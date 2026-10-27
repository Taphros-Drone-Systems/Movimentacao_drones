"""
vehicle.py

API de alto nível que integra todos os módulos da biblioteca ppymavlink.

Fases implementadas:
- Fase 1: Base da comunicação (connection, reader, dispatcher)
- Fase 2: Estado interno (state, telemetry, heartbeat)
- Fase 3: Controle do veículo (commands, transaction)
- Fase 4: Recursos MAVLink avançados (parameters, mission, navigation)
- Fase 5: Robustez (failsafe)

Melhorias adicionadas:
- Logging estruturado
- Validação de entrada
- Rate limiting
- Health monitor
"""

from typing import Callable, Optional, List, Tuple, Any

from connection import MAVLinkConnection
from dispatcher import MAVLinkDispatcher
from state import StateManager
from telemetry import Telemetry
from heartbeat import HeartbeatManager
from reader import MAVLinkReader
from commands import VehicleCommands, CommandResult
from transaction import TransactionManager
from parameters import ParameterManager, ParamResult
from mission import MissionManager, MissionItem, MissionResult
from navigation import NavigationController, NavResult, PositionTarget
from failsafe import FailsafeManager, FailsafeState, FailsafeAction, LinkStats

# Novas importações para melhorias
from logging_config import get_logger, LoggerManager
from validators import (
    CoordinateValidator, AltitudeValidator, SpeedValidator,
    ParameterValidator, MissionValidator, ValidationResult, ValidationSeverity
)
from rate_limiter import MAVLinkRateLimiter, LimitStrategy
from health import HealthMonitor, HealthIssue, HealthSeverity


class Vehicle:
    """
    Classe principal que integra todos os subsistemas.
    
    Uso básico:
        drone = Vehicle("udp:127.0.0.1:14550")
        drone.connect()
        
        # Estado
        print(drone.state.flight_mode())
        print(drone.state.is_armed())
        
        # Comandos
        drone.arm()
        drone.takeoff(10)
        drone.land()
        
        # Parâmetros
        drone.parameters.download_all()
        value = drone.get_param("PARAM_NAME")
        
        # Missões
        drone.add_takeoff(10)
        drone.add_waypoint(lat, lon, alt)
        drone.add_land()
        drone.upload_mission()
        
        # Navegação
        drone.set_mode("GUIDED")
        drone.goto(lat, lon, alt)
        drone.set_velocity(5, 0, 0)
        
        # Failsafe
        drone.configure_failsafe(auto_land_on_failsafe=True)
        stats = drone.get_link_stats()
        
        # Melhorias
        drone.on_health_issues(lambda issues: print(f"Problemas: {issues}"))
        drone.configure_rate_limit("GOTO", 0.5)
        
        drone.disconnect()
    """

    def __init__(self, connection_string: str, baudrate: int = 57600):
        """
        Inicializa o veículo com todos os subsistemas.
        
        Args:
            connection_string: String de conexão MAVLink
                              Ex: "udp:127.0.0.1:14550"
                              Ex: "/dev/ttyACM0"
                              Ex: "tcp:127.0.0.1:5760"
            baudrate: Baud rate para conexões seriais
        """
        # Configura logging
        LoggerManager.setup(level="INFO")
        self.logger = get_logger("vehicle")
        self.logger.info("Inicializando Vehicle...")
        
        # ===================================================================
        # Fase 1 — Base da comunicação
        # ===================================================================
        self.connection = MAVLinkConnection(connection_string, baudrate)
        self.dispatcher = MAVLinkDispatcher()

        # ===================================================================
        # Fase 2 — Estado interno do veículo
        # ===================================================================
        self.state = StateManager()
        self.telemetry = Telemetry(self.dispatcher, self.state)
        self.heartbeat = HeartbeatManager(self.connection, self.dispatcher)
        self.reader = MAVLinkReader(self.connection, self.dispatcher)

        # ===================================================================
        # Fase 3 — Controle do veículo
        # ===================================================================
        self.commands = VehicleCommands(
            self.connection, self.dispatcher, self.state, self.heartbeat
        )
        self.transaction = TransactionManager(self.dispatcher)

        # ===================================================================
        # Fase 4 — Recursos MAVLink avançados
        # ===================================================================
        self.parameters = ParameterManager(
            self.connection, self.dispatcher, self.transaction
        )
        self.mission = MissionManager(
            self.connection, self.dispatcher, self.transaction
        )
        self.navigation = NavigationController(
            self.connection, self.dispatcher, self.state, self.transaction
        )

        # ===================================================================
        # Fase 5 — Robustez
        # ===================================================================
        self.failsafe = FailsafeManager(
            self.connection, self.dispatcher, self.heartbeat,
            self.commands, self.state
        )

        # ===================================================================
        # Melhorias
        # ===================================================================
        self.rate_limiter = MAVLinkRateLimiter()
        self.health = HealthMonitor(self)

        # Estado interno
        self._is_connected = False
        self._connection_string = connection_string
        self._baudrate = baudrate
        
        self.logger.info(f"Vehicle inicializado com connection_string={connection_string}")

    # ===================================================================
    # CONEXÃO / DESCONEXÃO
    # ===================================================================

    def connect(self) -> bool:
        """
        Conecta ao veículo e inicia todos os subsistemas.
        
        Ordem de inicialização:
        1. Conexão MAVLink
        2. Aguarda HEARTBEAT
        3. Reader (loop de leitura)
        4. Heartbeat Manager (envio e monitoramento)
        5. Transaction Manager (ACKs)
        6. Failsafe Manager (monitoramento de link)
        7. Rate Limiter
        8. Health Monitor
        
        Returns:
            bool: True se conectado com sucesso
        """
        try:
            self.logger.info(f"Conectando em {self._connection_string}...")
            self.connection.connect()
            self.connection.wait_heartbeat()
            self._is_connected = True
            self.logger.info(f"Conectado - SysID: {self.connection.master.target_system}")
        except Exception as e:
            self.logger.error(f"Falha na conexão: {e}")
            return False

        # Inicia subsistemas
        self.reader.start()
        self.heartbeat.start()
        self.transaction.start()
        self.failsafe.start()
        
        # Inicia melhorias
        self.rate_limiter.start()
        self.health.start()

        self.logger.info("Todos os subsistemas iniciados")
        return True

    def disconnect(self):
        """
        Desconecta e para todos os subsistemas de forma ordenada.
        
        Ordem de parada:
        1. Failsafe (para monitoramento e reconexão)
        2. Health Monitor
        3. Rate Limiter
        4. Navigation (para monitoramento)
        5. Mission (limpa handlers)
        6. Parameters (limpa downloads)
        7. Commands (limpa ACKs pendentes)
        8. Transaction (cancela transações)
        9. Heartbeat (para envio/monitoramento)
        10. Reader (para loop de leitura)
        11. Telemetry (remove handlers)
        12. Conexão (fecha porta/soquete)
        """
        self.logger.info("Desconectando...")

        # Para subsistemas na ordem correta
        if hasattr(self, 'failsafe'):
            self.failsafe.stop()

        if hasattr(self, 'health'):
            self.health.stop()

        if hasattr(self, 'rate_limiter'):
            self.rate_limiter.stop()

        if hasattr(self, 'navigation'):
            self.navigation.stop_monitor()

        if hasattr(self, 'mission'):
            self.mission.stop()

        if hasattr(self, 'parameters'):
            self.parameters.stop()

        if hasattr(self, 'commands'):
            self.commands.stop()

        if hasattr(self, 'transaction'):
            self.transaction.stop()

        if hasattr(self, 'heartbeat'):
            self.heartbeat.stop()

        if hasattr(self, 'reader'):
            self.reader.stop()

        if hasattr(self, 'telemetry'):
            self.telemetry.stop()

        if hasattr(self, 'connection'):
            self.connection.disconnect()

        self._is_connected = False
        self.logger.info("Desconectado")

    # ===================================================================
    # PROPRIEDADES
    # ===================================================================

    @property
    def is_connected(self) -> bool:
        """
        Verifica se está conectado e com link ativo.
        """
        return self._is_connected and self.heartbeat.connected

    @property
    def connection_string(self) -> str:
        """Retorna a string de conexão."""
        return self._connection_string

    @property
    def target_system(self) -> int:
        """Retorna o ID do sistema alvo."""
        if self.connection.master:
            return self.connection.master.target_system
        return 0

    @property
    def target_component(self) -> int:
        """Retorna o ID do componente alvo."""
        if self.connection.master:
            return self.connection.master.target_component
        return 0

    # ===================================================================
    # MÉTODOS DE VALIDAÇÃO
    # ===================================================================

    def validate_coordinates(self, lat: float, lon: float) -> ValidationResult:
        """
        Valida coordenadas geográficas.
        
        Args:
            lat: Latitude em graus (-90 a 90)
            lon: Longitude em graus (-180 a 180)
        
        Returns:
            ValidationResult com resultado da validação
        """
        return CoordinateValidator.validate_coordinates(lat, lon)
    
    def validate_altitude(self, altitude: float, operation: str = None) -> ValidationResult:
        """
        Valida altitude.
        
        Args:
            altitude: Altitude em metros
            operation: Tipo de operação ('takeoff', 'land', 'waypoint', 'loiter')
        
        Returns:
            ValidationResult com resultado da validação
        """
        return AltitudeValidator.validate_altitude(altitude, operation)
    
    def validate_ground_speed(self, speed: float) -> ValidationResult:
        """
        Valida velocidade de solo.
        
        Args:
            speed: Velocidade em m/s
        
        Returns:
            ValidationResult com resultado da validação
        """
        return SpeedValidator.validate_ground_speed(speed)
    
    def validate_vertical_speed(self, speed: float) -> ValidationResult:
        """
        Valida velocidade vertical.
        
        Args:
            speed: Velocidade vertical em m/s
        
        Returns:
            ValidationResult com resultado da validação
        """
        return SpeedValidator.validate_vertical_speed(speed)
    
    def validate_yaw_rate(self, rate: float) -> ValidationResult:
        """
        Valida taxa de yaw.
        
        Args:
            rate: Taxa de yaw em graus/s
        
        Returns:
            ValidationResult com resultado da validação
        """
        return SpeedValidator.validate_yaw_rate(rate)
    
    def validate_param(self, param_name: str, value: float = None) -> ValidationResult:
        """
        Valida parâmetro.
        
        Args:
            param_name: Nome do parâmetro
            value: Valor do parâmetro (opcional)
        
        Returns:
            ValidationResult com resultado da validação
        """
        # Valida nome
        name_result = ParameterValidator.validate_param_name(param_name)
        if not name_result:
            return name_result
        
        # Valida valor se fornecido
        if value is not None:
            return ParameterValidator.validate_param_value(param_name, value)
        
        return ValidationResult(True, "OK")

    # ===================================================================
    # FASE 3 — COMANDOS DE CONTROLE
    # ===================================================================

    def arm(self, timeout: float = 5.0) -> CommandResult:
        """
        Arma os motores do veículo.
        
        Args:
            timeout: Tempo máximo de espera pelo ACK
            
        Returns:
            CommandResult
        """
        self.logger.info("Armando veículo...")
        return self.commands.arm(timeout)

    def disarm(self, timeout: float = 5.0) -> CommandResult:
        """
        Desarma os motores do veículo.
        
        Args:
            timeout: Tempo máximo de espera pelo ACK
            
        Returns:
            CommandResult
        """
        self.logger.info("Desarmando veículo...")
        return self.commands.disarm(timeout)

    def takeoff(self, altitude: float, timeout: float = 10.0) -> CommandResult:
        """
        Inicia decolagem até a altitude especificada (metros).
        Requer veículo armado e modo GUIDED.
        
        Args:
            altitude: Altitude alvo em metros
            timeout: Tempo máximo de espera pelo ACK
            
        Returns:
            CommandResult
        """
        # Valida altitude
        alt_result = self.validate_altitude(altitude, 'takeoff')
        if not alt_result:
            self.logger.error(f"Altitude inválida: {alt_result.message}")
            return CommandResult.FAILED
        elif alt_result.severity == ValidationSeverity.WARNING:
            self.logger.warning(alt_result.message)
        
        # Verifica pré-condições
        if not self.state.is_armed():
            self.logger.warning("Veículo não armado antes da decolagem")
        
        self.logger.info(f"Takeoff para {altitude}m")
        return self.commands.takeoff(altitude, timeout)

    def land(self, timeout: float = 10.0) -> CommandResult:
        """
        Inicia pouso na posição atual.
        Força modo LAND se necessário.
        
        Args:
            timeout: Tempo máximo de espera pelo ACK
            
        Returns:
            CommandResult
        """
        self.logger.info("Iniciando pouso...")
        return self.commands.land(timeout)

    def set_mode(self, mode: str, timeout: float = 3.0) -> CommandResult:
        """
        Altera o modo de voo do veículo.
        
        Args:
            mode: Modo desejado ('GUIDED', 'AUTO', 'LOITER', 'LAND', etc.)
            timeout: Tempo máximo de espera pelo ACK
            
        Returns:
            CommandResult
        """
        self.logger.info(f"Alterando modo para {mode}...")
        return self.commands.set_mode(mode, timeout)

    def reboot(self, autopilot_only: bool = True, timeout: float = 5.0) -> CommandResult:
        """
        Reinicia o autopilot ou sistema completo.
        
        Args:
            autopilot_only: True = reinicia só o autopilot
                           False = reinicia o sistema operacional inteiro
            timeout: Tempo máximo de espera pelo ACK
            
        Returns:
            CommandResult
        """
        reboot_type = "autopilot" if autopilot_only else "sistema completo"
        self.logger.warning(f"Solicitando reboot do {reboot_type}...")
        return self.commands.reboot(autopilot_only, timeout)

    # ===================================================================
    # FASE 4 — PARÂMETROS
    # ===================================================================

    def get_param(self, param_name: str, timeout: float = 3.0):
        """
        Lê um parâmetro do veículo (com cache).
        
        Args:
            param_name: Nome do parâmetro (ex: 'MIS_TAKEOFF_ALT')
            timeout: Timeout em segundos
            
        Returns:
            Tuple[ParamResult, Optional[ParamValue]]
        """
        # Valida nome do parâmetro
        result = self.validate_param(param_name)
        if not result:
            self.logger.warning(result.message)
        
        return self.parameters.get_param(param_name, timeout)

    def set_param(self, param_name: str, value: float, timeout: float = 3.0) -> ParamResult:
        """
        Escreve um parâmetro no veículo.
        
        Args:
            param_name: Nome do parâmetro
            value: Novo valor
            timeout: Timeout em segundos
            
        Returns:
            ParamResult
        """
        # Valida parâmetro
        result = self.validate_param(param_name, value)
        if not result:
            self.logger.error(f"Parâmetro inválido: {result.message}")
            return ParamResult.INVALID_VALUE
        elif result.severity == ValidationSeverity.WARNING:
            self.logger.warning(result.message)
        
        # Rate limiting
        if not self.rate_limiter.can_send("PARAM_SET"):
            self.logger.warning("Rate limit: aguarde antes de enviar outro comando PARAM_SET")
        
        self.logger.info(f"Set param: {param_name} = {value}")
        return self.parameters.set_param(param_name, value, timeout)

    def download_params(self, timeout: float = 30.0, progress_callback=None) -> ParamResult:
        """
        Faz download de todos os parâmetros do veículo.
        
        Args:
            timeout: Timeout total em segundos
            progress_callback: Callback opcional para progresso
            
        Returns:
            ParamResult
        """
        self.logger.info("Iniciando download de parâmetros...")
        return self.parameters.download_all(timeout, progress_callback)

    def find_params(self, pattern: str) -> dict:
        """
        Busca parâmetros por nome parcial.
        
        Args:
            pattern: Padrão de busca (case-insensitive)
            
        Returns:
            Dict com parâmetros encontrados
        """
        return self.parameters.find_params(pattern)

    def get_all_params(self) -> dict:
        """Retorna todos os parâmetros em cache."""
        return self.parameters.get_all_params()

    def save_params(self, filepath: str) -> bool:
        """Exporta parâmetros para arquivo."""
        return self.parameters.export_params(filepath)

    def load_params(self, filepath: str) -> bool:
        """Importa parâmetros de arquivo (apenas cache)."""
        return self.parameters.import_params(filepath)

    # ===================================================================
    # FASE 4 — MISSÕES
    # ===================================================================

    def upload_mission(self, items=None, timeout: float = 30.0,
                      progress_callback=None) -> MissionResult:
        """
        Envia missão para o veículo.
        
        Args:
            items: Lista de MissionItem (usa cache interno se None)
            timeout: Timeout total
            progress_callback: Callback para progresso
            
        Returns:
            MissionResult
        """
        # Valida tamanho da missão
        mission_items = items if items is not None else self.mission.get_mission()
        
        size_result = MissionValidator.validate_mission_size(len(mission_items))
        if not size_result:
            self.logger.error(size_result.message)
            return MissionResult.ERROR
        elif size_result.severity == ValidationSeverity.WARNING:
            self.logger.warning(size_result.message)
        
        # Rate limiting
        if not self.rate_limiter.can_send("MISSION_UPLOAD"):
            self.logger.warning("Rate limit: aguarde antes de enviar outra missão")
        
        self.logger.info(f"Uploading mission: {len(mission_items)} items")
        return self.mission.upload(items, timeout, progress_callback)

    def download_mission(self, timeout: float = 30.0,
                        progress_callback=None):
        """
        Baixa missão do veículo.
        
        Args:
            timeout: Timeout total
            progress_callback: Callback para progresso
            
        Returns:
            Tuple[MissionResult, List[MissionItem]]
        """
        self.logger.info("Downloading mission...")
        return self.mission.download(timeout, progress_callback)

    def clear_mission(self, timeout: float = 10.0) -> MissionResult:
        """Limpa missão do veículo."""
        self.logger.info("Limpando missão...")
        return self.mission.clear(timeout)

    def add_waypoint(self, latitude: float, longitude: float, altitude: float,
                    delay: float = 0.0, acceptance_radius: float = 2.0) -> MissionItem:
        """
        Adiciona waypoint à missão local.
        """
        # Valida coordenadas
        coord_result = self.validate_coordinates(latitude, longitude)
        if not coord_result:
            self.logger.error(f"Coordenadas inválidas: {coord_result.message}")
            raise ValueError(coord_result.message)
        
        # Valida altitude
        alt_result = self.validate_altitude(altitude, 'waypoint')
        if not alt_result:
            self.logger.error(f"Altitude inválida: {alt_result.message}")
            raise ValueError(alt_result.message)
        
        seq = self.mission.mission_count()
        item = MissionItem.waypoint(seq, latitude, longitude, altitude, delay, acceptance_radius)
        self.mission.add_item(item)
        self.logger.debug(f"Waypoint {seq} adicionado: ({latitude}, {longitude}, {altitude})")
        return item

    def add_takeoff(self, altitude: float = 10.0) -> MissionItem:
        """Adiciona comando de decolagem à missão local."""
        # Valida altitude
        alt_result = self.validate_altitude(altitude, 'takeoff')
        if not alt_result:
            self.logger.error(f"Altitude inválida: {alt_result.message}")
            raise ValueError(alt_result.message)
        
        item = MissionItem.takeoff(0, altitude)
        self.mission.add_item(item)
        self.logger.debug(f"Takeoff adicionado: {altitude}m")
        return item

    def add_land(self) -> MissionItem:
        """Adiciona comando de pouso à missão local."""
        item = MissionItem.land(0)
        self.mission.add_item(item)
        self.logger.debug("Land adicionado")
        return item

    def add_rtl(self) -> MissionItem:
        """Adiciona comando Return to Launch à missão local."""
        item = MissionItem.rtl(0)
        self.mission.add_item(item)
        self.logger.debug("RTL adicionado")
        return item

    def add_loiter(self, time_seconds: float, radius: float = 10.0,
                  latitude: float = 0, longitude: float = 0,
                  altitude: float = 0) -> MissionItem:
        """Adiciona comando de loiter à missão local."""
        seq = self.mission.mission_count()
        item = MissionItem.loiter(seq, time_seconds, latitude, longitude, altitude, radius)
        self.mission.add_item(item)
        self.logger.debug(f"Loiter adicionado: {time_seconds}s, radius={radius}m")
        return item

    def add_loiter_unlimited(self, radius: float = 10.0,
                            latitude: float = 0, longitude: float = 0,
                            altitude: float = 0) -> MissionItem:
        """Adiciona loiter ilimitado à missão local."""
        seq = self.mission.mission_count()
        item = MissionItem.loiter_unlimited(seq, latitude, longitude, altitude, radius)
        self.mission.add_item(item)
        self.logger.debug(f"Loiter unlimited adicionado: radius={radius}m")
        return item

    def upload_waypoints(self, waypoints: list, takeoff_alt: float = 10.0,
                        include_takeoff: bool = True,
                        include_land: bool = True) -> MissionResult:
        """
        Converte lista de coordenadas em missão e faz upload.
        """
        # Valida waypoints
        seq_result = MissionValidator.validate_waypoint_sequence(waypoints)
        if not seq_result:
            self.logger.error(seq_result.message)
            return MissionResult.ERROR
        
        # Valida altitude de decolagem
        alt_result = self.validate_altitude(takeoff_alt, 'takeoff')
        if not alt_result:
            self.logger.warning(f"Altitude de decolagem: {alt_result.message}")
        
        self.logger.info(f"Uploading {len(waypoints)} waypoints, takeoff={takeoff_alt}m")
        return self.mission.upload_waypoints(waypoints, takeoff_alt, include_takeoff, include_land)

    def get_mission(self) -> list:
        """Retorna missão atual (cache local)."""
        return self.mission.get_mission()

    def get_mission_item(self, seq: int):
        """Retorna item de missão por sequência."""
        return self.mission.get_item(seq)

    def mission_count(self) -> int:
        """Retorna número de itens na missão."""
        return self.mission.mission_count()

    def clear_local_mission(self):
        """Limpa missão local (cache, não afeta o veículo)."""
        self.mission.set_mission([])
        self.logger.debug("Missão local limpa")

    def export_mission(self, filepath: str) -> bool:
        """Exporta missão para arquivo (formato QGC WPL)."""
        return self.mission.export_mission(filepath)

    # ===================================================================
    # FASE 4 — NAVEGAÇÃO
    # ===================================================================

    def goto(self, latitude: float, longitude: float, altitude: float,
             yaw: float = None, ground_speed: float = 5.0,
             acceptance_radius: float = 2.0, timeout: float = 30.0,
             callback: Optional[Callable] = None) -> NavResult:
        """
        Navega para posição absoluta (requer modo GUIDED).
        """
        # Valida coordenadas
        coord_result = self.validate_coordinates(latitude, longitude)
        if not coord_result:
            self.logger.error(f"Coordenadas inválidas: {coord_result.message}")
            return NavResult.INVALID_TARGET
        
        # Valida altitude
        alt_result = self.validate_altitude(altitude, 'waypoint')
        if not alt_result:
            self.logger.error(f"Altitude inválida: {alt_result.message}")
            return NavResult.INVALID_TARGET
        elif alt_result.severity == ValidationSeverity.WARNING:
            self.logger.warning(alt_result.message)
        
        # Valida velocidade
        speed_result = self.validate_ground_speed(ground_speed)
        if not speed_result:
            self.logger.warning(f"Velocidade: {speed_result.message}")
        
        # Verifica rate limiting
        if not self.rate_limiter.can_send("GOTO"):
            self.logger.warning("Rate limit: aguarde antes de enviar outro comando GOTO")
            return NavResult.REJECTED
        
        self.logger.info(f"GOTO: ({latitude:.6f}, {longitude:.6f}, {altitude:.1f}m) speed={ground_speed}m/s")
        
        return self.navigation.goto(
            latitude, longitude, altitude, yaw,
            ground_speed, acceptance_radius, timeout, callback
        )

    def goto_relative(self, north: float, east: float, up: float = 0,
                     yaw: float = None, timeout: float = 30.0) -> NavResult:
        """
        Move relativo à posição atual.
        """
        # Valida deslocamentos (limites razoáveis)
        if abs(north) > 1000:
            self.logger.warning(f"Deslocamento norte muito grande: {north}m")
        if abs(east) > 1000:
            self.logger.warning(f"Deslocamento leste muito grande: {east}m")
        if abs(up) > 500:
            self.logger.warning(f"Deslocamento vertical muito grande: {up}m")
        
        # Verifica posição atual
        current_pos = self.state.position
        if current_pos.latitude is None or current_pos.longitude is None:
            self.logger.error("Posição atual desconhecida")
            return NavResult.INVALID_TARGET
        
        self.logger.info(f"GOTO relative: N={north}m, E={east}m, U={up}m")
        
        return self.navigation.goto_relative(north, east, up, yaw, timeout)

    def set_velocity(self, vx: float, vy: float, vz: float = 0,
                    yaw: float = None, duration: float = None) -> NavResult:
        """
        Controle por velocidade (modo GUIDED).
        """
        # Valida velocidades
        h_speed = (vx**2 + vy**2)**0.5
        speed_result = self.validate_ground_speed(h_speed)
        if not speed_result:
            self.logger.warning(speed_result.message)
        
        v_speed_result = self.validate_vertical_speed(vz)
        if not v_speed_result:
            self.logger.warning(v_speed_result.message)
        
        # Rate limiting
        if not self.rate_limiter.can_send("VELOCITY"):
            self.logger.warning("Rate limit: velocidade controlada")
        
        self.logger.info(f"Set velocity: vx={vx}, vy={vy}, vz={vz}")
        
        return self.navigation.velocity_control(vx, vy, vz, yaw, duration)

    def set_velocity_body(self, vx: float, vy: float, vz: float = 0,
                         duration: float = None) -> NavResult:
        """Controle por velocidade no referencial do corpo."""
        self.logger.info(f"Set body velocity: vx={vx}, vy={vy}, vz={vz}")
        return self.navigation.velocity_body(vx, vy, vz, duration)

    def set_yaw(self, yaw: float, is_relative: bool = False,
               direction: int = 0) -> NavResult:
        """Controla orientação (yaw) do veículo."""
        self.logger.info(f"Set yaw: {yaw}°, relative={is_relative}")
        return self.navigation.yaw_control(yaw, is_relative, direction)

    def set_yaw_rate(self, yaw_rate: float, duration: float = None) -> NavResult:
        """Controla taxa de rotação (yaw rate)."""
        # Valida taxa de yaw
        rate_result = self.validate_yaw_rate(yaw_rate)
        if not rate_result:
            self.logger.warning(rate_result.message)
        
        self.logger.info(f"Set yaw rate: {yaw_rate}°/s")
        return self.navigation.yaw_rate_control(yaw_rate, duration)

    def set_altitude(self, altitude: float, ground_speed: float = 1.0,
                    timeout: float = 30.0) -> NavResult:
        """Altera altitude mantendo posição horizontal."""
        # Valida altitude
        alt_result = self.validate_altitude(altitude)
        if not alt_result:
            self.logger.error(f"Altitude inválida: {alt_result.message}")
            return NavResult.INVALID_TARGET
        
        self.logger.info(f"Set altitude: {altitude}m, speed={ground_speed}m/s")
        return self.navigation.set_altitude(altitude, ground_speed, timeout)

    def stop_navigation(self):
        """Para qualquer navegação ativa (loiter)."""
        self.logger.info("Parando navegação")
        self.navigation.stop()

    def loiter(self):
        """Ativa modo loiter (pairar na posição atual)."""
        self.logger.info("Ativando loiter")
        self.navigation.loiter()

    def is_navigating(self) -> bool:
        """Verifica se há navegação ativa."""
        return self.navigation.is_navigating()

    def get_navigation_target(self):
        """Retorna alvo atual de navegação."""
        return self.navigation.get_target()

    def takeoff_guided(self, altitude: float, timeout: float = 30.0) -> NavResult:
        """Decolagem em modo GUIDED (alternativa ao takeoff)."""
        # Valida altitude
        alt_result = self.validate_altitude(altitude, 'takeoff')
        if not alt_result:
            self.logger.error(f"Altitude inválida: {alt_result.message}")
            return NavResult.INVALID_TARGET
        
        self.logger.info(f"Takeoff guided: {altitude}m")
        return self.navigation.takeoff_guided(altitude, timeout)

    def land_guided(self, timeout: float = 60.0) -> NavResult:
        """Pouso em modo GUIDED."""
        self.logger.info("Land guided")
        return self.navigation.land_guided(timeout)

    # ===================================================================
    # FASE 5 — FAILSAFE
    # ===================================================================

    def get_link_stats(self) -> dict:
        """Retorna estatísticas do link de comunicação."""
        return self.failsafe.get_stats_dict()

    def print_link_stats(self):
        """Imprime estatísticas do link no console."""
        self.failsafe.print_stats()

    def get_failsafe_state(self) -> FailsafeState:
        """Retorna estado atual do failsafe."""
        return self.failsafe.state

    @property
    def is_failsafe_active(self) -> bool:
        """
        Verifica se failsafe está ativo.
        CORRIGIDO: Agora chama corretamente o método.
        """
        return self.failsafe.is_failsafe_active

    def reconnect(self) -> bool:
        """Tenta reconectar manualmente ao veículo."""
        self.logger.info("Tentando reconexão manual...")
        return self.failsafe.reconnect()

    def configure_failsafe(self, **kwargs):
        """Configura parâmetros do failsafe."""
        self.logger.info(f"Configurando failsafe: {kwargs}")
        self.failsafe.configure(**kwargs)

    def reset_link_stats(self):
        """Reseta estatísticas do link."""
        self.failsafe.reset_stats()

    # ===================================================================
    # MELHORIAS — HEALTH MONITOR
    # ===================================================================

    def get_health_issues(self) -> list:
        """Retorna problemas de saúde atuais."""
        return self.health.get_issues()
    
    def get_health_history(self) -> list:
        """Retorna histórico de problemas."""
        return self.health.get_history()
    
    def get_health_stats(self) -> dict:
        """Retorna estatísticas do health monitor."""
        return self.health.get_stats()
    
    def on_health_issues(self, callback: Callable):
        """
        Registra callback para problemas de saúde.
        
        Args:
            callback: Função chamada quando problemas são detectados
                     Recebe lista de HealthIssue como argumento
        """
        self.health.on_issues(callback)
    
    def kick_watchdog(self):
        """Atualiza watchdog (chamar ao receber heartbeat)."""
        self.health.kick_watchdog()

    # ===================================================================
    # MELHORIAS — RATE LIMITER
    # ===================================================================

    def configure_rate_limit(self, msg_type: str, interval: float):
        """
        Configura intervalo de rate limit para tipo de mensagem.
        
        Args:
            msg_type: Tipo da mensagem ('GOTO', 'VELOCITY', 'PARAM_SET', etc.)
            interval: Intervalo mínimo entre mensagens (segundos)
        """
        self.rate_limiter.set_interval(msg_type, interval)
        self.logger.info(f"Rate limit configurado: {msg_type} = {interval}s")
    
    def configure_rate_limit_strategy(self, msg_type: str, strategy: LimitStrategy):
        """
        Configura estratégia de rate limit para tipo de mensagem.
        
        Args:
            msg_type: Tipo da mensagem
            strategy: Estratégia (DROP, QUEUE, BLOCK)
        """
        self.rate_limiter.set_strategy(msg_type, strategy)
        self.logger.info(f"Rate limit strategy: {msg_type} = {strategy.value}")
    
    def get_rate_limiter_stats(self) -> dict:
        """Retorna estatísticas do rate limiter."""
        return self.rate_limiter.get_stats()

    # ===================================================================
    # UTILITÁRIOS
    # ===================================================================

    def wait_ready(self, timeout: float = 30.0) -> bool:
        """
        Aguarda o veículo estar pronto (conectado + telemetria).
        """
        import time
        start = time.time()

        while time.time() - start < timeout:
            if self.is_connected:
                state = self.state.get_state()
                if (state.position.latitude is not None and
                    state.system.system_status is not None):
                    self.logger.info("Veículo pronto!")
                    return True
            time.sleep(0.5)

        self.logger.warning(f"Timeout aguardando veículo pronto ({timeout}s)")
        return False

    def get_status_dict(self) -> dict:
        """
        Retorna um dicionário com o estado completo do veículo.
        """
        state = self.state.get_state()
        link_stats = self.failsafe.get_stats_dict()
        health_stats = self.get_health_stats()
        rate_stats = self.get_rate_limiter_stats()

        return {
            'connected': self.is_connected,
            'mode': state.system.flight_mode,
            'armed': state.system.armed,
            'battery': {
                'voltage': state.battery.voltage,
                'current': state.battery.current,
                'remaining': state.battery.remaining
            },
            'position': {
                'latitude': state.position.latitude,
                'longitude': state.position.longitude,
                'altitude_rel': state.position.relative_altitude,
                'altitude_abs': state.position.absolute_altitude,
                'heading': state.position.heading
            },
            'attitude': {
                'roll': state.attitude.roll,
                'pitch': state.attitude.pitch,
                'yaw': state.attitude.yaw
            },
            'gps': {
                'fix_type': state.gps.fix_type,
                'satellites': state.gps.satellites_visible
            },
            'link': {
                'state': self.heartbeat.state.name,
                'heartbeat_age': self.heartbeat.heartbeat_age,
                'connected': self.heartbeat.connected,
                'packet_loss': link_stats['packet_loss_rate'],
                'failsafe_state': link_stats['state']
            },
            'parameters_cached': self.parameters.param_count(),
            'mission_items': self.mission.mission_count(),
            'navigating': self.navigation.is_navigating(),
            'health': {
                'active_issues': health_stats.get('active_issues', 0),
                'issues_detected': health_stats.get('issues_detected', 0),
                'watchdog_fired': health_stats.get('watchdog_fired', False)
            },
            'rate_limiter': {
                'intervals': rate_stats.get('intervals', {}),
                'queue_sizes': rate_stats.get('queue_sizes', {})
            }
        }
    
    def print_status(self):
        """
        Imprime status resumido do veículo no console.
        """
        s = self.get_status_dict()

        print("\n" + "=" * 60)
        print("STATUS DO VEÍCULO")
        print("=" * 60)
        print(f"Conexão: {'OK' if s['connected'] else 'FALHA'}")
        print(f"Modo: {s['mode']} | Armado: {'SIM' if s['armed'] else 'NÃO'}")
        
        # Bateria - trata None
        voltage = s['battery']['voltage']
        remaining = s['battery']['remaining']
        voltage_str = f"{voltage:.1f}V" if voltage is not None else "N/A"
        remaining_str = f"{remaining}%" if remaining is not None else "N/A"
        print(f"Bateria: {voltage_str} {remaining_str}")
        
        # Posição - trata None
        lat = s['position']['latitude']
        lon = s['position']['longitude']
        if lat is not None and lon is not None:
            print(f"Posição: Lat={lat:.6f} Lon={lon:.6f}")
        else:
            print("Posição: N/A")
        
        # Altitude - trata None
        alt = s['position']['altitude_rel']
        alt_str = f"{alt:.1f}m" if alt is not None else "N/A"
        print(f"Altitude: {alt_str}")
        
        # GPS - trata None
        fix = s['gps']['fix_type']
        sats = s['gps']['satellites']
        fix_str = f"Fix {fix}" if fix is not None else "N/A"
        sats_str = f"({sats} sat)" if sats is not None else ""
        print(f"GPS: {fix_str} {sats_str}")
        
        # Link
        print(f"Link: {s['link']['state']} | Failsafe: {s['link']['failsafe_state']}")
        print(f"Packet Loss: {s['link']['packet_loss']}")
        
        # Health
        print(f"Saúde: {s['health']['active_issues']} problemas ativos")
        
        # Outros
        print(f"Parâmetros: {s['parameters_cached']} em cache")
        print(f"Missão: {s['mission_items']} itens")
        print(f"Navegando: {'SIM' if s['navigating'] else 'NÃO'}")
        print("=" * 60)

    # ===================================================================
    # CONTEXTO (with statement)
    # ===================================================================

    def __enter__(self):
        """
        Suporte para context manager (with statement).
        """
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Garante desconexão ao sair do contexto."""
        self.disconnect()
        return False

    # ===================================================================
    # REPRESENTAÇÃO
    # ===================================================================

    def __repr__(self):
        status = "conectado" if self.is_connected else "desconectado"
        failsafe = f" | failsafe={self.failsafe.state.name}" if hasattr(self, 'failsafe') else ""
        health = f" | issues={len(self.get_health_issues())}" if hasattr(self, 'health') else ""
        return (
            f"Vehicle({self._connection_string}, "
            f"status={status}, "
            f"mode={self.state.flight_mode()}, "
            f"armed={self.state.is_armed()}{failsafe}{health})"
        )

    def __str__(self):
        return self.__repr__()