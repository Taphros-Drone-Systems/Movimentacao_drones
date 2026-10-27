"""
navigation.py

Controle de navegação do veículo.
Suporta goto, velocity control e yaw control.

Integrado com:
- Connection (envio de mensagens)
- StateManager (leitura de estado)
- Dispatcher (eventos)
- TransactionManager (ACK handling)
"""

import threading
import time
import math
from enum import Enum, auto
from typing import Optional, Tuple, Callable

from pymavlink import mavutil


class NavResult(Enum):
    """Resultados de comandos de navegação."""
    SUCCESS = auto()
    TIMEOUT = auto()
    FAILED = auto()
    REJECTED = auto()
    DISCONNECTED = auto()
    INVALID_TARGET = auto()
    NOT_ARMED = auto()


class PositionTarget:
    """
    Representa um alvo de posição para navegação.
    """
    
    def __init__(
        self,
        latitude: Optional[float] = None,
        longitude: Optional[float] = None,
        altitude: Optional[float] = None,
        yaw: Optional[float] = None
    ):
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude
        self.yaw = yaw

    def __repr__(self):
        return (f"PositionTarget(lat={self.latitude}, "
                f"lon={self.longitude}, alt={self.altitude}, "
                f"yaw={self.yaw})")


class NavigationController:
    """
    Controlador de navegação do veículo.
    
    Funcionalidades:
    - goto: Vai para coordenada específica
    - goto_relative: Move relativo à posição atual
    - velocity_control: Controle por velocidade
    - yaw_control: Controle de orientação
    - takeoff_guided: Decolagem em modo GUIDED
    - land_guided: Pouso em modo GUIDED
    """

    def __init__(self, connection, dispatcher, state, transaction):
        self._connection = connection
        self._dispatcher = dispatcher
        self._state = state
        self._transaction = transaction

        # Estado do controle de navegação
        self._navigation_active = False
        self._target_position: Optional[PositionTarget] = None
        self._lock = threading.RLock()

        # Monitor de posição (verifica chegada ao alvo)
        self._monitor_thread: Optional[threading.Thread] = None
        self._monitor_running = False
        self._arrival_callback: Optional[Callable] = None

    # ===================================================================
    # GOTO - Navegação para posição absoluta
    # ===================================================================

    def goto(self, latitude: float, longitude: float, altitude: float,
         yaw: Optional[float] = None, 
         ground_speed: float = 5.0,
         acceptance_radius: float = 2.0,
         timeout: float = 30.0,
         callback: Optional[Callable] = None) -> NavResult:
        """
        Navega para uma posição absoluta.
        Versão otimizada para drone real com Pixhawk.
        """
        # Verifica se está armado
        if not self._state.is_armed():
            print(f"[Navigation] ERRO: Veículo não está armado")
            return NavResult.NOT_ARMED

        # Verifica modo GUIDED
        current_mode = self._state.flight_mode()
        if current_mode != "GUIDED":
            print(f"[Navigation] ERRO: Modo atual é {current_mode}, necessário GUIDED")
            return NavResult.REJECTED

        # Verifica se tem GPS fix (necessário para navegação)
        if not self._state.has_gps_fix():
            print(f"[Navigation] ERRO: Sem GPS fix 3D")
            return NavResult.REJECTED

        # Define alvo
        target = PositionTarget(latitude, longitude, altitude, yaw)
        
        with self._lock:
            self._target_position = target
            self._navigation_active = True

        print(f"[Navigation] GOTO:")
        print(f"  Lat: {latitude:.8f}")
        print(f"  Lon: {longitude:.8f}")
        print(f"  Alt: {altitude:.1f}m")
        print(f"  Velocidade: {ground_speed}m/s")

        # Comando padrão do ArduPilot para navegação em modo GUIDED
        # MAV_CMD_DO_REPOSITION = 84
        msg = self._connection.master.mav.command_int_send(
	    self._connection.master.target_system,
	    self._connection.master.target_component,
	    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
	    mavutil.mavlink.MAV_CMD_DO_REPOSITION,
	    0,  # current
	    0,  # autocontinue
	    ground_speed,  # param1
	    1,  # param2 (CRÍTICO: define como 1)
	    0,  # param3
	    0,  # param4
	    int(latitude * 1e7),  # x (latitude)
	    int(longitude * 1e7),  # y (longitude)
	    altitude,  # z (altitude)
	    0,  # unused
	)
        
        self._connection.send(msg)
        print(f"[Navigation] Comando MAV_CMD_DO_REPOSITION (84) enviado")

        # Aguarda resultado
        result = self._wait_arrival(
            latitude, longitude, altitude,
            acceptance_radius, timeout, callback
        )

        return result

    def goto_relative(self, north: float, east: float, up: float = 0,
                     yaw: Optional[float] = None,
                     timeout: float = 30.0) -> NavResult:
        """
        Move relativo à posição atual (em metros).
        
        Args:
            north: Deslocamento norte (metros)
            east: Deslocamento leste (metros)
            up: Deslocamento vertical (metros)
            yaw: Yaw alvo (graus, opcional)
            timeout: Timeout (segundos)
            
        Returns:
            NavResult
        """
        current_pos = self._state.position
        
        if current_pos.latitude is None or current_pos.longitude is None:
            print(f"[Navigation] ERRO: Posição atual desconhecida")
            return NavResult.INVALID_TARGET

        # Fatores de conversão precisos
        # 1 grau de latitude ≈ 111,111 metros
        lat_m_per_deg = 111111.0
        
        # 1 grau de longitude ≈ 111,111 * cos(latitude) metros
        lon_m_per_deg = 111111.0 * math.cos(math.radians(current_pos.latitude))

        # Converte metros para graus
        dlat = north / lat_m_per_deg
        dlon = east / lon_m_per_deg

        new_lat = current_pos.latitude + dlat
        new_lon = current_pos.longitude + dlon
        
        # Altitude (up é relativo à posição atual)
        current_alt = current_pos.relative_altitude or 0
        new_alt = current_alt + up
        
        print(f"[Navigation] GOTO RELATIVE:")
        print(f"  Deslocamento: N={north}m, E={east}m, U={up}m")
        print(f"  Pos atual: lat={current_pos.latitude:.8f}, lon={current_pos.longitude:.8f}, alt={current_alt:.1f}m")
        print(f"  Delta: dlat={dlat:.8f}°, dlon={dlon:.8f}°")
        print(f"  Novo alvo: lat={new_lat:.8f}, lon={new_lon:.8f}, alt={new_alt:.1f}m")

        return self.goto(new_lat, new_lon, new_alt, yaw, timeout=timeout)

    # ===================================================================
    # CONTROLE DE VELOCIDADE
    # ===================================================================

    def velocity_control(self, vx: float, vy: float, vz: float = 0,
                        yaw: Optional[float] = None,
                        duration: Optional[float] = None) -> NavResult:
        """
        Controle por velocidade (modo GUIDED).
        
        Args:
            vx: Velocidade X (norte, m/s)
            vy: Velocidade Y (leste, m/s)
            vz: Velocidade Z (up negativo, m/s)
            yaw: Yaw alvo (graus)
            duration: Duração do comando (None = contínuo)
            
        Returns:
            NavResult
        """
        if not self._state.is_armed():
            return NavResult.NOT_ARMED

        current_mode = self._state.flight_mode()
        if current_mode != "GUIDED":
            return NavResult.REJECTED

        # Type mask para velocidade
        type_mask = 0b0000111111000111  # Usa velocidades, ignora posição

        if yaw is not None:
            type_mask = 0b0000111111000011  # Inclui yaw

        msg = self._connection.master.mav.set_position_target_global_int_encode(
            0,
            self._connection.master.target_system,
            self._connection.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            type_mask,
            0, 0, 0,  # posição (ignorada)
            vx, vy, vz,  # velocidades
            0, 0, 0,  # acelerações (ignoradas)
            yaw if yaw is not None else 0.0,
            0
        )
        self._connection.send(msg)

        with self._lock:
            self._navigation_active = True

        if duration is not None:
            time.sleep(duration)
            self.stop()
            self._navigation_active = False

        return NavResult.SUCCESS

    def velocity_body(self, vx: float, vy: float, vz: float = 0,
                     duration: Optional[float] = None) -> NavResult:
        """
        Controle por velocidade no referencial do corpo (body frame).
        
        Args:
            vx: Velocidade X (frente, m/s)
            vy: Velocidade Y (direita, m/s)
            vz: Velocidade Z (baixo, m/s)
            duration: Duração do comando
            
        Returns:
            NavResult
        """
        if not self._state.is_armed():
            return NavResult.NOT_ARMED

        msg = self._connection.master.mav.set_position_target_local_ned_encode(
            0,
            self._connection.master.target_system,
            self._connection.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111,  # type_mask (velocidades)
            0, 0, 0,  # posição (ignorada)
            vx, vy, vz,  # velocidades
            0, 0, 0,  # acelerações (ignoradas)
            0, 0  # yaw, yaw_rate (ignorados)
        )
        self._connection.send(msg)

        if duration is not None:
            time.sleep(duration)
            self.stop()

        return NavResult.SUCCESS

    # ===================================================================
    # CONTROLE DE YAW
    # ===================================================================

    def yaw_control(self, yaw: float, is_relative: bool = False,
                   direction: int = 0) -> NavResult:
        """
        Controla o yaw do veículo.
        
        Args:
            yaw: Ângulo alvo (graus)
            is_relative: True = yaw relativo ao atual
            direction: 1 = horário, -1 = anti-horário, 0 = caminho mais curto
            
        Returns:
            NavResult
        """
        # command_long MAV_CMD_CONDITION_YAW
        msg = self._connection.master.mav.command_long_encode(
            self._connection.master.target_system,
            self._connection.master.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,  # confirmation
            yaw,  # param1: ângulo
            0,    # param2: velocidade (0 = default)
            direction,  # param3: direção
            1 if is_relative else 0,  # param4: relativo
            0,  # param5: unused
            0,  # param6: unused
            0   # param7: unused
        )
        self._connection.send(msg)
        return NavResult.SUCCESS

    def yaw_rate_control(self, yaw_rate: float, duration: Optional[float] = None) -> NavResult:
        """
        Controla taxa de yaw.
        
        Args:
            yaw_rate: Taxa de yaw (graus/s, positivo = horário)
            duration: Duração (None = contínuo)
            
        Returns:
            NavResult
        """
        msg = self._connection.master.mav.set_position_target_global_int_encode(
            0,
            self._connection.master.target_system,
            self._connection.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111011111,  # type_mask (só yaw_rate)
            0, 0, 0, 0, 0, 0, 0, 0, 0,
            0.0,  # yaw (ignorado)
            yaw_rate
        )
        self._connection.send(msg)

        if duration is not None:
            time.sleep(duration)
            self.stop()

        return NavResult.SUCCESS

    # ===================================================================
    # CONTROLE DE ALTITUDE
    # ===================================================================

    def set_altitude(self, altitude: float, ground_speed: float = 1.0,
                    timeout: float = 30.0) -> NavResult:
        """
        Altera altitude mantendo posição horizontal atual.
        
        Args:
            altitude: Altitude alvo (metros)
            ground_speed: Velocidade vertical (m/s)
            timeout: Timeout
            
        Returns:
            NavResult
        """
        pos = self._state.position
        
        if pos.latitude is None or pos.longitude is None:
            return NavResult.INVALID_TARGET

        return self.goto(
            pos.latitude,
            pos.longitude,
            altitude,
            ground_speed=ground_speed,
            acceptance_radius=0.5,
            timeout=timeout
        )

    # ===================================================================
    # UTILITÁRIOS DE ESPERA
    # ===================================================================

    def _wait_arrival(self, latitude: float, longitude: float, altitude: float,
                      acceptance_radius: float, timeout: float,
                      callback: Optional[Callable]) -> NavResult:
        """
        Aguarda chegada ao alvo com monitoramento.
        """
        start_time = time.time()
        last_print_time = 0
        last_distance = None

        print(f"[Navigation] Aguardando chegada ao destino...")

        while time.time() - start_time < timeout:
            pos = self._state.position

            if pos.latitude is None or pos.longitude is None:
                time.sleep(0.5)
                continue

            # Calcula distância horizontal
            distance = self._calculate_distance(
                pos.latitude, pos.longitude,
                latitude, longitude
            )

            # Calcula erro vertical
            current_alt = pos.relative_altitude or 0
            alt_error = abs(current_alt - altitude)

            # Mostra progresso a cada 2 segundos
            now = time.time()
            if now - last_print_time >= 2.0:
                # Mostra se está se movendo
                if last_distance is not None and abs(last_distance - distance) < 0.5:
                    print(f"  [AVISO] Drone parece estar parado. Distância: {distance:.1f}m")
                else:
                    print(f"  Distância: {distance:.1f}m | Erro alt: {alt_error:.1f}m | "
                        f"Alt atual: {current_alt:.1f}m | Alvo: {altitude:.1f}m")
                
                last_print_time = now
                last_distance = distance

            # Verifica chegada
            if distance <= acceptance_radius and alt_error <= max(acceptance_radius, 1.0):
                print(f"[Navigation] Chegou ao destino! Distância final: {distance:.1f}m")
                if callback:
                    try:
                        callback()
                    except Exception as e:
                        print(f"[Navigation] Erro no callback: {e}")

                with self._lock:
                    self._navigation_active = False
                    self._target_position = None

                return NavResult.SUCCESS

            time.sleep(0.5)

        # Timeout - mostra distância final
        pos = self._state.position
        if pos.latitude:
            final_dist = self._calculate_distance(pos.latitude, pos.longitude, latitude, longitude)
            final_alt_err = abs((pos.relative_altitude or 0) - altitude)
            print(f"  ⚠ Timeout! Dist final: {final_dist:.1f}m | Erro alt final: {final_alt_err:.1f}m")

        with self._lock:
            self._navigation_active = False
            self._target_position = None

        return NavResult.TIMEOUT

    @staticmethod
    def _calculate_distance(lat1: float, lon1: float, 
                           lat2: float, lon2: float) -> float:
        """
        Calcula distância horizontal entre duas coordenadas (metros).
        Fórmula de Haversine.
        """
        R = 6371000  # Raio da Terra em metros

        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)

        a = (math.sin(dlat/2)**2 + 
             math.cos(lat1_rad) * math.cos(lat2_rad) * 
             math.sin(dlon/2)**2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

        return R * c

    # ===================================================================
    # LAND / TAKEOFF GUIADO
    # ===================================================================

    def takeoff_guided(self, altitude: float, timeout: float = 30.0) -> NavResult:
        """
        Decolagem em modo GUIDED (sem missão).
        Similar ao comando takeoff do commands.py.
        
        Args:
            altitude: Altitude alvo (metros)
            timeout: Timeout
            
        Returns:
            NavResult
        """
        if not self._state.is_armed():
            return NavResult.NOT_ARMED

        current_mode = self._state.flight_mode()
        if current_mode != "GUIDED":
            return NavResult.REJECTED

        print(f"[Navigation] Takeoff guided para {altitude}m")

        # Comando NAV_TAKEOFF
        msg = self._connection.master.mav.command_long_encode(
            self._connection.master.target_system,
            self._connection.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,    # confirmation
            0,    # param1: pitch mínimo
            0,    # param2: unused
            0,    # param3: unused
            0,    # param4: yaw
            0,    # param5: latitude
            0,    # param6: longitude
            altitude  # param7: altitude
        )
        self._connection.send(msg)

        # Aguarda altitude
        start_time = time.time()
        while time.time() - start_time < timeout:
            current_alt = self._state.position.relative_altitude
            if current_alt is not None and current_alt >= altitude * 0.95:
                print(f"[Navigation] Takeoff concluído! Altitude: {current_alt:.1f}m")
                return NavResult.SUCCESS
            time.sleep(0.5)

        return NavResult.TIMEOUT

    def land_guided(self, timeout: float = 60.0) -> NavResult:
        """
        Pouso em modo GUIDED.
        
        Args:
            timeout: Timeout
            
        Returns:
            NavResult
        """
        current_mode = self._state.flight_mode()
        if current_mode != "GUIDED":
            return NavResult.REJECTED

        print(f"[Navigation] Land guided")

        msg = self._connection.master.mav.command_long_encode(
            self._connection.master.target_system,
            self._connection.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,    # confirmation
            0,    # param1: abort (0=land)
            0,    # param2: precision landing
            0,    # param3: unused
            0,    # param4: unused
            0,    # param5: latitude
            0,    # param6: longitude
            0     # param7: altitude
        )
        self._connection.send(msg)

        # Aguarda desarme (indica pouso completo)
        start_time = time.time()
        while time.time() - start_time < timeout:
            if not self._state.is_armed():
                print(f"[Navigation] Pouso concluído!")
                return NavResult.SUCCESS
            time.sleep(1)

        return NavResult.TIMEOUT

    # ===================================================================
    # PARADA DE NAVEGAÇÃO
    # ===================================================================

    def stop(self):
        """
        Para qualquer navegação ativa (envia velocidade zero).
        """
        msg = self._connection.master.mav.set_position_target_global_int_encode(
            0,
            self._connection.master.target_system,
            self._connection.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111000111,  # type_mask (velocidades)
            0, 0, 0,  # posição (ignorada)
            0, 0, 0,  # velocidades ZERO
            0, 0, 0,  # acelerações (ignoradas)
            0, 0
        )
        self._connection.send(msg)

        with self._lock:
            self._navigation_active = False
            self._target_position = None

    def loiter(self):
        """
        Ativa loiter (pairar na posição atual).
        """
        return self.stop()

    def is_navigating(self) -> bool:
        """
        Verifica se há navegação ativa.
        """
        with self._lock:
            return self._navigation_active

    def get_target(self) -> Optional[PositionTarget]:
        """
        Retorna o alvo atual de navegação.
        """
        with self._lock:
            return self._target_position

    # ===================================================================
    # CLEANUP
    # ===================================================================

    def stop_monitor(self):
        """
        Para thread de monitoramento.
        """
        self._monitor_running = False
        if self._monitor_thread:
            self._monitor_thread.join(timeout=2.0)

    def __repr__(self):
        return f"NavigationController(active={self.is_navigating()})"
