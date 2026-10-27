"""
commands.py

Camada de comandos MAVLink para controle do veículo.
Totalmente integrado com Connection, Dispatcher, StateManager e HeartbeatManager.
"""

import time
from enum import Enum, auto
from threading import Event, RLock

from pymavlink import mavutil


class CommandResult(Enum):
    SUCCESS = auto()
    TIMEOUT = auto()
    DENIED = auto()
    FAILED = auto()
    DISCONNECTED = auto()


class VehicleCommands:
    """
    Implementa comandos de controle do veículo.
    
    Dependências:
    - connection: MAVLinkConnection (envio de mensagens)
    - dispatcher: MAVLinkDispatcher (eventos)
    - state: StateManager (verificação de estado)
    - heartbeat: HeartbeatManager (monitoramento de conexão)
    """

    def __init__(self, connection, dispatcher, state, heartbeat):
        self._connection = connection
        self._dispatcher = dispatcher
        self._state = state
        self._heartbeat = heartbeat

        # Controle de ACK assíncrono via dispatcher
        self._pending_ack = None
        self._ack_event = Event()
        self._ack_result = None
        self._ack_lock = RLock()

        # Registrar listener COMMAND_ACK
        self._dispatcher.register("COMMAND_ACK", self._on_command_ack)

        # Monitorar desconexão
        self._dispatcher.register("link_disconnected", self._on_link_disconnected)
        self._disconnected = False

    # =====================================================
    # ARM / DISARM
    # =====================================================

    def arm(self, timeout: float = 5.0) -> CommandResult:
        """
        Arma os motores do veículo.
        Bloqueia até receber ACK ou timeout.
        """
        # Verificar se já está armado
        if self._state.is_armed():
            return CommandResult.SUCCESS

        # Diagnóstico pré-arm
        current_mode = self._state.flight_mode()
        has_gps = self._state.has_gps_fix()
        battery = self._state.battery
        
        print(f"[Commands] Tentando armar...")
        print(f"  Modo: {current_mode}")
        print(f"  GPS fix: {has_gps}")
        if battery.voltage is not None:
            print(f"  Bateria: {battery.voltage:.1f}V")
        
        # Lista de verificações comuns que podem impedir o arm
        if not has_gps:
            print("[Commands] ⚠ Aviso: Sem GPS fix 3D - ARM pode falhar")
        
        msg = self._connection.master.mav.command_long_encode(
            self._connection.master.target_system,
            self._connection.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,    # confirmation
            1,    # param1: 1 = arm
            0,    # param2: force (0 = safety checks)
            0,    # param3: unused
            0,    # param4: unused
            0,    # param5: unused
            0,    # param6: unused
            0     # param7: unused
        )

        self._connection.send(msg)
        result = self._wait_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, timeout)
        
        if result != CommandResult.SUCCESS:
            print(f"[Commands] ARM falhou: {result}")
            print("[Commands] Possíveis causas:")
            print("  - Pre-arm checks não passaram")
            print("  - GPS fix inadequado (precisa 3D fix ou melhor)")
            print("  - Modo não permite armar (use GUIDED, STABILIZE ou AUTO)")
            print("  - Battery failsafe ativo")
            print("  - Safety switch não pressionado")
            print("  - Arming check (ARMING_CHECK) está habilitado")
            print("  - RC não calibrado")
            print("  - Acelerômetro não calibrado")
            print("  - Barômetro não saudável")
        else:
            print("[Commands] ARM bem sucedido")
        
        return result

    def disarm(self, timeout: float = 5.0) -> CommandResult:
        """
        Desarma os motores do veículo.
        """
        # Verificar se já está desarmado
        if not self._state.is_armed():
            return CommandResult.SUCCESS

        msg = self._connection.master.mav.command_long_encode(
            self._connection.master.target_system,
            self._connection.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,    # confirmation
            0,    # param1: 0 = disarm
            0,    # param2: force (0 = normal, 21196 = force disarm)
            0,    # param3: unused
            0,    # param4: unused
            0,    # param5: unused
            0,    # param6: unused
            0     # param7: unused
        )

        self._connection.send(msg)
        result = self._wait_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, timeout)
        
        if result != CommandResult.SUCCESS:
            print(f"[Commands] DISARM falhou: {result}")
        
        return result

    def force_disarm(self, timeout: float = 5.0) -> CommandResult:
        """
        Força desarme dos motores (ignora safety checks).
        CUIDADO: Pode desarmar em voo!
        """
        if not self._state.is_armed():
            return CommandResult.SUCCESS

        msg = self._connection.master.mav.command_long_encode(
            self._connection.master.target_system,
            self._connection.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,    # confirmation
            0,    # param1: 0 = disarm
            21196,  # param2: magic number para force disarm
            0,    # param3: unused
            0,    # param4: unused
            0,    # param5: unused
            0,    # param6: unused
            0     # param7: unused
        )

        self._connection.send(msg)
        return self._wait_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, timeout)

    # =====================================================
    # TAKEOFF / LAND
    # =====================================================

    def takeoff(self, altitude: float, timeout: float = 10.0) -> CommandResult:
        """
        Inicia decolagem até altitude especificada (metros).
        Requer veículo armado e modo GUIDED.
        """
        # Verificar pré-condições
        if not self._state.is_armed():
            print("[Commands] Takeoff falhou: veículo não armado")
            return CommandResult.DENIED

        current_mode = self._state.flight_mode()
        print(f"[Commands] Modo atual para takeoff: {current_mode}")
        
        if current_mode != "GUIDED":
            # Tenta mudar para GUIDED automaticamente
            print(f"[Commands] Mudando para GUIDED (atual: {current_mode})...")
            result = self.set_mode("GUIDED", timeout=3.0)
            if result != CommandResult.SUCCESS:
                print(f"[Commands] Não foi possível mudar para GUIDED: {result}")
                return result
            time.sleep(0.5)  # Aguarda modo estabilizar

        msg = self._connection.master.mav.command_long_encode(
            self._connection.master.target_system,
            self._connection.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,          # confirmation
            0,          # param1: pitch (0 = default)
            0,          # param2: unused
            0,          # param3: unused
            0,          # param4: yaw (0 = heading atual)
            0,          # param5: latitude (0 = atual)
            0,          # param6: longitude (0 = atual)
            altitude    # param7: altitude em metros
        )

        self._connection.send(msg)
        result = self._wait_ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, timeout)
        
        if result == CommandResult.SUCCESS:
            print(f"[Commands] Takeoff iniciado: altitude alvo = {altitude}m")
        else:
            print(f"[Commands] Takeoff falhou: {result}")
        
        return result

    def land(self, timeout: float = 10.0) -> CommandResult:
        """
        Inicia pouso na posição atual.
        Funciona em qualquer modo (força LAND se necessário).
        """
        current_mode = self._state.flight_mode()
        print(f"[Commands] Modo atual para land: {current_mode}")
        
        if current_mode != "LAND":
            print(f"[Commands] Mudando para LAND (atual: {current_mode})...")
            result = self.set_mode("LAND", timeout=3.0)
            if result != CommandResult.SUCCESS:
                print(f"[Commands] Não foi possível mudar para LAND: {result}")
                return result
            time.sleep(0.5)

        msg = self._connection.master.mav.command_long_encode(
            self._connection.master.target_system,
            self._connection.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,    # confirmation
            0,    # param1: abort (0 = land, 1 = abort landing)
            0,    # param2: precision landing mode
            0,    # param3: unused
            0,    # param4: unused
            0,    # param5: latitude (0 = atual)
            0,    # param6: longitude (0 = atual)
            0     # param7: altitude (0 = ground level)
        )

        self._connection.send(msg)
        result = self._wait_ack(mavutil.mavlink.MAV_CMD_NAV_LAND, timeout)
        
        if result == CommandResult.SUCCESS:
            print("[Commands] Land iniciado")
        else:
            print(f"[Commands] Land falhou: {result}")
        
        return result

    def abort_landing(self, timeout: float = 5.0) -> CommandResult:
        """
        Aborta pouso em andamento.
        """
        msg = self._connection.master.mav.command_long_encode(
            self._connection.master.target_system,
            self._connection.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,    # confirmation
            1,    # param1: 1 = abort landing
            0,    # param2: unused
            0,    # param3: unused
            0,    # param4: unused
            0,    # param5: unused
            0,    # param6: unused
            0     # param7: unused
        )

        self._connection.send(msg)
        return self._wait_ack(mavutil.mavlink.MAV_CMD_NAV_LAND, timeout)

    # =====================================================
    # MODE CHANGE
    # =====================================================

    def set_mode(self, mode: str, timeout: float = 3.0) -> CommandResult:
        """
        Altera o modo de voo do veículo.
        mode: string como 'GUIDED', 'AUTO', 'LAND', 'LOITER', etc.
        """
        mode_upper = mode.upper()
        
        # Verificar se já está no modo desejado
        current_mode = self._state.flight_mode()
        if current_mode == mode_upper:
            print(f"[Commands] Já está no modo {mode_upper}")
            return CommandResult.SUCCESS

        # Obtém o ID do modo para o autopilot atual
        mode_mapping = self._connection.master.mode_mapping()
        
        if mode_mapping is None:
            print("[Commands] Erro: mode_mapping() retornou None")
            return CommandResult.FAILED
        
        mode_id = mode_mapping.get(mode_upper)
        if mode_id is None:
            # Tenta lowercase também
            mode_id = mode_mapping.get(mode.lower())
        
        if mode_id is None:
            # Lista modos disponíveis para diagnóstico
            available_modes = list(mode_mapping.keys())[:20]
            print(f"[Commands] Modo desconhecido: {mode}")
            print(f"[Commands] Modos disponíveis: {', '.join(available_modes)}...")
            return CommandResult.FAILED

        print(f"[Commands] Mudando modo: {current_mode} -> {mode_upper} (ID: {mode_id})")

        # Envia comando de mudança de modo via MAV_CMD_DO_SET_MODE
        msg = self._connection.master.mav.command_long_encode(
            self._connection.master.target_system,
            self._connection.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,                                                        # confirmation
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,        # param1: base mode
            mode_id,                                                  # param2: custom mode
            0, 0, 0, 0, 0                                             # param3-7: unused
        )

        self._connection.send(msg)
        result = self._wait_ack(mavutil.mavlink.MAV_CMD_DO_SET_MODE, timeout)
        
        if result == CommandResult.SUCCESS:
            # Aguarda confirmação da mudança
            time.sleep(0.3)
            new_mode = self._state.flight_mode()
            if new_mode == mode_upper:
                print(f"[Commands] Modo alterado com sucesso: {new_mode}")
            else:
                print(f"[Commands] ACK recebido mas modo ainda é {new_mode} (aguardando...)")
        else:
            print(f"[Commands] Falha ao mudar modo: {result}")
        
        return result

    # =====================================================
    # REBOOT / SHUTDOWN
    # =====================================================

    def reboot(self, autopilot_only: bool = True, timeout: float = 5.0) -> CommandResult:
        """
        Reinicia o autopilot ou sistema completo.

        Parâmetros:
            autopilot_only: True = reinicia só o autopilot
                           False = reinicia o sistema operacional inteiro
        """
        reboot_type = "autopilot" if autopilot_only else "sistema completo"
        print(f"[Commands] Solicitando reboot do {reboot_type}...")
        
        msg = self._connection.master.mav.command_long_encode(
            self._connection.master.target_system,
            self._connection.master.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            0,                         # confirmation
            1 if autopilot_only else 2,  # param1: 1=autopilot, 2=OS
            0,                         # param2: unused
            0,                         # param3: unused
            0,                         # param4: unused
            0,                         # param5: unused
            0,                         # param6: unused
            0                          # param7: unused
        )

        self._connection.send(msg)
        return self._wait_ack(mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, timeout)

    def shutdown(self, timeout: float = 5.0) -> CommandResult:
        """
        Desliga o sistema completamente.
        """
        print("[Commands] Solicitando shutdown...")
        
        msg = self._connection.master.mav.command_long_encode(
            self._connection.master.target_system,
            self._connection.master.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            0,    # confirmation
            0,    # param1: 0 = shutdown
            0,    # param2: unused
            0,    # param3: unused
            0,    # param4: unused
            0,    # param5: unused
            0,    # param6: unused
            0     # param7: unused
        )

        self._connection.send(msg)
        return self._wait_ack(mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, timeout)

    # =====================================================
    # RTL (Return to Launch)
    # =====================================================

    def rtl(self, timeout: float = 3.0) -> CommandResult:
        """
        Ativa Return to Launch.
        Equivalente a set_mode("RTL").
        """
        return self.set_mode("RTL", timeout)

    # =====================================================
    # MODO HOLD / LOITER
    # =====================================================

    def hold(self, timeout: float = 3.0) -> CommandResult:
        """
        Ativa modo HOLD (pairar na posição).
        Tenta LOITER primeiro, depois HOLD.
        """
        # Tenta LOITER primeiro (mais comum em ArduPilot)
        result = self.set_mode("LOITER", timeout=2.0)
        if result == CommandResult.SUCCESS:
            return result
        
        # Tenta HOLD (PX4)
        return self.set_mode("HOLD", timeout)

    # =====================================================
    # MODO AUTO
    # =====================================================

    def auto(self, timeout: float = 3.0) -> CommandResult:
        """
        Ativa modo AUTO (executar missão).
        """
        return self.set_mode("AUTO", timeout)

    # =====================================================
    # MODO STABILIZE
    # =====================================================

    def stabilize(self, timeout: float = 3.0) -> CommandResult:
        """
        Ativa modo STABILIZE.
        """
        return self.set_mode("STABILIZE", timeout)

    # =====================================================
    # MODO ALT_HOLD
    # =====================================================

    def alt_hold(self, timeout: float = 3.0) -> CommandResult:
        """
        Ativa modo ALT_HOLD.
        """
        return self.set_mode("ALT_HOLD", timeout)

    # =====================================================
    # MODO POSHOLD
    # =====================================================

    def pos_hold(self, timeout: float = 3.0) -> CommandResult:
        """
        Ativa modo POSHOLD.
        """
        return self.set_mode("POSHOLD", timeout)

    # =====================================================
    # ACK HANDLING (INTEGRADO COM DISPATCHER)
    # =====================================================

    def _on_command_ack(self, msg):
        """
        Callback do dispatcher para COMMAND_ACK.
        Integrado com o sistema de eventos, sem polling bloqueante.
        """
        with self._ack_lock:
            if self._pending_ack is not None and msg.command == self._pending_ack:
                if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    self._ack_result = CommandResult.SUCCESS
                elif msg.result == mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED:
                    self._ack_result = CommandResult.DENIED
                elif msg.result == mavutil.mavlink.MAV_RESULT_UNSUPPORTED:
                    self._ack_result = CommandResult.FAILED
                elif msg.result == mavutil.mavlink.MAV_RESULT_DENIED:
                    self._ack_result = CommandResult.DENIED
                elif msg.result == mavutil.mavlink.MAV_RESULT_FAILED:
                    self._ack_result = CommandResult.FAILED
                else:
                    self._ack_result = CommandResult.FAILED

                self._ack_event.set()

    def _on_link_disconnected(self):
        """
        Callback para detectar desconexão durante comandos.
        """
        self._disconnected = True
        # Se estiver esperando ACK, cancela
        with self._ack_lock:
            if self._pending_ack is not None:
                self._ack_result = CommandResult.DISCONNECTED
                self._ack_event.set()

    def _wait_ack(self, command_id: int, timeout: float) -> CommandResult:
        """
        Aguarda COMMAND_ACK para um comando específico.
        
        USA O DISPATCHER/EVENTOS ao invés de polling direto.
        Totalmente integrado com o loop de leitura existente.
        """
        # Reset do estado
        self._disconnected = False
        self._ack_event.clear()

        with self._ack_lock:
            self._pending_ack = command_id
            self._ack_result = None

        # Aguarda o ACK via evento (não-bloqueante para o reader)
        if not self._ack_event.wait(timeout=timeout):
            # Timeout
            with self._ack_lock:
                self._pending_ack = None
            return CommandResult.TIMEOUT

        # Resultado recebido
        with self._ack_lock:
            result = self._ack_result or CommandResult.FAILED
            self._pending_ack = None
            self._ack_result = None

        return result

    # =====================================================
    # UTILITÁRIOS
    # =====================================================

    def get_available_modes(self) -> list:
        """
        Retorna lista de modos disponíveis.
        """
        mode_mapping = self._connection.master.mode_mapping()
        if mode_mapping:
            return list(mode_mapping.keys())
        return []

    def print_available_modes(self):
        """
        Imprime modos disponíveis.
        """
        modes = self.get_available_modes()
        if modes:
            print("[Commands] Modos disponíveis:")
            for i, mode in enumerate(sorted(modes)):
                print(f"  {i+1:2d}. {mode}")
        else:
            print("[Commands] Não foi possível obter modos disponíveis")

    # =====================================================
    # CLEANUP
    # =====================================================

    def stop(self):
        """
        Remove listeners do dispatcher e limpa recursos.
        """
        self._dispatcher.unregister("COMMAND_ACK", self._on_command_ack)
        self._dispatcher.unregister("link_disconnected", self._on_link_disconnected)

        # Libera qualquer thread esperando ACK
        with self._ack_lock:
            if self._pending_ack is not None:
                self._ack_result = CommandResult.FAILED
                self._ack_event.set()
        
        print("[Commands] Parado")

    def __repr__(self):
        return (f"VehicleCommands(armed={self._state.is_armed()}, "
                f"mode={self._state.flight_mode()})")