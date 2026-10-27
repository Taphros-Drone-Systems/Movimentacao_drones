"""
failsafe.py

Sistema de failsafe e recuperação de conexão.
Monitora a saúde do link e toma ações automáticas.

Funcionalidades:
- Reconexão automática
- Monitoramento de heartbeat timeout
- Monitoramento de packet loss
- Ações automáticas em falhas
- Callbacks de eventos de failsafe
- Recuperação de estado
"""

import threading
import time
from enum import Enum, auto
from typing import Optional, Callable, Dict
from dataclasses import dataclass, field
from collections import deque


class FailsafeState(Enum):
    """Estados do sistema de failsafe."""
    NORMAL = auto()           # Operação normal
    WARNING = auto()          # Sinais de degradação
    CRITICAL = auto()         # Falha iminente
    FAILSAFE = auto()         # Failsafe ativo
    RECOVERING = auto()       # Tentando recuperar
    DISCONNECTED = auto()     # Totalmente desconectado


class FailsafeAction(Enum):
    """Ações automáticas em caso de failsafe."""
    NONE = auto()             # Nenhuma ação
    LAND = auto()             # Iniciar pouso
    RTL = auto()              # Return to Launch
    LOITER = auto()           # Pairar na posição
    HOLD = auto()             # Manter posição
    DISARM = auto()           # Desarmar (CUIDADO!)


@dataclass
class LinkStats:
    """Estatísticas do link de comunicação."""
    packets_sent: int = 0
    packets_received: int = 0
    packets_lost: int = 0
    last_packet_time: float = 0.0
    
    # Janela deslizante para packet loss recente
    recent_packets: deque = field(default_factory=lambda: deque(maxlen=100))
    
    @property
    def packet_loss_rate(self) -> float:
        """Taxa de perda de pacotes recente (0.0 a 1.0)."""
        if not self.recent_packets:
            return 0.0
        return self.recent_packets.count(False) / len(self.recent_packets)
    
    @property
    def total_packets(self) -> int:
        """Total de pacotes (enviados + recebidos)."""
        return self.packets_sent + self.packets_received


@dataclass
class FailsafeConfig:
    """Configuração do sistema de failsafe."""
    # Heartbeat timeout (segundos)
    heartbeat_timeout: float = 3.0
    
    # Máximo de reconexões
    max_reconnect_attempts: int = 5
    
    # Intervalo entre tentativas de reconexão (segundos)
    reconnect_interval: float = 2.0
    
    # Backoff exponencial para reconexão
    reconnect_backoff: bool = True
    max_reconnect_interval: float = 30.0
    
    # Packet loss
    packet_loss_warning: float = 0.3    # 30% de perda = warning
    packet_loss_critical: float = 0.5   # 50% de perda = critical
    
    # Ações automáticas
    auto_land_on_failsafe: bool = True
    auto_rtl_on_failsafe: bool = False
    
    # Tempo máximo em failsafe antes de ação (segundos)
    failsafe_action_delay: float = 5.0
    
    # Recuperação
    auto_reconnect: bool = True
    auto_arm_after_reconnect: bool = False  # CUIDADO: segurança
    
    # Tempo de estabilização após conexão (evita falsos positivos)
    stabilization_time: float = 5.0


class FailsafeManager:
    """
    Gerenciador de failsafe e recuperação de conexão.
    
    Responsabilidades:
    - Monitorar saúde do link
    - Detectar falhas de comunicação
    - Executar ações automáticas de recuperação
    - Notificar outros módulos sobre mudanças de estado
    - Registrar estatísticas de link
    """

    def __init__(self, connection, dispatcher, heartbeat, commands, state):
        """
        Args:
            connection: MAVLinkConnection
            dispatcher: MAVLinkDispatcher
            heartbeat: HeartbeatManager
            commands: VehicleCommands
            state: StateManager
        """
        self._connection = connection
        self._dispatcher = dispatcher
        self._heartbeat = heartbeat
        self._commands = commands
        self._state = state

        # Configuração
        self.config = FailsafeConfig()
        
        # Estado atual
        self._failsafe_state = FailsafeState.DISCONNECTED
        self._lock = threading.RLock()
        
        # Estatísticas
        self._stats = LinkStats()
        
        # Tempo de início do sistema (para período de estabilização)
        self._start_time = 0.0
        
        # Controle de reconexão
        self._reconnect_attempts = 0
        self._last_reconnect_time = 0.0
        self._reconnect_event = threading.Event()
        self._reconnect_thread: Optional[threading.Thread] = None
        
        # Controle de failsafe
        self._failsafe_start_time: Optional[float] = None
        self._failsafe_action_taken = False
        self._was_armed_before_failsafe = False
        self._mode_before_failsafe: Optional[str] = None
        
        # Monitor thread
        self._running = False
        self._monitor_thread: Optional[threading.Thread] = None
        
        # Callbacks
        self._callbacks: Dict[str, list] = {
            'on_failsafe': [],
            'on_recover': [],
            'on_disconnect': [],
            'on_reconnect': [],
            'on_state_change': [],
            'on_packet_loss': [],
        }
        
        # Registra handlers no dispatcher
        self._dispatcher.register("heartbeat_received", self._on_packet_received)
        self._dispatcher.register("heartbeat_timeout", self._on_heartbeat_timeout)
        self._dispatcher.register("link_connected", self._on_link_connected)
        self._dispatcher.register("link_disconnected", self._on_link_disconnected)
        
        # Registra para monitorar mensagens (packet stats)
        self._dispatcher.register("*", self._on_any_message)

    # ===================================================================
    # INÍCIO / PARADA
    # ===================================================================

    def start(self):
        """
        Inicia monitoramento de failsafe.
        """
        with self._lock:
            if self._running:
                return
            self._running = True
            # Marca tempo de início para período de estabilização
            self._start_time = time.time()
            # Inicializa estatísticas
            self._stats.last_packet_time = time.time()
            self._stats.packets_received = 0
            self._stats.packets_lost = 0
            self._stats.recent_packets.clear()
        
        self._monitor_thread = threading.Thread(
            target=self._monitor_loop,
            daemon=True,
            name="FailsafeMonitor"
        )
        self._monitor_thread.start()
        
        print("[Failsafe] Monitoramento iniciado")

    def stop(self):
        """
        Para monitoramento.
        """
        with self._lock:
            self._running = False
        
        if self._monitor_thread:
            self._monitor_thread.join(timeout=2.0)
        
        if self._reconnect_thread:
            self._reconnect_event.set()
            self._reconnect_thread.join(timeout=2.0)
        
        # Limpa handlers
        self._dispatcher.unregister("heartbeat_received", self._on_packet_received)
        self._dispatcher.unregister("heartbeat_timeout", self._on_heartbeat_timeout)
        self._dispatcher.unregister("link_connected", self._on_link_connected)
        self._dispatcher.unregister("link_disconnected", self._on_link_disconnected)
        self._dispatcher.unregister("*", self._on_any_message)
        
        print("[Failsafe] Monitoramento parado")

    # ===================================================================
    # PROPRIEDADES
    # ===================================================================

    @property
    def state(self) -> FailsafeState:
        """Estado atual do failsafe."""
        with self._lock:
            return self._failsafe_state

    @property
    def stats(self) -> LinkStats:
        """Estatísticas do link."""
        with self._lock:
            return self._stats

    @property
    def is_failsafe_active(self) -> bool:
        """Verifica se failsafe está ativo."""
        return self.state in [FailsafeState.CRITICAL, FailsafeState.FAILSAFE]
    
    def is_failsafe_active_method(self) -> bool:
        """
        Verifica se failsafe está ativo (versão método explícita).
        
        Returns:
            bool: True se failsafe está ativo
        """
        return self.state in [FailsafeState.CRITICAL, FailsafeState.FAILSAFE]

    @property
    def is_recovering(self) -> bool:
        """Verifica se está em recuperação."""
        return self.state == FailsafeState.RECOVERING

    # ===================================================================
    # CALLBACKS
    # ===================================================================

    def on_failsafe(self, callback: Callable):
        """
        Registra callback para evento de failsafe.
        callback(state: FailsafeState)
        """
        self._callbacks['on_failsafe'].append(callback)

    def on_recover(self, callback: Callable):
        """
        Registra callback para recuperação.
        callback()
        """
        self._callbacks['on_recover'].append(callback)

    def on_disconnect(self, callback: Callable):
        """
        Registra callback para desconexão.
        callback()
        """
        self._callbacks['on_disconnect'].append(callback)

    def on_reconnect(self, callback: Callable):
        """
        Registra callback para reconexão.
        callback()
        """
        self._callbacks['on_reconnect'].append(callback)

    def on_state_change(self, callback: Callable):
        """
        Registra callback para mudança de estado.
        callback(old_state, new_state)
        """
        self._callbacks['on_state_change'].append(callback)

    def on_packet_loss(self, callback: Callable):
        """
        Registra callback para perda de pacotes.
        callback(loss_rate: float)
        """
        self._callbacks['on_packet_loss'].append(callback)

    # ===================================================================
    # CONFIGURAÇÃO
    # ===================================================================

    def configure(self, **kwargs):
        """
        Atualiza configuração.
        
        Args:
            heartbeat_timeout: Timeout de heartbeat
            max_reconnect_attempts: Máximo de tentativas
            reconnect_interval: Intervalo entre tentativas
            auto_land_on_failsafe: Pouso automático
            auto_rtl_on_failsafe: RTL automático
            failsafe_action_delay: Delay antes da ação
            auto_reconnect: Reconexão automática
            stabilization_time: Tempo de estabilização após conexão
        """
        for key, value in kwargs.items():
            if hasattr(self.config, key):
                setattr(self.config, key, value)

    # ===================================================================
    # LOOP DE MONITORAMENTO
    # ===================================================================

    def _monitor_loop(self):
        """
        Loop principal de monitoramento.
        Verifica condições de failsafe periodicamente.
        """
        while self._is_running():
            try:
                self._check_conditions()
            except Exception as e:
                print(f"[Failsafe] Erro no monitor: {e}")
            
            time.sleep(0.5)

    def _check_conditions(self):
        """
        Verifica todas as condições de failsafe.
        """
        with self._lock:
            state = self._failsafe_state
        
        now = time.time()
        
        # Período de estabilização - não verifica logo após início
        if now - self._start_time < self.config.stabilization_time:
            return
        
        # Se está desconectado, tenta reconectar
        if state == FailsafeState.DISCONNECTED and self.config.auto_reconnect:
            self._attempt_reconnect()
            return
        
        # Se está em RECOVERING, não interfere
        if state == FailsafeState.RECOVERING:
            return
        
        with self._lock:
            stats = self._stats
        
        # 1. Verifica packet loss
        packet_loss = stats.packet_loss_rate
        
        if packet_loss >= self.config.packet_loss_critical:
            self._set_state(FailsafeState.CRITICAL)
            self._trigger_callbacks('on_packet_loss', packet_loss)
            
        elif packet_loss >= self.config.packet_loss_warning:
            if state == FailsafeState.NORMAL:
                self._set_state(FailsafeState.WARNING)
            self._trigger_callbacks('on_packet_loss', packet_loss)
        
        # 2. Verifica heartbeat age
        heartbeat_age = self._heartbeat.heartbeat_age
        
        if heartbeat_age is not None:
            if heartbeat_age > self.config.heartbeat_timeout * 3:
                # Muito tempo sem heartbeat
                self._handle_failsafe()
                
            elif heartbeat_age > self.config.heartbeat_timeout * 2:
                # Timeout estendido
                if state not in [FailsafeState.CRITICAL, FailsafeState.FAILSAFE, FailsafeState.RECOVERING]:
                    self._set_state(FailsafeState.CRITICAL)
        
        # 3. Verifica se ação de failsafe deve ser executada
        if state == FailsafeState.FAILSAFE and not self._failsafe_action_taken:
            if self._failsafe_start_time is not None:
                if now - self._failsafe_start_time >= self.config.failsafe_action_delay:
                    self._execute_failsafe_action()

    # ===================================================================
    # HANDLERS DE EVENTOS
    # ===================================================================

    def _on_packet_received(self, msg=None):
        """
        Registra recebimento de pacote (heartbeat recebido).
        """
        with self._lock:
            self._stats.packets_received += 1
            self._stats.last_packet_time = time.time()
            self._stats.recent_packets.append(True)
        
        # Se estava em warning/critical e recebeu heartbeat, recupera
        current_state = self.state
        if current_state in [FailsafeState.WARNING, FailsafeState.CRITICAL]:
            self._set_state(FailsafeState.NORMAL)
            self._trigger_callbacks('on_recover')

    def _on_heartbeat_timeout(self):
        """
        Handler para timeout de heartbeat.
        """
        self._set_state(FailsafeState.CRITICAL)

    def _on_link_connected(self):
        """
        Handler para conexão estabelecida.
        """
        with self._lock:
            self._reconnect_attempts = 0
            self._start_time = time.time()  # Reinicia período de estabilização
        
        self._set_state(FailsafeState.NORMAL)
        self._trigger_callbacks('on_reconnect')
        
        # Restaura estado anterior (com delay para estabilizar)
        threading.Thread(
            target=self._delayed_restore,
            daemon=True,
            name="StateRestore"
        ).start()

    def _on_link_disconnected(self):
        """
        Handler para desconexão.
        """
        self._handle_failsafe()
        self._trigger_callbacks('on_disconnect')

    def _on_any_message(self, msg):
        """
        Monitora todas as mensagens para estatísticas.
        """
        with self._lock:
            self._stats.packets_received += 1
            self._stats.last_packet_time = time.time()
            self._stats.recent_packets.append(True)

    # ===================================================================
    # GESTÃO DE FAILSAFE
    # ===================================================================

    def _handle_failsafe(self):
        """
        Inicia procedimento de failsafe.
        """
        with self._lock:
            if self._failsafe_state == FailsafeState.FAILSAFE:
                return  # Já está em failsafe
            
            # Salva estado atual do veículo
            self._was_armed_before_failsafe = self._state.is_armed()
            self._mode_before_failsafe = self._state.flight_mode()
            
            # Marca início do failsafe
            self._failsafe_start_time = time.time()
            self._failsafe_action_taken = False
        
        self._set_state(FailsafeState.FAILSAFE)
        self._trigger_callbacks('on_failsafe', self._failsafe_state)
        
        print("[Failsafe] FAILSAFE ATIVADO!")
        if self._mode_before_failsafe:
            print(f"[Failsafe] Estado anterior - Armado: {self._was_armed_before_failsafe}, "
                  f"Modo: {self._mode_before_failsafe}")

    def _execute_failsafe_action(self):
        """
        Executa ação automática de failsafe.
        """
        with self._lock:
            if self._failsafe_action_taken:
                return
            self._failsafe_action_taken = True
        
        print("[Failsafe] Executando ação de failsafe...")
        
        try:
            if self.config.auto_land_on_failsafe:
                print("[Failsafe] Enviando comando LAND...")
                self._commands.land(timeout=1.0)
                
            elif self.config.auto_rtl_on_failsafe:
                print("[Failsafe] Enviando comando RTL...")
                self._commands.set_mode("RTL", timeout=1.0)
                
        except Exception as e:
            print(f"[Failsafe] Erro ao executar ação: {e}")

    def _restore_vehicle_state(self):
        """
        Tenta restaurar estado do veículo após reconexão.
        """
        try:
            # Não rearma automaticamente por segurança
            if self.config.auto_arm_after_reconnect and self._was_armed_before_failsafe:
                print("[Failsafe] Rearmando veículo...")
                time.sleep(1)
                result = self._commands.arm(timeout=3.0)
                print(f"[Failsafe] Rearm: {result}")
            
            # Restaura modo anterior se não era failsafe
            if self._mode_before_failsafe and self._mode_before_failsafe not in ["LAND", "RTL", "UNKNOWN", "None", None]:
                current_mode = self._state.flight_mode()
                if current_mode != self._mode_before_failsafe:
                    print(f"[Failsafe] Restaurando modo: {self._mode_before_failsafe}")
                    self._commands.set_mode(self._mode_before_failsafe, timeout=3.0)
                    
        except Exception as e:
            print(f"[Failsafe] Erro ao restaurar estado: {e}")

    def _delayed_restore(self):
        """
        Restaura estado com delay para estabilização.
        """
        time.sleep(2.0)  # Aguarda estabilizar
        self._restore_vehicle_state()

    # ===================================================================
    # RECONEXÃO AUTOMÁTICA
    # ===================================================================

    def _attempt_reconnect(self):
        """
        Tenta reconectar ao veículo.
        """
        with self._lock:
            if self._reconnect_attempts >= self.config.max_reconnect_attempts:
                return
            
            now = time.time()
            
            # Calcula intervalo com backoff exponencial
            if self.config.reconnect_backoff:
                interval = min(
                    self.config.reconnect_interval * (2 ** self._reconnect_attempts),
                    self.config.max_reconnect_interval
                )
            else:
                interval = self.config.reconnect_interval
            
            if now - self._last_reconnect_time < interval:
                return
            
            self._last_reconnect_time = now
            self._reconnect_attempts += 1
            
            attempts = self._reconnect_attempts
        
        # Inicia thread de reconexão
        if self._reconnect_thread is None or not self._reconnect_thread.is_alive():
            self._set_state(FailsafeState.RECOVERING)
            
            self._reconnect_thread = threading.Thread(
                target=self._reconnect_worker,
                args=(attempts,),
                daemon=True,
                name="ReconnectWorker"
            )
            self._reconnect_thread.start()

    def _reconnect_worker(self, attempt: int):
        """
        Worker de reconexão.
        """
        print(f"[Failsafe] Tentativa de reconexão {attempt}/{self.config.max_reconnect_attempts}...")
        
        try:
            # Tenta reconectar
            try:
                self._connection.disconnect()
            except Exception:
                pass
            
            time.sleep(1)
            
            self._connection.connect()
            self._connection.wait_heartbeat(timeout=10)
            
            print(f"[Failsafe] Reconexão bem sucedida!")
            
            # Reinicia heartbeat
            self._heartbeat.start()
            
            # Notifica
            self._dispatcher.dispatch("link_connected")
            
        except Exception as e:
            print(f"[Failsafe] Falha na reconexão {attempt}: {e}")
            
            if attempt >= self.config.max_reconnect_attempts:
                print("[Failsafe] Máximo de tentativas excedido")
                self._set_state(FailsafeState.DISCONNECTED)
                self._dispatcher.dispatch("reconnect_failed")
            else:
                self._set_state(FailsafeState.DISCONNECTED)

    def reconnect(self) -> bool:
        """
        Tenta reconexão manual.
        
        Returns:
            bool: True se reconectado
        """
        with self._lock:
            self._reconnect_attempts = 0
        
        self._set_state(FailsafeState.RECOVERING)
        self._reconnect_worker(1)
        
        return self._heartbeat.connected

    # ===================================================================
    # MUDANÇA DE ESTADO
    # ===================================================================

    def _set_state(self, new_state: FailsafeState):
        """
        Altera estado do failsafe e notifica.
        """
        with self._lock:
            old_state = self._failsafe_state
            
            if old_state == new_state:
                return
            
            self._failsafe_state = new_state
            
            # Se voltou ao normal, reseta failsafe
            if new_state == FailsafeState.NORMAL:
                self._failsafe_start_time = None
                self._failsafe_action_taken = False
                self._reconnect_attempts = 0
        
        print(f"[Failsafe] Estado: {old_state.name} -> {new_state.name}")
        self._trigger_callbacks('on_state_change', old_state, new_state)

    def _trigger_callbacks(self, event: str, *args):
        """
        Dispara callbacks registrados.
        """
        for callback in self._callbacks.get(event, []):
            try:
                callback(*args)
            except Exception as e:
                print(f"[Failsafe] Erro no callback {event}: {e}")

    # ===================================================================
    # PACOTE PERDIDO (REGISTRO MANUAL)
    # ===================================================================

    def register_packet_lost(self, count: int = 1):
        """
        Registra pacotes perdidos (para uso externo).
        
        Args:
            count: Número de pacotes perdidos
        """
        with self._lock:
            self._stats.packets_lost += count
            for _ in range(count):
                self._stats.recent_packets.append(False)

    def register_packet_sent(self, count: int = 1):
        """
        Registra pacotes enviados.
        
        Args:
            count: Número de pacotes enviados
        """
        with self._lock:
            self._stats.packets_sent += count

    # ===================================================================
    # UTILITÁRIOS
    # ===================================================================

    def get_stats_dict(self) -> dict:
        """
        Retorna estatísticas em formato dict.
        """
        with self._lock:
            return {
                'state': self._failsafe_state.name,
                'packets_sent': self._stats.packets_sent,
                'packets_received': self._stats.packets_received,
                'packets_lost': self._stats.packets_lost,
                'packet_loss_rate': f"{self._stats.packet_loss_rate:.1%}",
                'reconnect_attempts': self._reconnect_attempts,
                'heartbeat_age': self._heartbeat.heartbeat_age,
                'is_failsafe_active': self.is_failsafe_active,
                'total_packets': self._stats.total_packets,
                'stabilization_time': f"{max(0, self.config.stabilization_time - (time.time() - self._start_time)):.1f}s"
            }

    def print_stats(self):
        """
        Imprime estatísticas do link.
        """
        stats = self.get_stats_dict()
        
        print("\n" + "=" * 50)
        print("ESTATÍSTICAS DO LINK")
        print("=" * 50)
        print(f"Estado: {stats['state']}")
        print(f"Pacotes: {stats['total_packets']} total "
              f"({stats['packets_sent']} env, {stats['packets_received']} rec, "
              f"{stats['packets_lost']} perd)")
        print(f"Perda: {stats['packet_loss_rate']}")
        print(f"Reconexões: {stats['reconnect_attempts']}")
        if stats['heartbeat_age'] is not None:
            print(f"Último heartbeat: {stats['heartbeat_age']:.1f}s atrás")
        print("=" * 50)

    def reset_stats(self):
        """
        Reseta estatísticas.
        """
        with self._lock:
            self._stats = LinkStats()
            self._reconnect_attempts = 0
            self._start_time = time.time()

    # ===================================================================
    # HELPERS
    # ===================================================================

    def _is_running(self) -> bool:
        with self._lock:
            return self._running

    def __repr__(self):
        return (f"FailsafeManager(state={self.state.name}, "
                f"loss={self.stats.packet_loss_rate:.1%})")