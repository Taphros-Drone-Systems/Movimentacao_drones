"""
health.py
Sistema de monitoramento de saúde do veículo.
"""

import threading
import time
from typing import List, Dict, Optional, Callable, Any
from dataclasses import dataclass, field
from enum import Enum
from collections import deque


class HealthSeverity(Enum):
    """Severidade do problema de saúde"""
    OK = 0
    INFO = 1
    WARNING = 2
    ERROR = 3
    CRITICAL = 4


@dataclass
class HealthIssue:
    """Representa um problema detectado"""
    name: str
    message: str
    severity: HealthSeverity
    timestamp: float = field(default_factory=time.time)
    
    def __repr__(self):
        return f"[{self.severity.name}] {self.name}: {self.message}"


class HealthMonitor:
    """
    Monitora a saúde do veículo e link de comunicação.
    """

    def __init__(self, vehicle, check_interval: float = 1.0):
        """
        Args:
            vehicle: Instância do Vehicle
            check_interval: Intervalo entre verificações (segundos)
        """
        self.vehicle = vehicle
        self.check_interval = check_interval
        
        self._running = False
        self._monitor_thread: Optional[threading.Thread] = None
        self._watchdog_thread: Optional[threading.Thread] = None
        self._lock = threading.RLock()
        
        # Histórico de problemas
        self._issues: deque = deque(maxlen=100)
        self._current_issues: Dict[str, HealthIssue] = {}
        
        # Callbacks
        self._callbacks: List[Callable[[List[HealthIssue]], None]] = []
        
        # Watchdog
        self._watchdog_timeout: float = 5.0
        self._last_heartbeat: float = 0
        self._watchdog_fired = False
        
        # Estatísticas
        self._checks_performed = 0
        self._issues_detected = 0
    
    def start(self):
        """Inicia monitoramento"""
        if self._running:
            return
        
        self._running = True
        self._last_heartbeat = time.time()
        
        self._monitor_thread = threading.Thread(
            target=self._monitor_loop,
            daemon=True,
            name="HealthMonitor"
        )
        self._monitor_thread.start()
        
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop,
            daemon=True,
            name="HealthWatchdog"
        )
        self._watchdog_thread.start()
        
        self._log_info("HealthMonitor iniciado")
    
    def stop(self):
        """Para monitoramento"""
        self._running = False
        
        if self._monitor_thread:
            self._monitor_thread.join(timeout=2.0)
        
        if self._watchdog_thread:
            self._watchdog_thread.join(timeout=2.0)
        
        self._log_info("HealthMonitor parado")
    
    def on_issues(self, callback: Callable[[List[HealthIssue]], None]):
        """
        Registra callback para problemas detectados.
        
        Args:
            callback: Função chamada quando problemas são detectados
        """
        self._callbacks.append(callback)
    
    def set_watchdog_timeout(self, timeout: float):
        """Define timeout do watchdog (segundos)"""
        self._watchdog_timeout = timeout
    
    def kick_watchdog(self):
        """Atualiza timestamp do watchdog (chamado ao receber heartbeat)"""
        self._last_heartbeat = time.time()
        if self._watchdog_fired:
            self._watchdog_fired = False
            self._log_info("Watchdog resetado - link recuperado")
    
    def get_issues(self) -> List[HealthIssue]:
        """Retorna lista de problemas atuais"""
        with self._lock:
            return list(self._current_issues.values())
    
    def get_history(self) -> List[HealthIssue]:
        """Retorna histórico de problemas"""
        with self._lock:
            return list(self._issues)
    
    def clear_history(self):
        """Limpa histórico de problemas"""
        with self._lock:
            self._issues.clear()
            self._current_issues.clear()
    
    def _monitor_loop(self):
        """Loop principal de monitoramento"""
        while self._running:
            try:
                self._check_conditions()
            except Exception as e:
                self._log_error(f"Erro no monitor: {e}")
            
            time.sleep(self.check_interval)
    
    def _watchdog_loop(self):
        """Loop do watchdog de heartbeat"""
        while self._running:
            time.sleep(0.5)
            
            if not self.vehicle.is_connected:
                continue
            
            now = time.time()
            if now - self._last_heartbeat > self._watchdog_timeout:
                if not self._watchdog_fired:
                    self._watchdog_fired = True
                    issue = HealthIssue(
                        name="heartbeat_timeout",
                        message=f"Sem heartbeat por {self._watchdog_timeout}s",
                        severity=HealthSeverity.CRITICAL
                    )
                    self._add_issue(issue)
            else:
                # Remove issue se existir
                with self._lock:
                    if "heartbeat_timeout" in self._current_issues:
                        del self._current_issues["heartbeat_timeout"]
    
    def _check_conditions(self):
        """Verifica todas as condições de saúde"""
        self._checks_performed += 1
        new_issues = []
        
        # 1. Verifica conexão
        if not self.vehicle.is_connected:
            new_issues.append(HealthIssue(
                name="disconnected",
                message="Veículo desconectado",
                severity=HealthSeverity.CRITICAL
            ))
        
        # 2. Verifica bateria
        battery = self.vehicle.state.battery
        if battery.remaining is not None:
            if battery.remaining <= 10:
                new_issues.append(HealthIssue(
                    name="battery_critical",
                    message=f"Bateria crítica: {battery.remaining}%",
                    severity=HealthSeverity.CRITICAL
                ))
            elif battery.remaining <= 25:
                new_issues.append(HealthIssue(
                    name="battery_low",
                    message=f"Bateria baixa: {battery.remaining}%",
                    severity=HealthSeverity.WARNING
                ))
        
        if battery.voltage is not None:
            if battery.voltage < 12.0:
                new_issues.append(HealthIssue(
                    name="battery_voltage_low",
                    message=f"Voltagem baixa: {battery.voltage:.1f}V",
                    severity=HealthSeverity.WARNING
                ))
        
        # 3. Verifica GPS
        gps = self.vehicle.state.gps
        if gps.fix_type is not None:
            fix_names = {0: "sem GPS", 1: "sem fix", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK float", 6: "RTK fix"}
            
            if gps.fix_type < 3:
                new_issues.append(HealthIssue(
                    name="gps_no_fix",
                    message=f"GPS sem fix 3D: {fix_names.get(gps.fix_type, 'desconhecido')}",
                    severity=HealthSeverity.ERROR if gps.fix_type < 2 else HealthSeverity.WARNING
                ))
            
            if gps.satellites_visible is not None and gps.satellites_visible < 6:
                new_issues.append(HealthIssue(
                    name="gps_few_satellites",
                    message=f"Poucos satélites: {gps.satellites_visible}",
                    severity=HealthSeverity.WARNING
                ))
        
        # 4. Verifica packet loss
        link_stats = self.vehicle.get_link_stats()
        loss_str = link_stats.get('packet_loss_rate', '0%')
        try:
            loss = float(loss_str.strip('%')) / 100 if isinstance(loss_str, str) else 0
        except:
            loss = 0
        
        if loss >= 0.5:
            new_issues.append(HealthIssue(
                name="packet_loss_critical",
                message=f"Perda de pacotes crítica: {loss:.1%}",
                severity=HealthSeverity.CRITICAL
            ))
        elif loss >= 0.3:
            new_issues.append(HealthIssue(
                name="packet_loss_high",
                message=f"Alta perda de pacotes: {loss:.1%}",
                severity=HealthSeverity.ERROR
            ))
        elif loss >= 0.1:
            new_issues.append(HealthIssue(
                name="packet_loss",
                message=f"Perda de pacotes: {loss:.1%}",
                severity=HealthSeverity.WARNING
            ))
        
        # 5. Verifica failsafe
        if self.vehicle.is_failsafe_active:
            new_issues.append(HealthIssue(
                name="failsafe_active",
                message="Failsafe ativo",
                severity=HealthSeverity.CRITICAL
            ))
        
        # Atualiza issues
        self._update_issues(new_issues)
    
    def _update_issues(self, new_issues: List[HealthIssue]):
        """Atualiza lista de problemas atuais"""
        with self._lock:
            # Mapeia novos problemas por nome
            new_issues_map = {issue.name: issue for issue in new_issues}
            
            # Verifica problemas resolvidos
            resolved = []
            for name, issue in self._current_issues.items():
                if name not in new_issues_map:
                    resolved.append(issue)
            
            # Remove resolvidos
            for issue in resolved:
                del self._current_issues[issue.name]
                self._log_info(f"Problema resolvido: {issue.name}")
            
            # Adiciona novos problemas
            for name, issue in new_issues_map.items():
                if name not in self._current_issues:
                    self._current_issues[name] = issue
                    self._issues.append(issue)
                    self._issues_detected += 1
                    
                    # Log do problema
                    if issue.severity == HealthSeverity.CRITICAL:
                        self._log_critical(f"{issue.name}: {issue.message}")
                    elif issue.severity == HealthSeverity.ERROR:
                        self._log_error(f"{issue.name}: {issue.message}")
                    elif issue.severity == HealthSeverity.WARNING:
                        self._log_warning(f"{issue.name}: {issue.message}")
            
            # Dispara callbacks se houver problemas
            if self._current_issues:
                for callback in self._callbacks:
                    try:
                        callback(list(self._current_issues.values()))
                    except Exception as e:
                        self._log_error(f"Erro no callback: {e}")
    
    def _add_issue(self, issue: HealthIssue):
        """Adiciona problema manualmente"""
        with self._lock:
            self._current_issues[issue.name] = issue
            self._issues.append(issue)
            self._issues_detected += 1
        
        for callback in self._callbacks:
            try:
                callback([issue])
            except Exception:
                pass
    
    def get_stats(self) -> dict:
        """Retorna estatísticas do monitor"""
        with self._lock:
            return {
                'running': self._running,
                'checks_performed': self._checks_performed,
                'issues_detected': self._issues_detected,
                'active_issues': len(self._current_issues),
                'history_size': len(self._issues),
                'watchdog_fired': self._watchdog_fired,
                'watchdog_timeout': self._watchdog_timeout
            }
    
    def _log_info(self, msg: str):
        """Log informativo"""
        print(f"[HealthMonitor] {msg}")
    
    def _log_warning(self, msg: str):
        """Log de warning"""
        print(f"[HealthMonitor] ⚠ {msg}")
    
    def _log_error(self, msg: str):
        """Log de erro"""
        print(f"[HealthMonitor] ✗ {msg}")
    
    def _log_critical(self, msg: str):
        """Log crítico"""
        print(f"[HealthMonitor] 🔴 {msg}")