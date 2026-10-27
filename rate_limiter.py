"""
rate_limiter.py
Limitador de taxa para mensagens MAVLink.
"""

import threading
import time
from collections import defaultdict
from typing import Optional, Dict, Callable, Any
from enum import Enum


class LimitStrategy(Enum):
    """Estratégia de rate limiting"""
    DROP = "drop"           # Descarta mensagens que excedem o limite
    QUEUE = "queue"         # Enfileira mensagens para enviar depois
    BLOCK = "block"         # Bloqueia até poder enviar


class RateLimiter:
    """Limitador de taxa para diferentes tipos de mensagem"""
    
    def __init__(self):
        self._last_sent: Dict[str, float] = defaultdict(float)
        self._lock = threading.RLock()
        self._queues: Dict[str, list] = defaultdict(list)
        self._queue_threads: Dict[str, threading.Thread] = {}
        self._running = False
        
        # Configurações por tipo de mensagem (intervalo mínimo em segundos)
        self._intervals: Dict[str, float] = {
            # Comandos críticos - alta prioridade
            'ARM': 0.5,
            'DISARM': 0.5,
            'TAKEOFF': 1.0,
            'LAND': 1.0,
            'RTL': 0.5,
            
            # Comandos de navegação
            'GOTO': 0.2,
            'POSITION_TARGET': 0.05,  # 20Hz máximo
            'VELOCITY': 0.05,
            
            # Parâmetros
            'PARAM_SET': 0.2,
            'PARAM_REQUEST': 0.1,
            
            # Missão
            'MISSION_UPLOAD': 2.0,
            'MISSION_DOWNLOAD': 2.0,
            'MISSION_ITEM': 0.05,
            
            # Heartbeat
            'HEARTBEAT': 0.5,
            
            # Default
            'DEFAULT': 0.1
        }
        
        # Estratégia por tipo
        self._strategies: Dict[str, LimitStrategy] = {
            'HEARTBEAT': LimitStrategy.DROP,
            'DEFAULT': LimitStrategy.BLOCK
        }
    
    def start(self):
        """Inicia threads de processamento de filas"""
        self._running = True
    
    def stop(self):
        """Para processamento de filas"""
        self._running = False
        for thread in self._queue_threads.values():
            if thread and thread.is_alive():
                thread.join(timeout=2.0)
    
    def set_interval(self, msg_type: str, interval: float):
        """Define intervalo mínimo entre mensagens do tipo"""
        with self._lock:
            self._intervals[msg_type.upper()] = interval
    
    def set_strategy(self, msg_type: str, strategy: LimitStrategy):
        """Define estratégia para tipo de mensagem"""
        with self._lock:
            self._strategies[msg_type.upper()] = strategy
    
    def can_send(self, msg_type: str, timestamp: Optional[float] = None) -> bool:
        """
        Verifica se pode enviar mensagem do tipo.
        
        Args:
            msg_type: Tipo da mensagem
            timestamp: Timestamp opcional (default: time.time())
        
        Returns:
            True se pode enviar, False caso contrário
        """
        msg_type = msg_type.upper()
        
        if timestamp is None:
            timestamp = time.time()
        
        with self._lock:
            interval = self._intervals.get(msg_type, self._intervals['DEFAULT'])
            last = self._last_sent.get(msg_type, 0)
            
            if timestamp - last >= interval:
                self._last_sent[msg_type] = timestamp
                return True
            
            return False
    
    def wait_and_send(self, msg_type: str, send_func: Callable, *args, **kwargs):
        """
        Aguarda até poder enviar e então executa a função de envio.
        
        Args:
            msg_type: Tipo da mensagem
            send_func: Função que envia a mensagem
            *args, **kwargs: Argumentos para send_func
        """
        msg_type = msg_type.upper()
        strategy = self._strategies.get(msg_type, self._strategies['DEFAULT'])
        
        if strategy == LimitStrategy.DROP:
            if self.can_send(msg_type):
                return send_func(*args, **kwargs)
            return False
        
        elif strategy == LimitStrategy.QUEUE:
            with self._lock:
                self._queues[msg_type].append((send_func, args, kwargs))
                
                # Inicia thread de processamento se não existir
                if msg_type not in self._queue_threads or not self._queue_threads[msg_type].is_alive():
                    self._queue_threads[msg_type] = threading.Thread(
                        target=self._process_queue,
                        args=(msg_type,),
                        daemon=True,
                        name=f"RateLimiterQueue_{msg_type}"
                    )
                    self._queue_threads[msg_type].start()
            return True
        
        else:  # BLOCK
            while not self.can_send(msg_type):
                time.sleep(0.01)
            return send_func(*args, **kwargs)
    
    def _process_queue(self, msg_type: str):
        """Processa fila de mensagens pendentes"""
        while self._running:
            with self._lock:
                if not self._queues[msg_type]:
                    break
                
                # Pega próximo item da fila
                send_func, args, kwargs = self._queues[msg_type].pop(0)
            
            # Aguarda intervalo
            interval = self._intervals.get(msg_type, self._intervals['DEFAULT'])
            time.sleep(interval)
            
            # Envia
            try:
                send_func(*args, **kwargs)
            except Exception as e:
                print(f"[RateLimiter] Erro ao enviar {msg_type}: {e}")
    
    def get_stats(self) -> dict:
        """Retorna estatísticas do rate limiter"""
        with self._lock:
            return {
                'intervals': dict(self._intervals),
                'strategies': {k: v.value for k, v in self._strategies.items()},
                'queue_sizes': {k: len(v) for k, v in self._queues.items()},
                'last_sent': {k: v for k, v in self._last_sent.items()}
            }
    
    def reset(self):
        """Reseta estatísticas"""
        with self._lock:
            self._last_sent.clear()
            self._queues.clear()


class MAVLinkRateLimiter(RateLimiter):
    """Rate limiter específico para mensagens MAVLink"""
    
    # Mapeamento de nomes de mensagens MAVLink para tipos
    MAV_MSG_TO_TYPE = {
        'HEARTBEAT': 'HEARTBEAT',
        'COMMAND_LONG': 'COMMAND',
        'COMMAND_INT': 'COMMAND',
        'SET_POSITION_TARGET_GLOBAL_INT': 'POSITION_TARGET',
        'SET_POSITION_TARGET_LOCAL_NED': 'POSITION_TARGET',
        'PARAM_SET': 'PARAM_SET',
        'PARAM_REQUEST_READ': 'PARAM_REQUEST',
        'MISSION_COUNT': 'MISSION_UPLOAD',
        'MISSION_ITEM': 'MISSION_ITEM',
        'MISSION_WRITE_PARTIAL_LIST': 'MISSION_UPLOAD',
        'MISSION_CLEAR_ALL': 'MISSION_CLEAR',
    }
    
    @classmethod
    def get_type_for_message(cls, msg_type: str) -> str:
        """Obtém tipo de rate limit para mensagem MAVLink"""
        return cls.MAV_MSG_TO_TYPE.get(msg_type, msg_type)