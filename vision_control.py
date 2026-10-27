"""
vision_control.py

Módulo para controle visual do drone usando detecções da câmera.
Integra com YOLO e o sistema de navegação do drone.
"""

import math
import time
import threading
from typing import Optional, Tuple, Callable, Any
from dataclasses import dataclass
from enum import Enum, auto

import cv2
import numpy as np


class AlignmentMode(Enum):
    """Modos de alinhamento disponíveis"""
    CONTINUOUS = auto()      # Alinhamento contínuo (segue objeto)
    ONE_SHOT = auto()        # Alinhamento único (move uma vez)
    PRECISION = auto()       # Alinhamento de precisão (velocidade reduzida)


class TrackingState(Enum):
    """Estados do rastreamento visual"""
    IDLE = auto()            # Parado
    ALIGNING = auto()        # Alinhando
    TRACKING = auto()        # Seguindo
    LOST = auto()            # Perdeu objeto
    ERROR = auto()           # Erro


@dataclass
class AlignmentConfig:
    """Configuração para alinhamento visual"""
    # Parâmetros do PID para controle visual
    pid_p: float = 0.05      # Proporcional - sensibilidade ao erro
    pid_i: float = 0.01      # Integral - correção de erro acumulado
    pid_d: float = 0.02      # Derivativo - antecipação
    
    # Zona morta (pixels) - dentro disso considera alinhado
    dead_zone_x: int = 30     # Pixels horizontais
    dead_zone_y: int = 30     # Pixels verticais
    
    # Limites de velocidade (m/s)
    max_speed_x: float = 2.0   # Velocidade horizontal máxima
    max_speed_y: float = 2.0   # Velocidade vertical máxima
    
    # Limites de rotação (graus/s)
    max_yaw_rate: float = 45.0  # Taxa de rotação máxima
    
    # Tempo máximo de alinhamento (segundos)
    max_alignment_time: float = 10.0
    
    # Pausa entre comandos (segundos)
    command_interval: float = 0.1
    
    # Zoom/ganho para correção (ajuste fino)
    zoom_factor: float = 0.8    # Quanto mais longe do centro, mais rápido


@dataclass
class DetectionInfo:
    """Informações da detecção YOLO"""
    x: int           # Centro x (pixels)
    y: int           # Centro y (pixels)
    width: int       # Largura da bounding box
    height: int      # Altura da bounding box
    confidence: float  # Confiança da detecção
    class_id: int    # ID da classe detectada
    class_name: str  # Nome da classe
    timestamp: float # Timestamp da detecção


class VisualAlignmentController:
    """
    Controlador para alinhar centro da câmera com centro da bounding box detectada.
    
    Funcionalidades:
    - Alinhamento horizontal (yaw) e vertical (pitch)
    - Rastreamento contínuo de objetos
    - Controle PID para movimentos suaves
    - Modos one-shot, contínuo e precisão
    - Callbacks para eventos de alinhamento
    """
    
    def __init__(self, drone, frame_width: int = 640, frame_height: int = 480):
        """
        Args:
            drone: Instância do Vehicle (ppymavlink)
            frame_width: Largura do frame da câmera (pixels)
            frame_height: Altura do frame da câmera (pixels)
        """
        self.drone = drone
        self.frame_center_x = frame_width // 2
        self.frame_center_y = frame_height // 2
        self.frame_width = frame_width
        self.frame_height = frame_height
        
        # Configuração
        self.config = AlignmentConfig()
        
        # Estado do PID
        self._integral_x = 0.0
        self._integral_y = 0.0
        self._prev_error_x = 0.0
        self._prev_error_y = 0.0
        self._last_time = 0.0
        
        # Estado do rastreamento
        self._tracking_state = TrackingState.IDLE
        self._current_mode = AlignmentMode.ONE_SHOT
        self._target_box: Optional[DetectionInfo] = None
        self._alignment_start_time = 0.0
        
        # Thread de controle
        self._running = False
        self._control_thread: Optional[threading.Thread] = None
        self._lock = threading.RLock()
        
        # Callbacks
        self._callbacks = {
            'on_aligned': [],      # Quando alinhado
            'on_tracking_lost': [],# Quando perde o objeto
            'on_alignment_start': [],  # Quando começa alinhamento
            'on_alignment_stop': [],   # Quando para alinhamento
            'on_error': []         # Em caso de erro
        }
        
        # Estatísticas
        self._stats = {
            'alignments_completed': 0,
            'total_alignment_time': 0.0,
            'tracking_time': 0.0,
            'tracking_start_time': 0.0
        }
        
        print(f"[VisualControl] Inicializado - Frame center: ({self.frame_center_x}, {self.frame_center_y})")
    
    # ===================================================================
    # CONFIGURAÇÃO
    # ===================================================================
    
    def configure(self, **kwargs):
        """Configura parâmetros do controlador"""
        for key, value in kwargs.items():
            if hasattr(self.config, key):
                setattr(self.config, key, value)
                print(f"[VisualControl] Config: {key} = {value}")
    
    def set_frame_size(self, width: int, height: int):
        """Atualiza tamanho do frame da câmera"""
        self.frame_width = width
        self.frame_height = height
        self.frame_center_x = width // 2
        self.frame_center_y = height // 2
        print(f"[VisualControl] Frame size: {width}x{height}, center: ({self.frame_center_x}, {self.frame_center_y})")
    
    # ===================================================================
    # CALLBACKS
    # ===================================================================
    
    def on_aligned(self, callback: Callable):
        """Callback quando o drone está alinhado com o objeto"""
        self._callbacks['on_aligned'].append(callback)
    
    def on_tracking_lost(self, callback: Callable):
        """Callback quando perde o rastreamento do objeto"""
        self._callbacks['on_tracking_lost'].append(callback)
    
    def on_alignment_start(self, callback: Callable):
        """Callback quando inicia alinhamento"""
        self._callbacks['on_alignment_start'].append(callback)
    
    def on_alignment_stop(self, callback: Callable):
        """Callback quando para alinhamento"""
        self._callbacks['on_alignment_stop'].append(callback)
    
    # ===================================================================
    # FUNÇÃO PRINCIPAL DE ALINHAMENTO
    # ===================================================================
    
    def align_to_box(self, detection: dict, mode: AlignmentMode = AlignmentMode.ONE_SHOT,
                     wait_for_completion: bool = True, timeout: float = 10.0) -> bool:
        """
        Alinha o drone ao centro da bounding box detectada.
        
        Args:
            detection: Dicionário da detecção YOLO com 'box' (xywh) e 'conf'
            mode: Modo de alinhamento (ONE_SHOT, CONTINUOUS, PRECISION)
            wait_for_completion: Aguarda até completar o alinhamento
            timeout: Timeout máximo para alinhamento (segundos)
            
        Returns:
            bool: True se alinhado com sucesso
        """
        # Verifica se drone está armado e em modo GUIDED
        if not self._check_prerequisites():
            self._trigger_callbacks('on_error', "Drone não pronto para navegação")
            return False
        
        # Extrai informações da detecção
        box = detection.get('box') or detection.get('xywh')
        if box is None:
            self._trigger_callbacks('on_error', "Detecção sem bounding box")
            return False
        
        # Cria objeto DetectionInfo
        detection_info = DetectionInfo(
            x=int(box[0]),           # Centro x (se xywh) ou x1
            y=int(box[1]),           # Centro y (se xywh) ou y1
            width=int(box[2]),
            height=int(box[3]),
            confidence=detection.get('conf', 1.0),
            class_id=detection.get('class_id', 0),
            class_name=detection.get('class_name', 'unknown'),
            timestamp=time.time()
        )
        
        # Ajusta se for xyxy (x1,y1,x2,y2) em vez de xywh
        if box[2] > 1.0 and box[3] > 1.0 and (box[2] > self.frame_width or box[3] > self.frame_height):
            # Provavelmente é xyxy
            center_x = int((box[0] + box[2]) / 2)
            center_y = int((box[1] + box[3]) / 2)
            detection_info.x = center_x
            detection_info.y = center_y
            detection_info.width = int(box[2] - box[0])
            detection_info.height = int(box[3] - box[1])
        
        self._target_box = detection_info
        self._current_mode = mode
        
        # Modo contínuo - inicia thread de rastreamento
        if mode == AlignmentMode.CONTINUOUS:
            self.start_tracking()
            return True
        
        # Modo one-shot ou precision - alinha e para
        return self._align_single(detection_info, mode, timeout)
    
    def _align_single(self, detection: DetectionInfo, mode: AlignmentMode,
                      timeout: float = 10.0) -> bool:
        """
        Alinhamento único (one-shot ou precision)
        """
        print(f"\n[VisualControl] Iniciando alinhamento {mode.name}")
        print(f"  Objeto: {detection.class_name} (conf={detection.confidence:.2f})")
        print(f"  Centro atual: ({detection.x}, {detection.y})")
        print(f"  Centro alvo: ({self.frame_center_x}, {self.frame_center_y})")
        
        self._tracking_state = TrackingState.ALIGNING
        self._alignment_start_time = time.time()
        self._trigger_callbacks('on_alignment_start', detection)
        
        # Reseta PID
        self._reset_pid()
        
        start_time = time.time()
        last_command_time = 0
        
        # Loop de alinhamento
        while time.time() - start_time < timeout:
            # Verifica se ainda está conectado
            if not self.drone.is_connected:
                print("[VisualControl] Drone desconectado durante alinhamento")
                return False
            
            # Verifica timeout
            if time.time() - self._alignment_start_time > self.config.max_alignment_time:
                print("[VisualControl] Timeout de alinhamento")
                return False
            
            # Obtém detecção atualizada (para modo precision)
            if mode == AlignmentMode.PRECISION:
                current_detection = self._get_current_detection()
                if current_detection:
                    detection = current_detection
            
            # Calcula erro
            error_x = detection.x - self.frame_center_x
            error_y = detection.y - self.frame_center_y
            
            # Verifica se está alinhado
            if self._is_aligned(error_x, error_y):
                print(f"[VisualControl] ✅ Alinhado! Erros: ({error_x}, {error_y}) pixels")
                self._stop_alignment(success=True)
                return True
            
            # Calcula comandos via PID
            yaw_rate, pitch_speed = self._compute_pid_commands(error_x, error_y, mode)
            
            # Aplica comando (rate limiting)
            now = time.time()
            if now - last_command_time >= self.config.command_interval:
                self._apply_correction(yaw_rate, pitch_speed, mode)
                last_command_time = now
                
                # Log de progresso
                if int(now) % 2 == 0:
                    print(f"  Progresso: erro_x={error_x}, erro_y={error_y}, yaw_rate={yaw_rate:.1f}°/s")
            
            time.sleep(0.05)  # Pequena pausa para não sobrecarregar
        
        print("[VisualControl] ❌ Timeout - não foi possível alinhar")
        self._stop_alignment(success=False)
        return False
    
    def _compute_pid_commands(self, error_x: int, error_y: int, mode: AlignmentMode) -> Tuple[float, float]:
        """
        Calcula comandos via PID.
        
        Returns:
            Tuple[yaw_rate, pitch_speed]: Taxa de rotação (graus/s) e velocidade vertical (m/s)
        """
        current_time = time.time()
        dt = current_time - self._last_time if self._last_time > 0 else 0.1
        self._last_time = current_time
        
        # Aplica zoom/ganho baseado na distância ao centro
        gain_x = min(1.0, abs(error_x) / self.frame_width * 2) * self.config.zoom_factor
        gain_y = min(1.0, abs(error_y) / self.frame_height * 2) * self.config.zoom_factor
        
        # Ajuste de sensibilidade por modo
        if mode == AlignmentMode.PRECISION:
            gain_x *= 0.5   # Metade da velocidade para precisão
            gain_y *= 0.5
        
        # PID para X (yaw)
        self._integral_x += error_x * dt * self.config.pid_i
        # Anti-windup
        self._integral_x = max(-100, min(100, self._integral_x))
        
        derivative_x = (error_x - self._prev_error_x) / dt if dt > 0 else 0
        
        yaw_rate = (self.config.pid_p * error_x + 
                   self._integral_x + 
                   self.config.pid_d * derivative_x) * gain_x
        
        # Limita yaw rate
        yaw_rate = max(-self.config.max_yaw_rate, min(self.config.max_yaw_rate, yaw_rate))
        
        # PID para Y (pitch/altitude)
        # Nota: Para drones, pitch controla movimento frente/trás
        # Para controle vertical (subir/descer), usamos vz
        self._integral_y += error_y * dt * self.config.pid_i
        self._integral_y = max(-100, min(100, self._integral_y))
        
        derivative_y = (error_y - self._prev_error_y) / dt if dt > 0 else 0
        
        # Velocidade vertical (positivo = subir)
        vertical_speed = (self.config.pid_p * error_y + 
                         self._integral_y + 
                         self.config.pid_d * derivative_y) * gain_y * 0.5
        
        # Limita velocidade vertical
        vertical_speed = max(-self.config.max_speed_y, min(self.config.max_speed_y, vertical_speed))
        
        # Atualiza erros anteriores
        self._prev_error_x = error_x
        self._prev_error_y = error_y
        
        # Para movimento horizontal, na verdade usamos velocidade
        # yaw_rate controla rotação, e small forward/backward pode ser adicionado
        # mas para simplificar, focamos em yaw e altitude
        
        return yaw_rate, vertical_speed
    
    def _apply_correction(self, yaw_rate: float, vertical_speed: float, mode: AlignmentMode):
        """Aplica correção ao drone"""
        try:
            # Aplica yaw rate
            if abs(yaw_rate) > 1.0:  # Apenas se significativo
                self.drone.set_yaw_rate(yaw_rate)
            
            # Aplica velocidade vertical (subir/descer)
            if abs(vertical_speed) > 0.1:
                # Mantém posição horizontal, ajusta altitude
                self.drone.set_velocity(0, 0, -vertical_speed)  # vz positivo = descer
                
        except Exception as e:
            print(f"[VisualControl] Erro ao aplicar correção: {e}")
    
    def _is_aligned(self, error_x: int, error_y: int) -> bool:
        """Verifica se está alinhado (dentro da zona morta)"""
        return (abs(error_x) <= self.config.dead_zone_x and 
                abs(error_y) <= self.config.dead_zone_y)
    
    # ===================================================================
    # RASTREAMENTO CONTÍNUO
    # ===================================================================
    
    def start_tracking(self):
        """Inicia rastreamento contínuo do objeto"""
        if self._tracking_state == TrackingState.TRACKING:
            print("[VisualControl] Já está rastreando")
            return
        
        self._tracking_state = TrackingState.TRACKING
        self._running = True
        self._stats['tracking_start_time'] = time.time()
        
        self._control_thread = threading.Thread(
            target=self._tracking_loop,
            daemon=True,
            name="VisualTrackingThread"
        )
        self._control_thread.start()
        
        print("[VisualControl] Rastreamento contínuo iniciado")
    
    def stop_tracking(self):
        """Para rastreamento contínuo"""
        self._running = False
        self._tracking_state = TrackingState.IDLE
        
        # Para navegação
        self.drone.stop_navigation()
        
        if self._control_thread:
            self._control_thread.join(timeout=2.0)
        
        # Atualiza estatísticas
        if self._stats['tracking_start_time'] > 0:
            self._stats['tracking_time'] += time.time() - self._stats['tracking_start_time']
        
        self._trigger_callbacks('on_alignment_stop')
        print("[VisualControl] Rastreamento parado")
    
    def _tracking_loop(self):
        """Loop principal de rastreamento contínuo"""
        print("[VisualControl] Thread de rastreamento iniciada")
        
        while self._running and self._tracking_state == TrackingState.TRACKING:
            try:
                # Verifica pré-requisitos
                if not self._check_prerequisites():
                    time.sleep(1)
                    continue
                
                # Obtém detecção atual (deve ser fornecida externamente)
                # Por padrão, usa o último alvo - a aplicação deve atualizar
                if self._target_box is None:
                    time.sleep(0.5)
                    continue
                
                # Verifica se detecção está atualizada (< 0.5 segundos)
                if time.time() - self._target_box.timestamp > 0.5:
                    print("[VisualControl] Detecção desatualizada")
                    self._tracking_state = TrackingState.LOST
                    self._trigger_callbacks('on_tracking_lost')
                    time.sleep(0.5)
                    continue
                
                # Calcula erro
                error_x = self._target_box.x - self.frame_center_x
                error_y = self._target_box.y - self.frame_center_y
                
                # Verifica alinhamento
                if self._is_aligned(error_x, error_y):
                    self._trigger_callbacks('on_aligned', self._target_box)
                    # No tracking contínuo, continua mesmo se alinhado
                
                # Calcula e aplica correção
                yaw_rate, vertical_speed = self._compute_pid_commands(error_x, error_y, self._current_mode)
                self._apply_correction(yaw_rate, vertical_speed, self._current_mode)
                
                time.sleep(self.config.command_interval)
                
            except Exception as e:
                print(f"[VisualControl] Erro no tracking loop: {e}")
                time.sleep(0.5)
        
        print("[VisualControl] Thread de rastreamento encerrada")
    
    # ===================================================================
    # MÉTODOS AUXILIARES
    # ===================================================================
    
    def update_detection(self, detection: dict):
        """
        Atualiza a detecção atual para rastreamento contínuo.
        Deve ser chamado a cada frame de detecção.
        """
        if self._tracking_state != TrackingState.TRACKING:
            return
        
        box = detection.get('box') or detection.get('xywh')
        if box is None:
            return
        
        # Calcula centro
        if len(box) >= 4:
            if box[2] > 1.0 and box[2] < self.frame_width:
                # xywh
                center_x = int(box[0])
                center_y = int(box[1])
            else:
                # xyxy
                center_x = int((box[0] + box[2]) / 2)
                center_y = int((box[1] + box[3]) / 2)
            
            with self._lock:
                self._target_box = DetectionInfo(
                    x=center_x,
                    y=center_y,
                    width=int(box[2]),
                    height=int(box[3]),
                    confidence=detection.get('conf', 1.0),
                    class_id=detection.get('class_id', 0),
                    class_name=detection.get('class_name', 'unknown'),
                    timestamp=time.time()
                )
            
            # Se estava perdido, recupera
            if self._tracking_state == TrackingState.LOST:
                self._tracking_state = TrackingState.TRACKING
                print("[VisualControl] Objeto recuperado!")
    
    def _get_current_detection(self) -> Optional[DetectionInfo]:
        """Retorna detecção atual (para modo precision)"""
        with self._lock:
            return self._target_box
    
    def _reset_pid(self):
        """Reseta acumuladores do PID"""
        self._integral_x = 0.0
        self._integral_y = 0.0
        self._prev_error_x = 0.0
        self._prev_error_y = 0.0
        self._last_time = 0.0
    
    def _stop_alignment(self, success: bool):
        """Para alinhamento e atualiza estatísticas"""
        self._tracking_state = TrackingState.IDLE
        
        # Para navegação
        self.drone.stop_navigation()
        
        # Atualiza estatísticas
        if success:
            self._stats['alignments_completed'] += 1
            alignment_time = time.time() - self._alignment_start_time
            self._stats['total_alignment_time'] += alignment_time
            print(f"[VisualControl] Alinhamento completado em {alignment_time:.2f}s")
        
        self._trigger_callbacks('on_alignment_stop', success)
    
    def _check_prerequisites(self) -> bool:
        """Verifica se drone está pronto para navegação visual"""
        if not self.drone.is_connected:
            return False
        
        # Verifica modo
        current_mode = self.drone.state.flight_mode()
        if current_mode != "GUIDED":
            print(f"[VisualControl] Modo incorreto: {current_mode} (necessário GUIDED)")
            return False
        
        # Verifica se está armado
        if not self.drone.state.is_armed():
            print("[VisualControl] Drone não armado")
            return False
        
        return True
    
    def _trigger_callbacks(self, event: str, *args):
        """Dispara callbacks registrados"""
        for callback in self._callbacks.get(event, []):
            try:
                callback(*args)
            except Exception as e:
                print(f"[VisualControl] Erro no callback {event}: {e}")
    
    # ===================================================================
    # UTILITÁRIOS
    # ===================================================================
    
    def draw_alignment_overlay(self, frame: np.ndarray, detection: dict) -> np.ndarray:
        """
        Desenha overlay de alinhamento no frame.
        
        Args:
            frame: Frame da câmera (numpy array)
            detection: Detecção YOLO com 'box' e 'conf'
            
        Returns:
            Frame com overlay desenhado
        """
        frame_with_overlay = frame.copy()
        
        # Extrai bounding box
        box = detection.get('box') or detection.get('xywh')
        if box is None:
            return frame_with_overlay
        
        # Desenha bounding box
        if len(box) >= 4:
            if box[2] > 1.0 and box[2] < self.frame_width:
                # xywh
                x1 = int(box[0] - box[2]//2)
                y1 = int(box[1] - box[3]//2)
                x2 = int(box[0] + box[2]//2)
                y2 = int(box[1] + box[3]//2)
            else:
                # xyxy
                x1, y1, x2, y2 = map(int, box[:4])
            
            # Cor por classe
            class_name = detection.get('class_name', 'object')
            colors = {
                'person': (0, 255, 0),
                'car': (255, 0, 0),
                'default': (0, 255, 255)
            }
            color = colors.get(class_name, colors['default'])
            
            # Desenha bounding box
            cv2.rectangle(frame_with_overlay, (x1, y1), (x2, y2), color, 2)
            
            # Desenha centro do objeto
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2
            cv2.circle(frame_with_overlay, (center_x, center_y), 5, color, -1)
        
        # Desenha centro da tela
        cv2.circle(frame_with_overlay, (self.frame_center_x, self.frame_center_y), 10, (255, 255, 255), 2)
        cv2.line(frame_with_overlay, 
                (self.frame_center_x - 20, self.frame_center_y),
                (self.frame_center_x + 20, self.frame_center_y),
                (255, 255, 255), 1)
        cv2.line(frame_with_overlay,
                (self.frame_center_x, self.frame_center_y - 20),
                (self.frame_center_x, self.frame_center_y + 20),
                (255, 255, 255), 1)
        
        # Desenha zona morta (dead zone)
        cv2.rectangle(frame_with_overlay,
                     (self.frame_center_x - self.config.dead_zone_x,
                      self.frame_center_y - self.config.dead_zone_y),
                     (self.frame_center_x + self.config.dead_zone_x,
                      self.frame_center_y + self.config.dead_zone_y),
                     (100, 100, 100), 1)
        
        # Adiciona texto com informações
        info_text = f"Mode: {self._tracking_state.name}"
        cv2.putText(frame_with_overlay, info_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        if self._target_box:
            error_x = self._target_box.x - self.frame_center_x
            error_y = self._target_box.y - self.frame_center_y
            error_text = f"Error: ({error_x}, {error_y}) px"
            cv2.putText(frame_with_overlay, error_text, (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return frame_with_overlay
    
    def get_stats(self) -> dict:
        """Retorna estatísticas do controlador"""
        return {
            'tracking_state': self._tracking_state.name,
            'alignments_completed': self._stats['alignments_completed'],
            'avg_alignment_time': (self._stats['total_alignment_time'] / self._stats['alignments_completed'] 
                                   if self._stats['alignments_completed'] > 0 else 0),
            'tracking_time': self._stats['tracking_time'],
            'config': {
                'dead_zone_x': self.config.dead_zone_x,
                'dead_zone_y': self.config.dead_zone_y,
                'max_yaw_rate': self.config.max_yaw_rate,
                'pid_p': self.config.pid_p,
                'pid_i': self.config.pid_i,
                'pid_d': self.config.pid_d
            }
        }
    
    def cleanup(self):
        """Limpeza final"""
        self.stop_tracking()
        print("[VisualControl] Cleanup concluído")
    
    def __repr__(self):
        return f"VisualAlignmentController(state={self._tracking_state.name}, alignments={self._stats['alignments_completed']})"