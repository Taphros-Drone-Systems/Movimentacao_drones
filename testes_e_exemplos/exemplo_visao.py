"""
exemplo_basico_visao.py
Exemplo mínimo para testar o alinhamento visual do drone com YOLO
"""

import cv2
import time
import numpy as np
from ultralytics import YOLO
from vehicle import Vehicle
from vision_control import VisualAlignmentController, AlignmentMode


# ============================================================================
# CONFIGURAÇÕES BÁSICAS (AJUSTE CONFORME SEU CASO)
# ============================================================================

CONNECTION_STRING = "udp:127.0.0.1:14550"  # Para SITL
# CONNECTION_STRING = "/dev/ttyACM0"        # Para drone real (USB)
# CONNECTION_STRING = "/dev/ttyUSB0"        # Para telemetria

MODEL_PATH = "treinamentos/referencia_esfera.pt"  # Seu modelo YOLO
TARGET_CLASS = "esfera_laranja"        # Classe que você quer detectar
ALTITUDE = 3.0               # Altitude de voo (metros)

# Parâmetros da câmera
FRAME_WIDTH = 640
FRAME_HEIGHT = 480


# ============================================================================
# FUNÇÃO PRINCIPAL
# ============================================================================

def main():
    print("=" * 60)
    print("   EXEMPLO BÁSICO - Alinhamento Visual com Drone")
    print("=" * 60)
    
    # ------------------------------------------------------------------------
    # 1. INICIALIZA O DRONE
    # ------------------------------------------------------------------------
    print("\n[1] Conectando ao drone...")
    drone = Vehicle(CONNECTION_STRING)
    
    if not drone.connect():
        print("   ❌ Falha na conexão!")
        return
    
    print("   ✅ Conectado!")
    
    # Aguarda telemetria
    print("   Aguardando telemetria...")
    if not drone.wait_ready(timeout=10):
        print("   ❌ Timeout!")
        drone.disconnect()
        return
    
    # ------------------------------------------------------------------------
    # 2. PREPARA O DRONE PARA VOO
    # ------------------------------------------------------------------------
    print("\n[2] Preparando drone para voo...")
    
    # Muda para modo GUIDED
    print("   Mudando para modo GUIDED...")
    result = drone.set_mode("GUIDED", timeout=5.0)
    if result != 1:  # CommandResult.SUCCESS = 1
        print("   ❌ Falha ao mudar para GUIDED!")
        drone.disconnect()
        return
    
    # Arma os motores
    print("   Armando motores...")
    result = drone.arm(timeout=5.0)
    if result != 1:
        print("   ❌ Falha ao armar!")
        drone.disconnect()
        return
    
    # Decola
    print(f"   Decolando para {ALTITUDE} metros...")
    result = drone.takeoff(ALTITUDE, timeout=15.0)
    if result != 1:
        print("   ❌ Falha na decolagem!")
        drone.disconnect()
        return
    
    print("   ✅ Drone no ar e estabilizado!")
    time.sleep(2)  # Aguarda estabilizar
    
    # ------------------------------------------------------------------------
    # 3. INICIALIZA O CONTROLADOR VISUAL
    # ------------------------------------------------------------------------
    print("\n[3] Inicializando controlador visual...")
    
    vision = VisualAlignmentController(drone, FRAME_WIDTH, FRAME_HEIGHT)
    
    # Configura parâmetros (ajuste fino)
    vision.configure(
        pid_p=0.05,      # Sensibilidade
        pid_i=0.01,      # Correção integral
        pid_d=0.02,      # Suavização
        dead_zone_x=30,  # Zona morta horizontal (pixels)
        dead_zone_y=30,  # Zona morta vertical (pixels)
        max_yaw_rate=30, # Rotação máxima (graus/s)
        max_alignment_time=10.0  # Timeout máximo
    )
    
    # ------------------------------------------------------------------------
    # 4. CARREGA MODELO YOLO
    # ------------------------------------------------------------------------
    print("\n[4] Carregando modelo YOLO...")
    
    try:
        model = YOLO(MODEL_PATH)
        print(f"   ✅ Modelo carregado: {MODEL_PATH}")
    except Exception as e:
        print(f"   ❌ Erro ao carregar modelo: {e}")
        print("   Usando modo de simulação (sem YOLO real)...")
        model = None
    
    # ------------------------------------------------------------------------
    # 5. ABRE A CÂMERA
    # ------------------------------------------------------------------------
    print("\n[5] Abrindo câmera...")
    
    cap = cv2.VideoCapture(0)  # 0 = webcam padrão
    if not cap.isOpened():
        print("   ⚠️ Não foi possível abrir a câmera!")
        print("   Usando modo de simulação...")
        usar_camera = False
    else:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        usar_camera = True
        print("   ✅ Câmera aberta!")
    
    # ------------------------------------------------------------------------
    # 6. LOOP PRINCIPAL
    # ------------------------------------------------------------------------
    print("\n" + "=" * 60)
    print("   SISTEMA PRONTO!")
    print("=" * 60)
    print("\nControles:")
    print("   'a' - Alinhar com o objeto detectado (one-shot)")
    print("   't' - Rastrear objeto continuamente")
    print("   's' - Parar rastreamento")
    print("   'l' - Pousar drone")
    print("   'q' - Sair\n")
    
    tracking_active = False
    frame_count = 0
    
    while True:
        # Captura frame
        if usar_camera:
            ret, frame = cap.read()
            if not ret:
                print("   Erro na captura de frame!")
                break
        else:
            # Modo simulação: cria frame preto com um retângulo falso
            frame = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)
            # Desenha um retângulo "falso" que se move
            t = time.time()
            x = int(FRAME_WIDTH//2 + 100 * np.sin(t))
            y = int(FRAME_HEIGHT//2 + 50 * np.cos(t * 0.7))
            cv2.rectangle(frame, (x-50, y-50), (x+50, y+50), (0, 255, 0), 2)
            cv2.putText(frame, "SIMULATED OBJECT", (x-80, y-60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # Detecta objeto a cada 3 frames (para desempenho)
        detection = None
        if frame_count % 3 == 0 and model is not None:
            results = model(frame, conf=0.5, verbose=False)
            
            for result in results:
                boxes = result.boxes
                if boxes is not None and len(boxes) > 0:
                    for box in boxes:
                        # Obtém informações
                        cls_id = int(box.cls[0])
                        class_name = result.names[cls_id]
                        
                        # Verifica se é a classe alvo
                        if TARGET_CLASS.lower() in class_name.lower():
                            xywh = box.xywh[0].cpu().numpy()
                            conf = float(box.conf[0])
                            
                            detection = {
                                'box': xywh,
                                'conf': conf,
                                'class_id': cls_id,
                                'class_name': class_name
                            }
                            break  # Pega só a primeira detecção
        
        # Desenha overlay de alinhamento
        if detection:
            frame = vision.draw_alignment_overlay(frame, detection)
            
            # Mostra informações
            cv2.putText(frame, 
                       f"Target: {detection['class_name']} ({detection['conf']:.2f})",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Mostra estado do drone
        status = f"Mode: {drone.state.flight_mode()} | Armed: {drone.state.is_armed()}"
        cv2.putText(frame, status, (10, FRAME_HEIGHT - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        if tracking_active:
            cv2.putText(frame, "TRACKING ACTIVE", (FRAME_WIDTH - 200, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        # Mostra frame
        cv2.imshow("Drone Vision - Basico", frame)
        
        # Processa teclas
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            print("\n[Q] Saindo...")
            break
        
        elif key == ord('a') and detection:
            # ALINHAMENTO ONE-SHOT
            print(f"\n[A] Alinhando com {detection['class_name']}...")
            success = vision.align_to_box(
                detection,
                mode=AlignmentMode.ONE_SHOT,
                wait_for_completion=True,
                timeout=10.0
            )
            if success:
                print("   ✅ Alinhado com sucesso!")
            else:
                print("   ❌ Falha no alinhamento!")
        
        elif key == ord('t') and detection:
            # RASTREAMENTO CONTÍNUO
            if not tracking_active:
                print(f"\n[T] Iniciando rastreamento de {detection['class_name']}...")
                vision.start_tracking()
                tracking_active = True
            else:
                print("   Rastreamento já está ativo!")
        
        elif key == ord('s'):
            # PARAR RASTREAMENTO
            if tracking_active:
                print("\n[S] Parando rastreamento...")
                vision.stop_tracking()
                tracking_active = False
        
        elif key == ord('l'):
            # POUSAR
            print("\n[L] Iniciando pouso...")
            drone.land()
            time.sleep(1)
            print("   Comando de pouso enviado!")
            
            # Aguarda pouso por 20 segundos
            for i in range(20):
                if not drone.state.is_armed():
                    print("   ✅ Drone pousado!")
                    break
                time.sleep(1)
            
            # Se ainda estiver armado, força desarme
            if drone.state.is_armed():
                print("   Forçando desarme...")
                drone.disarm()
        
        # Atualiza rastreamento se ativo
        if tracking_active and detection:
            vision.update_detection(detection)
        
        frame_count += 1
    
    # ------------------------------------------------------------------------
    # 7. LIMPEZA FINAL
    # ------------------------------------------------------------------------
    print("\n[6] Finalizando...")
    
    # Para rastreamento
    if tracking_active:
        vision.stop_tracking()
    
    # Pousa se ainda estiver no ar
    if drone.state.is_armed():
        print("   Pousando...")
        drone.land()
        time.sleep(3)
        drone.disarm()
    
    # Desconecta
    drone.disconnect()
    
    # Fecha câmera
    if usar_camera:
        cap.release()
    
    cv2.destroyAllWindows()
    
    print("\n✅ Programa finalizado com sucesso!")


# ============================================================================
# VERSÃO SUPER SIMPLIFICADA (APENAS TESTE DE CONCEITO)
# ============================================================================

def exemplo_super_simples():
    """
    Versão extremamente simples para testar apenas o conceito.
    Não requer drone real - simula o movimento.
    """
    print("\n" + "=" * 60)
    print("   VERSÃO SUPER SIMPLES (Modo Demonstração)")
    print("=" * 60)
    
    # Cria um drone mockado (simulado)
    class MockDrone:
        def __init__(self):
            self.yaw = 0
            self.altitude = 5
        
        def set_yaw_rate(self, rate):
            self.yaw += rate * 0.1
            print(f"   🎮 Yaw: {self.yaw:.1f}° (rate: {rate:.1f}°/s)")
        
        def set_velocity(self, vx, vy, vz):
            self.altitude -= vz * 0.1
            print(f"   🎮 Altitude: {self.altitude:.1f}m (vz: {vz:.1f}m/s)")
        
        def stop_navigation(self):
            print("   🛑 Navegação parada")
    
    # Cria controlador visual com drone mockado
    mock_drone = MockDrone()
    vision = VisualAlignmentController(mock_drone, 640, 480)
    
    # Configura
    vision.configure(dead_zone_x=30, dead_zone_y=30)
    
    print("\nSimulando detecção de objeto no canto direito da tela...")
    
    # Simula detecção
    detection = {
        'box': [500, 240, 100, 100],  # Objeto à direita
        'conf': 0.95,
        'class_name': 'hook'
    }
    
    print(f"   Objeto detectado em x=500 (centro da tela é 320)")
    print(f"   Erro horizontal: {500 - 320} pixels")
    
    # Alinha
    print("\nExecutando alinhamento...")
    success = vision.align_to_box(detection, mode=AlignmentMode.ONE_SHOT, timeout=5.0)
    
    if success:
        print("\n✅ Conceito validado! O algoritmo funciona!")
    else:
        print("\n⚠️ Alinhamento não completou (timeout)")
    
    print("\nDica: Para testar com drone real, execute a função main()")


# ============================================================================
# EXECUÇÃO
# ============================================================================

if __name__ == "__main__":
    print("\nEscolha uma opção:")
    print("1 - Versão completa (com drone real/SITL)")
    print("2 - Versão super simples (demonstração)")
    
    opcao = input("\nOpção (1/2): ").strip()
    
    if opcao == "1":
        main()
    else:
        exemplo_super_simples()