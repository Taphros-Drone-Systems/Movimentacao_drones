from ultralytics import YOLO

# Load a model
model = YOLO("Hang_the_hook/hang_the_hook_best.pt")  # load a custom model

# Predict with the model
results = model.predict(source="2", show=True, stream=True, verbose=True, conf=0.5)

for result in results:
    # Caixas delimitadoras (bounding boxes)
    boxes = result.boxes  # Objeto com as detecções
    if boxes is not None:
        # Coordenadas das caixas (x1, y1, x2, y2)
        xyxy = boxes.xyxy  # Tensor com coordenadas
        xywh = boxes.xywh  # Formato centro x, centro y, largura, altura
        
        # Confiança das detecções
        conf = boxes.conf  # Valores de confiança
        
        # Classes detectadas
        cls = boxes.cls  # Índices das classes
        
        # IDs dos objetos (se tracking estiver ativado)
        ids = boxes.id  # IDs de tracking
        
    # Máscaras de segmentação (se aplicável)
    masks = result.masks
    
    # Pontos-chave (pose estimation)
    keypoints = result.keypoints
    
    # Imagem original
    orig_img = result.orig_img
    
    # Caminho do arquivo
    path = result.path
    
    # Nomes das classes
    names = result.names
    
    # Velocidade de processamento
    speed = result.speed  # {'preprocess': ..., 'inference': ..., 'postprocess': ...}