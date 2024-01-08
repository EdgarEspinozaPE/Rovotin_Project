import cv2
import mediapipe as mp
import numpy as np
import yaml


# Inicializar el módulo de MediaPipe para la detección holística
mp_holistic = mp.solutions.holistic
holistic = mp_holistic.Holistic()

# Inicializar la captura de video
cap = cv2.VideoCapture(0)  # Puedes cambiar el argumento a la ruta de un archivo de video si lo prefieres

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Convertir la imagen a RGB para que sea compatible con MediaPipe
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Realizar la detección holística
    results = holistic.process(rgb_frame)

    shoulderPos = np.empty(3)
    elbowPos = np.empty(3)
    handPos = np.empty(3)
    # Si se detecta el cuerpo y las manos, dibujar landmarks en la imagen
    if results.pose_landmarks and results.left_hand_landmarks:
        # Dibujar landmarks del cuerpo
        if results.pose_landmarks:
            for i, landmark in enumerate(results.pose_landmarks.landmark):
                height, width, _ = frame.shape
                cx, cy, cz = int(landmark.x * width), int(landmark.y * height), int(landmark.z * width)
                landmark_name = f'PoseLandmark_{i}'

                #
                cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
                if i == 11:
                    shoulderPos = np.array([cx, cy, cz])
                    #print(f"{landmark_name}: ({cx}, {cy}, {cz})")
                elif i == 13:
                    elbowPos = np.array([cx, cy, cz])
                    #print(f"{landmark_name}: ({cx}, {cy}, {cz})")

        # Dibujar landmarks de las manos
        if results.left_hand_landmarks:
            for i, landmark in enumerate(results.left_hand_landmarks.landmark):
                height, width, _ = frame.shape
                cx, cy, cz = int(landmark.x * width), int(landmark.y * height), int(landmark.z * width)
                landmark_name = f'LeftHandLandmark_{i}'
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                if i== 12:
                    handPos = np.array([cx, cy, cz])
                    #print(f"{landmark_name}: ({cx}, {cy}, {cz})")    
    
        #TRANSFORMACIONES
        right = np.array([1.0, 0.0, 0.0])
        up = np.array([0.0, 1.0, 0.0])
        forward = np.array([0.0, 0.0, 1.0])

        # Calcular la dirección del codo
        elbowDirection = (elbowPos - shoulderPos)
        elbowDirection = elbowDirection / np.linalg.norm(elbowDirection)

        # Calcular vectores de la base S
        W = elbowDirection
        V = np.cross(W, right)
        V = V / np.linalg.norm(V)
        U = np.cross(V, W)

        # Calcular la dirección del brazo
        armDirection = (handPos - elbowPos)
        armDirection = armDirection / np.linalg.norm(armDirection)

        # Calcular el coseno del ángulo entre armDirection y elbowDirection
        cosTheta = np.dot(armDirection, elbowDirection)

        # Comprobación sospechosa
        if cosTheta <= 0:
            armDirection = V

        # Crear la matriz S
        S = np.vstack((U, V, W))

        S = np.transpose(S)

        #print(U,V,W,S)
        # Calcular la inversa de la matriz S
        S_inv = np.linalg.inv(S)

        # Calcular diffBasis
        diffBasis = np.dot(S_inv, armDirection)
        diffBasis = diffBasis / np.linalg.norm(diffBasis)
        data = {
            "target": {
                "x": diffBasis.x,
                "y": diffBasis.y,
                "z": diffBasis.z
            }
        }
        yaml_output = yaml.dump(data, default_flow_style=False)
        
    # Mostrar el resultado
    cv2.imshow('Holistic Landmark Detection', frame)

    # Salir con la tecla 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Liberar recursos
cap.release()
cv2.destroyAllWindows()
