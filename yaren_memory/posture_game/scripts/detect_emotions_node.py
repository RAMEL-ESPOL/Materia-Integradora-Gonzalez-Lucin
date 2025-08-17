#!/usr/bin/env python3

# Nodo ROS 2 para detección de emociones faciales en tiempo real.
# Flujo general:
# - Suscribe imágenes de /image_raw (sensor_msgs/Image)
# - Convierte a OpenCV con CvBridge
# - Detecta rostro y landmarks con MediaPipe FaceMesh
# - Extrae ROI de la cara, lo normaliza y redimensiona a 48x48
# - Predice la emoción con un modelo Keras (archivo .h5)
# - Publica el índice de la emoción (Int16) en /emotion
# - Escucha /shutdown_all para apagado limpio del nodo

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int16, Bool
import numpy as np
import tensorflow as tf
import cv2
import mediapipe as mp
import os
from ament_index_python.packages import get_package_share_directory 

class FacialExpressionModel(object):
    def __init__(self, model):
        
        self.model=model
        
        self.EMOTIONS_LIST = ["Enojo", "Disgusto", "Miedo", "Felicidad",
                            "Tristeza", "Sorpresa", "Neutral"]

    def predict_emotion(self, img):
        # Preprocesamiento mínimo: añade dimensión batch, normaliza a [0,1] y asegura tipo float32
        img = np.expand_dims(img, axis=0)
        img = img / 255.0
        img = img.astype(np.float32)

        # Infiere probabilidades por clase; verbose=0 para uso en tiempo real
        self.preds = self.model.predict(img, verbose=0)
        # Devuelve la etiqueta con mayor probabilidad
        return self.EMOTIONS_LIST[np.argmax(self.preds)]

class EmotionDetectionNode(Node):
    def __init__(self):
        # Inicializa el nodo con nombre fijo para rastreo en ROS 2
        super().__init__('emotion_detection_node')
        
        # Resuelve ruta del paquete para localizar el modelo dentro de share/
        pkg_path = get_package_share_directory('posture_game')
        model_path = os.path.join(pkg_path, 'models', 'emotions','model_mbn_1.h5')

        # Carga el modelo Keras desde archivo .h5
        self.model = tf.keras.models.load_model(model_path)
        # Wrapper para manejar preprocesamiento y etiquetas
        self.emotion_model = FacialExpressionModel(self.model)
        
        # Inicializa MediaPipe FaceMesh para detectar landmarks faciales
        # max_num_faces=1: asume un solo niño frente a la cámara
        # refine_landmarks=True: mejora precisión en ojos y labios
        # min_*_confidence=0.5: umbrales balanceados para velocidad/precisión
        mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = mp_face_mesh.FaceMesh(max_num_faces=1, refine_landmarks=True, 
                                min_detection_confidence=0.5, 
                                min_tracking_confidence=0.5)
        
        # Puente ROS <-> OpenCV para convertir imágenes
        self.bridge = CvBridge()

        # Publicador del índice de emoción en /emotion (Int16)
        self.publisher = self.create_publisher(Int16, '/emotion', 10)
        
        # Suscripción a la cámara y a señal de apagado global
        self.subscription = self.create_subscription(Image,'/image_raw',self.image_callback,10)
        self.create_subscription(Bool, '/shutdown_all', self.shutdown_callback, 10)

        
        self.get_logger().info('Emotion Detection Node initialized')

    def shutdown_callback(self, msg):
        # Maneja solicitud de apagado publicada en /shutdown_all
        if msg.data:
            # Mensaje de advertencia antes de cerrar el nodo y el contexto rclpy
            self.get_logger().warn("🛑 Apagando emotion_detection_node (señal /shutdown_all)")
            self.destroy_node()
            rclpy.shutdown()


    def image_callback(self, msg):
        # Callback por cada frame recibido desde /image_raw
        try:
            
            fr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            
            rgb_frame = cv2.cvtColor(fr, cv2.COLOR_BGR2RGB)
            # Procesa landmarks faciales
            results = self.face_mesh.process(rgb_frame)

            if results.multi_face_landmarks:
                # Se asume un solo rostro; se itera por consistencia con la API
                for face_landmarks in results.multi_face_landmarks:
                    # Obtiene dimensiones y proyecta landmarks normalizados a píxeles
                    h, w, _ = fr.shape
                    x_coords = [lm.x * w for lm in face_landmarks.landmark]
                    y_coords = [lm.y * h for lm in face_landmarks.landmark]

                    # Calcula bounding box mínimo que contiene todos los landmarks
                    x_min, x_max = int(min(x_coords)), int(max(x_coords))
                    y_min, y_max = int(min(y_coords)), int(max(y_coords))

                    # Expande el ROI para capturar contexto facial adicional
                    expand = 45
                    x_min = max(0, x_min - expand)
                    y_min = max(0, y_min - expand)
                    x_max = min(w, x_max + expand)
                    y_max = min(h, y_max + expand)

                    # Recorta la región de la cara sobre el frame RGB
                    fc = rgb_frame[y_min:y_max, x_min:x_max]
                    # Redimensiona al tamaño de entrada esperado por el modelo (48x48)
                    roi = cv2.resize(fc, (48, 48))

                    # Verificación defensiva: evita publicar si el ROI está vacío
                    if roi.size > 0:
                        # Predice etiqueta de emoción
                        pred = self.emotion_model.predict_emotion(roi)
                        # Publica el índice correspondiente a la etiqueta predicha
                        self.publisher.publish(Int16(data=self.emotion_model.EMOTIONS_LIST.index(pred)))

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    # Inicializa contexto ROS 2
    rclpy.init(args=args)
    
    # Instancia y arranca el nodo de detección de emociones
    emotion_detection_node = EmotionDetectionNode()
    
    try:
        # Mantiene el nodo activo procesando callbacks
        rclpy.spin(emotion_detection_node)
    except KeyboardInterrupt:
        # Permite detener con Ctrl+C sin traza de error
        pass
    finally:
        # Libera recursos gráficos si se usó OpenCV y cierra el nodo y ROS
        cv2.destroyAllWindows()
        emotion_detection_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    # Punto de entrada estándar en Python
    main()
