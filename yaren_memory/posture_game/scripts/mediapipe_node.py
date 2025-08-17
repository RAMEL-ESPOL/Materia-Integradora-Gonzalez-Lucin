#!/usr/bin/env python3

# Nodo ROS 2 que usa MediaPipe Holistic para extraer landmarks de cuerpo y manos.
# Flujo:
# - Suscribe imágenes crudas desde /image_raw (sensor_msgs/Image), en formato YUYV.
# - Convierte a BGR con OpenCV y luego a RGB (lo que espera MediaPipe).
# - Procesa con MediaPipe Holistic para obtener pose y manos.
# - Aplana los landmarks (x, y, z) en listas 1D y publica un mensaje personalizado
#   posture_game_interfaces/HolisticLandmarks en /holistic_landmarks.
# Notas:
# - El mensaje personalizado incluye arrays planos para compatibilidad con otros nodos.
# - Manejo defensivo de resultados vacíos para evitar errores cuando no hay detecciones.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from posture_game_interfaces.msg import HolisticLandmarks
from cv_bridge import CvBridge
import mediapipe as mp
import cv2

class MediaPipeNode(Node):
    def __init__(self):

        super().__init__('mediapipe_node')

        self.bridge = CvBridge()


        self.publisher = self.create_publisher(HolisticLandmarks, '/holistic_landmarks', 10)

        # Suscripción a la cámara cruda; el callback procesa cada frame entrante
        self.subscription = self.create_subscription(Image,'/image_raw',self.image_callback,10)

        # Configuración de MediaPipe Holistic:
        # - static_image_mode=False: optimizado para video (usa tracking entre frames)
        # - model_complexity=1: equilibrio entre precisión y latencia
        # - smooth_landmarks=True: filtra jitter en landmarks
        # - enable_segmentation=False: no se requiere máscara segmentada aquí
        # - refine_face_landmarks=False: no se usan landmarks faciales detallados
        # - min_detection_confidence/min_tracking_confidence: umbrales de robustez
        mp_holistic = mp.solutions.holistic
        self.holistic = mp_holistic.Holistic(
            static_image_mode=False,
            model_complexity=1,
            smooth_landmarks=True,
            enable_segmentation=False,
            refine_face_landmarks=False,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )


        self.get_logger().info("MediaPipe Node iniciado correctamente ✅")

    def image_callback(self, msg):

        try:

            frame = self.bridge.imgmsg_to_cv2(msg)

            # La cámara publica YUYV; convierte a BGR para trabajar en OpenCV.
            # Si la fuente fuese ya BGR8, esta conversión debería ajustarse.
            bgr = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_YUY2)

            # MediaPipe espera imágenes RGB
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

            # Procesa el frame para extraer landmarks de pose y manos
            results = self.holistic.process(rgb)

            # Si no hay detección, results.pose_landmarks o hands pueden ser None
            pose = results.pose_landmarks.landmark if results.pose_landmarks else []
            left_hand = results.left_hand_landmarks.landmark if results.left_hand_landmarks else []
            right_hand = results.right_hand_landmarks.landmark if results.right_hand_landmarks else []

            # Aplanado de estructuras: por cada landmark, extrae (x, y, z) en una lista 1D
            # Esto simplifica el transporte y parsing en nodos subsiguientes.
            flat_pose = [coord for lm in pose for coord in (lm.x, lm.y, lm.z)]
            flat_left = [coord for lm in left_hand for coord in (lm.x, lm.y, lm.z)]
            flat_right = [coord for lm in right_hand for coord in (lm.x, lm.y, lm.z)]

            # Construye el mensaje de salida con timestamp actual del clock del nodo
            msg_out = HolisticLandmarks()
            msg_out.header.stamp = self.get_clock().now().to_msg()
            msg_out.pose_landmarks = flat_pose
            msg_out.left_hand_landmarks = flat_left
            msg_out.right_hand_landmarks = flat_right

            # Publica en /holistic_landmarks para consumo de otros nodos (e.g., checker_node)
            self.publisher.publish(msg_out)

        except Exception as e:
            # Registro de errores para diagnóstico sin detener el nodo
            self.get_logger().error(f"Error en MediaPipeNode: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MediaPipeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.holistic.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
