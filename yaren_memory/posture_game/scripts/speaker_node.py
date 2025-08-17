#!/usr/bin/env python3

# Nodo de síntesis de voz con Piper que:
# - Carga un modelo ONNX de TTS desde el share del paquete (resolución robusta de rutas con ament).
# - Serializa la reproducción de audio con un lock para evitar solapamientos.
# - Publica el estado /audio_playing (Bool) antes y después de hablar para sincronizar con otros nodos.
# - Ejecuta la síntesis y reproducción en un hilo aparte para no bloquear el ejecutor de ROS.
# - Usa un archivo WAV temporal en disco para pasar audio a playsound (interfaz simple, pero bloqueante).

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import os
from piper import PiperVoice
from ament_index_python.packages import get_package_share_directory
import tempfile
import wave
from playsound import playsound
import threading

class CocoSpeakerNode(Node):
    def __init__(self):
        super().__init__('coco_speaker_node')
        
        # Rutas del modelo y config TTS dentro de share/posture_game/models/TTS
        pkg_share_dir_tts = get_package_share_directory('posture_game')
        self.tts_model_path = os.path.join(pkg_share_dir_tts, 'models', 'TTS', 'es_MX-claude-high.onnx')
        self.tts_config_path = os.path.join(pkg_share_dir_tts, 'models', 'TTS', 'es_MX-claude-high.onnx.json')

        # Se inicializa Piper en GPU si está disponible (use_cuda=True)
        self.voice = None
        self.init_tts()
        
        # Canal de feedback para coordinación con el game manager
        self.audio_playing_publisher = self.create_publisher(Bool, '/audio_playing', 10)
        
        # Suscribe textos a hablar y señal global de apagado
        self.create_subscription(String, '/game_feedback', self.speak_game_feedback, 10)
        self.create_subscription(Bool, '/shutdown_all', self.shutdown_callback, 10)

        # Lock para garantizar que solo un hilo hable a la vez
        self.speaking_lock = threading.Lock()
        
        self.get_logger().info('Yaren Speaker Node started successfully')

    def shutdown_callback(self, msg):
        # Apagado ordenado del nodo por señal externa
        if msg.data:
            self.get_logger().warn("🛑 Apagando speaker_node (señal /shutdown_all)")
            self.destroy_node()
            rclpy.shutdown()

    def init_tts(self):
        # Carga del motor TTS Piper desde archivos ONNX/JSON.
        # En caso de fallo, el nodo seguirá vivo pero no podrá hablar.
        try:
            self.voice = PiperVoice.load(
                model_path=self.tts_model_path,
                config_path=self.tts_config_path,
                use_cuda=True
            )
            self.get_logger().info("TTS engine initialized successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize TTS engine: {str(e)}")
    
    def speak_game_feedback(self, msg):
        # Lanza la síntesis en un hilo para no bloquear el callback ni el executor.
        if msg.data:
            info_data = msg.data
            threading.Thread(target=self.speak_text, args=(info_data,)).start()
    
    def speak_text(self, text):
        # Secuencia de síntesis y reproducción:
        # - Publica True en /audio_playing
        # - Genera WAV temporal y sintetiza frames PCM con Piper (parámetros de prosodia ajustables)
        # - Reproduce de forma bloqueante con playsound
        # - Publica False en /audio_playing al terminar o ante error
        if self.voice is None:
            self.get_logger().error("TTS engine not initialized")
            return
            
        with self.speaking_lock:
            audio_status_msg = Bool()
            audio_status_msg.data = True
            self.audio_playing_publisher.publish(audio_status_msg)
            
            try:
                # Archivo temporal autolimpiable; wave.open escribe header WAV PCM lineal.
                with tempfile.NamedTemporaryFile(suffix=".wav", delete=True) as fp:
                    with wave.open(fp.name, 'wb') as wav_file:
                        wav_file.setnchannels(1)
                        wav_file.setsampwidth(2)  # 16 bits PCM
                        wav_file.setframerate(self.voice.config.sample_rate)
                        # length_scale/noise_scale/noise_w ajustan duración y variabilidad del habla
                        self.voice.synthesize(text, wav_file, length_scale=1.2, noise_scale=0.5, noise_w=0.8)
                    
                    # Reproducción bloqueante; este bloqueo ocurre dentro del hilo dedicado
                    playsound(fp.name)
            except Exception as e:
                self.get_logger().error(f"Error generating or playing speech: {str(e)}")
            finally:
                audio_status_msg.data = False
                self.audio_playing_publisher.publish(audio_status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CocoSpeakerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()