#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from coco_services.srv import LaunchService  # <- tu servicio personalizado
import subprocess
import os
import signal

class LaunchMasterService(Node):
    def __init__(self):
        super().__init__('launch_master_service')

        self.extras_process = None
        self.game_process = None
        self.timer = None
        self.player_name = "jugador"  # valor por defecto

        # Servicio personalizado que ahora recibe el nombre
        self.start_all_srv = self.create_service(LaunchService, 'launch_start_all', self.start_all_callback)

        # Servicio de parada sigue usando Trigger
        from std_srvs.srv import Trigger
        self.stop_master_srv = self.create_service(Trigger, 'launch_stop_master', self.stop_master_callback)

    def start_all_callback(self, request, response):
        if self.extras_process or self.game_process:
            response.success = False
            response.message = 'Ya hay un launch activo.'
            return response

        self.player_name = request.player_name
        self.get_logger().info(f'Nombre del jugador recibido: {self.player_name}')

        self.get_logger().info('Lanzando extras_launch.py...')
        self.extras_process = subprocess.Popen(
            ['ros2', 'launch', 'posture_game', 'extras_launch.py'],
            preexec_fn=os.setsid
        )

        self.get_logger().info('Esperando 2 segundos antes de lanzar game_launch.py...')
        self.timer = self.create_timer(2.0, self.launch_game_once)

        response.success = True
        response.message = f'Lanzamiento iniciado. Jugador: {self.player_name}'
        return response

    def launch_game_once(self):
        if self.timer:
            self.timer.cancel()
            self.timer = None

        if self.game_process is None:
            self.get_logger().info('Lanzando game_launch.py...')
            self.game_process = subprocess.Popen(
                ['ros2', 'launch', 'posture_game', 'game_launch.py',
                 f'name:={self.player_name}'],
                preexec_fn=os.setsid
            )

    def stop_master_callback(self, request, response):
        if self.timer:
            self.get_logger().info('Cancelando temporizador de game_launch.py...')
            self.timer.cancel()
            self.timer = None

        if self.extras_process:
            self.get_logger().info('Terminando extras_launch.py...')
            os.killpg(os.getpgid(self.extras_process.pid), signal.SIGTERM)
            self.extras_process = None

        if self.game_process:
            self.get_logger().info('Terminando game_launch.py...')
            os.killpg(os.getpgid(self.game_process.pid), signal.SIGTERM)
            self.game_process = None

        response.success = True
        response.message = 'Launches detenidos.'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = LaunchMasterService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


