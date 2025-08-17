#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import subprocess

class LaunchListener(Node):
    def __init__(self):
        super().__init__('launch_listener')
        self.subscription = self.create_subscription(
            Bool,
            '/launch_request',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        if msg.data:
            self.get_logger().info('Recibido TRUE: lanzando simulador...')
            try:
                subprocess.Popen([
                    'bash', '-c',
                    'ros2 launch coco_gazebo_sim coco_robot.launch.py'
                ])
            except Exception as e:
                self.get_logger().error(f'Error al lanzar: {e}')
        else:
            self.get_logger().info('Recibido FALSE: ignorado')

def main(args=None):
    rclpy.init(args=args)
    node = LaunchListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

