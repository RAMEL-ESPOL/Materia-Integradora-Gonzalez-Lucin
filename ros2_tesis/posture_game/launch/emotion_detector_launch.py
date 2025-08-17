from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[
                os.path.join(
                    get_package_share_directory('posture_game'),
                    'config',
                    'usb_cam_params.yaml'
                ),
                {'camera_name': 'cam_1', 'camera_info_url': ''}
            ],
            output='log'
        ),

        Node(
            package='posture_game',
            executable='detect_emotions_node.py',
            name='emotion_detector',
            output='screen'
        ),

        Node(
            package='posture_game',
            executable='emotion_viewer_backup.py',   # o 'emotion_viewer' si así registraste el entry point
            name='emotion_viewer',
            output='screen',
            parameters=[{
                'display_width': 1280,          # <-- tamaño de ventana
                'display_height': 720,
                'resizable_window': True,       # <-- ventana redimensionable
                'fullscreen': False,            # <-- pon True si quieres pantalla completa
                'software_scale': 1.0           # <-- 2.0 duplica el frame por software
            }]
        ),
    ])
