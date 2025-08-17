from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'name',
            default_value='jugador',
            description='Nombre del jugador'
        ),

        Node(
            package='posture_game',
            executable='game_manage_node.py',
            output='screen',
            parameters=[{
                'name': LaunchConfiguration('name')
            }]
        )
    ])