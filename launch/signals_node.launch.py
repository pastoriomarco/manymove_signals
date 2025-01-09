# launch/signals_node.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_model',
            default_value='lite6',
            description='Model of the robot (e.g., lite6, uf850, xarm)'
        ),

        Node(
            package='manymove_signals',
            executable='signals_node',
            name='manymove_signals_node',
            output='screen',
            parameters=[{
                'robot_model': LaunchConfiguration('robot_model')
            }]
        )
    ])
