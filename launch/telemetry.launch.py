import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robonaut-telemetry',
            executable='telemetry_node',
            name='telemetry',
            output='screen',
            parameters=[
                os.path.join(
                    get_package_share_directory('robonaut-telemetry'),
                    'config',
                    'telemetry.yaml'
                )
            ]
        )
    ])