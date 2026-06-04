from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='dynamic_nav_benchmark', executable='dummy_algorithm', output='screen')
    ])
