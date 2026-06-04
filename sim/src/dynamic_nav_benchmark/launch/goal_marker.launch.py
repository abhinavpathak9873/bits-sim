from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dynamic_nav_benchmark',
            executable='goal_marker',
            output='screen',
            parameters=[{
                'goal_topic': '/goal_pose',
                'marker_name': 'dynamic_nav_goal_marker',
            }],
        )
    ])
