from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('dynamic_nav_bringup'), 'launch', 'simulation.launch.py'])
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'backend': LaunchConfiguration('backend'),
            'robot': LaunchConfiguration('robot'),
            'seed': LaunchConfiguration('seed'),
            'crowd_density': LaunchConfiguration('crowd_density'),
            'dynamic_obstacles': LaunchConfiguration('dynamic_obstacles'),
            'gui': LaunchConfiguration('gui'),
            'run_id': LaunchConfiguration('run_id'),
        }.items(),
    )
    runner = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'dynamic_nav_benchmark', 'experiment_runner',
            '--config', LaunchConfiguration('experiment'),
            '--run-id', LaunchConfiguration('run_id'),
        ],
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument('experiment'),
        DeclareLaunchArgument('world', default_value='mall'),
        DeclareLaunchArgument('backend', default_value='classic'),
        DeclareLaunchArgument('robot', default_value='tb3_waffle_pi'),
        DeclareLaunchArgument('seed', default_value='1'),
        DeclareLaunchArgument('crowd_density', default_value='medium'),
        DeclareLaunchArgument('dynamic_obstacles', default_value='true'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('run_id', default_value='benchmark_run'),
        simulation,
        runner,
    ])
