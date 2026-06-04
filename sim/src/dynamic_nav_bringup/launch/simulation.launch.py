import os
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


TB3_MODELS = {
    'tb3_burger': ('burger', 'turtlebot3_burger'),
    'tb3_waffle': ('waffle', 'turtlebot3_waffle'),
    'tb3_waffle_pi': ('waffle_pi', 'turtlebot3_waffle_pi'),
}
TB4_MODELS = {'tb4_standard': 'standard', 'tb4_lite': 'lite'}


def _require_package(name: str) -> str:
    try:
        return get_package_share_directory(name)
    except PackageNotFoundError as exc:
        raise RuntimeError(
            f'Missing required ROS package {name!r}. Install/source the official TurtleBot '
            'simulation packages before launching this robot backend.'
        ) from exc


def _launch_setup(context, *args, **kwargs):
    backend = LaunchConfiguration('backend').perform(context)
    world = LaunchConfiguration('world').perform(context)
    robot = LaunchConfiguration('robot').perform(context)
    gui = LaunchConfiguration('gui').perform(context)
    seed = LaunchConfiguration('seed').perform(context)
    crowd_density = LaunchConfiguration('crowd_density').perform(context)
    dynamic_obstacles = LaunchConfiguration('dynamic_obstacles').perform(context)
    robot_name = LaunchConfiguration('robot_name').perform(context)

    worlds_share = Path(get_package_share_directory('dynamic_nav_worlds'))
    actions = [
        Node(
            package='dynamic_nav_actors',
            executable='actor_manager',
            output='screen',
            parameters=[{
                'world': world,
                'backend': backend,
                'seed': int(seed),
                'crowd_density': crowd_density,
                'dynamic_obstacles': dynamic_obstacles.lower() == 'true',
                'max_actors': 4,
                'update_rate_hz': 2.0,
            }],
        )
    ]

    if backend == 'classic':
        if robot not in TB3_MODELS:
            raise RuntimeError(f'Gazebo Classic backend supports only TurtleBot3 robots, got {robot!r}')
        tb3_model, model_dir = TB3_MODELS[robot]
        tb3_gazebo = _require_package('turtlebot3_gazebo')
        world_path = worlds_share / 'worlds' / 'classic' / f'{world}.world'
        model_path = Path(tb3_gazebo) / 'models' / model_dir / 'model.sdf'
        if not model_path.exists():
            raise RuntimeError(f'Official TurtleBot3 model file not found: {model_path}')
        start = _scenario_start(worlds_share, world)
        actions.insert(0, SetEnvironmentVariable('TURTLEBOT3_MODEL', tb3_model))
        server_cmd = (
            'source /usr/share/gazebo/setup.bash && '
            'source /opt/ros/humble/setup.bash && '
            f'gzserver {world_path} --seed={seed} '
            '-s /opt/ros/humble/lib/libgazebo_ros_init.so '
            '-s /opt/ros/humble/lib/libgazebo_ros_factory.so '
            '-s /opt/ros/humble/lib/libgazebo_ros_state.so '
            '-s /opt/ros/humble/lib/libgazebo_ros_force_system.so'
        )
        actions.insert(1, ExecuteProcess(
            cmd=['bash', '-lc', server_cmd],
            output='log',
        ))
        client_cmd = (
            'source /usr/share/gazebo/setup.bash && '
            'gzclient --gui-client-plugin=libgazebo_ros_eol_gui.so'
        )
        actions.insert(2, ExecuteProcess(
            cmd=['bash', '-lc', client_cmd],
            output='log',
            condition=IfCondition(LaunchConfiguration('gui')),
        ))
        actions.append(TimerAction(period=8.0, actions=[Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=[
                '-entity', robot_name,
                '-file', str(model_path),
                '-x', str(start['x']),
                '-y', str(start['y']),
                '-z', '0.01',
                '-Y', str(start.get('yaw', 0.0)),
                '-robot_namespace', robot_name,
            ],
        )]))
        return actions

    if backend == 'ignition':
        if robot not in TB4_MODELS:
            raise RuntimeError(f'Ignition/Gazebo Sim backend supports only TurtleBot4 robots, got {robot!r}')
        tb4_bringup = _require_package('turtlebot4_ignition_bringup')
        world_path = worlds_share / 'worlds' / 'ignition' / f'{world}.sdf'
        launch_name = 'ignition.launch.py'
        candidate = Path(tb4_bringup) / 'launch' / launch_name
        if not candidate.exists():
            launch_name = 'turtlebot4_ignition.launch.py'
            candidate = Path(tb4_bringup) / 'launch' / launch_name
        if not candidate.exists():
            raise RuntimeError(f'No known TurtleBot4 Ignition launch file found in {tb4_bringup}/launch')
        actions.insert(0, IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(candidate)),
            launch_arguments={
                'model': TB4_MODELS[robot],
                'world': str(world_path),
                'rviz': 'false',
                'nav2': 'false',
            }.items(),
        ))
        return actions

    raise RuntimeError(f'Unknown backend {backend!r}; expected classic or ignition')


def _scenario_start(worlds_share: Path, world: str) -> dict:
    import yaml

    scenario_path = worlds_share / 'config' / 'scenarios' / f'{world}.yaml'
    with scenario_path.open('r', encoding='utf-8') as stream:
        scenario = yaml.safe_load(stream)
    return scenario.get('robot_start', {'x': 0.0, 'y': 0.0, 'yaw': 0.0})


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='mall', description='mall or airport'),
        DeclareLaunchArgument('backend', default_value='classic', description='classic or ignition'),
        DeclareLaunchArgument('robot', default_value='tb3_burger', description='tb3_* for classic, tb4_* for ignition'),
        DeclareLaunchArgument('robot_name', default_value='robot'),
        DeclareLaunchArgument('seed', default_value='1'),
        DeclareLaunchArgument('crowd_density', default_value='low'),
        DeclareLaunchArgument('dynamic_obstacles', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('record', default_value='false'),
        DeclareLaunchArgument('run_id', default_value='manual_run'),
        OpaqueFunction(function=_launch_setup),
    ])
