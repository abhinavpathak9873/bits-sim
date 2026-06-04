import math
import random
from dataclasses import dataclass
from pathlib import Path
from typing import Any, List, Sequence, Tuple

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState, SpawnEntity
from geometry_msgs.msg import Pose
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


Point = Tuple[float, float]


@dataclass
class MovingActor:
    name: str
    route: List[Point]
    speed: float
    radius: float
    height: float
    color: str
    segment: int
    progress: float
    sway_phase: float
    sway_amplitude: float
    sway_rate: float
    bob_amplitude: float
    speed_phase: float
    kind: str


def _load_scenario(world: str) -> dict[str, Any]:
    share = Path(get_package_share_directory('dynamic_nav_worlds'))
    path = share / 'config' / 'scenarios' / f'{world}.yaml'
    with path.open('r', encoding='utf-8') as stream:
        scenario = yaml.safe_load(stream)
    if not isinstance(scenario, dict):
        raise ValueError(f'Scenario config {path} must contain a YAML mapping')
    return scenario


def _boxless_cylinder_sdf(name: str, radius: float, height: float, color: str) -> str:
    return f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{name}'>
    <static>false</static>
    <link name='body'>
      <inertial><mass>70.0</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
      <collision name='collision'><geometry><cylinder><radius>{radius}</radius><length>{height}</length></cylinder></geometry></collision>
      <visual name='visual'>
        <geometry><cylinder><radius>{radius}</radius><length>{height}</length></cylinder></geometry>
        <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>"""


class ActorManager(Node):
    def __init__(self) -> None:
        super().__init__('dynamic_nav_actor_manager')
        self.declare_parameter('world', 'mall')
        self.declare_parameter('backend', 'classic')
        self.declare_parameter('crowd_density', 'medium')
        self.declare_parameter('dynamic_obstacles', True)
        self.declare_parameter('seed', 1)
        self.declare_parameter('update_rate_hz', 10.0)
        self.declare_parameter('max_actors', 4)
        self.declare_parameter('control_mode', 'spawn_service')
        self.declare_parameter('speed_profile', 'low')

        self.world = self.get_parameter('world').value
        self.backend = self.get_parameter('backend').value
        self.crowd_density = self.get_parameter('crowd_density').value
        self.dynamic_obstacles = bool(self.get_parameter('dynamic_obstacles').value)
        self.seed = int(self.get_parameter('seed').value)
        self.update_period = 1.0 / float(self.get_parameter('update_rate_hz').value)
        self.max_actors = int(self.get_parameter('max_actors').value)
        self.control_mode = self.get_parameter('control_mode').value
        self.speed_profile = self.get_parameter('speed_profile').value
        self.random = random.Random(self.seed)
        self.scenario = _load_scenario(self.world)
        self.actors: List[MovingActor] = []
        self.spawned: set[str] = set()
        self.sim_time = 0.0

        self.event_pub = self.create_publisher(String, '/benchmark/events', 10)
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.state_client = self.create_client(SetEntityState, '/set_entity_state')

        if self.backend != 'classic':
            self.get_logger().warn(
                'ActorManager currently controls Gazebo Classic entities. '
                'Ignition/TB4 worlds launch, but dynamic obstacle control is Classic-only in this package.'
            )
            return
        if not self.dynamic_obstacles:
            self.get_logger().info('dynamic_obstacles is false; actor spawning is disabled.')
            return

        self._prepare_actors()
        if self.control_mode == 'existing_models':
            self.spawned = {actor.name for actor in self.actors}
            self.get_logger().info(f'Controlling {len(self.spawned)} preloaded crowd models.')
        else:
            self.spawn_timer = self.create_timer(0.5, self._spawn_once)
        self.create_timer(self.update_period, self._tick)

    def _prepare_actors(self) -> None:
        profile = self.scenario['density_profiles'].get(self.crowd_density)
        if profile is None:
            raise ValueError(f'Unknown crowd_density={self.crowd_density!r}')
        pedestrian_routes = self.scenario['pedestrian_routes']
        cart_routes = self.scenario['cart_routes']
        speed_scale = {
            'static': 0.0,
            'low': 0.55,
            'medium': 1.0,
            'mid': 1.0,
            'high': 1.55,
        }.get(self.speed_profile)
        if speed_scale is None:
            raise ValueError(f'Unknown speed_profile={self.speed_profile!r}')
        cart_limit = min(int(profile['carts']), max(0, self.max_actors // 4))
        if int(profile['carts']) > 0 and self.max_actors >= 4:
            cart_limit = max(1, cart_limit)
        pedestrian_limit = max(0, self.max_actors - cart_limit)
        for index in range(min(int(profile['pedestrians']), pedestrian_limit)):
            route = [tuple(p) for p in self.random.choice(pedestrian_routes)]
            if self.random.random() < 0.5:
                route = list(reversed(route))
            progress = ((index % 8) / 8.0 + self.random.uniform(-0.025, 0.025)) % 1.0
            self.actors.append(MovingActor(
                name=f'pedestrian_{index:03d}',
                route=route,
                speed=self.random.uniform(0.55, 1.25) * speed_scale,
                radius=self.random.uniform(0.18, 0.27),
                height=self.random.uniform(1.45, 1.85),
                color='0.20 0.32 0.75 1',
                segment=self.random.randrange(max(1, len(route) - 1)),
                progress=progress,
                sway_phase=self.random.uniform(0.0, math.tau),
                sway_amplitude=self.random.uniform(0.035, 0.11),
                sway_rate=self.random.uniform(0.8, 1.6),
                bob_amplitude=self.random.uniform(0.006, 0.018),
                speed_phase=self.random.uniform(0.0, math.tau),
                kind='pedestrian',
            ))
        for index in range(cart_limit):
            route = [tuple(p) for p in cart_routes[index % len(cart_routes)]]
            progress = ((index % 4) / 4.0 + 0.12) % 1.0
            self.actors.append(MovingActor(
                name=f'service_cart_{index:03d}',
                route=route,
                speed=self.random.uniform(0.35, 0.75) * speed_scale,
                radius=0.42,
                height=0.75,
                color='0.85 0.58 0.16 1',
                segment=self.random.randrange(max(1, len(route) - 1)),
                progress=progress,
                sway_phase=self.random.uniform(0.0, math.tau),
                sway_amplitude=self.random.uniform(0.0, 0.035),
                sway_rate=self.random.uniform(0.25, 0.55),
                bob_amplitude=0.0,
                speed_phase=self.random.uniform(0.0, math.tau),
                kind='cart',
            ))
        self.get_logger().info(
            f'Prepared {len(self.actors)} actors for {self.world}/{self.crowd_density} seed={self.seed}'
        )

    def _spawn_once(self) -> None:
        if not self.spawn_client.service_is_ready():
            self.get_logger().info('Waiting for /spawn_entity...')
            return
        for actor in self.actors:
            if actor.name in self.spawned:
                continue
            x, y, yaw = self._actor_pose(actor)
            request = SpawnEntity.Request()
            request.name = actor.name
            request.xml = _boxless_cylinder_sdf(actor.name, actor.radius, actor.height, actor.color)
            request.initial_pose = self._pose(x, y, actor.height / 2.0, yaw)
            future = self.spawn_client.call_async(request)
            future.add_done_callback(lambda _future, name=actor.name: self._spawn_done(name, _future))
            self.spawned.add(actor.name)

    def _spawn_done(self, name: str, future) -> None:
        spawn_error = future.exception()
        if spawn_error is not None:
            self.get_logger().warn(f'Failed to spawn {name}: {spawn_error}')
            self.spawned.discard(name)
            return
        spawn_reply = future.result()
        if spawn_reply.success:
            self.event_pub.publish(String(data=f'actor_spawned:{name}'))
            if len(self.spawned) == len(self.actors):
                self.get_logger().info(f'Spawned {len(self.spawned)} dynamic actors.')
                self.destroy_timer(self.spawn_timer)
        else:
            self.spawned.discard(name)
            self.get_logger().warn(f'Spawn rejected for {name}: {spawn_reply.status_message}')

    def _tick(self) -> None:
        if not self.actors or len(self.spawned) != len(self.actors):
            return
        if self.speed_profile == 'static':
            return
        self.sim_time += self.update_period
        self._advance_crowd(self.update_period)
        if not self.state_client.service_is_ready():
            return
        for actor in self.actors:
            x, y, yaw = self._actor_pose(actor, include_motion=True)
            request = SetEntityState.Request()
            request.state = EntityState()
            request.state.name = actor.name
            request.state.pose = self._pose(x, y, self._actor_z(actor), yaw)
            request.state.reference_frame = 'world'
            self.state_client.call_async(request)

    def _advance(self, actor: MovingActor, dt: float) -> None:
        start = actor.route[actor.segment]
        end = actor.route[(actor.segment + 1) % len(actor.route)]
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        if distance <= 1e-6:
            actor.segment = (actor.segment + 1) % len(actor.route)
            actor.progress = 0.0
            return
        speed_modulation = 1.0 + 0.12 * math.sin(self.sim_time * 0.75 + actor.speed_phase)
        actor.progress += actor.speed * speed_modulation * dt / distance
        while actor.progress >= 1.0:
            actor.progress -= 1.0
            actor.segment = (actor.segment + 1) % len(actor.route)

    def _advance_crowd(self, dt: float) -> None:
        for actor in self.actors:
            old_segment = actor.segment
            old_progress = actor.progress
            self._advance(actor, dt)
            x, y, _ = self._actor_pose(actor, include_motion=True)
            if self._blocked_by_static(actor, x, y) or self._blocked_by_actor(actor, x, y):
                actor.segment = old_segment
                actor.progress = old_progress

    def _blocked_by_static(self, actor: MovingActor, x: float, y: float) -> bool:
        for obstacle in self.scenario.get('static_obstacles', []):
            clearance = actor.radius + 0.18
            half_x = float(obstacle['sx']) / 2.0 + clearance
            half_y = float(obstacle['sy']) / 2.0 + clearance
            if abs(x - float(obstacle['x'])) <= half_x and abs(y - float(obstacle['y'])) <= half_y:
                return True
        return False

    def _blocked_by_actor(self, actor: MovingActor, x: float, y: float) -> bool:
        for other in self.actors:
            if other.name == actor.name:
                continue
            ox, oy, _ = self._actor_pose(other, include_motion=True)
            min_distance = actor.radius + other.radius + 0.30
            if math.hypot(x - ox, y - oy) < min_distance:
                return True
        return False

    def _actor_pose(self, actor: MovingActor, include_motion: bool = False) -> Tuple[float, float, float]:
        start = actor.route[actor.segment]
        end = actor.route[(actor.segment + 1) % len(actor.route)]
        t = actor.progress
        x = start[0] + (end[0] - start[0]) * t
        y = start[1] + (end[1] - start[1]) * t
        yaw = math.atan2(end[1] - start[1], end[0] - start[0])
        if include_motion and actor.kind == 'pedestrian':
            lateral = (
                actor.sway_amplitude * math.sin(self.sim_time * actor.sway_rate + actor.sway_phase)
                + 0.025 * math.sin(self.sim_time * (actor.sway_rate * 2.7) + actor.sway_phase * 0.41)
            )
            normal_x = -math.sin(yaw)
            normal_y = math.cos(yaw)
            x += normal_x * lateral
            y += normal_y * lateral
        return x, y, yaw

    def _actor_z(self, actor: MovingActor) -> float:
        if actor.kind != 'pedestrian':
            return actor.height / 2.0
        bob = actor.bob_amplitude * math.sin(self.sim_time * actor.sway_rate * 2.0 + actor.sway_phase)
        return actor.height / 2.0 + bob

    @staticmethod
    def _pose(x: float, y: float, z: float, yaw: float) -> Pose:
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)
        return pose


def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ActorManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        return
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
