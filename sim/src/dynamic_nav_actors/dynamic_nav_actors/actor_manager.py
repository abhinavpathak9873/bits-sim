import math
import random
from dataclasses import dataclass
from pathlib import Path
from typing import List, Sequence, Tuple

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState, SpawnEntity
from geometry_msgs.msg import Pose
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


def _load_scenario(world: str) -> dict:
    share = Path(get_package_share_directory('dynamic_nav_worlds'))
    path = share / 'config' / 'scenarios' / f'{world}.yaml'
    with path.open('r', encoding='utf-8') as stream:
        return yaml.safe_load(stream)


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
        self.declare_parameter('update_rate_hz', 2.0)
        self.declare_parameter('max_actors', 4)

        self.world = self.get_parameter('world').value
        self.backend = self.get_parameter('backend').value
        self.crowd_density = self.get_parameter('crowd_density').value
        self.dynamic_obstacles = bool(self.get_parameter('dynamic_obstacles').value)
        self.seed = int(self.get_parameter('seed').value)
        self.update_period = 1.0 / float(self.get_parameter('update_rate_hz').value)
        self.max_actors = int(self.get_parameter('max_actors').value)
        self.random = random.Random(self.seed)
        self.scenario = _load_scenario(self.world)
        self.actors: List[MovingActor] = []
        self.spawned: set[str] = set()
        self.move_index = 0

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
        self.spawn_timer = self.create_timer(0.5, self._spawn_once)
        self.create_timer(self.update_period, self._tick)

    def _prepare_actors(self) -> None:
        profile = self.scenario['density_profiles'].get(self.crowd_density)
        if profile is None:
            raise ValueError(f'Unknown crowd_density={self.crowd_density!r}')
        pedestrian_routes = self.scenario['pedestrian_routes']
        cart_routes = self.scenario['cart_routes']
        for index in range(int(profile['pedestrians'])):
            if len(self.actors) >= self.max_actors:
                break
            route = [tuple(p) for p in self.random.choice(pedestrian_routes)]
            if self.random.random() < 0.5:
                route = list(reversed(route))
            self.actors.append(MovingActor(
                name=f'pedestrian_{index:03d}',
                route=route,
                speed=self.random.uniform(0.55, 1.25),
                radius=self.random.uniform(0.18, 0.27),
                height=self.random.uniform(1.45, 1.85),
                color='0.20 0.32 0.75 1',
                segment=self.random.randrange(max(1, len(route) - 1)),
                progress=self.random.random(),
            ))
        for index in range(int(profile['carts'])):
            if len(self.actors) >= self.max_actors:
                break
            route = [tuple(p) for p in cart_routes[index % len(cart_routes)]]
            self.actors.append(MovingActor(
                name=f'service_cart_{index:03d}',
                route=route,
                speed=self.random.uniform(0.35, 0.75),
                radius=0.42,
                height=0.75,
                color='0.85 0.58 0.16 1',
                segment=self.random.randrange(max(1, len(route) - 1)),
                progress=self.random.random(),
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
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f'Failed to spawn {name}: {exc}')
            self.spawned.discard(name)
            return
        if response.success:
            self.event_pub.publish(String(data=f'actor_spawned:{name}'))
            if len(self.spawned) == len(self.actors):
                self.get_logger().info(f'Spawned {len(self.spawned)} dynamic actors.')
                self.destroy_timer(self.spawn_timer)
        else:
            self.spawned.discard(name)
            self.get_logger().warn(f'Spawn rejected for {name}: {response.status_message}')

    def _tick(self) -> None:
        if not self.actors or len(self.spawned) != len(self.actors) or not self.state_client.service_is_ready():
            return
        for actor in self.actors:
            self._advance(actor, self.update_period)
        actor = self.actors[self.move_index % len(self.actors)]
        self.move_index += 1
        x, y, yaw = self._actor_pose(actor)
        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = actor.name
        request.state.pose = self._pose(x, y, actor.height / 2.0, yaw)
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
        actor.progress += actor.speed * dt / distance
        while actor.progress >= 1.0:
            actor.progress -= 1.0
            actor.segment = (actor.segment + 1) % len(actor.route)

    def _actor_pose(self, actor: MovingActor) -> Tuple[float, float, float]:
        start = actor.route[actor.segment]
        end = actor.route[(actor.segment + 1) % len(actor.route)]
        t = actor.progress
        x = start[0] + (end[0] - start[0]) * t
        y = start[1] + (end[1] - start[1]) * t
        yaw = math.atan2(end[1] - start[1], end[0] - start[0])
        return x, y, yaw

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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
