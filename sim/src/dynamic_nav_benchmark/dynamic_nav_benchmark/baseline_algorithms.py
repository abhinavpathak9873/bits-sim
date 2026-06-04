import math
import heapq
from pathlib import Path
from typing import Iterable, Sequence

import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _angle_wrap(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _yaw_from_pose(pose: Pose) -> float:
    q = pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _finite_ranges(scan: LaserScan) -> list[tuple[float, float]]:
    values = []
    for index, distance in enumerate(scan.ranges):
        if not math.isfinite(distance):
            continue
        if distance < scan.range_min or distance > scan.range_max:
            continue
        angle = scan.angle_min + index * scan.angle_increment
        values.append((angle, distance))
    return values


class BaselineController(Node):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('max_linear_speed', 0.28)
        self.declare_parameter('max_angular_speed', 0.9)
        self.declare_parameter('goal_tolerance_m', 0.35)
        self.declare_parameter('control_period_s', 0.1)

        self.goal: Pose | None = None
        self.pose: Pose | None = None
        self.scan: LaserScan | None = None

        self.pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.create_subscription(PoseStamped, self.get_parameter('goal_topic').value, self._goal_cb, 10)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self._odom_cb, 20)
        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value, self._scan_cb, 10)
        self.create_timer(float(self.get_parameter('control_period_s').value), self._tick)

    def _goal_cb(self, msg: PoseStamped) -> None:
        self.goal = msg.pose
        self.on_new_goal()

    def _odom_cb(self, msg: Odometry) -> None:
        self.pose = msg.pose.pose

    def _scan_cb(self, msg: LaserScan) -> None:
        self.scan = msg

    def _tick(self) -> None:
        self.pub.publish(self.compute_cmd())

    def compute_cmd(self) -> Twist:
        return Twist()

    def on_new_goal(self) -> None:
        pass

    def goal_vector(self) -> tuple[float, float, float, float]:
        if self.goal is None or self.pose is None:
            return 0.0, 0.0, 0.0, 0.0
        dx = self.goal.position.x - self.pose.position.x
        dy = self.goal.position.y - self.pose.position.y
        distance = math.hypot(dx, dy)
        heading = math.atan2(dy, dx)
        yaw = _yaw_from_pose(self.pose)
        return dx, dy, distance, _angle_wrap(heading - yaw)

    def stop_if_done(self, cmd: Twist, distance: float) -> Twist:
        if distance < float(self.get_parameter('goal_tolerance_m').value):
            return Twist()
        return cmd

    def sector_min(self, center: float, width: float, default: float = math.inf) -> float:
        if self.scan is None:
            return default
        values = [
            distance for angle, distance in _finite_ranges(self.scan)
            if abs(_angle_wrap(angle - center)) <= width * 0.5
        ]
        return min(values) if values else default

    def clear_toward(self, heading_error: float, clearance: float) -> bool:
        if self.scan is None:
            return True
        width = 0.65
        values = [
            distance for angle, distance in _finite_ranges(self.scan)
            if abs(_angle_wrap(angle - heading_error)) <= width * 0.5
        ]
        return not values or min(values) > clearance


class GoToGoalController(BaselineController):
    """Pure proportional go-to-goal baseline with no obstacle avoidance."""

    def __init__(self) -> None:
        super().__init__('baseline_go_to_goal')

    def compute_cmd(self) -> Twist:
        cmd = Twist()
        if self.goal is None or self.pose is None:
            return cmd
        _, _, distance, error = self.goal_vector()
        max_v = float(self.get_parameter('max_linear_speed').value)
        max_w = float(self.get_parameter('max_angular_speed').value)
        cmd.angular.z = _clamp(1.7 * error, -max_w, max_w)
        if abs(error) < 0.9:
            cmd.linear.x = min(max_v, 0.55 * distance)
        return self.stop_if_done(cmd, distance)


class Bug2Controller(BaselineController):
    """Bug-style baseline: head to goal, follow left wall when blocked."""

    def __init__(self) -> None:
        super().__init__('baseline_bug2')
        self.declare_parameter('obstacle_enter_m', 0.85)
        self.declare_parameter('obstacle_exit_m', 1.25)
        self.declare_parameter('desired_wall_distance_m', 0.75)
        self.mode = 'go_to_goal'

    def compute_cmd(self) -> Twist:
        cmd = Twist()
        if self.goal is None or self.pose is None:
            return cmd
        _, _, distance, error = self.goal_vector()
        max_v = float(self.get_parameter('max_linear_speed').value)
        max_w = float(self.get_parameter('max_angular_speed').value)
        enter = float(self.get_parameter('obstacle_enter_m').value)
        exit_distance = float(self.get_parameter('obstacle_exit_m').value)
        front = self.sector_min(0.0, 0.7)
        left = self.sector_min(math.pi / 2.0, 0.9)
        left_front = self.sector_min(math.pi / 4.0, 0.9)

        if self.mode == 'go_to_goal' and front < enter:
            self.mode = 'wall_follow'
        elif self.mode == 'wall_follow' and self.clear_toward(error, exit_distance) and abs(error) < 0.6:
            self.mode = 'go_to_goal'

        if self.mode == 'go_to_goal':
            cmd.angular.z = _clamp(1.6 * error, -max_w, max_w)
            if abs(error) < 0.9:
                cmd.linear.x = min(max_v, 0.5 * distance)
        else:
            desired = float(self.get_parameter('desired_wall_distance_m').value)
            wall_error = desired - min(left, left_front)
            cmd.linear.x = 0.16 if front > 0.55 else 0.04
            cmd.angular.z = _clamp(0.75 + 1.2 * wall_error, -max_w, max_w)

        return self.stop_if_done(cmd, distance)


class VFHController(BaselineController):
    """Lightweight vector-field histogram style local planner."""

    def __init__(self) -> None:
        super().__init__('baseline_vfh')
        self.declare_parameter('influence_distance_m', 2.1)
        self.declare_parameter('front_stop_m', 0.45)
        self.declare_parameter('repulsive_gain', 1.25)

    def compute_cmd(self) -> Twist:
        cmd = Twist()
        if self.goal is None or self.pose is None:
            return cmd
        _, _, distance, error = self.goal_vector()
        max_v = float(self.get_parameter('max_linear_speed').value)
        max_w = float(self.get_parameter('max_angular_speed').value)
        influence = float(self.get_parameter('influence_distance_m').value)
        repulsive_gain = float(self.get_parameter('repulsive_gain').value)

        target_x = math.cos(error)
        target_y = math.sin(error)
        repulse_x = 0.0
        repulse_y = 0.0
        if self.scan is not None:
            for angle, obstacle_distance in _sample_scan(_finite_ranges(self.scan), stride=6):
                if obstacle_distance > influence:
                    continue
                weight = repulsive_gain * (1.0 / max(obstacle_distance, 0.05) - 1.0 / influence)
                repulse_x -= weight * math.cos(angle)
                repulse_y -= weight * math.sin(angle)

        vx = target_x + repulse_x
        vy = target_y + repulse_y
        desired_heading = math.atan2(vy, vx)
        front = self.sector_min(0.0, 0.65)
        front_stop = float(self.get_parameter('front_stop_m').value)
        speed_scale = _clamp((front - front_stop) / 1.1, 0.0, 1.0)

        cmd.angular.z = _clamp(1.9 * desired_heading, -max_w, max_w)
        if abs(desired_heading) < 1.0:
            cmd.linear.x = min(max_v, 0.45 * distance) * speed_scale
        return self.stop_if_done(cmd, distance)


class FollowGapController(BaselineController):
    """Follow-the-gap lidar baseline for local obstacle avoidance."""

    def __init__(self) -> None:
        super().__init__('baseline_follow_gap')
        self.declare_parameter('free_distance_m', 1.15)
        self.declare_parameter('bubble_radius_rad', 0.28)
        self.declare_parameter('heading_weight', 0.65)

    def compute_cmd(self) -> Twist:
        cmd = Twist()
        if self.goal is None or self.pose is None:
            return cmd
        _, _, distance, goal_error = self.goal_vector()
        if self.scan is None:
            cmd.angular.z = _clamp(1.5 * goal_error, -0.8, 0.8)
            if abs(goal_error) < 0.9:
                cmd.linear.x = min(float(self.get_parameter('max_linear_speed').value), 0.45 * distance)
            return self.stop_if_done(cmd, distance)

        free_distance = float(self.get_parameter('free_distance_m').value)
        bubble = float(self.get_parameter('bubble_radius_rad').value)
        ranges = _finite_ranges(self.scan)
        usable = [item for item in ranges if abs(item[0]) <= 2.35]
        if not usable:
            return cmd

        nearest_angle, nearest_distance = min(usable, key=lambda item: item[1])
        gaps = []
        current = []
        for angle, obstacle_distance in usable:
            blocked_by_bubble = nearest_distance < free_distance and abs(_angle_wrap(angle - nearest_angle)) < bubble
            if obstacle_distance > free_distance and not blocked_by_bubble:
                current.append((angle, obstacle_distance))
            elif current:
                gaps.append(current)
                current = []
        if current:
            gaps.append(current)

        if not gaps:
            cmd.angular.z = 0.7
            return cmd

        heading_weight = float(self.get_parameter('heading_weight').value)
        best_gap = max(
            gaps,
            key=lambda gap: len(gap) - heading_weight * abs(_angle_wrap(gap[len(gap) // 2][0] - goal_error)),
        )
        best_angle = best_gap[len(best_gap) // 2][0]
        max_v = float(self.get_parameter('max_linear_speed').value)
        max_w = float(self.get_parameter('max_angular_speed').value)
        front = self.sector_min(0.0, 0.65)
        cmd.angular.z = _clamp(1.8 * best_angle, -max_w, max_w)
        if abs(best_angle) < 1.0:
            cmd.linear.x = max_v * _clamp((front - 0.45) / 1.2, 0.0, 1.0)
        return self.stop_if_done(cmd, distance)


class AStarController(BaselineController):
    """Grid A* global planner over scenario static obstacles plus waypoint following."""

    def __init__(self) -> None:
        super().__init__('baseline_astar')
        self.declare_parameter('world', 'mall')
        self.declare_parameter('grid_resolution_m', 0.35)
        self.declare_parameter('obstacle_inflation_m', 0.55)
        self.declare_parameter('waypoint_tolerance_m', 0.45)
        self.declare_parameter('replan_distance_m', 0.65)
        self.declare_parameter('front_stop_m', 0.55)
        self.path: list[tuple[float, float]] = []
        self.path_goal: tuple[float, float] | None = None
        self.bounds, self.static_obstacles = self._load_world()

    def on_new_goal(self) -> None:
        self.path = []
        self.path_goal = None

    def compute_cmd(self) -> Twist:
        cmd = Twist()
        if self.goal is None or self.pose is None:
            return cmd
        _, _, goal_distance, _ = self.goal_vector()
        if goal_distance < float(self.get_parameter('goal_tolerance_m').value):
            return cmd

        goal_xy = (self.goal.position.x, self.goal.position.y)
        if not self.path or self.path_goal != goal_xy:
            self.path = self._plan((self.pose.position.x, self.pose.position.y), goal_xy)
            self.path_goal = goal_xy
        if not self.path:
            cmd.angular.z = 0.45
            return cmd

        while self.path and math.hypot(self.path[0][0] - self.pose.position.x, self.path[0][1] - self.pose.position.y) < float(
            self.get_parameter('waypoint_tolerance_m').value
        ):
            self.path.pop(0)
        if not self.path:
            return cmd

        front = self.sector_min(0.0, 0.65)
        if front < float(self.get_parameter('front_stop_m').value):
            cmd.angular.z = 0.65
            return cmd

        target_x, target_y = self.path[0]
        dx = target_x - self.pose.position.x
        dy = target_y - self.pose.position.y
        waypoint_distance = math.hypot(dx, dy)
        yaw = _yaw_from_pose(self.pose)
        error = _angle_wrap(math.atan2(dy, dx) - yaw)
        max_v = float(self.get_parameter('max_linear_speed').value)
        max_w = float(self.get_parameter('max_angular_speed').value)
        cmd.angular.z = _clamp(1.7 * error, -max_w, max_w)
        if abs(error) < 0.9:
            cmd.linear.x = min(max_v, 0.55 * waypoint_distance)
        return cmd

    def _load_world(self) -> tuple[tuple[float, float, float, float], list[dict]]:
        world = str(self.get_parameter('world').value)
        scenario_path = Path(get_package_share_directory('dynamic_nav_worlds')) / 'config' / 'scenarios' / f'{world}.yaml'
        scenario = yaml.safe_load(scenario_path.read_text(encoding='utf-8')) or {}
        if world == 'airport':
            bounds = (-15.2, 15.2, -9.2, 9.2)
        else:
            bounds = (-14.2, 14.2, -10.2, 10.2)
        obstacles = [
            {'x': float(item['x']), 'y': float(item['y']), 'sx': float(item['sx']), 'sy': float(item['sy'])}
            for item in scenario.get('static_obstacles', [])
        ]
        return bounds, obstacles

    def _plan(self, start_xy: tuple[float, float], goal_xy: tuple[float, float]) -> list[tuple[float, float]]:
        start = self._to_cell(start_xy)
        goal = self._to_cell(goal_xy)
        if self._blocked(goal):
            return []
        queue = [(0.0, start)]
        came_from: dict[tuple[int, int], tuple[int, int] | None] = {start: None}
        cost_so_far = {start: 0.0}
        neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
        while queue:
            _, current = heapq.heappop(queue)
            if current == goal:
                break
            for dx, dy in neighbors:
                nxt = (current[0] + dx, current[1] + dy)
                if self._blocked(nxt):
                    continue
                step_cost = math.hypot(dx, dy)
                new_cost = cost_so_far[current] + step_cost
                if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                    cost_so_far[nxt] = new_cost
                    priority = new_cost + math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
                    heapq.heappush(queue, (priority, nxt))
                    came_from[nxt] = current
        if goal not in came_from:
            return []
        cells = []
        current = goal
        while current is not None:
            cells.append(current)
            current = came_from[current]
        cells.reverse()
        return [self._to_world(cell) for cell in cells[1::3]] + [goal_xy]

    def _to_cell(self, xy: tuple[float, float]) -> tuple[int, int]:
        xmin, _, ymin, _ = self.bounds
        resolution = float(self.get_parameter('grid_resolution_m').value)
        return (round((xy[0] - xmin) / resolution), round((xy[1] - ymin) / resolution))

    def _to_world(self, cell: tuple[int, int]) -> tuple[float, float]:
        xmin, _, ymin, _ = self.bounds
        resolution = float(self.get_parameter('grid_resolution_m').value)
        return (xmin + cell[0] * resolution, ymin + cell[1] * resolution)

    def _blocked(self, cell: tuple[int, int]) -> bool:
        x, y = self._to_world(cell)
        xmin, xmax, ymin, ymax = self.bounds
        if x < xmin or x > xmax or y < ymin or y > ymax:
            return True
        inflation = float(self.get_parameter('obstacle_inflation_m').value)
        for obstacle in self.static_obstacles:
            if abs(x - obstacle['x']) <= obstacle['sx'] * 0.5 + inflation and abs(y - obstacle['y']) <= obstacle['sy'] * 0.5 + inflation:
                return True
        return False


def _sample_scan(values: Iterable[tuple[float, float]], stride: int) -> Iterable[tuple[float, float]]:
    for index, value in enumerate(values):
        if index % stride == 0:
            yield value


def _spin(node: Node, args: Sequence[str] | None = None) -> None:
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main_go_to_goal(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    _spin(GoToGoalController(), args)


def main_bug2(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    _spin(Bug2Controller(), args)


def main_vfh(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    _spin(VFHController(), args)


def main_follow_gap(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    _spin(FollowGapController(), args)


def main_astar(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    _spin(AStarController(), args)
