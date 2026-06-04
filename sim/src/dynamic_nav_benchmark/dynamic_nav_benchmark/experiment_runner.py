import argparse
import csv
import json
import math
import shlex
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, List, Optional, Sequence

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


@dataclass
class Sample:
    t: float
    x: float
    y: float
    speed: float
    min_scan: float


@dataclass
class Metrics:
    samples: List[Sample] = field(default_factory=list)
    events: List[str] = field(default_factory=list)
    cmd_speeds: List[float] = field(default_factory=list)
    last_scan_min: float = math.inf
    collision_count: int = 0
    near_miss_count: int = 0


class BenchmarkNode(Node):
    def __init__(self, config: dict, run_id: str, output_dir: Path) -> None:
        super().__init__('dynamic_nav_benchmark_runner')
        self.config = config
        self.run_id = run_id
        self.output_dir = output_dir
        self.metrics = Metrics()
        self.goal = config['goals'][0]
        self.start_time = self.get_clock().now()
        self.success = False
        self.timeout_s = float(config.get('timeout_s', 180.0))
        self.goal_tolerance_m = float(config.get('goal_tolerance_m', 0.45))
        self.collision_distance_m = float(config.get('collision_distance_m', 0.18))
        self.near_miss_distance_m = float(config.get('near_miss_distance_m', 0.65))
        self.goal_pub = self.create_publisher(PoseStamped, config.get('goal_topic', '/goal_pose'), 10)
        self.event_pub = self.create_publisher(String, '/benchmark/events', 10)
        self.create_subscription(Odometry, config.get('odom_topic', '/odom'), self._odom_cb, 50)
        self.create_subscription(LaserScan, config.get('scan_topic', '/scan'), self._scan_cb, 50)
        self.create_subscription(Twist, config.get('cmd_vel_topic', '/cmd_vel'), self._cmd_cb, 50)
        self.create_subscription(String, '/benchmark/events', self._event_cb, 50)
        self.create_timer(1.0, self._publish_goal)
        self.event_pub.publish(String(data=f'run_start:{self.run_id}'))

    def done(self) -> bool:
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        return self.success or elapsed >= self.timeout_s

    def timed_out(self) -> bool:
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        return (not self.success) and elapsed >= self.timeout_s

    def _publish_goal(self) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.config.get('frame_id', 'map')
        msg.pose.position.x = float(self.goal['x'])
        msg.pose.position.y = float(self.goal['y'])
        yaw = float(self.goal.get('yaw', 0.0))
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_pub.publish(msg)

    def _odom_cb(self, msg: Odometry) -> None:
        now = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = math.hypot(vx, vy)
        self.metrics.samples.append(Sample(now, x, y, speed, self.metrics.last_scan_min))
        if self.metrics.last_scan_min <= self.collision_distance_m:
            self.metrics.collision_count += 1
        elif self.metrics.last_scan_min <= self.near_miss_distance_m:
            self.metrics.near_miss_count += 1
        if math.hypot(x - float(self.goal['x']), y - float(self.goal['y'])) <= self.goal_tolerance_m:
            if not self.success:
                self.success = True
                self.event_pub.publish(String(data=f'goal_reached:{self.goal.get("name", "goal")}'))

    def _scan_cb(self, msg: LaserScan) -> None:
        finite = [value for value in msg.ranges if math.isfinite(value)]
        self.metrics.last_scan_min = min(finite) if finite else math.inf

    def _cmd_cb(self, msg: Twist) -> None:
        self.metrics.cmd_speeds.append(abs(msg.linear.x) + abs(msg.angular.z))

    def _event_cb(self, msg: String) -> None:
        self.metrics.events.append(msg.data)


def _launch_command(command: str | Sequence[str]) -> subprocess.Popen:
    if isinstance(command, str):
        argv = shlex.split(command)
    else:
        argv = list(command)
    if not argv:
        raise ValueError('algorithm_launch cannot be empty')
    return subprocess.Popen(argv)


def run_ros_trial(config: dict[str, Any], run_id: str, output_dir: Path) -> dict:
    command = config.get('algorithm_launch')
    process: Optional[subprocess.Popen] = None
    bag_process: Optional[subprocess.Popen] = None
    if command:
        process = _launch_command(command)
    if config.get('record', False):
        bag_dir = output_dir / run_id / 'bags' / 'topics'
        bag_dir.parent.mkdir(parents=True, exist_ok=True)
        topics = config.get('bag_topics', [
            config.get('scan_topic', '/scan'),
            config.get('odom_topic', '/odom'),
            config.get('cmd_vel_topic', '/cmd_vel'),
            config.get('goal_topic', '/goal_pose'),
            '/benchmark/events',
        ])
        bag_process = subprocess.Popen(['ros2', 'bag', 'record', '-o', str(bag_dir), *topics])
    rclpy.init()
    node = BenchmarkNode(config, run_id, output_dir)
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.1)
        timed_out = node.timed_out()
        node.event_pub.publish(String(data='timeout' if timed_out else 'run_complete'))
        summary = write_outputs(config, run_id, output_dir, node.metrics, node.success, timed_out)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if process:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
        if bag_process:
            bag_process.terminate()
            try:
                bag_process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                bag_process.kill()
    return summary


def run_dry_trial(config: dict, run_id: str, output_dir: Path) -> dict:
    metrics = Metrics(events=[f'run_start:{run_id}', 'dry_run', 'goal_reached:synthetic'])
    goal = config['goals'][0]
    start = config.get('robot_start', {'x': 0.0, 'y': 0.0})
    steps = 80
    for i in range(steps + 1):
        t = i / 10.0
        alpha = i / steps
        x = start['x'] + (goal['x'] - start['x']) * alpha
        y = start['y'] + (goal['y'] - start['y']) * alpha
        metrics.samples.append(Sample(t, x, y, 0.55, 1.5 - 0.4 * math.sin(alpha * math.pi)))
        metrics.cmd_speeds.append(0.55)
    return write_outputs(config, run_id, output_dir, metrics, True, False)


def write_outputs(config: dict, run_id: str, output_dir: Path, metrics: Metrics, success: bool, timed_out: bool) -> dict:
    run_dir = output_dir / run_id
    plots_dir = run_dir / 'plots'
    bags_dir = run_dir / 'bags'
    plots_dir.mkdir(parents=True, exist_ok=True)
    bags_dir.mkdir(parents=True, exist_ok=True)

    trajectory_path = run_dir / 'trajectory.csv'
    with trajectory_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(['t', 'x', 'y', 'speed', 'min_scan'])
        for sample in metrics.samples:
            writer.writerow([sample.t, sample.x, sample.y, sample.speed, sample.min_scan])

    path_length = 0.0
    for prev, cur in zip(metrics.samples, metrics.samples[1:]):
        path_length += math.hypot(cur.x - prev.x, cur.y - prev.y)
    speeds = [sample.speed for sample in metrics.samples]
    min_distance = min((sample.min_scan for sample in metrics.samples), default=math.inf)
    stops = sum(1 for value in metrics.cmd_speeds if value < 0.03)
    duration = metrics.samples[-1].t if metrics.samples else 0.0
    final_error = None
    if metrics.samples:
        final = metrics.samples[-1]
        goal = config['goals'][0]
        final_error = math.hypot(final.x - float(goal['x']), final.y - float(goal['y']))

    summary = {
        'run_id': run_id,
        'world': config.get('world'),
        'backend': config.get('backend'),
        'robot': config.get('robot'),
        'seed': config.get('seed'),
        'crowd_density': config.get('crowd_density'),
        'success': success,
        'timed_out': timed_out,
        'duration_s': duration,
        'path_length_m': path_length,
        'average_speed_mps': sum(speeds) / len(speeds) if speeds else 0.0,
        'peak_speed_mps': max(speeds) if speeds else 0.0,
        'collision_count': metrics.collision_count,
        'near_miss_count': metrics.near_miss_count,
        'minimum_obstacle_distance_m': None if math.isinf(min_distance) else min_distance,
        'stop_or_replan_count': stops,
        'final_pose_error_m': final_error,
        'events': metrics.events,
        'config_snapshot': config,
    }
    with (run_dir / 'summary.json').open('w', encoding='utf-8') as stream:
        json.dump(summary, stream, indent=2)
    with (run_dir / 'metrics.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(['metric', 'value'])
        for key, value in summary.items():
            if key not in {'events', 'config_snapshot'}:
                writer.writerow([key, value])

    if metrics.samples:
        plt.figure(figsize=(7, 5))
        plt.plot([s.x for s in metrics.samples], [s.y for s in metrics.samples], label='robot')
        goal = config['goals'][0]
        plt.scatter([goal['x']], [goal['y']], marker='x', label='goal')
        plt.axis('equal')
        plt.grid(True)
        plt.legend()
        plt.title(f'{run_id} trajectory')
        plt.savefig(plots_dir / 'trajectory.png', dpi=140)
        plt.close()
        plt.figure(figsize=(7, 4))
        plt.plot([s.t for s in metrics.samples], [s.speed for s in metrics.samples])
        plt.grid(True)
        plt.xlabel('time [s]')
        plt.ylabel('speed [m/s]')
        plt.title(f'{run_id} speed profile')
        plt.savefig(plots_dir / 'speed_profile.png', dpi=140)
        plt.close()
        finite_clearance = [(s.t, s.min_scan) for s in metrics.samples if math.isfinite(s.min_scan)]
        if finite_clearance:
            plt.figure(figsize=(7, 4))
            plt.plot([p[0] for p in finite_clearance], [p[1] for p in finite_clearance])
            plt.grid(True)
            plt.xlabel('time [s]')
            plt.ylabel('minimum scan range [m]')
            plt.title(f'{run_id} obstacle clearance')
            plt.savefig(plots_dir / 'clearance.png', dpi=140)
            plt.close()
    return summary


def load_config(path: Path) -> dict[str, Any]:
    with path.open('r', encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError(f'Experiment config {path} must contain a YAML mapping')
    if 'goals' not in config or not config['goals']:
        raise ValueError('Experiment config must define at least one goal')
    launch_command = config.get('algorithm_launch')
    if launch_command is not None and not isinstance(launch_command, (str, list, tuple)):
        raise ValueError('algorithm_launch must be a shell-style string or an argument list')
    return config


def main(argv: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    parser.add_argument('--run-id')
    parser.add_argument('--output-dir', default='results')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args(argv)

    config = load_config(Path(args.config))
    run_id = args.run_id or config.get('run_id') or f"{config.get('world', 'world')}_{int(time.time())}"
    output_dir = Path(args.output_dir)
    if args.dry_run:
        summary = run_dry_trial(config, run_id, output_dir)
    else:
        summary = run_ros_trial(config, run_id, output_dir)
    print(json.dumps({k: summary[k] for k in ['run_id', 'success', 'duration_s', 'path_length_m']}, indent=2))


if __name__ == '__main__':
    main()
