import math
from typing import Sequence

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class DummyAlgorithm(Node):
    def __init__(self) -> None:
        super().__init__('dynamic_nav_dummy_algorithm')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('max_linear_speed', 0.35)
        self.declare_parameter('max_angular_speed', 0.8)
        self.goal = None
        self.pose = None
        self.pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self._goal_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 20)
        self.create_timer(0.1, self._tick)

    def _goal_cb(self, msg: PoseStamped) -> None:
        self.goal = msg.pose.position

    def _odom_cb(self, msg: Odometry) -> None:
        self.pose = msg.pose.pose

    def _tick(self) -> None:
        cmd = Twist()
        if self.goal is None or self.pose is None:
            self.pub.publish(cmd)
            return
        dx = self.goal.x - self.pose.position.x
        dy = self.goal.y - self.pose.position.y
        distance = math.hypot(dx, dy)
        yaw = self._yaw_from_pose()
        heading = math.atan2(dy, dx)
        error = math.atan2(math.sin(heading - yaw), math.cos(heading - yaw))
        max_v = float(self.get_parameter('max_linear_speed').value)
        max_w = float(self.get_parameter('max_angular_speed').value)
        cmd.angular.z = max(-max_w, min(max_w, 1.6 * error))
        if abs(error) < 0.8:
            cmd.linear.x = min(max_v, 0.45 * distance)
        if distance < 0.35:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        self.pub.publish(cmd)

    def _yaw_from_pose(self) -> float:
        q = self.pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DummyAlgorithm()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
