import threading
import time
from typing import Sequence

import rclpy
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node


MARKER_NAME = 'dynamic_nav_goal_marker'


class GoalMarker(Node):
    def __init__(self) -> None:
        super().__init__('dynamic_nav_goal_marker')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('marker_name', MARKER_NAME)
        self.declare_parameter('marker_z', 0.04)
        self.declare_parameter('spawn_service', '/spawn_entity')
        self.declare_parameter('delete_service', '/delete_entity')
        self.marker_name = str(self.get_parameter('marker_name').value)
        self.spawn_client = self.create_client(SpawnEntity, str(self.get_parameter('spawn_service').value))
        self.delete_client = self.create_client(DeleteEntity, str(self.get_parameter('delete_service').value))
        self._lock = threading.Lock()
        self._worker: threading.Thread | None = None
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('goal_topic').value),
            self._goal_cb,
            10,
        )
        self.get_logger().info(
            f'Goal marker listening on {self.get_parameter("goal_topic").value}; '
            f'Gazebo model name is {self.marker_name}'
        )

    def _goal_cb(self, msg: PoseStamped) -> None:
        pose = Pose()
        pose.position.x = msg.pose.position.x
        pose.position.y = msg.pose.position.y
        pose.position.z = float(self.get_parameter('marker_z').value)
        pose.orientation.w = 1.0
        with self._lock:
            if self._worker and self._worker.is_alive():
                return
            self._worker = threading.Thread(target=self._replace_marker, args=(pose,), daemon=True)
            self._worker.start()

    def _replace_marker(self, pose: Pose) -> None:
        if not self.spawn_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning('Gazebo /spawn_entity service is not available; goal marker not shown')
            return
        self.delete_client.wait_for_service(timeout_sec=1.0)
        if self.delete_client.service_is_ready():
            delete_request = DeleteEntity.Request()
            delete_request.name = self.marker_name
            self._wait_future(self.delete_client.call_async(delete_request), timeout_s=2.0)

        spawn_request = SpawnEntity.Request()
        spawn_request.name = self.marker_name
        spawn_request.xml = self._marker_sdf(self.marker_name)
        spawn_request.robot_namespace = ''
        spawn_request.initial_pose = pose
        spawn_request.reference_frame = 'world'
        future = self.spawn_client.call_async(spawn_request)
        response = self._wait_future(future, timeout_s=4.0)
        if response is not None and getattr(response, 'success', False):
            self.get_logger().info(f'Goal marker shown at x={pose.position.x:.2f}, y={pose.position.y:.2f}')
        elif response is not None:
            self.get_logger().warning(f'Goal marker spawn failed: {response.status_message}')

    @staticmethod
    def _wait_future(future, timeout_s: float):
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        return future.result() if future.done() else None

    @staticmethod
    def _marker_sdf(name: str) -> str:
        return f"""
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="goal_marker_link">
      <visual name="goal_disk">
        <pose>0 0 0.02 0 0 0</pose>
        <geometry><cylinder><radius>0.48</radius><length>0.04</length></cylinder></geometry>
        <material>
          <ambient>0.0 1.0 0.18 0.85</ambient>
          <diffuse>0.0 1.0 0.18 0.85</diffuse>
          <emissive>0.0 0.7 0.12 1.0</emissive>
        </material>
      </visual>
      <visual name="goal_ring">
        <pose>0 0 0.10 0 0 0</pose>
        <geometry><cylinder><radius>0.62</radius><length>0.03</length></cylinder></geometry>
        <material>
          <ambient>1.0 0.95 0.0 0.65</ambient>
          <diffuse>1.0 0.95 0.0 0.65</diffuse>
          <emissive>0.8 0.7 0.0 1.0</emissive>
        </material>
      </visual>
      <visual name="goal_beacon">
        <pose>0 0 0.75 0 0 0</pose>
        <geometry><cylinder><radius>0.055</radius><length>1.35</length></cylinder></geometry>
        <material>
          <ambient>0.0 0.95 1.0 0.75</ambient>
          <diffuse>0.0 0.95 1.0 0.75</diffuse>
          <emissive>0.0 0.55 0.8 1.0</emissive>
        </material>
      </visual>
      <visual name="goal_top">
        <pose>0 0 1.48 0 0 0</pose>
        <geometry><sphere><radius>0.18</radius></sphere></geometry>
        <material>
          <ambient>0.0 1.0 0.18 1.0</ambient>
          <diffuse>0.0 1.0 0.18 1.0</diffuse>
          <emissive>0.0 0.8 0.15 1.0</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GoalMarker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
