from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from vgm_interfaces.action import PickPlace
from vgm_interfaces.msg import DetectedObjectArray

from .model import Candidate, TaskCoordinator


class TaskManager(Node):
    """Selects a stable perception target and dispatches one guarded manipulation task."""

    def __init__(self) -> None:
        super().__init__("task_manager")
        target_class = self.declare_parameter("target_class", "workpiece").value
        minimum_confidence = self.declare_parameter("minimum_confidence", 0.80).value
        self.plan_only = self.declare_parameter("plan_only", True).value
        self.place_frame = self.declare_parameter("place_frame", "world").value
        self.place_xyz = (
            self.declare_parameter("place_x", 0.35).value,
            self.declare_parameter("place_y", -0.25).value,
            self.declare_parameter("place_z", 0.78).value,
        )
        self.coordinator = TaskCoordinator(str(target_class), float(minimum_confidence))
        self.client = ActionClient(self, PickPlace, "/pick_place")
        self.subscription = self.create_subscription(
            DetectedObjectArray, "/perception/objects", self._on_objects, 10
        )

    def _on_objects(self, message: DetectedObjectArray) -> None:
        if not self.client.server_is_ready():
            return
        candidates = [
            Candidate(
                object_id=item.object_id,
                class_label=item.class_label,
                confidence=float(item.confidence),
                depth_m=float(item.pose.pose.position.z),
            )
            for item in message.objects
        ]
        selected = self.coordinator.select(candidates)
        if selected is None:
            return
        target = next(
            (item for item in message.objects if item.object_id == selected.object_id), None
        )
        if target is None:
            return
        goal = PickPlace.Goal()
        goal.target = target
        goal.place_pose = self._place_pose()
        goal.execute = not self.plan_only
        self.coordinator.mark_dispatched()
        future = self.client.send_goal_async(goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)

    def _place_pose(self) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = str(self.place_frame)
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = self.place_xyz
        pose.pose.orientation.w = 1.0
        return pose

    def _on_feedback(self, feedback_message) -> None:
        if self.coordinator.state.name == "DISPATCHED":
            self.coordinator.mark_running()
        self.get_logger().info(
            f"stage={feedback_message.feedback.stage} "
            f"progress={feedback_message.feedback.progress:.0%}"
        )

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.coordinator.complete(False)
            self.get_logger().error("manipulation goal rejected")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        result = future.result().result
        self.coordinator.complete(bool(result.success))
        log = self.get_logger().info if result.success else self.get_logger().error
        log(result.message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TaskManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

