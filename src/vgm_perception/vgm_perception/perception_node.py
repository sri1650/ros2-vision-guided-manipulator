"""ROS 2 node connecting RGB-D images to typed object-pose messages."""

from __future__ import annotations

import json
from dataclasses import replace

import message_filters
import rclpy
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped conversions)
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from vgm_interfaces.msg import DetectedObjectArray

from .depth_projection import DepthProjector
from .filtering import TemporalPositionFilter
from .message_adapter import to_detected_object_message
from .models import CameraIntrinsics
from .segmentation import ColorRange, ColorSegmenter
from .synthetic import SyntheticDetector


class PerceptionNode(Node):
    """Estimate position of coloured or synthetic objects from aligned RGB-D."""

    def __init__(self) -> None:
        super().__init__("perception_node")
        self._declare_parameters()
        self._bridge = CvBridge()
        self._projector = DepthProjector(
            min_depth_m=self.get_parameter("depth.min_m").value,
            max_depth_m=self.get_parameter("depth.max_m").value,
            roi_fraction=self.get_parameter("depth.roi_fraction").value,
            minimum_valid_samples=self.get_parameter("depth.minimum_valid_samples").value,
            assumed_thickness_m=self.get_parameter("object.assumed_thickness_m").value,
        )
        self._filter = TemporalPositionFilter(
            alpha=self.get_parameter("filter.alpha").value,
            max_age_s=self.get_parameter("filter.max_age_s").value,
        )
        self._detector = self._create_detector()
        self._output_frame = self.get_parameter("output_frame").value
        self._tf_timeout = Duration(
            seconds=self.get_parameter("transform_timeout_s").value
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(
            DetectedObjectArray,
            self.get_parameter("topics.detections").value,
            10,
        )

        rgb_subscriber = message_filters.Subscriber(
            self,
            Image,
            self.get_parameter("topics.rgb_image").value,
            qos_profile=qos_profile_sensor_data,
        )
        depth_subscriber = message_filters.Subscriber(
            self,
            Image,
            self.get_parameter("topics.depth_image").value,
            qos_profile=qos_profile_sensor_data,
        )
        info_subscriber = message_filters.Subscriber(
            self,
            CameraInfo,
            self.get_parameter("topics.camera_info").value,
            qos_profile=qos_profile_sensor_data,
        )
        synchronizer = message_filters.ApproximateTimeSynchronizer(
            [rgb_subscriber, depth_subscriber, info_subscriber],
            queue_size=self.get_parameter("synchronizer.queue_size").value,
            slop=self.get_parameter("synchronizer.slop_s").value,
        )
        synchronizer.registerCallback(self._on_images)
        # Keep subscriptions alive for the lifetime of this node.
        self._subscribers = (rgb_subscriber, depth_subscriber, info_subscriber)
        self._synchronizer = synchronizer
        self.get_logger().info(
            f"RGB-D perception ready in '{self.get_parameter('detector.mode').value}' mode"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("topics.rgb_image", "/camera/color/image_raw")
        self.declare_parameter("topics.depth_image", "/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("topics.camera_info", "/camera/color/camera_info")
        self.declare_parameter("topics.detections", "/perception/objects")
        self.declare_parameter("detector.mode", "colour")
        self.declare_parameter("detector.colour_ranges_json", json.dumps([
            {
                "label": "workpiece",
                "hue_min": 345.0,
                "hue_max": 15.0,
                "saturation_min": 0.45,
                "value_min": 0.25,
                "min_area_px": 120,
            }
        ]))
        self.declare_parameter("detector.synthetic.label", "workpiece")
        self.declare_parameter("detector.synthetic.center_x", 0.50)
        self.declare_parameter("detector.synthetic.center_y", 0.50)
        self.declare_parameter("detector.synthetic.width", 0.20)
        self.declare_parameter("detector.synthetic.height", 0.20)
        self.declare_parameter("depth.min_m", 0.10)
        self.declare_parameter("depth.max_m", 2.00)
        self.declare_parameter("depth.roi_fraction", 0.45)
        self.declare_parameter("depth.minimum_valid_samples", 5)
        self.declare_parameter("depth.uint16_scale", 0.001)
        self.declare_parameter("object.assumed_thickness_m", 0.03)
        self.declare_parameter("filter.alpha", 0.45)
        self.declare_parameter("filter.max_age_s", 0.75)
        self.declare_parameter("pose.position_variance", 0.0004)
        self.declare_parameter("output_frame", "")
        self.declare_parameter("transform_timeout_s", 0.05)
        self.declare_parameter("synchronizer.queue_size", 10)
        self.declare_parameter("synchronizer.slop_s", 0.08)

    def _create_detector(self):
        mode = self.get_parameter("detector.mode").value.lower()
        if mode == "synthetic":
            return SyntheticDetector(
                label=self.get_parameter("detector.synthetic.label").value,
                center_x=self.get_parameter("detector.synthetic.center_x").value,
                center_y=self.get_parameter("detector.synthetic.center_y").value,
                width=self.get_parameter("detector.synthetic.width").value,
                height=self.get_parameter("detector.synthetic.height").value,
            )
        if mode != "colour":
            raise ValueError("detector.mode must be 'colour' or 'synthetic'")
        try:
            configurations = json.loads(
                self.get_parameter("detector.colour_ranges_json").value
            )
            return ColorSegmenter([ColorRange(**entry) for entry in configurations])
        except (TypeError, ValueError) as error:
            raise ValueError(f"invalid detector.colour_ranges_json: {error}") from error

    def _on_images(self, rgb_message, depth_message, info_message) -> None:
        try:
            rgb_image = self._bridge.imgmsg_to_cv2(rgb_message, desired_encoding="rgb8")
            depth_image = self._bridge.imgmsg_to_cv2(
                depth_message, desired_encoding="passthrough"
            )
        except CvBridgeError as error:
            self.get_logger().warning(f"image conversion failed: {error}")
            return

        if depth_image.ndim != 2:
            self.get_logger().warning("depth image is not single-channel; frame dropped")
            return
        depth_scale = (
            self.get_parameter("depth.uint16_scale").value
            if depth_message.encoding in ("16UC1", "mono16")
            else 1.0
        )
        source_frame = info_message.header.frame_id or depth_message.header.frame_id
        if not source_frame:
            self.get_logger().warning("camera frame_id is empty; frame dropped")
            return
        try:
            intrinsics = CameraIntrinsics(
                fx=info_message.k[0],
                fy=info_message.k[4],
                cx=info_message.k[2],
                cy=info_message.k[5],
                width=info_message.width,
                height=info_message.height,
                frame_id=source_frame,
            )
            detections = self._detector.detect(rgb_image)
        except ValueError as error:
            self.get_logger().warning(f"invalid RGB-D input: {error}")
            return
        estimates = []
        for index, detection in enumerate(detections):
            object_id = f"{detection.label}-{index:02d}"
            try:
                estimate = self._projector.estimate(
                    detection=detection,
                    depth_image=depth_image,
                    intrinsics=intrinsics,
                    frame_id=source_frame,
                    object_id=object_id,
                    depth_scale=depth_scale,
                )
            except ValueError as error:
                self.get_logger().warning(f"invalid aligned depth input: {error}")
                return
            if estimate is None:
                continue
            transformed = self._transform_estimate(estimate, depth_message.header.stamp)
            if transformed is None:
                continue
            timestamp_s = (
                depth_message.header.stamp.sec
                + depth_message.header.stamp.nanosec * 1e-9
            )
            estimates.append(self._filter.update(transformed, timestamp_s))

        output = DetectedObjectArray()
        output.header.stamp = depth_message.header.stamp
        output.header.frame_id = self._output_frame or source_frame
        covariance = self.get_parameter("pose.position_variance").value
        output.objects = [
            to_detected_object_message(estimate, output.header.stamp, covariance)
            for estimate in estimates
        ]
        self._publisher.publish(output)

    def _transform_estimate(self, estimate, stamp):
        target_frame = self._output_frame
        if not target_frame or target_frame == estimate.frame_id:
            return estimate
        point = PointStamped()
        point.header.stamp = stamp
        point.header.frame_id = estimate.frame_id
        point.point.x, point.point.y, point.point.z = estimate.position
        try:
            transformed = self._tf_buffer.transform(
                point,
                target_frame,
                timeout=self._tf_timeout,
            )
        except TransformException as error:
            self.get_logger().warning(
                f"transform {estimate.frame_id}->{target_frame} unavailable: {error}",
                throttle_duration_sec=2.0,
            )
            return None
        return replace(
            estimate,
            position=(
                transformed.point.x,
                transformed.point.y,
                transformed.point.z,
            ),
            frame_id=target_frame,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
