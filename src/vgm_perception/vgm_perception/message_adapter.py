"""Conversion between pure pipeline models and vgm_interfaces messages."""

from __future__ import annotations


def to_detected_object_message(estimate, stamp, covariance_xyz: float):
    """Build a DetectedObject message while keeping ROS out of core modules."""

    from vgm_interfaces.msg import DetectedObject

    message = DetectedObject()
    message.header.stamp = stamp
    message.header.frame_id = estimate.frame_id
    message.object_id = estimate.object_id
    message.class_label = estimate.label
    message.confidence = float(estimate.confidence)
    message.pose.pose.position.x = estimate.position[0]
    message.pose.pose.position.y = estimate.position[1]
    message.pose.pose.position.z = estimate.position[2]
    message.pose.pose.orientation.w = 1.0
    message.pose.covariance[0] = covariance_xyz
    message.pose.covariance[7] = covariance_xyz
    message.pose.covariance[14] = covariance_xyz
    # Orientation is intentionally unobserved by this position-only baseline.
    message.pose.covariance[21] = -1.0
    message.pose.covariance[28] = -1.0
    message.pose.covariance[35] = -1.0
    message.size.x = estimate.size[0]
    message.size.y = estimate.size[1]
    message.size.z = estimate.size[2]
    return message
