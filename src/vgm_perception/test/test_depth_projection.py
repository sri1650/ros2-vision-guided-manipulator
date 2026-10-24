import numpy as np
import pytest

from vgm_perception.depth_projection import DepthProjector
from vgm_perception.models import BoundingBox, CameraIntrinsics, Detection2D


def _intrinsics():
    return CameraIntrinsics(
        fx=100.0,
        fy=100.0,
        cx=4.5,
        cy=4.5,
        width=10,
        height=10,
        frame_id="camera_optical_frame",
    )


def test_projects_box_center_and_metric_size():
    detection = Detection2D("part", 0.9, BoundingBox(3, 3, 4, 4))
    depth = np.full((10, 10), 2_000, dtype=np.uint16)
    projector = DepthProjector(
        min_depth_m=0.1,
        max_depth_m=3.0,
        roi_fraction=1.0,
        minimum_valid_samples=4,
    )

    estimate = projector.estimate(
        detection,
        depth,
        _intrinsics(),
        "camera_optical_frame",
        "part-00",
        depth_scale=0.001,
    )

    assert estimate is not None
    assert estimate.position == pytest.approx((0.0, 0.0, 2.0))
    assert estimate.size == pytest.approx((0.08, 0.08, 0.03))
    assert estimate.valid_depth_samples == 16


def test_uses_median_and_rejects_invalid_depth():
    detection = Detection2D("part", 0.9, BoundingBox(3, 3, 4, 4))
    depth = np.full((10, 10), np.nan, dtype=np.float32)
    depth[3:7, 3:7] = 0.8
    depth[3, 3] = 100.0
    projector = DepthProjector(minimum_valid_samples=5, roi_fraction=1.0)

    estimate = projector.estimate(
        detection, depth, _intrinsics(), "camera", "part-00"
    )

    assert estimate is not None
    assert estimate.position[2] == pytest.approx(0.8)
    assert estimate.valid_depth_samples == 15


def test_returns_none_when_valid_depth_is_insufficient():
    detection = Detection2D("part", 0.9, BoundingBox(3, 3, 4, 4))
    depth = np.zeros((10, 10), dtype=np.float32)
    projector = DepthProjector(minimum_valid_samples=2)

    assert (
        projector.estimate(detection, depth, _intrinsics(), "camera", "part-00")
        is None
    )


def test_rejects_unregistered_depth_dimensions():
    detection = Detection2D("part", 0.9, BoundingBox(3, 3, 4, 4))
    with pytest.raises(ValueError, match="dimensions"):
        DepthProjector().estimate(
            detection,
            np.ones((8, 8)),
            _intrinsics(),
            "camera",
            "part-00",
        )
