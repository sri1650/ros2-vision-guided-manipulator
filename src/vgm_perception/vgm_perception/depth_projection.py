"""Robust depth sampling and pinhole projection without ROS dependencies."""

from __future__ import annotations

import numpy as np

from .models import BoundingBox, CameraIntrinsics, Detection2D, PoseEstimate


class DepthProjector:
    """Project 2D detections into 3D using aligned depth images.

    The input depth image must already be registered to the colour image. This
    class deliberately does not hide calibration or image-registration errors.
    """

    def __init__(
        self,
        min_depth_m: float = 0.10,
        max_depth_m: float = 2.00,
        roi_fraction: float = 0.45,
        minimum_valid_samples: int = 5,
        assumed_thickness_m: float = 0.03,
    ) -> None:
        if not 0.0 < min_depth_m < max_depth_m:
            raise ValueError("depth range must satisfy 0 < minimum < maximum")
        if not 0.0 < roi_fraction <= 1.0:
            raise ValueError("roi_fraction must be in (0, 1]")
        if minimum_valid_samples < 1:
            raise ValueError("minimum_valid_samples must be positive")
        self.min_depth_m = float(min_depth_m)
        self.max_depth_m = float(max_depth_m)
        self.roi_fraction = float(roi_fraction)
        self.minimum_valid_samples = int(minimum_valid_samples)
        self.assumed_thickness_m = max(0.0, float(assumed_thickness_m))

    def estimate(
        self,
        detection: Detection2D,
        depth_image: np.ndarray,
        intrinsics: CameraIntrinsics,
        frame_id: str,
        object_id: str,
        depth_scale: float = 1.0,
    ) -> PoseEstimate | None:
        """Return a 3D estimate, or ``None`` when depth is insufficient."""

        if depth_image.ndim != 2:
            raise ValueError("depth_image must be a two-dimensional array")
        if depth_scale <= 0.0:
            raise ValueError("depth_scale must be positive")
        if depth_image.shape != (intrinsics.height, intrinsics.width):
            raise ValueError("depth image dimensions do not match CameraInfo")

        clipped = detection.bbox.clipped(intrinsics.width, intrinsics.height)
        if clipped is None:
            return None
        roi = self._central_roi(clipped)
        values_m = depth_image[
            roi.y : roi.y + roi.height, roi.x : roi.x + roi.width
        ].astype(np.float64, copy=False) * depth_scale
        valid = values_m[
            np.isfinite(values_m)
            & (values_m >= self.min_depth_m)
            & (values_m <= self.max_depth_m)
        ]
        if valid.size < self.minimum_valid_samples:
            return None

        depth_m = float(np.median(valid))
        u, v = clipped.center
        x_m = (u - intrinsics.cx) * depth_m / intrinsics.fx
        y_m = (v - intrinsics.cy) * depth_m / intrinsics.fy
        width_m = clipped.width * depth_m / intrinsics.fx
        height_m = clipped.height * depth_m / intrinsics.fy

        return PoseEstimate(
            object_id=object_id,
            label=detection.label,
            confidence=detection.confidence,
            position=(x_m, y_m, depth_m),
            size=(width_m, height_m, self.assumed_thickness_m),
            frame_id=frame_id,
            valid_depth_samples=int(valid.size),
        )

    def _central_roi(self, bbox: BoundingBox) -> BoundingBox:
        width = max(1, int(round(bbox.width * self.roi_fraction)))
        height = max(1, int(round(bbox.height * self.roi_fraction)))
        x = bbox.x + (bbox.width - width) // 2
        y = bbox.y + (bbox.height - height) // 2
        return BoundingBox(x=x, y=y, width=width, height=height)
