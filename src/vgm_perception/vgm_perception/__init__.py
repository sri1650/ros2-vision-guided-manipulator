"""RGB-D perception primitives for the vision-guided manipulator."""

from .depth_projection import DepthProjector
from .filtering import TemporalPositionFilter
from .models import BoundingBox, CameraIntrinsics, Detection2D, PoseEstimate
from .segmentation import ColorRange, ColorSegmenter
from .synthetic import SyntheticDetector

__all__ = [
    "BoundingBox",
    "CameraIntrinsics",
    "ColorRange",
    "ColorSegmenter",
    "DepthProjector",
    "Detection2D",
    "PoseEstimate",
    "SyntheticDetector",
    "TemporalPositionFilter",
]
