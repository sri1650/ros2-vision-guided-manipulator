"""ROS-independent data models used by the perception pipeline."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True)
class BoundingBox:
    """Axis-aligned image-space bounding box, in pixels."""

    x: int
    y: int
    width: int
    height: int

    def __post_init__(self) -> None:
        if self.x < 0 or self.y < 0:
            raise ValueError("bounding-box origin must be non-negative")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("bounding-box dimensions must be positive")

    @property
    def center(self) -> tuple[float, float]:
        return (self.x + (self.width - 1) / 2.0, self.y + (self.height - 1) / 2.0)

    def clipped(self, image_width: int, image_height: int) -> BoundingBox | None:
        x0 = max(0, min(self.x, image_width))
        y0 = max(0, min(self.y, image_height))
        x1 = max(0, min(self.x + self.width, image_width))
        y1 = max(0, min(self.y + self.height, image_height))
        if x1 <= x0 or y1 <= y0:
            return None
        return BoundingBox(x=x0, y=y0, width=x1 - x0, height=y1 - y0)


@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole camera model in pixel units."""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    frame_id: str = ""

    def __post_init__(self) -> None:
        values = (self.fx, self.fy, self.cx, self.cy)
        if not all(isfinite(value) for value in values):
            raise ValueError("camera intrinsics must be finite")
        if self.fx <= 0.0 or self.fy <= 0.0:
            raise ValueError("camera focal lengths must be positive")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("camera image dimensions must be positive")


@dataclass(frozen=True)
class Detection2D:
    """A class-labelled 2D observation produced by a detector."""

    label: str
    confidence: float
    bbox: BoundingBox

    def __post_init__(self) -> None:
        if not self.label:
            raise ValueError("detection label must not be empty")
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("detection confidence must be between zero and one")


@dataclass(frozen=True)
class PoseEstimate:
    """A position-only object estimate expressed in a named coordinate frame."""

    object_id: str
    label: str
    confidence: float
    position: tuple[float, float, float]
    size: tuple[float, float, float]
    frame_id: str
    valid_depth_samples: int

    def __post_init__(self) -> None:
        if not self.object_id or not self.label or not self.frame_id:
            raise ValueError("object_id, label and frame_id are required")
        if not all(isfinite(value) for value in (*self.position, *self.size)):
            raise ValueError("pose estimate values must be finite")
        if any(value < 0.0 for value in self.size):
            raise ValueError("object size cannot be negative")
