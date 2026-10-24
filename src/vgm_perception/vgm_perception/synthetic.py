"""Synthetic detections for deterministic integration testing."""

from __future__ import annotations

from .models import BoundingBox, Detection2D


class SyntheticDetector:
    """Generate a fixed, normalized bounding box for each incoming image."""

    def __init__(
        self,
        label: str = "demo_part",
        confidence: float = 0.99,
        center_x: float = 0.50,
        center_y: float = 0.50,
        width: float = 0.20,
        height: float = 0.20,
    ) -> None:
        values = (center_x, center_y, width, height)
        if not all(0.0 <= value <= 1.0 for value in values):
            raise ValueError("normalized synthetic box values must be in [0, 1]")
        if width <= 0.0 or height <= 0.0:
            raise ValueError("synthetic box dimensions must be positive")
        self.label = label
        self.confidence = confidence
        self.center_x = center_x
        self.center_y = center_y
        self.width = width
        self.height = height

    def detect(self, image) -> list[Detection2D]:
        image_height, image_width = image.shape[:2]
        box_width = max(1, int(round(self.width * image_width)))
        box_height = max(1, int(round(self.height * image_height)))
        center_x = int(round(self.center_x * (image_width - 1)))
        center_y = int(round(self.center_y * (image_height - 1)))
        x = max(0, min(center_x - box_width // 2, image_width - box_width))
        y = max(0, min(center_y - box_height // 2, image_height - box_height))
        return [
            Detection2D(
                label=self.label,
                confidence=self.confidence,
                bbox=BoundingBox(x=x, y=y, width=box_width, height=box_height),
            )
        ]
