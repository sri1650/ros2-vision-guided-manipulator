"""Deterministic colour segmentation intended for simulation and calibration."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np

from .models import BoundingBox, Detection2D

try:
    import cv2
except ImportError:  # pragma: no cover - exercised on lean, non-ROS test hosts
    cv2 = None


@dataclass(frozen=True)
class ColorRange:
    """HSV threshold range; hue uses degrees, saturation/value use [0, 1]."""

    label: str
    hue_min: float
    hue_max: float
    saturation_min: float = 0.35
    value_min: float = 0.20
    min_area_px: int = 120
    max_area_px: int = 1_000_000

    def __post_init__(self) -> None:
        if not self.label:
            raise ValueError("colour range requires a label")
        if not (0.0 <= self.hue_min <= 360.0 and 0.0 <= self.hue_max <= 360.0):
            raise ValueError("hue thresholds must be in [0, 360]")
        if not 0.0 <= self.saturation_min <= 1.0:
            raise ValueError("saturation_min must be in [0, 1]")
        if not 0.0 <= self.value_min <= 1.0:
            raise ValueError("value_min must be in [0, 1]")
        if self.min_area_px < 1 or self.max_area_px < self.min_area_px:
            raise ValueError("invalid component area range")


class ColorSegmenter:
    """Find connected blobs matching configurable HSV ranges.

    This transparent baseline is useful for the supplied synthetic scene. It is
    not represented as a trained perception model and should be replaced or
    supplemented before operation in uncontrolled lighting.
    """

    def __init__(self, ranges: list[ColorRange]) -> None:
        if not ranges:
            raise ValueError("at least one colour range is required")
        self.ranges = tuple(ranges)

    def detect(self, rgb_image: np.ndarray) -> list[Detection2D]:
        if rgb_image.ndim != 3 or rgb_image.shape[2] != 3:
            raise ValueError("rgb_image must have shape (height, width, 3)")
        if rgb_image.dtype != np.uint8:
            raise ValueError("rgb_image must use uint8 channels")

        hue, saturation, value = _rgb_to_hsv(rgb_image)
        detections: list[Detection2D] = []
        for colour in self.ranges:
            if colour.hue_min <= colour.hue_max:
                hue_mask = (hue >= colour.hue_min) & (hue <= colour.hue_max)
            else:
                hue_mask = (hue >= colour.hue_min) | (hue <= colour.hue_max)
            mask = hue_mask & (saturation >= colour.saturation_min) & (
                value >= colour.value_min
            )
            for bbox, area in _connected_components(mask):
                if colour.min_area_px <= area <= colour.max_area_px:
                    fill_ratio = area / float(bbox.width * bbox.height)
                    confidence = min(1.0, 0.55 + 0.45 * fill_ratio)
                    detections.append(
                        Detection2D(
                            label=colour.label,
                            confidence=confidence,
                            bbox=bbox,
                        )
                    )
        return sorted(detections, key=lambda item: (item.label, item.bbox.x, item.bbox.y))


def _rgb_to_hsv(image: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    rgb = image.astype(np.float32) / 255.0
    red, green, blue = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    maximum = np.max(rgb, axis=2)
    minimum = np.min(rgb, axis=2)
    delta = maximum - minimum

    hue = np.zeros_like(maximum)
    nonzero = delta > 1e-8
    red_max = nonzero & (maximum == red)
    green_max = nonzero & (maximum == green)
    blue_max = nonzero & (maximum == blue)
    hue[red_max] = 60.0 * np.mod((green[red_max] - blue[red_max]) / delta[red_max], 6.0)
    hue[green_max] = 60.0 * (
        (blue[green_max] - red[green_max]) / delta[green_max] + 2.0
    )
    hue[blue_max] = 60.0 * (
        (red[blue_max] - green[blue_max]) / delta[blue_max] + 4.0
    )

    saturation = np.zeros_like(maximum)
    nonblack = maximum > 1e-8
    saturation[nonblack] = delta[nonblack] / maximum[nonblack]
    return hue, saturation, maximum


def _connected_components(mask: np.ndarray) -> list[tuple[BoundingBox, int]]:
    """Return 4-connected boxes, with a NumPy/Python fallback for lean tests."""

    if cv2 is not None:
        count, _, statistics, _ = cv2.connectedComponentsWithStats(
            mask.astype(np.uint8), connectivity=4
        )
        return [
            (
                BoundingBox(
                    x=int(statistics[index, cv2.CC_STAT_LEFT]),
                    y=int(statistics[index, cv2.CC_STAT_TOP]),
                    width=int(statistics[index, cv2.CC_STAT_WIDTH]),
                    height=int(statistics[index, cv2.CC_STAT_HEIGHT]),
                ),
                int(statistics[index, cv2.CC_STAT_AREA]),
            )
            for index in range(1, count)
        ]

    height, width = mask.shape
    visited = np.zeros_like(mask, dtype=bool)
    components: list[tuple[BoundingBox, int]] = []
    for y in range(height):
        for x in range(width):
            if not mask[y, x] or visited[y, x]:
                continue
            queue = deque([(x, y)])
            visited[y, x] = True
            min_x = max_x = x
            min_y = max_y = y
            area = 0
            while queue:
                current_x, current_y = queue.popleft()
                area += 1
                min_x = min(min_x, current_x)
                max_x = max(max_x, current_x)
                min_y = min(min_y, current_y)
                max_y = max(max_y, current_y)
                for next_x, next_y in (
                    (current_x - 1, current_y),
                    (current_x + 1, current_y),
                    (current_x, current_y - 1),
                    (current_x, current_y + 1),
                ):
                    if (
                        0 <= next_x < width
                        and 0 <= next_y < height
                        and mask[next_y, next_x]
                        and not visited[next_y, next_x]
                    ):
                        visited[next_y, next_x] = True
                        queue.append((next_x, next_y))
            components.append(
                (
                    BoundingBox(
                        x=min_x,
                        y=min_y,
                        width=max_x - min_x + 1,
                        height=max_y - min_y + 1,
                    ),
                    area,
                )
            )
    return components
