import numpy as np
import pytest
from vgm_perception.models import BoundingBox, CameraIntrinsics, Detection2D
from vgm_perception.synthetic import SyntheticDetector


def test_synthetic_detector_scales_normalized_box():
    image = np.zeros((100, 200, 3), dtype=np.uint8)

    detection = SyntheticDetector(width=0.2, height=0.4).detect(image)[0]

    assert detection.bbox.width == 40
    assert detection.bbox.height == 40
    assert detection.bbox.center == pytest.approx((99.5, 49.5))


def test_model_validation_rejects_invalid_inputs():
    with pytest.raises(ValueError):
        BoundingBox(0, 0, 0, 10)
    with pytest.raises(ValueError):
        Detection2D("part", 1.1, BoundingBox(0, 0, 1, 1))
    with pytest.raises(ValueError):
        CameraIntrinsics(0.0, 100.0, 10.0, 10.0, 20, 20)
