import numpy as np
from vgm_perception.segmentation import ColorRange, ColorSegmenter


def test_segments_red_blob_and_ignores_small_noise():
    image = np.zeros((20, 30, 3), dtype=np.uint8)
    image[5:15, 8:20] = (255, 0, 0)
    image[1, 1] = (255, 0, 0)
    segmenter = ColorSegmenter(
        [ColorRange("red_part", 345.0, 15.0, min_area_px=10)]
    )

    detections = segmenter.detect(image)

    assert len(detections) == 1
    assert detections[0].label == "red_part"
    assert detections[0].bbox.x == 8
    assert detections[0].bbox.y == 5
    assert detections[0].bbox.width == 12
    assert detections[0].bbox.height == 10


def test_separates_disconnected_blobs():
    image = np.zeros((12, 20, 3), dtype=np.uint8)
    image[2:6, 2:6] = (0, 0, 255)
    image[5:10, 13:18] = (0, 0, 255)
    segmenter = ColorSegmenter(
        [ColorRange("blue_part", 200.0, 260.0, min_area_px=8)]
    )

    detections = segmenter.detect(image)

    assert len(detections) == 2
    assert [item.bbox.x for item in detections] == [2, 13]
