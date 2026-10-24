import pytest

from vgm_perception.filtering import TemporalPositionFilter
from vgm_perception.models import PoseEstimate


def _estimate(position):
    return PoseEstimate(
        object_id="part-00",
        label="part",
        confidence=0.9,
        position=position,
        size=(0.1, 0.1, 0.03),
        frame_id="camera",
        valid_depth_samples=10,
    )


def test_smooths_repeated_measurements():
    filter_ = TemporalPositionFilter(alpha=0.25)
    first = filter_.update(_estimate((0.0, 0.0, 1.0)), timestamp_s=1.0)
    second = filter_.update(_estimate((1.0, 0.0, 1.0)), timestamp_s=1.1)

    assert first.position == (0.0, 0.0, 1.0)
    assert second.position == pytest.approx((0.25, 0.0, 1.0))


def test_expires_stale_tracks_before_update():
    filter_ = TemporalPositionFilter(alpha=0.1, max_age_s=0.5)
    filter_.update(_estimate((0.0, 0.0, 1.0)), timestamp_s=1.0)
    new_track = filter_.update(_estimate((1.0, 0.0, 1.0)), timestamp_s=2.0)

    assert new_track.position == (1.0, 0.0, 1.0)
    assert filter_.track_count == 1
