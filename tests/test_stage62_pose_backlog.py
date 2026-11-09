"""Targeted Stage 6.2 backlog / timestamp / quality tests."""
import pytest
from reliability.capture import select_fresh_pose_batch
from reliability.metrics import Metrics
from reliability.safety import ObstacleError


def frame(stamp, received):
    return (stamp, {}, received)


def test_old_frames_do_not_kill_a_fresh_batch():
    fresh, dropped, last, maximum = select_fresh_pose_batch(
        [frame(1.0, 9.0), frame(1.1, 9.8), frame(1.2, 9.95)], None, 10.0)
    assert [f[0] for f in fresh] == [1.1, 1.2]
    assert dropped == 1 and last == 1.2 and maximum == pytest.approx(1.0)


def test_latest_frame_still_must_be_fresh():
    with pytest.raises(ObstacleError, match='EVALUATION_POSE_STREAM_STALE'):
        select_fresh_pose_batch([frame(1, 9.0), frame(2, 9.5)], None, 10.0)


@pytest.mark.parametrize('frames,previous', [
    ([frame(1, 9.0), frame(1, 9.99)], None),
    ([frame(2, 9.0), frame(1, 9.99)], None),
    ([frame(1, 9.99)], 1.0),
])
def test_timestamp_reset_detected_even_in_discarded_frames(frames, previous):
    with pytest.raises(ObstacleError, match='EVALUATION_SIM_CLOCK_RESET'):
        select_fresh_pose_batch(frames, previous, 10.0)


def test_empty_batch_does_not_fabricate_pose():
    assert select_fresh_pose_batch([], 5.0, 10.0) == ([], 0, 5.0, None)


def test_drop_is_reported_as_incomplete_not_valid():
    m = Metrics({}); m.pose_stale_dropped = 3; m.pose_max_drain_age_s = 1.2
    quality = m.report()['quality']
    assert quality['status'] == 'INCOMPLETE'
    assert quality['pose_stale_messages_dropped'] == 3
    assert quality['max_pose_drain_age_s'] == 1.2
