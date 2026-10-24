from types import SimpleNamespace

import pytest

from ur5_capture.replay import log_frames, select_episodes


def test_episode_selection():
    assert select_episodes("0,2,4-6,2", 8) == [0, 2, 4, 5, 6]
    assert select_episodes("all", 3) == [0, 1, 2]


@pytest.mark.parametrize("spec", ["", "-1", "3", "2-1", "0-1-2", "all,0"])
def test_invalid_episode_selection(spec):
    with pytest.raises(ValueError):
        select_episodes(spec, 3)


def test_continuous_time_rebases_episode_timestamps():
    times = []
    recording = SimpleNamespace(set_time=lambda name, **value: times.append((name, value)),
                                log=lambda *args: None)
    rr = SimpleNamespace(Scalars=lambda x: x)
    frames = [{"timestamp": t, "episode_index": 2} for t in [0.5, 0.6]]
    offset, seconds = log_frames(frames, recording, rr, [], 10)
    frames = [{"timestamp": t, "episode_index": 5} for t in [0, 0.1]]
    offset, seconds = log_frames(frames, recording, rr, [], 10, offset, seconds)
    assert offset == 4
    assert seconds == pytest.approx(0.4)
    assert [v["duration"] for k, v in times if k == "timestamp"] == pytest.approx([0, .1, .2, .3])
    assert [v["sequence"] for k, v in times if k == "frame_index"] == [0, 1, 2, 3]
    assert [v["sequence"] for k, v in times if k == "episode_index"] == [2, 2, 5, 5]


def test_separate_time_starts_at_zero():
    times = []
    recording = SimpleNamespace(set_time=lambda name, **value: times.append((name, value)),
                                log=lambda *args: None)
    rr = SimpleNamespace(Scalars=lambda x: x)
    for episode in [2, 5]:
        log_frames([{"timestamp": 5, "episode_index": episode}], recording, rr, [], 30)
    assert [v["duration"] for k, v in times if k == "timestamp"] == [0, 0]
