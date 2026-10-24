"""Episode boundaries must not replay cached HID motion or paused button clicks."""
from types import SimpleNamespace
import time

import numpy as np
import pytest

from ur5_capture import teleop
from ur5_capture.config import load_config


class CachedHID:
    """Match PySpaceMouse: read mutates and returns one cached state object."""

    def __init__(self):
        self.state = SimpleNamespace(t=1.0, x=0.6, y=0., z=0., roll=0., pitch=0., yaw=0.,
                                     buttons=[0, 0])
        self.pending = []

    def read(self):
        if self.pending:
            self.state.__dict__.update(self.pending.pop(0))
        return self.state

    def close(self):
        pass


def test_boundary_ignores_cached_motion_but_active_staleness_still_fails(monkeypatch):
    cfg = load_config("configs/mock.yaml")
    mouse = teleop.SpaceMouseInput(cfg.teleop, gripper_cfg=cfg.gripper)
    mouse.device = CachedHID()
    clock = SimpleNamespace(now=1.)
    monkeypatch.setattr(teleop, "time", SimpleNamespace(monotonic=lambda: clock.now))
    mouse.start_episode()
    clock.now += 10  # saving / user prompt takes much longer than stale_timeout_s
    for _ in range(3):
        sample = mouse.read()
        assert not sample.enabled
        np.testing.assert_array_equal(sample.axes, np.zeros(6))
    mouse.device.pending.append(dict(t=2., x=0.4))
    sample = mouse.read()
    assert sample.enabled and sample.axes[0] == 0.4
    clock.now += cfg.teleop.stale_timeout_s + 0.01
    with pytest.raises(RuntimeError, match="SpaceMouse motion report is stale"):
        mouse.read()
    mouse.start_episode()
    assert not mouse.read().enabled  # second episode suppresses the same stale report
    mouse.device.pending.append(dict(t=3., x=0.2))
    assert mouse.read().axes[0] == 0.2


def test_paused_clicks_are_dropped_and_command_latches_preserved():
    cfg = load_config("configs/mock.yaml")
    cfg.gripper.toggle_button = 0
    mouse = teleop.SpaceMouseInput(cfg.teleop, gripper_cfg=cfg.gripper)
    mouse.device = CachedHID()
    mouse.grasp = 1
    mouse.high_speed = True
    mouse.device.pending.extend([dict(t=2., buttons=[0, 0]), dict(t=3., buttons=[1, 1])])
    mouse.start_episode()
    assert mouse.read().grasp == 1
    assert mouse.high_speed  # held buttons must not synthesize another press
    mouse.device.pending.extend([dict(t=4., buttons=[0, 0]), dict(t=5., buttons=[1, 1])])
    sample = mouse.read()
    assert sample.grasp == 0 and not sample.high_speed


def test_second_episode_after_saving_starts_without_replaying_old_motion(tmp_path, monkeypatch):
    from ur5_capture import lerobot_bridge, record as recorder

    cfg = load_config("configs/mock.yaml")
    cfg.dataset.root = str(tmp_path / "dataset")
    cfg.dataset.episode_seconds = 0.1
    captured = [[], []]
    boundaries = []

    def connect(mouse, calibrate=True):
        # Exercise real HID handling while robot, gripper, and cameras stay mocked.
        mouse.mock = False
        mouse.device = CachedHID()
        mouse.connected = True

    def start_episode(mouse):
        teleop.SpaceMouseInput.start_episode(mouse)
        boundaries.append(True)
        if len(boundaries) == 1:
            mouse.device.pending.append(dict(t=2., x=0.4))

    class Writer:
        def __init__(self, config, metadata):
            self.root = tmp_path
            (tmp_path / "meta").mkdir()
            self.count = self.saved = 0

        def add(self, frame):
            captured[self.saved].append(frame)
            self.count += 1

        def save(self):
            self.saved += 1
            self.count = 0
            time.sleep(cfg.teleop.stale_timeout_s + 0.05)

        def close(self):
            pass

    monkeypatch.setattr(lerobot_bridge.SpaceMouseTeleoperator, "connect", connect)
    monkeypatch.setattr(lerobot_bridge.SpaceMouseTeleoperator, "start_episode", start_episode)
    monkeypatch.setattr(recorder, "V3Writer", Writer)
    result = recorder.record(cfg, unattended=True)
    assert result["saved_episodes"] == 2
    assert len(boundaries) == 2
    assert [len(frames) for frames in captured] == [3, 3]
    assert any(np.any(frame["action"][:6]) for frame in captured[0])
    for frame in captured[1]:
        np.testing.assert_array_equal(frame["action"][:6], np.zeros(6))
        assert not frame["complementary_info.motion_enabled"].item()
