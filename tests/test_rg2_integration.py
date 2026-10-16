"""Local bridge/recorder tests; upstream HF base classes are explicit test doubles.

These tests do not validate the official LeRobot writer or upstream class contracts.
"""
from dataclasses import dataclass
import importlib.util
from pathlib import Path
import sys
import types

import numpy as np
import pytest

from ur5_capture.config import load_config
from ur5_capture.robot import UR5Backend


@pytest.fixture
def bridge(monkeypatch):
    @dataclass
    class Config:
        id: str = "test"

        @classmethod
        def register_subclass(cls, name):
            return lambda subclass: subclass

    class Device:
        def __init__(self, config):
            self.id = config.id

    for name in ("lerobot", "lerobot.robots", "lerobot.teleoperators"):
        module = types.ModuleType(name)
        module.__path__ = []
        monkeypatch.setitem(sys.modules, name, module)
    for name, symbol, value in (
        ("lerobot.robots.config", "RobotConfig", Config),
        ("lerobot.robots.robot", "Robot", Device),
        ("lerobot.teleoperators.config", "TeleoperatorConfig", Config),
        ("lerobot.teleoperators.teleoperator", "Teleoperator", Device),
    ):
        module = types.ModuleType(name)
        setattr(module, symbol, value)
        monkeypatch.setitem(sys.modules, name, module)
    source = Path(__file__).resolve().parents[1] / "src/ur5_capture/lerobot_bridge.py"
    spec = importlib.util.spec_from_file_location("ur5_capture.lerobot_bridge", source)
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)
    return module


def test_bridge_forwards_deadman_and_stops_after_gripper_failure(bridge, monkeypatch):
    cfg = load_config("configs/mock.yaml")
    robot = bridge.UR5Robot(bridge.UR5RobotConfig(), cfg)
    calls = []
    robot.gripper_feedback = types.SimpleNamespace(
        update_command=lambda grasp, enabled: calls.append((grasp, enabled)),
        stop=lambda: calls.append("gripper_stop"))
    state = types.SimpleNamespace(pose=np.array(cfg.robot.mock_initial_pose))
    monkeypatch.setattr(robot, "get_state", lambda age: state)
    monkeypatch.setattr(robot, "send_target", lambda *args: calls.append("target"))
    monkeypatch.setattr(UR5Backend, "stop", lambda self: calls.append("robot_stop"))
    action = dict.fromkeys(bridge.ACTION_NAMES, 0.0)
    action.update(grasp=1.0, enabled=0.0)
    try:
        result = robot.send_action(action)
        assert calls == ["target", (1, False)]
        assert result["grasp"] == 1.0
        def fail(*args, **kwargs):
            raise RuntimeError("Gripper fault")
        robot.gripper_feedback.update_command = fail
        action["enabled"] = 1.0
        with pytest.raises(RuntimeError, match="Gripper fault"):
            robot.send_action(action)
        assert calls[-2:] == ["robot_stop", "gripper_stop"]
    finally:
        robot.connection.close()
        robot.child_connection.close()


def test_bridge_cleans_up_robot_if_startup_feedback_fails(bridge, monkeypatch):
    robot = bridge.UR5Robot(bridge.UR5RobotConfig(), load_config("configs/mock.yaml"))
    calls = []
    robot.gripper_feedback = types.SimpleNamespace(
        connect=lambda: calls.append("gripper_connect"),
        disconnect=lambda: calls.append("gripper_disconnect"))
    monkeypatch.setattr(UR5Backend, "connect", lambda self: calls.append("robot_connect"))
    monkeypatch.setattr(UR5Backend, "disconnect", lambda self: calls.append("robot_disconnect"))
    def fail(*args):
        raise RuntimeError("Startup feedback fault")
    monkeypatch.setattr(robot, "get_state", fail)
    try:
        with pytest.raises(RuntimeError, match="Startup feedback fault"):
            robot.connect()
        assert calls == ["gripper_connect", "robot_connect", "robot_disconnect", "gripper_disconnect"]
    finally:
        robot.connection.close()
        robot.child_connection.close()


def test_recorder_samples_before_command_and_stops_before_save(bridge, monkeypatch, tmp_path):
    from ur5_capture import record as recorder
    cfg = load_config("configs/mock.yaml")
    cfg.dataset.root = str(tmp_path / "capture")
    cfg.dataset.episodes = 1
    cfg.dataset.episode_seconds = 0.1
    cfg.synchronization.max_loop_lateness_s = 0.5
    frames, events = [], []
    original_update = bridge.UR5Robot.update_gripper_command
    original_stop = bridge.UR5Robot.stop
    def update(self, grasp, enabled=False):
        events.append("grasp")
        return original_update(self, grasp, enabled)
    def stop(self):
        events.append("stop")
        return original_stop(self)
    monkeypatch.setattr(bridge.UR5Robot, "update_gripper_command", update)
    monkeypatch.setattr(bridge.UR5Robot, "stop", stop)
    class Writer:
        def __init__(self, *args): self.count = 0
        def add(self, frame):
            events.append("frame")
            frames.append(frame)
            self.count += 1
        def save(self):
            assert events[-1] == "stop"
            self.count = 0
        def discard(self): self.count = 0
        def close(self): pass
    monkeypatch.setattr(recorder, "V3Writer", Writer)
    # Change grasp every frame so we can check observation-before-action semantics.
    def read(self):
        from ur5_capture.teleop import MouseSample
        self.toggle_grasp()
        return MouseSample(np.zeros(6), True, False, self.grasp)
    monkeypatch.setattr(bridge.SpaceMouseTeleoperator, "read", read)
    result = recorder.record(cfg, unattended=True)
    assert result["saved_episodes"] == 1
    assert len(frames) == 3
    assert events[:6] == ["grasp", "frame"] * 3
    np.testing.assert_allclose([f["observation.state"][6] for f in frames], [.04, 0, .04])
    assert [f["action"][6] for f in frames] == [1, 0, 1]
    assert all(f["action"].shape == (7,) for f in frames)
