"""8D absolute state must use feedback, never copy the 7D action's grasp command."""
from pathlib import Path
import sys
import types

import numpy as np
import pytest
import yaml

from ur5_capture.config import CaptureConfig
from ur5_capture.gripper import GripperFeedback

ROOT = Path(__file__).resolve().parents[1]


def test_mock_absolute_joint_positions_come_from_previous_command():
    cfg = CaptureConfig.model_validate(yaml.safe_load((ROOT / "configs/mock.yaml").read_text()))
    reader = GripperFeedback(cfg.gripper, mock=True)
    reader.connect()
    first = reader.read()
    np.testing.assert_allclose(first.qpos, [0.04, -0.04])
    reader.update_command(1)
    np.testing.assert_array_equal(first.qpos, [0.04, -0.04])  # immutable past observation
    np.testing.assert_allclose(reader.read().qpos, [0, 0])
    reader.update_command(0)
    np.testing.assert_allclose(reader.read().qpos, [0.04, -0.04])
    reader.disconnect()


def test_real_capture_rejects_mock_finger_feedback():
    cfg = yaml.safe_load((ROOT / "configs/ur5_three_rgb.yaml").read_text())
    cfg['gripper']['feedback_source'] = 'mock'
    cfg['gripper']['feedback_plugin'] = None
    cfg['gripper']['actuation_enabled'] = False
    with pytest.raises(ValueError, match='measured gripper feedback'):
        CaptureConfig.model_validate(cfg)


def test_real_feedback_plugin_required_before_connection():
    cfg = CaptureConfig.model_validate(yaml.safe_load((ROOT / "configs/ur5_three_rgb.yaml").read_text()))
    cfg.gripper.feedback_plugin = None
    with pytest.raises(RuntimeError, match='requires gripper.feedback_source=plugin'):
        GripperFeedback(cfg.gripper).connect()


def test_feedback_plugin_reads_measured_joint_positions(monkeypatch):
    class Sensor:
        def connect(self):
            self.connected = True
        def read_qpos(self):
            return [0.013, -0.014]
        def disconnect(self):
            self.connected = False
    module = types.ModuleType('fake_gripper_sensor')
    module.Sensor = Sensor
    monkeypatch.setitem(sys.modules, 'fake_gripper_sensor', module)
    cfg = CaptureConfig.model_validate(yaml.safe_load((ROOT / 'configs/ur5_three_rgb.yaml').read_text()))
    cfg.gripper.actuation_enabled = False
    cfg.gripper.feedback_plugin = 'fake_gripper_sensor:Sensor'
    cfg.gripper.feedback_kwargs = {}
    reader = GripperFeedback(cfg.gripper)
    reader.connect()
    np.testing.assert_allclose(reader.read().qpos, [.013, -.014])
    reader.update_command(1)
    np.testing.assert_allclose(reader.read().qpos, [.013, -.014])  # command != measured feedback
    reader.disconnect()


def test_invalid_feedback_rejected(monkeypatch):
    class Sensor:
        def connect(self): pass
        def read_qpos(self): return [float('nan'), 0]
        def disconnect(self): pass
    module = types.ModuleType('bad_gripper_sensor')
    module.Sensor = Sensor
    monkeypatch.setitem(sys.modules, 'bad_gripper_sensor', module)
    cfg = CaptureConfig.model_validate(yaml.safe_load((ROOT / 'configs/ur5_three_rgb.yaml').read_text()))
    cfg.gripper.actuation_enabled = False
    cfg.gripper.feedback_plugin = 'bad_gripper_sensor:Sensor'
    cfg.gripper.feedback_kwargs = {}
    reader = GripperFeedback(cfg.gripper)
    reader.connect()
    with pytest.raises(RuntimeError, match='Invalid gripper feedback'):
        reader.read()
    reader.disconnect()
