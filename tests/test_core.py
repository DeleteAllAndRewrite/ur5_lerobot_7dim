import copy
import threading
from pathlib import Path
import time

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
import yaml

from ur5_capture.cameras import CameraFrame, select_frames
from ur5_capture.config import CaptureConfig, SyncConfig
from ur5_capture.geometry import check_target, compose, difference, target_from_axes
from ur5_capture.robot import UR5Backend
from ur5_capture.teleop import SpaceMouseInput
from ur5_capture.dataset import ACTION_NAMES, STATE_NAMES, build_frame, make_features
from ur5_capture.gripper import GripperSample

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def config():
    return CaptureConfig.model_validate(yaml.safe_load((ROOT / "configs/mock.yaml").read_text()))


def test_rotation_delta_across_pi():
    a = np.array([0, 0, 0, 0, 0, np.deg2rad(179)])
    b = np.array([0, 0, 0, 0, 0, np.deg2rad(-179)])
    delta = difference(a, b)
    assert np.isclose(np.linalg.norm(delta[3:]), np.deg2rad(2))
    reconstructed = compose(a, delta)
    np.testing.assert_allclose(Rotation.from_rotvec(reconstructed[3:]).as_matrix(),
                               Rotation.from_rotvec(b[3:]).as_matrix(), atol=1e-10)


def test_tool_translation_and_rotation_are_composed():
    pose = np.array([0, 0, 0, 0, 0, np.pi / 2])
    target = compose(pose, np.array([0.01, 0, 0, 0.03, 0, 0]), "tool")
    np.testing.assert_allclose(target[:3], [0, 0.01, 0], atol=1e-10)
    np.testing.assert_allclose(difference(pose, target)[3:], [0, 0.03, 0], atol=1e-10)


def test_disabled_motion_and_workspace_clipping(config):
    pose = np.array(config.robot.mock_initial_pose)
    target, delta = target_from_axes(pose, np.ones(6), config, False)
    np.testing.assert_allclose(delta, 0, atol=1e-10)
    pose[0] = config.robot.workspace_max[0]
    target, delta = target_from_axes(pose, np.array([1, 0, 0, 0, 0, 0]), config, True)
    assert target[0] == pose[0]
    assert delta[0] == 0


def test_vector_speed_limit(config):
    _, delta = target_from_axes(config.robot.mock_initial_pose, np.ones(6), config, True)
    assert np.linalg.norm(delta[:3]) <= config.robot.max_translation_speed / config.dataset.fps + 1e-10
    assert np.linalg.norm(delta[3:]) <= config.robot.max_rotation_speed / config.dataset.fps + 1e-10


def test_high_speed_profile_uses_configured_limits(config):
    pose = np.array(config.robot.mock_initial_pose)
    _, low = target_from_axes(pose, np.ones(6), config, True)
    _, high = target_from_axes(pose, np.ones(6), config, True, high_speed=True)
    assert np.isclose(np.linalg.norm(low[:3]), config.robot.max_translation_speed / config.dataset.fps)
    assert np.isclose(np.linalg.norm(high[:3]), config.robot.high_translation_speed / config.dataset.fps)
    assert np.isclose(np.linalg.norm(low[3:]), config.robot.max_rotation_speed / config.dataset.fps)
    assert np.isclose(np.linalg.norm(high[3:]), config.robot.high_rotation_speed / config.dataset.fps)


@pytest.mark.parametrize("mutation", [
    lambda c: c.update(typo=1),
    lambda c: c['cameras'][1].update(serial=c['cameras'][0]['serial']),
    lambda c: c['cameras'][2].update(role='global'),
    lambda c: c['robot'].update(control_hz=500),
    lambda c: c['robot'].update(high_translation_speed=0.41),
    lambda c: c['robot'].update(high_translation_speed=0.04),
    lambda c: c['teleop'].update(speed_toggle_button=-1),
    lambda c: c['teleop'].update(rotation_axes=[0, 1, 2]),
    lambda c: c['robot'].update(max_translation_speed=float('nan')),
    lambda c: c['cameras'][0].update(fps=15),
])
def test_invalid_configs_rejected(config, mutation):
    data = copy.deepcopy(config.model_dump())
    mutation(data)
    with pytest.raises(ValueError):
        CaptureConfig.model_validate(data)


def test_real_motion_requires_explicit_tcp_and_workspace(config):
    data = config.model_dump()
    data['mode'] = 'real'
    data['robot']['motion_enabled'] = True
    with pytest.raises(ValueError):
        CaptureConfig.model_validate(data)


def frame(t, i):
    return CameraFrame(np.zeros((2, 2, 3), np.uint8), t, t * 1000, i)


def test_synchronization_rejects_duplicate_stale_and_skewed():
    sync = SyncConfig()
    buffers = {'a': [frame(1.00, 1)], 'b': [frame(1.01, 1)], 'c': [frame(1.02, 1)]}
    selected = select_frames(buffers, {}, 1.03, sync)
    assert len(selected) == 3
    with pytest.raises(RuntimeError):
        select_frames(buffers, {'a': 1}, 1.03, sync)
    with pytest.raises(RuntimeError):
        select_frames(buffers, {}, 1.5, sync)
    buffers['c'] = [frame(1.07, 2)]
    with pytest.raises(RuntimeError):
        select_frames(buffers, {}, 1.08, sync)


def test_tracking_guard_rejects_large_target(config):
    pose = np.array(config.robot.mock_initial_pose)
    target = pose.copy()
    target[0] += 0.1
    with pytest.raises(RuntimeError):
        check_target(pose, target, config.robot)


def test_mock_servo_ack_stop_and_command_timeout(config):
    robot = UR5Backend(config.robot, mock=True)
    try:
        robot.connect()
        before = robot.get_state(max_age=0.2).pose
        target = before.copy()
        target[0] += 0.001
        robot.send_target(target, 0.03, True)
        time.sleep(0.06)
        np.testing.assert_allclose(robot.get_state(max_age=0.2).pose[:3], target[:3], atol=1e-5)
        robot.stop()
        time.sleep(config.robot.command_timeout_s + 0.02)
        robot.check_health()  # Idle periods do not falsely trigger the active-command watchdog.
        robot.send_target(target, 0.03, True)
        time.sleep(config.robot.command_timeout_s + 0.07)
        with pytest.raises(RuntimeError):
            robot.check_health()
    finally:
        robot.disconnect()


def test_action_schema_has_seven_dims_and_grasp_command(config):
    assert ACTION_NAMES == ['dx_m', 'dy_m', 'dz_m', 'drx_rad', 'dry_rad', 'drz_rad', 'grasp']
    assert make_features(config)['action']['shape'] == (7,)
    assert make_features(config)['observation.state']['shape'] == (8,)
    assert STATE_NAMES[-2:] == ['left_finger_qpos_m', 'right_finger_qpos_m']
    robot_state = type('State', (), dict(pose=np.zeros(6), joints=np.zeros(6),
                                         host_time=1., device_time=2.))()
    images = {c.name: type('Frame', (), dict(rgb=np.zeros((c.height, c.width, 3), np.uint8),
                                             host_time=1., device_time_ms=2., frame_number=3))()
              for c in config.cameras}
    feedback = GripperSample(np.array([0.02, -0.02]), 0.5)
    frame = build_frame(config, robot_state, images, np.zeros(6), np.ones(6), 1, True, 1., feedback)
    np.testing.assert_array_equal(frame['action'], [1, 1, 1, 1, 1, 1, 1])
    np.testing.assert_allclose(frame['observation.state'][6:], [.02, -.02])
    assert frame['observation.state'].shape == (8,)
    assert frame['action'].dtype == np.float32
    assert frame['complementary_info.gripper_host_time'].item() == 0.5
    with pytest.raises(ValueError):
        build_frame(config, robot_state, images, np.zeros(6), np.ones(6), 2, True, 1., feedback)
    with pytest.raises(ValueError):
        build_frame(config, robot_state, images, np.zeros(6), np.ones(7), 1, True, 1., feedback)
    with pytest.raises(ValueError):
        build_frame(config, robot_state, images, np.zeros(6), np.ones(6), 1, True, 1.,
                    GripperSample(np.array([0.02]), 0.5))


def test_keyboard_toggle_and_mock_grasp_state(config):
    mouse = SpaceMouseInput(config.teleop, mock=True, gripper_cfg=config.gripper)
    first = mouse.read()
    assert first.grasp == 0
    assert first.axes.shape == (6,)
    assert mouse.toggle_grasp() == 1
    assert mouse.read().grasp == 1
    assert mouse.toggle_grasp() == 0
    for _ in range(7):
        mouse.read()
    assert mouse.read().grasp == 1  # mock automatically toggles every tenth sample


def test_gripper_and_speed_buttons_must_differ(config):
    data = config.model_dump()
    data['gripper']['toggle_button'] = data['teleop']['speed_toggle_button']
    with pytest.raises(ValueError):
        CaptureConfig.model_validate(data)


def test_repeated_grasp_button_reports_only_toggle_once(config):
    class HIDState:
        def __init__(self, seq, grasp_pressed):
            self.t = float(seq)
            self.buttons = [False, False, grasp_pressed]
            self.x = self.y = self.z = self.roll = self.pitch = self.yaw = 0.0

    class FakeDevice:
        def __init__(self):
            self.events = []
            self.latest = HIDState(0, False)

        def read(self):
            if self.events:
                self.latest = self.events.pop(0)
            return self.latest

    config.gripper.toggle_button = 2
    mouse = SpaceMouseInput(config.teleop, gripper_cfg=config.gripper)
    mouse.device = FakeDevice()
    mouse.device.events = [HIDState(1, False), HIDState(2, True)]
    assert mouse.read().grasp == 1
    assert mouse.read().grasp == 1  # no repeated toggles on held key
    mouse.device.events = [HIDState(3, False), HIDState(4, True)]
    assert mouse.read().grasp == 0  # rising edge again


def test_button_zero_toggles_gripper_and_button_one_toggles_speed(config):
    class HIDState:
        def __init__(self, seq, button0=False, button1=False):
            self.t = float(seq)
            self.buttons = [button0, button1]
            self.x = self.y = self.z = self.roll = self.pitch = self.yaw = 0.0

    class FakeDevice:
        def __init__(self):
            self.events = []
            self.latest = HIDState(0)

        def read(self):
            if self.events:
                self.latest = self.events.pop(0)
            return self.latest

    config.gripper.toggle_button = 0
    config.teleop.speed_toggle_button = 1
    mouse = SpaceMouseInput(config.teleop, gripper_cfg=config.gripper)
    mouse.device = FakeDevice()

    mouse.device.events = [HIDState(1, button0=True)]
    sample = mouse.read()
    assert sample.grasp == 1 and sample.enabled and not sample.high_speed
    mouse.device.events = [HIDState(2, button0=True)]
    assert mouse.read().grasp == 1  # held key does not retrigger

    mouse.device.events = [HIDState(3), HIDState(4, button1=True)]
    sample = mouse.read()
    assert sample.high_speed
    mouse.device.events = [HIDState(5, button1=True)]
    assert mouse.read().high_speed  # held key does not toggle back
    mouse.device.events = [HIDState(6), HIDState(7, button1=True)]
    assert not mouse.read().high_speed


def test_mock_stale_robot_sample_waits_for_a_fresh_update(config, monkeypatch):
    """A starved mock worker must recover without returning a stale pose."""
    robot = UR5Backend(config.robot, mock=True)
    monkeypatch.setattr(robot, "check_health", lambda: None)
    with robot.shared.get_lock():
        robot.shared[0] = time.monotonic() - 1.0

    def publish_new_state():
        time.sleep(0.06)
        with robot.shared.get_lock():
            robot.shared[0] = time.monotonic()
            robot.shared[2:8] = list(config.robot.mock_initial_pose)

    update = threading.Thread(target=publish_new_state)
    update.start()
    try:
        started = time.monotonic()
        result = robot.get_state(max_age=0.05)
        assert time.monotonic() - started >= 0.04
        assert time.monotonic() - result.host_time <= 0.05
        np.testing.assert_allclose(result.pose, config.robot.mock_initial_pose)
    finally:
        update.join(timeout=1)
        robot.connection.close()
        robot.child_connection.close()


def test_real_stale_robot_sample_fails_immediately(config, monkeypatch):
    robot = UR5Backend(config.robot, mock=False)
    monkeypatch.setattr(robot, "check_health", lambda: None)
    with robot.shared.get_lock():
        robot.shared[0] = time.monotonic() - 1.0
    try:
        started = time.monotonic()
        with pytest.raises(RuntimeError, match="Robot state is stale"):
            robot.get_state(max_age=0.05)
        assert time.monotonic() - started < 0.1
    finally:
        robot.connection.close()
        robot.child_connection.close()
