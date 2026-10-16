"""Official Robot/Teleoperator subclasses plus timing-aware extensions for our recorder.

The custom recorder uses get_state/read/send_target so timestamps and acknowledgments
are retained. Generic get_observation/get_action/send_action are also available.
Import this module to register the config types; it does not patch upstream factories.
"""
from dataclasses import dataclass

import numpy as np
from lerobot.robots.config import RobotConfig as HFRobotConfig
from lerobot.robots.robot import Robot
from lerobot.teleoperators.config import TeleoperatorConfig as HFTeleoperatorConfig
from lerobot.teleoperators.teleoperator import Teleoperator

from .config import load_config
from .dataset import ACTION_NAMES, DELTA_NAMES, POSE_NAMES, GRIPPER_QPOS_NAMES
from .geometry import check_target, compose, difference, limit_norm
from .gripper import GripperFeedback
from .robot import UR5Backend
from .teleop import SpaceMouseInput


@HFRobotConfig.register_subclass("ur5_rtde_eef")
@dataclass
class UR5RobotConfig(HFRobotConfig):
    capture_yaml: str = "configs/ur5_three_rgb.yaml"


class UR5Robot(UR5Backend, Robot):
    config_class = UR5RobotConfig
    name = "ur5_rtde_eef"

    def __init__(self, config: UR5RobotConfig, capture_config=None):
        Robot.__init__(self, config)
        self.capture_config = capture_config or load_config(config.capture_yaml)
        UR5Backend.__init__(self, self.capture_config.robot, self.capture_config.mode == "mock")
        self.gripper_feedback = GripperFeedback(self.capture_config.gripper, self.capture_config.mode == "mock")

    @property
    def observation_features(self):
        return {**{f"eef.{n}": float for n in POSE_NAMES},
                **{f"gripper.{n}": float for n in GRIPPER_QPOS_NAMES},
                **{f"joint.q{i}": float for i in range(6)}}

    @property
    def action_features(self):
        return {**dict.fromkeys(ACTION_NAMES, float), "enabled": float}

    @property
    def is_connected(self):
        return self.started and self.process.is_alive()

    @property
    def is_calibrated(self):
        return self.mock or self.cfg.tcp_configured

    def calibrate(self):
        if not self.is_calibrated:
            raise ValueError("Configure the measured TCP offset in YAML; no automatic UR calibration")

    def configure(self):
        if not self.is_calibrated:
            self.calibrate()

    def connect(self, calibrate=True):
        self.configure()
        # Validate physical feedback BEFORE starting UR motion control.
        self.gripper_feedback.connect()
        try:
            UR5Backend.connect(self)
            state = self.get_state(self.capture_config.synchronization.max_robot_age_s)
            check_target(state.pose, state.pose, self.cfg)
            self.read_gripper_qpos()
        except BaseException:
            self.gripper_feedback.disconnect()
            raise

    def read_gripper_qpos(self):
        return self.gripper_feedback.read()

    def update_gripper_command(self, grasp):
        self.gripper_feedback.update_command(grasp)

    def get_observation(self):
        state = self.get_state(self.capture_config.synchronization.max_robot_age_s)
        gripper = self.read_gripper_qpos()
        return {**{f"eef.{n}": float(v) for n, v in zip(POSE_NAMES, state.pose, strict=True)},
                **{f"gripper.{n}": float(v) for n, v in zip(GRIPPER_QPOS_NAMES, gripper.qpos, strict=True)},
                **{f"joint.q{i}": float(v) for i, v in enumerate(state.joints)}}

    def send_action(self, action):
        if set(action) != set(self.action_features):
            raise ValueError("send_action expects 6D TCP delta, grasp command, and deadman enabled")
        state = self.get_state(self.capture_config.synchronization.max_robot_age_s)
        delta = np.array([action[n] for n in DELTA_NAMES], dtype=float)
        grasp = float(action["grasp"])
        if not np.isfinite(grasp) or grasp not in (0.0, 1.0):
            raise ValueError("grasp must be 0 (open) or 1 (closed)")
        enabled = bool(action["enabled"])
        period = 1 / self.capture_config.dataset.fps
        if not enabled:
            delta[:] = 0
        delta[:3] = limit_norm(delta[:3], self.cfg.max_translation_speed * period)
        delta[3:] = limit_norm(delta[3:], self.cfg.max_rotation_speed * period)
        target = compose(state.pose, delta, self.capture_config.teleop.frame)
        target[:3] = np.clip(target[:3], self.cfg.workspace_min, self.cfg.workspace_max)
        self.send_target(target, period, enabled)
        self.update_gripper_command(int(grasp))
        actual_delta = difference(state.pose, target)
        # Returned action is always the accepted base-frame delta, matching dataset semantics.
        # Grasp is command-only until a physical gripper driver has been integrated.
        return {**{n: float(v) for n, v in zip(DELTA_NAMES, actual_delta, strict=True)},
                "grasp": grasp, "enabled": float(enabled)}

    def disconnect(self):
        try:
            UR5Backend.disconnect(self)
        finally:
            self.gripper_feedback.disconnect()


@HFTeleoperatorConfig.register_subclass("ur5_spacemouse")
@dataclass
class SpaceMouseTeleoperatorConfig(HFTeleoperatorConfig):
    capture_yaml: str = "configs/ur5_three_rgb.yaml"


class SpaceMouseTeleoperator(SpaceMouseInput, Teleoperator):
    config_class = SpaceMouseTeleoperatorConfig
    name = "ur5_spacemouse"

    def __init__(self, config: SpaceMouseTeleoperatorConfig, capture_config=None):
        Teleoperator.__init__(self, config)
        self.capture_config = capture_config or load_config(config.capture_yaml)
        SpaceMouseInput.__init__(self, self.capture_config.teleop, self.capture_config.mode == "mock",
                                 self.capture_config.gripper)
        self.connected = False

    @property
    def action_features(self):
        return {**dict.fromkeys(ACTION_NAMES, float), "enabled": float}

    @property
    def feedback_features(self):
        return {}

    @property
    def is_connected(self):
        return self.connected

    @property
    def is_calibrated(self):
        return True

    def calibrate(self):
        pass

    def configure(self):
        pass

    def connect(self, calibrate=True):
        SpaceMouseInput.connect(self)
        self.connected = True

    def get_action(self):
        sample = self.read()
        if sample.stop:
            raise RuntimeError("SpaceMouse stop button pressed")
        dz = self.cfg.deadzone
        axes = np.clip(np.sign(sample.axes) * np.maximum(np.abs(sample.axes) - dz, 0) / (1 - dz), -1, 1)
        if not sample.enabled:
            axes[:] = 0
        period = 1 / self.capture_config.dataset.fps
        translation = axes[list(self.cfg.translation_axes)] * self.cfg.translation_signs
        rotation = axes[list(self.cfg.rotation_axes)] * self.cfg.rotation_signs
        delta = np.r_[limit_norm(translation, 1) * self.capture_config.robot.max_translation_speed * period,
                      limit_norm(rotation, 1) * self.capture_config.robot.max_rotation_speed * period]
        return {**{n: float(v) for n, v in zip(DELTA_NAMES, delta, strict=True)},
                "grasp": float(sample.grasp), "enabled": float(sample.enabled)}

    def send_feedback(self, feedback):
        if feedback:
            raise ValueError("SpaceMouse has no feedback channel")

    def disconnect(self):
        SpaceMouseInput.disconnect(self)
        self.connected = False
