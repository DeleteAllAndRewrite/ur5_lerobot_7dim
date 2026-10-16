from pathlib import Path

import numpy as np
import yaml

from ur5_capture.config import CaptureConfig
from ur5_capture.dataset import ACTION_NAMES
from ur5_capture.lerobot_bridge import (
    SpaceMouseTeleoperator, SpaceMouseTeleoperatorConfig,
    UR5Robot, UR5RobotConfig,
)


def test_official_robot_and_teleoperator_contracts():
    from lerobot.robots.robot import Robot
    from lerobot.teleoperators.teleoperator import Teleoperator
    cfg = CaptureConfig.model_validate(yaml.safe_load(
        (Path(__file__).resolve().parents[1] / 'configs/mock.yaml').read_text()))
    robot = UR5Robot(UR5RobotConfig(id='test'), cfg)
    teleop = SpaceMouseTeleoperator(SpaceMouseTeleoperatorConfig(id='test'), cfg)
    try:
        assert isinstance(robot, Robot)
        assert isinstance(teleop, Teleoperator)
        robot.connect()
        teleop.connect()
        obs = robot.get_observation()
        assert obs['gripper.left_finger_qpos_m'] == 0.04
        assert obs['gripper.right_finger_qpos_m'] == -0.04
        action = teleop.get_action()
        assert set(obs) == set(robot.observation_features)
        assert set(action) == set(robot.action_features) == set(teleop.action_features)
        accepted = robot.send_action(action)
        assert np.isfinite([accepted[k] for k in ACTION_NAMES]).all()
        assert len(ACTION_NAMES) == 7
        assert accepted['grasp'] in (0.0, 1.0)
        action['grasp'] = 1.0
        assert robot.send_action(action)['grasp'] == 1.0
        assert robot.get_observation()['gripper.left_finger_qpos_m'] == 0.0
        robot.stop()
    finally:
        teleop.disconnect()
        robot.disconnect()
