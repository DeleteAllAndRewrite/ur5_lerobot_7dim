"""Independent gripper feedback source for absolute 2-joint observation.

A live gripper needs a hardware-specific reader; SpaceMouse grasp commands must
never be substituted for measurements. The mock backend is explicitly synthetic.
"""
from dataclasses import dataclass
import importlib
import time

import numpy as np


@dataclass(frozen=True)
class GripperSample:
    qpos: np.ndarray  # [left_finger_joint_m, right_finger_joint_m] in configured signed convention
    host_time: float


class GripperFeedback:
    def __init__(self, cfg, mock=False):
        self.cfg = cfg
        self.mock = mock
        self.driver = None
        self.connected = False
        self.command = int(cfg.initial_grasp)

    def connect(self):
        if self.connected:
            raise RuntimeError("Gripper feedback already connected")
        if self.mock:
            if self.cfg.feedback_source != "mock":
                raise ValueError("Mock gripper requires feedback_source=mock")
        else:
            if self.cfg.feedback_source != "plugin" or not self.cfg.feedback_plugin:
                raise RuntimeError(
                    "8D real observation requires gripper.feedback_source=plugin and "
                    "gripper.feedback_plugin=module:Class returning measured two-finger qpos; "
                    "the grasp command cannot be used as feedback"
                )
            module_name, cls_name = self.cfg.feedback_plugin.split(":", 1)
            driver_type = getattr(importlib.import_module(module_name), cls_name)
            self.driver = driver_type(**self.cfg.feedback_kwargs)
            # The adapter intentionally only reads measurements; it never sends
            # a grasp command to an unknown physical gripper.
            try:
                self.driver.connect()
            except BaseException:
                try:
                    self.driver.disconnect()
                finally:
                    self.driver = None
                raise
        self.connected = True

    def read(self):
        if not self.connected:
            raise RuntimeError("Gripper feedback not connected")
        if self.mock:
            half_width = self.cfg.mock_open_width_m / 2 if self.command == 0 else 0.0
            qpos = np.array([half_width, -half_width], dtype=np.float64)
        else:
            qpos = np.asarray(self.driver.read_qpos(), dtype=np.float64)
        if qpos.shape != (2,) or not np.isfinite(qpos).all():
            raise RuntimeError(f"Invalid gripper feedback: expected finite (2,), received {qpos}")
        return GripperSample(qpos=qpos.copy(), host_time=time.monotonic())

    def update_command(self, grasp):
        if grasp not in (0, 1):
            raise ValueError("Grasp command must be 0 or 1")
        # Mock simulates an instantaneous response for the NEXT observation.
        # For physical robots this only updates the local command label. Actual
        # gripper actuation must be implemented by an independently validated driver.
        self.command = int(grasp)

    def disconnect(self):
        try:
            if self.driver is not None:
                self.driver.disconnect()
        finally:
            self.driver = None
            self.connected = False
