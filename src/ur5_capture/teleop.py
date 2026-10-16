"""SpaceMouse 2.0 object API; HID event timestamp prevents replay of stale input."""
import time
from dataclasses import dataclass

import numpy as np


@dataclass
class MouseSample:
    axes: np.ndarray
    enabled: bool
    stop: bool
    grasp: int  # commanded grasp: 0=open, 1=closed; never implied hardware feedback


class SpaceMouseInput:
    def __init__(self, cfg, mock=False, gripper_cfg=None):
        self.cfg, self.mock = cfg, mock
        self.grasp = int(gripper_cfg.initial_grasp) if gripper_cfg is not None else 0
        self.gripper_button = gripper_cfg.toggle_button if gripper_cfg is not None else None
        self.gripper_button_was_down = False
        self.device = None
        self.last_event = None
        self.last_event_host = 0.0
        self.step = 0

    def connect(self):
        if not self.mock:
            import pyspacemouse
            self.device = pyspacemouse.open(device_index=self.cfg.device_index, nonblocking=True)
            if self.device is None:
                raise RuntimeError("Could not open SpaceMouse")

    def toggle_grasp(self):
        """Latch a desired command; does not move or read any real gripper."""
        self.grasp = 1 - self.grasp
        return self.grasp

    def read(self):
        if self.mock:
            self.step += 1
            # Simulate changing commanded grasp, so dataset tests exercise both states.
            if self.step % 10 == 0:
                self.toggle_grasp()
            return MouseSample(np.array([0.4 * np.sin(self.step / 30), 0, 0, 0, 0, 0.15]),
                               True, False, self.grasp)
        # Each read consumes one HID report; drain queued reports before using the axes.
        # Snapshot t before the next read because the driver mutates a shared state object.
        state = self.device.read()
        stop_seen = False
        for _ in range(128):
            if state is None:
                break
            if self.cfg.stop_button < len(state.buttons):
                stop_seen |= bool(state.buttons[self.cfg.stop_button])
            if self.gripper_button is not None:
                if self.gripper_button >= len(state.buttons):
                    raise ValueError("Configured gripper button is unavailable on this SpaceMouse")
                down = bool(state.buttons[self.gripper_button])
                if down and not self.gripper_button_was_down:
                    self.toggle_grasp()
                self.gripper_button_was_down = down
            previous_t = state.t
            state = self.device.read()
            if state is None or state.t == previous_t:
                break
        else:
            raise RuntimeError("SpaceMouse input queue did not drain")
        if state is None:
            raise RuntimeError("SpaceMouse returned no state")
        now = time.monotonic()
        event_time = state.t
        if event_time != self.last_event:
            self.last_event, self.last_event_host = event_time, now
        buttons = state.buttons
        if max(self.cfg.deadman_button, self.cfg.stop_button) >= len(buttons):
            raise ValueError("Configured SpaceMouse button index is unavailable")
        stop = stop_seen or bool(buttons[self.cfg.stop_button])
        held = bool(buttons[self.cfg.deadman_button])
        axes = np.array([state.x, state.y, state.z, state.roll, state.pitch, state.yaw], dtype=float)
        if not np.isfinite(axes).all():
            raise RuntimeError("Non-finite SpaceMouse input")
        if now - self.last_event_host > self.cfg.stale_timeout_s:
            if held and np.any(np.abs(axes) > self.cfg.deadzone):
                raise RuntimeError("SpaceMouse motion report is stale")
            held = False
        return MouseSample(axes, held, stop, self.grasp)

    def disconnect(self):
        if self.device is not None:
            self.device.close()
