"""SpaceMouse 2.0 object API; HID event timestamp prevents replay of stale input."""
import time
from dataclasses import dataclass

import numpy as np


@dataclass
class MouseSample:
    axes: np.ndarray
    enabled: bool
    grasp: int  # commanded grasp: 0=open, 1=closed; never implied hardware feedback
    high_speed: bool = False


class SpaceMouseInput:
    def __init__(self, cfg, mock=False, gripper_cfg=None):
        self.cfg, self.mock = cfg, mock
        self.grasp = int(gripper_cfg.initial_grasp) if gripper_cfg is not None else 0
        self.gripper_button = gripper_cfg.toggle_button if gripper_cfg is not None else None
        self.gripper_button_was_down = False
        self.speed_button_was_down = False
        self.high_speed = False
        self.device = None
        self.last_event = None
        self.last_event_host = 0.0
        self.awaiting_fresh_input = False
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

    def start_episode(self):
        """Discard input from waiting/encoding; require a new HID event to move."""
        if self.mock:
            return
        # Synchronize held buttons without executing clicks made while paused.
        # Do not reset the cached report's age and then replay its nonzero axes.
        state = self._drain_state(apply_edges=False)
        self.last_event = state.t
        self.last_event_host = time.monotonic()
        self.awaiting_fresh_input = True

    def _buttons(self, state, *, apply_edges):
        if self.gripper_button is not None:
            if self.gripper_button >= len(state.buttons):
                raise ValueError("Configured gripper button is unavailable on this SpaceMouse")
            down = bool(state.buttons[self.gripper_button])
            if apply_edges and down and not self.gripper_button_was_down:
                self.toggle_grasp()
            self.gripper_button_was_down = down
        if self.cfg.speed_toggle_button >= len(state.buttons):
            raise ValueError("Configured speed button is unavailable on this SpaceMouse")
        speed_down = bool(state.buttons[self.cfg.speed_toggle_button])
        if apply_edges and speed_down and not self.speed_button_was_down:
            self.high_speed = not self.high_speed
        self.speed_button_was_down = speed_down

    def _drain_state(self, *, apply_edges=True):
        # The driver mutates and returns the SAME state object. Snapshot its
        # timestamp before every read so only genuinely new reports are drained.
        state = self.device.read()
        for _ in range(128):
            if state is None:
                break
            self._buttons(state, apply_edges=apply_edges)
            previous_t = state.t
            state = self.device.read()
            if state is None or state.t == previous_t:
                break
        else:
            raise RuntimeError("SpaceMouse input queue did not drain")
        if state is None:
            raise RuntimeError("SpaceMouse returned no state")
        return state

    def read(self):
        if self.mock:
            self.step += 1
            # Simulate changing commanded grasp, so dataset tests exercise both states.
            if self.step % 10 == 0:
                self.toggle_grasp()
            return MouseSample(np.array([0.4 * np.sin(self.step / 30), 0, 0, 0, 0, 0.15]),
                               True, self.grasp, self.high_speed)
        state = self._drain_state()
        now = time.monotonic()
        event_time = state.t
        if self.awaiting_fresh_input:
            if event_time == self.last_event:
                return MouseSample(np.zeros(6), False, self.grasp, self.high_speed)
            self.awaiting_fresh_input = False
        if event_time != self.last_event:
            self.last_event, self.last_event_host = event_time, now
        axes = np.array([state.x, state.y, state.z, state.roll, state.pitch, state.yaw], dtype=float)
        if not np.isfinite(axes).all():
            raise RuntimeError("Non-finite SpaceMouse input")
        if now - self.last_event_host > self.cfg.stale_timeout_s:
            if np.any(np.abs(axes) > self.cfg.deadzone):
                raise RuntimeError("SpaceMouse motion report is stale")
        return MouseSample(axes, True, self.grasp, self.high_speed)

    def disconnect(self):
        if self.device is not None:
            self.device.close()
