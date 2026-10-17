"""Validated YAML schema; unknown keys and non-finite values are errors."""
from pathlib import Path
from typing import Literal

import numpy as np
import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator


class StrictModel(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)


class DatasetConfig(StrictModel):
    repo_id: str = "local/ur5_three_rgb"
    root: str = "datasets/ur5_three_rgb"
    fps: int = Field(30, ge=1, le=60)
    task: str = Field(min_length=1)
    episodes: int = Field(10, ge=1)
    episode_seconds: float = Field(30, gt=0)
    image_writer_threads: int = Field(6, ge=0, le=32)
    vcodec: Literal["h264", "hevc", "libsvtav1"] = "h264"


class RobotConfig(StrictModel):
    ip: str = "192.168.1.10"
    controller: Literal["cb3", "e_series"] = "cb3"
    motion_backend: Literal["ur_rtde"] = "ur_rtde"
    control_hz: int = Field(125, ge=30, le=500)
    # Local port used by the URCap External Control server. Must not collide
    # with ur_robot_driver (50002 by default).
    ur_cap_port: int = Field(50005, ge=1024, le=65535)
    motion_enabled: bool = False
    tcp_offset: tuple[float, float, float, float, float, float] = (0, 0, 0, 0, 0, 0)
    tcp_configured: bool = False
    lookahead_time: float = Field(0.1, ge=0.03, le=0.2)
    gain: int = Field(300, ge=100, le=2000)
    max_translation_speed: float = Field(0.05, gt=0, le=0.25)
    max_rotation_speed: float = Field(0.3, gt=0, le=1)
    high_translation_speed: float = Field(0.15, gt=0, le=0.4)
    high_rotation_speed: float = Field(0.8, gt=0, le=1)
    max_tracking_translation: float = Field(0.03, gt=0)
    max_tracking_rotation: float = Field(0.2, gt=0)
    command_timeout_s: float = Field(0.15, gt=0, le=0.5)
    watchdog_hz: float = Field(10, ge=2, le=50)
    workspace_min: tuple[float, float, float] = (-0.6, -0.6, 0.1)
    workspace_max: tuple[float, float, float] = (0.6, 0.6, 0.8)
    workspace_configured: bool = False
    mock_initial_pose: tuple[float, float, float, float, float, float] = (0.3, 0, 0.3, 0, 0, 0)


class TeleopConfig(StrictModel):
    device_index: int = Field(0, ge=0)
    deadzone: float = Field(0.12, ge=0, lt=1)
    translation_axes: tuple[int, int, int] = (0, 1, 2)
    rotation_axes: tuple[int, int, int] = (3, 4, 5)
    translation_signs: tuple[Literal[-1, 1], Literal[-1, 1], Literal[-1, 1]] = (1, 1, 1)
    rotation_signs: tuple[Literal[-1, 1], Literal[-1, 1], Literal[-1, 1]] = (1, 1, 1)
    frame: Literal["base", "tool"] = "base"
    speed_toggle_button: int = Field(1, ge=0)
    stale_timeout_s: float = Field(0.15, gt=0, le=0.5)


class GripperConfig(StrictModel):
    # 7D action is commanded grasp; 8D observation requires a separate gripper-position source.
    initial_grasp: Literal[0, 1] = 0  # 0=open, 1=closed
    toggle_button: int | None = Field(default=None, ge=0)
    feedback_source: Literal["mock", "plugin"] = "mock"
    feedback_plugin: str | None = None  # module:Class implementing the physical gripper feedback API
    feedback_kwargs: dict = Field(default_factory=dict)
    actuation_enabled: bool = False  # explicitly opt in to a plugin's set_grasp()/stop() API
    mock_open_width_m: float = Field(0.08, gt=0, le=0.2)

    @model_validator(mode="after")
    def validate_feedback(self):
        if self.feedback_source == "plugin" and self.feedback_plugin is not None:
            if self.feedback_plugin.count(":") != 1 or not all(self.feedback_plugin.split(":")):
                raise ValueError("feedback_plugin must be 'module:Class'")
        if self.feedback_source == "mock" and self.feedback_plugin is not None:
            raise ValueError("feedback_plugin is only used with feedback_source=plugin")
        if self.actuation_enabled and (self.feedback_source != "plugin" or not self.feedback_plugin):
            raise ValueError("Gripper actuation requires a feedback plugin with set_grasp() and stop()")
        if self.feedback_plugin in ("ur5_capture.rg2:RG2Gripper", "ur5_capture.rg2:RTDEGripper"):
            from .rg2 import RG2Config, RTDEConfig
            (RG2Config if self.feedback_plugin.endswith(":RG2Gripper") else RTDEConfig).model_validate(
                self.feedback_kwargs
            )
        return self


class CameraConfig(StrictModel):
    name: str = Field(pattern=r"^[A-Za-z][A-Za-z0-9_]*$")
    role: Literal["global", "eye_in_hand"]
    serial: str
    width: int = Field(640, gt=0)
    height: int = Field(480, gt=0)
    fps: int = Field(30, ge=1, le=90)


class SyncConfig(StrictModel):
    max_camera_skew_s: float = Field(0.04, gt=0)
    max_camera_age_s: float = Field(0.10, gt=0)
    max_robot_age_s: float = Field(0.05, gt=0)
    max_loop_lateness_s: float = Field(0.05, gt=0)
    startup_timeout_s: float = Field(10, gt=0)


class CaptureConfig(StrictModel):
    mode: Literal["real", "mock"] = "real"
    dataset: DatasetConfig
    robot: RobotConfig = Field(default_factory=RobotConfig)
    teleop: TeleopConfig = Field(default_factory=TeleopConfig)
    gripper: GripperConfig = Field(default_factory=GripperConfig)
    cameras: list[CameraConfig]
    synchronization: SyncConfig = Field(default_factory=SyncConfig)

    @model_validator(mode="after")
    def cross_validate(self):
        if len(self.cameras) != 3 or sorted(c.role for c in self.cameras) != ["eye_in_hand", "global", "global"]:
            raise ValueError("This rig requires exactly 2 global RGB cameras and 1 eye_in_hand RGB camera")
        for attr in ("name", "serial"):
            if len({getattr(c, attr) for c in self.cameras}) != 3:
                raise ValueError(f"Camera {attr} values must be unique")
        if any(c.fps < self.dataset.fps for c in self.cameras):
            raise ValueError("Camera FPS must be >= dataset FPS")
        if any(c.width % 2 or c.height % 2 for c in self.cameras):
            raise ValueError("Video width and height must be even for yuv420p")
        r, t = self.robot, self.teleop
        if r.controller == "cb3" and r.control_hz > 125:
            raise ValueError("UR5 CB3 control_hz must be <=125")
        if r.control_hz < self.dataset.fps:
            raise ValueError("control_hz must be >= dataset FPS")
        if r.watchdog_hz >= r.control_hz:
            raise ValueError("watchdog_hz must be below control_hz")
        if r.command_timeout_s <= 1 / self.dataset.fps:
            raise ValueError("command timeout must exceed one recording period")
        if np.any(np.array(r.workspace_min) >= r.workspace_max):
            raise ValueError("workspace_min must be below workspace_max")
        if r.high_translation_speed < r.max_translation_speed:
            raise ValueError("high_translation_speed must be >= max_translation_speed")
        if r.high_rotation_speed < r.max_rotation_speed:
            raise ValueError("high_rotation_speed must be >= max_rotation_speed")
        if sorted(t.translation_axes + t.rotation_axes) != list(range(6)):
            raise ValueError("Axis mapping must be a permutation of 0..5")
        if self.gripper.toggle_button == t.speed_toggle_button:
            raise ValueError("Gripper and speed toggle buttons must differ")
        if self.mode == "real" and self.gripper.feedback_source != "plugin":
            raise ValueError("Real capture must use measured gripper feedback (feedback_source: plugin)")
        if self.mode == "mock" and self.gripper.feedback_source != "mock":
            raise ValueError("Mock capture must use simulated gripper feedback (feedback_source: mock)")
        if self.mode == "real" and r.motion_enabled and not (r.workspace_configured and r.tcp_configured):
            raise ValueError("Motion requires verified tcp_configured and workspace_configured")
        if round(self.dataset.episode_seconds * self.dataset.fps) < 2:
            raise ValueError("Episode must contain at least two frames")
        return self


def load_config(path: str | Path, project_root: Path | None = None) -> CaptureConfig:
    path = Path(path).resolve()
    cfg = CaptureConfig.model_validate(yaml.safe_load(path.read_text(encoding="utf-8")))
    base = project_root or Path(__file__).resolve().parents[2]
    root = Path(cfg.dataset.root).expanduser()
    cfg.dataset.root = str((base / root).resolve() if not root.is_absolute() else root.resolve())
    return cfg
