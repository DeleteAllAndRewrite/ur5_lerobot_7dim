"""All stored poses: base-frame xyz (m), rotation vector (rad), never Euler angles."""
import numpy as np
from scipy.spatial.transform import Rotation


def vector(value, n=6):
    value = np.asarray(value, dtype=np.float64)
    if value.shape != (n,) or not np.isfinite(value).all():
        raise ValueError(f"Expected finite vector of shape ({n},)")
    return value


def limit_norm(v, maximum):
    v = np.asarray(v, dtype=np.float64)
    return v * min(1.0, maximum / max(np.linalg.norm(v), 1e-12))


def compose(pose, delta, frame="base"):
    pose, delta = vector(pose), vector(delta)
    r = Rotation.from_rotvec(pose[3:])
    dr = Rotation.from_rotvec(delta[3:])
    if frame == "base":
        return np.r_[pose[:3] + delta[:3], (dr * r).as_rotvec()]
    if frame == "tool":
        return np.r_[pose[:3] + r.apply(delta[:3]), (r * dr).as_rotvec()]
    raise ValueError(frame)


def difference(start, end):
    start, end = vector(start), vector(end)
    return np.r_[end[:3] - start[:3],
                 (Rotation.from_rotvec(end[3:]) * Rotation.from_rotvec(start[3:]).inv()).as_rotvec()]


def interpolate(start, end, fraction):
    return compose(start, difference(start, end) * np.clip(fraction, 0, 1))


def target_from_axes(pose, axes, cfg, enabled):
    axes = vector(axes)
    dz = cfg.teleop.deadzone
    axes = np.sign(axes) * np.maximum(np.abs(axes) - dz, 0) / (1 - dz)
    axes = np.clip(axes, -1, 1) if enabled else np.zeros(6)
    dt = 1 / cfg.dataset.fps
    linear = axes[list(cfg.teleop.translation_axes)] * cfg.teleop.translation_signs
    angular = axes[list(cfg.teleop.rotation_axes)] * cfg.teleop.rotation_signs
    delta = np.r_[limit_norm(linear, 1) * cfg.robot.max_translation_speed * dt,
                  limit_norm(angular, 1) * cfg.robot.max_rotation_speed * dt]
    target = compose(pose, delta, cfg.teleop.frame)
    target[:3] = np.clip(target[:3], cfg.robot.workspace_min, cfg.robot.workspace_max)
    return target, difference(pose, target)


def check_target(actual, target, cfg):
    actual, target = vector(actual), vector(target)
    if np.any(target[:3] < cfg.workspace_min) or np.any(target[:3] > cfg.workspace_max):
        raise RuntimeError("TCP outside configured workspace")
    error = difference(actual, target)
    if np.linalg.norm(error[:3]) > cfg.max_tracking_translation:
        raise RuntimeError("TCP translation tracking limit exceeded")
    if np.linalg.norm(error[3:]) > cfg.max_tracking_rotation:
        raise RuntimeError("TCP rotation tracking limit exceeded")
