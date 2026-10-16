"""Use the official writer; do not manually fabricate v3 offsets or statistics."""
import importlib
import json
import subprocess
from pathlib import Path

import numpy as np

POSE_NAMES = ["x_m", "y_m", "z_m", "rx_rad", "ry_rad", "rz_rad"]
GRIPPER_QPOS_NAMES = ["left_finger_qpos_m", "right_finger_qpos_m"]
STATE_NAMES = [*POSE_NAMES, *GRIPPER_QPOS_NAMES]  # 8D absolute pose and measured/simulated finger positions
DELTA_NAMES = ["dx_m", "dy_m", "dz_m", "drx_rad", "dry_rad", "drz_rad"]
ACTION_NAMES = [*DELTA_NAMES, "grasp"]  # 7D: 6D TCP delta + latched binary grasp command


def numeric(names, dtype="float32"):
    return {"dtype": dtype, "shape": (len(names),), "names": names}


def make_features(cfg):
    names = [c.name for c in cfg.cameras]
    features = {
        "observation.state": numeric(STATE_NAMES),
        "action": numeric(ACTION_NAMES),
        "observation.joint_positions": numeric([f"q{i}_rad" for i in range(6)]),
        "complementary_info.target_pose": numeric(POSE_NAMES),
        "complementary_info.host_time": numeric(["monotonic_s"], "float64"),
        "complementary_info.robot_host_time": numeric(["monotonic_s"], "float64"),
        "complementary_info.robot_timestamp": numeric(["controller_s"], "float64"),
        "complementary_info.gripper_host_time": numeric(["monotonic_s"], "float64"),
        "complementary_info.camera_host_times": numeric(names, "float64"),
        "complementary_info.camera_device_times_ms": numeric(names, "float64"),
        "complementary_info.camera_frame_numbers": numeric(names, "int64"),
        "complementary_info.deadman": numeric(["enabled"], "int64"),
        "complementary_info.command_accepted_time": numeric(["monotonic_s"], "float64"),
    }
    for c in cfg.cameras:
        features[f"observation.images.{c.name}"] = {
            "dtype": "video", "shape": (c.height, c.width, 3), "names": ["height", "width", "channels"]}
    return features


def build_frame(cfg, state, images, target, delta, grasp, enabled, accepted_time, gripper_sample):
    frames = [images[c.name] for c in cfg.cameras]
    delta = np.asarray(delta, dtype=np.float64)
    if delta.shape != (6,) or not np.isfinite(delta).all():
        raise ValueError("Action TCP delta must be finite and have exactly six components")
    if grasp not in (0, 1):
        raise ValueError("Grasp command must be 0 (open) or 1 (closed)")
    if np.asarray(state.pose).shape != (6,) or not np.isfinite(state.pose).all():
        raise ValueError("EEF absolute pose must contain six finite values")
    qpos = np.asarray(gripper_sample.qpos, dtype=np.float64)
    if qpos.shape != (2,) or not np.isfinite(qpos).all():
        raise ValueError("Gripper feedback must contain two finite absolute joint positions")
    action = np.r_[delta, float(grasp)].astype(np.float32)
    frame = {
        "observation.state": np.r_[state.pose, qpos].astype(np.float32),
        "action": action,
        "observation.joint_positions": state.joints.astype(np.float32),
        "complementary_info.target_pose": target.astype(np.float32),
        "complementary_info.host_time": np.array([__import__("time").monotonic()], dtype=np.float64),
        "complementary_info.robot_host_time": np.array([state.host_time], dtype=np.float64),
        "complementary_info.robot_timestamp": np.array([state.device_time], dtype=np.float64),
        "complementary_info.gripper_host_time": np.array([gripper_sample.host_time], dtype=np.float64),
        "complementary_info.camera_host_times": np.array([f.host_time for f in frames], dtype=np.float64),
        "complementary_info.camera_device_times_ms": np.array([f.device_time_ms for f in frames], dtype=np.float64),
        "complementary_info.camera_frame_numbers": np.array([f.frame_number for f in frames], dtype=np.int64),
        "complementary_info.deadman": np.array([int(enabled)], dtype=np.int64),
        "complementary_info.command_accepted_time": np.array([accepted_time], dtype=np.float64),
    }
    frame.update({f"observation.images.{c.name}": images[c.name].rgb for c in cfg.cameras})
    return frame


class V3Writer:
    def __init__(self, cfg, camera_metadata):
        module = importlib.import_module("lerobot.datasets.lerobot_dataset")
        if module.CODEBASE_VERSION != "v3.0":
            raise RuntimeError(f"Expected LeRobot v3.0 writer; found {module.CODEBASE_VERSION}")
        self.root = Path(cfg.dataset.root)
        if self.root.exists():
            raise FileExistsError(f"Refusing to overwrite/resume existing dataset: {self.root}")
        self.dataset = module.LeRobotDataset.create(
            repo_id=cfg.dataset.repo_id, root=self.root, fps=cfg.dataset.fps,
            robot_type="ur5_eef", features=make_features(cfg), use_videos=True,
            image_writer_threads=cfg.dataset.image_writer_threads, image_writer_processes=0,
            video_backend="pyav", batch_encoding_size=1, vcodec=cfg.dataset.vcodec,
            metadata_buffer_size=1,
        )
        self.task = cfg.dataset.task
        self.count = 0
        provenance = {
            "config": cfg.model_dump(), "cameras": camera_metadata, "source_file": str(Path(module.__file__).resolve()),
            "pose": "Absolute TCP in UR base; [x,y,z,rx,ry,rz]; m, rad; rotation vector, not Euler",
            "observation.state": "8D absolute [TCP pose(6), left_finger_qpos_m, right_finger_qpos_m]; real feedback via configured plugin, mock data simulated",
            "action": "7D [dx,dy,dz,drx,dry,drz,grasp]; TCP translation in base, rotation Log(R_target R_observed^-1), grasp 0=open/1=closed",
            "action_timing": "Observation sampled before command; target approached over 1/fps. Not measured next-state delta.",
            "nominal_timestamp": "frame_index/fps; actual times retained separately using host monotonic clock",
            "synchronization": "Software selection by host frame arrival, not synchronized RGB exposure",
            "gripper": "action.grasp is a requested binary target (0=open, 1=closed), independent of measured feedback. Physical execution requires actuation_enabled and deadman. RG2 qpos is derived from measured raw width as [w/2,-w/2], not independent encoders.",
            "gripper_feedback_source": cfg.gripper.feedback_source,
            "gripper_actuation_enabled": cfg.gripper.actuation_enabled,
        }
        (self.root / "meta" / "capture_config.json").write_text(
            json.dumps(provenance, indent=2, ensure_ascii=False), encoding="utf-8")
        try:
            freeze = subprocess.run([__import__("sys").executable, "-m", "pip", "freeze"],
                                    capture_output=True, text=True, timeout=10, check=True).stdout
            (self.root / "meta" / "environment.txt").write_text(freeze, encoding="utf-8")
        except (subprocess.SubprocessError, OSError):
            pass

    def add(self, frame):
        self.dataset.add_frame({**frame, "task": self.task})
        self.count += 1

    def save(self):
        if self.count:
            self.dataset.save_episode(parallel_encoding=False)
        self.count = 0

    def discard(self):
        self.dataset.clear_episode_buffer()
        self.count = 0

    def close(self):
        try:
            if self.count:
                self.discard()
            self.dataset.finalize()
        finally:
            self.dataset.stop_image_writer()
