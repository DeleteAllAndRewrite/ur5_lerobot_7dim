"""Read a v3 capture with the official loader and validate every frame, including videos."""
import argparse
import json
import os
from pathlib import Path

import numpy as np

from ur5_capture.geometry import difference


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    parser.add_argument("--repo-id", default="local/ur5_three_rgb")
    args = parser.parse_args()
    if not (Path(args.root) / "meta/info.json").is_file():
        raise FileNotFoundError(f"No local dataset metadata: {args.root}")
    os.environ["HF_HUB_OFFLINE"] = "1"
    os.environ["HF_DATASETS_OFFLINE"] = "1"
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    ds = LeRobotDataset(args.repo_id, root=args.root, video_backend="pyav")
    ds._ensure_hf_dataset_loaded()
    raw = ds.hf_dataset.with_format(None)
    previous = {}
    durations = []
    for i in range(len(ds)):
        row, decoded = raw[i], ds[i]
        ep = int(row['episode_index'])
        state = np.array(row['observation.state'])
        if state.shape != (8,) or not np.isfinite(state).all():
            raise ValueError(f"Expected finite 8D absolute state at frame {i}, got {state.shape}")
        pose = state[:6]  # last two coordinates are absolute finger qpos, never EEF pose
        delta = np.array(row['action'])
        target = np.array(row['complementary_info.target_pose'])
        if delta.shape != (7,) or not np.isfinite(delta).all():
            raise ValueError(f"Expected 7D finite action at frame {i}, got {delta.shape}")
        if delta[6] not in (0, 1):
            raise ValueError(f"Invalid commanded grasp state at frame {i}: {delta[6]}")
        if not np.allclose(difference(pose, target), delta[:6], atol=1e-6):
            raise ValueError(f"Action TCP delta/target mismatch at frame {i}")
        frame_numbers = np.array(row['complementary_info.camera_frame_numbers'])
        host_time = float(np.array(row['complementary_info.host_time']).reshape(-1)[0])
        if ep in previous:
            numbers, prior_time = previous[ep]
            if not np.all(frame_numbers > numbers):
                raise ValueError(f"Duplicate/out-of-order camera frame at row {i}")
            if host_time <= prior_time:
                raise ValueError(f"Non-increasing host time at row {i}")
            durations.append(host_time - prior_time)
        previous[ep] = frame_numbers, host_time
        for key in ds.meta.video_keys:
            if decoded[key].shape[0] != 3:
                raise ValueError(f"Not RGB: {key}")
        for key in ('observation.state', 'action'):
            if not np.isfinite(row[key]).all():
                raise ValueError(f"Non-finite {key}")
    print(json.dumps({"passed": True, "format": ds.meta.info['codebase_version'],
                      "frames": len(ds), "episodes": len(previous), "cameras": ds.meta.video_keys,
                      "nominal_fps": ds.fps,
                      "actual_frame_interval_p50_s": float(np.median(durations)) if durations else None,
                      "actual_frame_interval_max_s": float(np.max(durations)) if durations else None}, indent=2))


if __name__ == "__main__":
    main()
