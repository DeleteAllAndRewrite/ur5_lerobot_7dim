"""Offline video/action/state playback through the official LeRobot Rerun viewer."""
import argparse
import importlib.util
import json
import os
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True, help="Local LeRobot dataset directory")
    parser.add_argument("--repo-id", default="local/ur5_three_rgb")
    parser.add_argument("--episode-index", type=int, default=0)
    parser.add_argument("--save", action="store_true", help="Export .rrd without opening a viewer")
    parser.add_argument("--output-dir", type=Path, default=Path("replays"),
                        help="Export parent directory; a dataset-name subdirectory is added")
    parser.add_argument("--mode", choices=("local", "distant"), default="local")
    parser.add_argument("--web-port", type=int, default=9090)
    parser.add_argument("--grpc-port", type=int, default=9876)
    args = parser.parse_args()

    root = args.root.expanduser().resolve()
    info_path = root / "meta/info.json"
    try:
        info = json.loads(info_path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        parser.error(f"Cannot read local dataset metadata {info_path}: {exc}")
    episodes = info.get("total_episodes", 0)
    if not episodes or not info.get("total_frames", 0):
        parser.error(f"Dataset has no saved episodes: {root}; choose a completed capture")
    if not 0 <= args.episode_index < episodes:
        parser.error(f"--episode-index must be between 0 and {episodes - 1}")
    if args.save and args.mode != "local":
        parser.error("--save requires --mode local")
    if importlib.util.find_spec("rerun") is None:
        parser.error("Rerun is not installed; run: python -m pip install '.[replay]'")

    # Import after offline mode is set. The viewer never imports our robot backend.
    os.environ["HF_HUB_OFFLINE"] = "1"
    os.environ["HF_DATASETS_OFFLINE"] = "1"
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.scripts.lerobot_dataset_viz import visualize_dataset

    # PyAV uses the project's existing decoder, without requiring torchcodec.
    dataset = LeRobotDataset(args.repo_id, root=root, episodes=[args.episode_index], video_backend="pyav")
    print(f"Replay: {root}, episode={args.episode_index}, frames={len(dataset)}", flush=True)
    rrd_path = visualize_dataset(
        dataset, episode_index=args.episode_index, batch_size=8, num_workers=0,
        mode=args.mode, web_port=args.web_port, grpc_port=args.grpc_port,
        save=args.save, output_dir=args.output_dir.expanduser().resolve() / root.name,
        display_compressed_images=True,
    )
    if rrd_path is not None:
        import rerun as rr
        recording = rr.get_data_recording()
        if recording is not None:
            recording.flush()
            recording.disconnect()
        print(f"Saved replay: {rrd_path}")


if __name__ == "__main__":
    main()
