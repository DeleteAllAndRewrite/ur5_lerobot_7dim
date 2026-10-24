"""Offline video/action/state playback through the official LeRobot Rerun viewer."""
import argparse
import importlib.util
import json
import os
from pathlib import Path


def select_episodes(spec, total):
    """Parse comma-separated indices and inclusive ranges, preserving requested order."""
    if spec == "all":
        return list(range(total))
    selected = []
    for part in spec.split(","):
        bounds = part.strip().split("-")
        if len(bounds) == 1:
            values = [int(bounds[0])]
        elif len(bounds) == 2:
            first, last = map(int, bounds)
            if first > last:
                raise ValueError("Episode ranges must be ascending")
            values = range(first, last + 1)
        else:
            raise ValueError("Use indices/ranges such as 0,2,5-8 or all")
        for value in values:
            if not 0 <= value < total:
                raise ValueError(f"Episode must be between 0 and {total - 1}")
            if value not in selected:
                selected.append(value)
    return selected


def log_frames(frames, recording, rr, camera_keys, fps, frame_offset=0, time_offset=0.0):
    """Log one episode on a rebased, monotonic timeline; return the next offsets."""
    import numpy as np

    first_time = None
    count = 0
    last_time = time_offset
    for frame in frames:
        timestamp = float(frame["timestamp"])
        if first_time is None:
            first_time = timestamp
        last_time = time_offset + timestamp - first_time
        recording.set_time("frame_index", sequence=frame_offset + count)
        recording.set_time("timestamp", duration=last_time)
        recording.set_time("episode_index", sequence=int(frame["episode_index"]))
        recording.log("episode_index", rr.Scalars(int(frame["episode_index"])))
        for key in camera_keys:
            value = frame[key].detach().cpu().numpy()
            rgb = np.rint(np.clip(value.transpose(1, 2, 0), 0, 1) * 255).astype(np.uint8)
            recording.log(key, rr.Image(rgb).compress())
        for key, path in (("action", "action"), ("observation.state", "state"),
                          ("next.done", "next.done"), ("next.reward", "next.reward"),
                          ("next.success", "next.success")):
            if key in frame:
                value = frame[key]
                recording.log(path, rr.Scalars(value.detach().cpu().numpy()))
        count += 1
    return frame_offset + count, last_time + 1 / fps if count else time_offset


def replay_multiple(args, root, selected, dataset_class):
    import time
    import rerun as rr
    import rerun.blueprint as rrb
    from lerobot.scripts.lerobot_dataset_viz import build_blueprint_from_dataset

    continuous = args.episode_mode == "continuous"
    streams = []
    output_dir = args.output_dir.expanduser().resolve() / root.name
    frame_offset, time_offset = 0, 0.0
    try:
        if args.mode == "local" and not args.save:
            # spawn() is silently ignored without an initialized global recording.
            rr.init(args.repo_id)
            rr.spawn(port=args.grpc_port, connect=False, server_memory_limit=args.server_memory_limit)
        for position, episode in enumerate(selected):
            dataset = dataset_class(args.repo_id, root=root, episodes=[episode], video_backend="pyav")
            if not continuous or position == 0:
                label = "continuous_" + "_".join(map(str, selected)) if continuous else f"episode_{episode}"
                recording = rr.RecordingStream(f"{args.repo_id}/{root.name}/{label}")
                streams.append(recording)
                blueprint = build_blueprint_from_dataset(dataset)
                # Also show which episode is active on the concatenated timeline.
                blueprint = rrb.Blueprint(rrb.Vertical(
                    blueprint.root_container,
                    rrb.TimeSeriesView(origin="episode_index", name="Episode"),
                    row_shares=[8, 1],
                ))
                if args.save:
                    output_dir.mkdir(parents=True, exist_ok=True)
                    # Keep names short even when all episodes are selected.
                    suffix = f"continuous_{selected[0]}_{selected[-1]}" if continuous else label
                    path = output_dir / f"{args.repo_id.replace('/', '_')}_{suffix}.rrd"
                    recording.save(path, default_blueprint=blueprint)
                    print(f"Replay output: {path}", flush=True)
                elif args.mode == "local":
                    recording.connect_grpc(f"rerun+http://127.0.0.1:{args.grpc_port}/proxy",
                                           default_blueprint=blueprint)
                else:
                    offset = 0 if continuous else position
                    uri = recording.serve_grpc(grpc_port=args.grpc_port + offset,
                                               default_blueprint=blueprint,
                                               server_memory_limit=args.server_memory_limit)
                    rr.serve_web_viewer(web_port=args.web_port + offset, open_browser=False, connect_to=uri)
                    print(f"{label}: {uri} ; browser port {args.web_port + offset}", flush=True)
                if not continuous:
                    frame_offset, time_offset = 0, 0.0
            print(f"Loading episode {episode}: {len(dataset)} frames", flush=True)
            frame_offset, time_offset = log_frames(
                iter(dataset), recording, rr, dataset.meta.camera_keys, dataset.fps,
                frame_offset, time_offset)
            recording.flush()
            if not continuous and args.mode != "distant":
                recording.disconnect()
        if args.mode == "distant":
            print("Serving recordings; Ctrl+C to exit.", flush=True)
            while True:
                time.sleep(1)
    except KeyboardInterrupt:
        print("Replay interrupted.", flush=True)
    finally:
        for recording in streams:
            recording.flush()
            recording.disconnect()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True, help="Local LeRobot dataset directory")
    parser.add_argument("--repo-id", default="local/ur5_three_rgb")
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument("--episode-index", type=int, default=None)
    selection.add_argument("--episodes", help="Episode indices/ranges, e.g. 0,2,5-8 or all")
    parser.add_argument("--episode-mode", choices=("separate", "continuous"), default="separate",
                        help="Separate recordings in one viewer, or one continuous timeline")
    parser.add_argument("--server-memory-limit", default="4GiB",
                        help="Per-recording history limit for multi-episode distant mode")
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
    try:
        selected = select_episodes(args.episodes if args.episodes is not None else
                                   str(args.episode_index if args.episode_index is not None else 0), episodes)
    except ValueError as exc:
        parser.error(str(exc))
    if args.save and args.mode != "local":
        parser.error("--save requires --mode local")
    if importlib.util.find_spec("rerun") is None:
        parser.error("Rerun is not installed; run: python -m pip install '.[replay]'")

    # Import after offline mode is set. The viewer never imports our robot backend.
    os.environ["HF_HUB_OFFLINE"] = "1"
    os.environ["HF_DATASETS_OFFLINE"] = "1"
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    if args.episodes is not None or args.episode_mode == "continuous":
        replay_multiple(args, root, selected, LeRobotDataset)
        return
    args.episode_index = selected[0]
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
