"""Standalone worker: intentionally imports no project package or hardware driver.

Invoked by absolute path with either version's Python. The official reader resolves
v3 video segment offsets; raw Arrow-backed values preserve float64 precision.
"""
import argparse
import inspect
import json
from pathlib import Path

import numpy as np

RESERVED = {"timestamp", "frame_index", "episode_index", "index", "task_index"}


def api(version):
    from lerobot.datasets.lerobot_dataset import CODEBASE_VERSION, LeRobotDataset
    if CODEBASE_VERSION != version:
        raise RuntimeError(f"Interpreter uses {CODEBASE_VERSION}; expected {version}")
    return LeRobotDataset


def load(root, repo_id, version):
    dataset = api(version)(repo_id=repo_id, root=Path(root), video_backend="pyav")
    if hasattr(dataset, "_ensure_hf_dataset_loaded"):
        dataset._ensure_hf_dataset_loaded()
    return dataset, dataset.hf_dataset.with_format(None)


def clean_features(features):
    result = {}
    for key, feature in features.items():
        if key in RESERVED:
            continue
        if key == "subtask_index":
            raise ValueError("Subtask metadata conversion is not supported")
        dtype = feature["dtype"]
        shape = tuple(feature["shape"])
        if dtype in ("image", "video"):
            if len(shape) != 3 or shape[2] != 3:
                raise ValueError(f"{key}: only HWC three-channel RGB features are supported")
        else:
            try:
                if np.dtype(dtype).kind not in "biuf":
                    raise ValueError(dtype)
            except (TypeError, ValueError) as exc:
                raise ValueError(f"{key}: unsupported feature dtype {dtype}") from exc
        result[key] = {"dtype": dtype, "shape": shape, "names": feature.get("names")}
    return result


def extract(dataset, raw, index, features):
    row = raw[index]
    # Public __getitem__ handles image/video decoding and the version-specific task table.
    item = dataset[index]
    output = {}
    for key, feature in features.items():
        if feature["dtype"] in ("image", "video"):
            value = item[key].detach().cpu().numpy()
            value = np.transpose(value, (1, 2, 0))
            output[key] = np.rint(np.clip(value, 0, 1) * 255).astype(np.uint8)
        else:
            output[key] = np.asarray(row[key], dtype=feature["dtype"]).reshape(feature["shape"])
        if output[key].shape != tuple(feature["shape"]):
            raise ValueError(f"Feature shape mismatch: {key}")
    output["__task__"] = np.array(item["task"])
    output["__timestamp__"] = np.array(row["timestamp"], dtype=np.float64)
    return row, output


def export_dataset(args):
    dataset, raw = load(args.root, args.repo_id, args.version)
    features = clean_features(dataset.features)
    info = json.loads((Path(args.root) / "meta/info.json").read_text())
    if len(dataset) == 0:
        raise ValueError("Cannot convert an empty dataset")
    bridge = Path(args.bridge)
    episodes = []
    previous_episode = -1
    expected_frame = 0
    for index in range(len(dataset)):
        row, values = extract(dataset, raw, index, features)
        ep, fi = int(row["episode_index"]), int(row["frame_index"])
        if ep != previous_episode:
            if ep != previous_episode + 1:
                raise ValueError("Episode indices must be contiguous and start at zero")
            episodes.append({"episode_index": ep, "length": 0})
            (bridge / f"episode_{ep:06d}").mkdir()
            previous_episode, expected_frame = ep, 0
        if fi != expected_frame or int(row["index"]) != index:
            raise ValueError("Non-contiguous frame indices")
        expected_frame += 1
        episodes[-1]["length"] += 1
        np.savez_compressed(bridge / f"episode_{ep:06d}/{fi:06d}.npz", **values)
    manifest = {"source_format": args.version, "fps": info["fps"], "robot_type": info.get("robot_type"),
                "features": features, "episodes": episodes, "total_frames": len(dataset)}
    if len(episodes) != info["total_episodes"] or len(dataset) != info["total_frames"]:
        raise ValueError("Source metadata counts do not match data")
    (bridge / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(f"Exported {len(episodes)} episodes, {len(dataset)} frames from {args.version}")


def import_dataset(args):
    cls = api(args.version)
    bridge = Path(args.bridge)
    manifest = json.loads((bridge / "manifest.json").read_text())
    features = clean_features(manifest["features"])
    kwargs = dict(repo_id=args.repo_id, root=Path(args.root), fps=manifest["fps"],
                  features=features, robot_type=manifest["robot_type"], use_videos=True,
                  image_writer_threads=0, image_writer_processes=0, video_backend="pyav", batch_encoding_size=1)
    if "vcodec" in inspect.signature(cls.create).parameters:
        kwargs.update(vcodec="h264", metadata_buffer_size=1)
    ds = cls.create(**kwargs)
    try:
        for ep in manifest["episodes"]:
            for fi in range(ep["length"]):
                path = bridge / f"episode_{ep['episode_index']:06d}/{fi:06d}.npz"
                with np.load(path, allow_pickle=False) as data:
                    frame = {key: data[key] for key in features}
                    task, timestamp = str(data["__task__"].item()), float(data["__timestamp__"])
                if args.version == "v2.1":
                    ds.add_frame(frame, task=task, timestamp=timestamp)
                else:
                    # v0.4.4's add_frame contains timestamp handling, but its validator
                    # rejects reserved columns in the input. Set the accepted episode
                    # buffer timestamp before official save_episode serializes it.
                    ds.add_frame({**frame, "task": task})
                    ds.episode_buffer["timestamp"][-1] = timestamp
            if args.version == "v3.0":
                ds.save_episode(parallel_encoding=False)
            else:
                ds.save_episode()
    finally:
        if hasattr(ds, "finalize"):
            ds.finalize()
        ds.stop_image_writer()
    print(f"Wrote {args.version} using official writer")


def verify_dataset(args):
    ds, raw = load(args.root, args.repo_id, args.version)
    bridge = Path(args.bridge)
    manifest = json.loads((bridge / "manifest.json").read_text())
    features = clean_features(manifest["features"])
    if len(ds) != manifest["total_frames"]:
        raise ValueError("Converted frame count differs")
    info = json.loads((Path(args.root) / "meta/info.json").read_text())
    if info["total_episodes"] != len(manifest["episodes"]) or info["fps"] != manifest["fps"]:
        raise ValueError("Converted episode count or FPS differs")
    if clean_features(ds.features) != features:
        raise ValueError("Converted feature schema differs")
    image_error = {}
    index = 0
    for ep in manifest["episodes"]:
        for fi in range(ep["length"]):
            row, actual = extract(ds, raw, index, features)
            if int(row["episode_index"]) != ep["episode_index"] or int(row["frame_index"]) != fi:
                raise ValueError("Converted episode boundary differs")
            with np.load(bridge / f"episode_{ep['episode_index']:06d}/{fi:06d}.npz", allow_pickle=False) as expected:
                for key, feature in features.items():
                    if feature["dtype"] in ("image", "video"):
                        error = float(np.abs(actual[key].astype(float) - expected[key].astype(float)).mean())
                        values = image_error.setdefault(key, [])
                        values.append(error)
                    elif not np.array_equal(actual[key], expected[key], equal_nan=True):
                        raise ValueError(f"Numeric feature changed at frame {index}: {key}")
                for key in ("__task__", "__timestamp__"):
                    if not np.array_equal(actual[key], expected[key]):
                        raise ValueError(f"{key} changed at frame {index}")
            index += 1
    report = {"passed": True, "frames": index, "episodes": len(manifest["episodes"]),
              "numeric_features_exact": True, "tasks_and_timestamps_exact": True,
              "rgb_mean_absolute_error_0_255": {k: float(np.mean(v)) for k, v in image_error.items()},
              "video_reencoded": True}
    (Path(args.root) / "meta/conversion_validation.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("operation", choices=["probe", "export", "import", "verify"])
    parser.add_argument("--version", required=True, choices=["v2.1", "v3.0"])
    parser.add_argument("--root")
    parser.add_argument("--bridge")
    parser.add_argument("--repo-id", default="local/conversion")
    args = parser.parse_args()
    api(args.version)
    if args.operation == "probe":
        print(f"Official {args.version} reader/writer available")
    else:
        {"export": export_dataset, "import": import_dataset, "verify": verify_dataset}[args.operation](args)


if __name__ == "__main__":
    main()
