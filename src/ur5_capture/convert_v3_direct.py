"""Bounded-disk v3 -> v2.1 conversion: exact Arrow values and lossless video splitting."""

import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import shutil
import subprocess
import uuid


def metadata(source, output, repo_id):
    import numpy as np
    import pyarrow.parquet as pq
    from lerobot.datasets.lerobot_dataset import CODEBASE_VERSION, LeRobotDatasetMetadata

    if CODEBASE_VERSION != "v2.1":
        raise ValueError("Metadata worker requires the v2.1 environment")
    info = json.loads((source / "meta/info.json").read_text())
    features = info["features"]
    for feature in features.values():
        if feature["dtype"] == "video":
            feature.pop("info", None)
    meta = LeRobotDatasetMetadata.create(repo_id, info["fps"], features,
                                        robot_type=info.get("robot_type"), root=output)
    tasks = pq.read_table(source / "meta/tasks.parquet").to_pylist()
    for task in sorted(tasks, key=lambda t: t["task_index"]):
        meta.add_task(task["task"])
    episodes = []
    for path in sorted((source / "meta/episodes").rglob("*.parquet")):
        episodes.extend(pq.read_table(path).to_pylist())
    for episode in sorted(episodes, key=lambda e: e["episode_index"]):
        stats = {}
        for key, value in episode.items():
            if key.startswith("stats/"):
                name, metric = key[6:].rsplit("/", 1)
                # v2.1 normalization uses these five standard metrics.
                if metric in ("min", "max", "mean", "std", "count"):
                    stats.setdefault(name, {})[metric] = np.asarray(value)
        meta.save_episode(episode["episode_index"], episode["length"], episode["tasks"], stats)


def convert_direct(source, destination, python_v21, repo_id, resume_staging=None):
    import pyarrow as pa
    import pyarrow.compute as pc
    import pyarrow.parquet as pq

    source, destination = Path(source).resolve(), Path(destination).resolve()
    if destination.exists() or destination == source or source in destination.parents:
        raise ValueError(f"Refusing existing/nested destination: {destination}")
    info = json.loads((source / "meta/info.json").read_text())
    if info["codebase_version"] != "v3.0":
        raise ValueError("Direct conversion requires v3.0 input")
    staging = (Path(resume_staging).resolve() if resume_staging else
               destination.with_name(destination.name + ".converting-" + uuid.uuid4().hex[:8]))
    if resume_staging and (staging.parent != destination.parent or
                           not staging.name.startswith(destination.name + ".converting-")):
        raise ValueError("Resume directory must be a staging sibling of the destination")
    env = os.environ.copy()
    env.pop("PYTHONPATH", None)
    env.update(HF_HUB_OFFLINE="1", HF_DATASETS_OFFLINE="1")
    if not resume_staging:
        subprocess.run([python_v21, __file__, str(source), str(staging), repo_id], env=env, check=True)
    else:
        staged_info = json.loads((staging / "meta/info.json").read_text())
        for key in ("total_episodes", "total_frames", "fps"):
            if staged_info[key] != info[key]:
                raise ValueError(f"Resume source mismatch: {key}")
    try:
        episodes = []
        for p in sorted((source / "meta/episodes").rglob("*.parquet")):
            episodes.extend(pq.read_table(p).to_pylist())
        episodes.sort(key=lambda e: e["episode_index"])
        data = pa.concat_tables([pq.read_table(p) for p in sorted((source / "data").rglob("*.parquet"))])
        videos = [k for k, f in info["features"].items() if f["dtype"] == "video"]
        jobs = []
        for ep in episodes:
            index = ep["episode_index"]
            table = data.filter(pc.equal(data["episode_index"], index))
            if table.num_rows != ep["length"]:
                raise ValueError(f"Episode length mismatch: {index}")
            path = staging / f"data/chunk-{index // 1000:03d}/episode_{index:06d}.parquet"
            path.parent.mkdir(parents=True, exist_ok=True)
            pq.write_table(table, path)
            if not pq.read_table(path).equals(table):
                raise ValueError(f"Numeric round trip differs: {index}")
            for key in videos:
                prefix = f"videos/{key}"
                src = source / info["video_path"].format(
                    video_key=key, chunk_index=ep[prefix + "/chunk_index"],
                    file_index=ep[prefix + "/file_index"])
                dst = staging / f"videos/chunk-{index // 1000:03d}/{key}/episode_{index:06d}.mp4"
                dst.parent.mkdir(parents=True, exist_ok=True)
                jobs.append((src, dst, ep[prefix + "/from_timestamp"], ep["length"]))

        def split(job):
            src, dst, start, length = job
            if dst.exists():
                probe = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0",
                                        "-show_entries", "stream=nb_frames", "-of", "json", str(dst)],
                                       capture_output=True, text=True)
                if probe.returncode == 0:
                    streams = json.loads(probe.stdout).get("streams", [])
                    if streams and int(streams[0].get("nb_frames", 0)) == length:
                        return dst
                dst.rename(dst.with_name(dst.name + ".incomplete-" + uuid.uuid4().hex[:8]))
            # Accurate seeking decodes from the preceding keyframe. CRF 0 preserves
            # decoded YUV pixels and hence the source image normalization statistics.
            subprocess.run(["ffmpeg", "-nostdin", "-v", "error", "-n", "-ss", str(start),
                            "-i", str(src), "-an", "-frames:v", str(length), "-c:v", "libx264",
                            "-preset", "veryfast", "-crf", "0", "-threads", "2", str(dst)], check=True)
            probe = json.loads(subprocess.check_output([
                "ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries",
                "stream=nb_frames", "-of", "json", str(dst)]))
            if int(probe["streams"][0]["nb_frames"]) != length:
                raise ValueError(f"Video frame count mismatch: {dst}")
            return dst

        with ThreadPoolExecutor(max_workers=8) as pool:
            for n, _ in enumerate(pool.map(split, jobs), 1):
                print(f"Converted video {n}/{len(jobs)}", flush=True)
        # Let the official v2.1 reader discover the new video codec metadata.
        subprocess.run([python_v21, __file__, str(source), str(staging), repo_id, "--finalize"],
                       env=env, check=True)
        for name in ("merge_manifest.json", "capture_config.json", "environment.txt"):
            p = source / "meta" / name
            if p.exists():
                shutil.copy2(p, staging / "meta" / name)
        (staging / "meta/conversion.json").write_text(json.dumps({
            "source": str(source), "source_version": "v3.0", "target_version": "v2.1",
            "method": "Exact Arrow split; accurate ffmpeg trim; lossless H264 CRF 0",
            "numeric_features_exact": True, "video_reencoded": True,
        }, indent=2) + "\n")
        if destination.exists():
            raise FileExistsError(destination)
        staging.rename(destination)
    except BaseException:
        (staging / "CONVERSION_FAILED.txt").write_text("Incomplete conversion; do not use.\n")
        raise
    return {"destination": str(destination), "format": "v2.1"}


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("repo_id")
    parser.add_argument("--finalize", action="store_true")
    args = parser.parse_args()
    if args.finalize:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        from lerobot.datasets.utils import write_info

        ds = LeRobotDataset(args.repo_id, root=args.output, video_backend="pyav")
        ds.meta.update_video_info()
        write_info(ds.meta.info, args.output)
        source_info = json.loads((args.source / "meta/info.json").read_text())
        assert ds.meta.total_episodes == source_info["total_episodes"]
        assert len(ds) == source_info["total_frames"]
        for ep in ds.meta.episodes.values():
            start = int(ds.episode_data_index["from"][ep["episode_index"]])
            for index in (start, start + ep["length"] - 1):
                frame = ds[index]
                for key in ds.meta.video_keys:
                    height, width, channels = source_info["features"][key]["shape"]
                    assert tuple(frame[key].shape) == (channels, height, width)
        (args.output / "meta/conversion_validation.json").write_text(json.dumps({
            "passed": True, "frames": len(ds), "episodes": ds.meta.total_episodes,
            "numeric_features_exact": True, "video_frame_counts_exact": True,
            "boundary_images_decoded": ds.meta.total_episodes * len(ds.meta.video_keys) * 2,
        }, indent=2) + "\n")
    else:
        metadata(args.source, args.output, args.repo_id)
