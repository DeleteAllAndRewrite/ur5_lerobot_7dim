"""Merge saved local captures into a new LeRobot v3 dataset, without re-encoding video."""

import argparse
import json
import os
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", type=Path, default=PROJECT_ROOT / "dataset")
    parser.add_argument("--pattern", default="ur5_three_rgb_[0-9][0-9][0-9][0-9]")
    parser.add_argument("--output", type=Path, default=PROJECT_ROOT / "dataset/ur5_three_rgb_merged")
    parser.add_argument("--repo-id", default="local/ur5_three_rgb_merged")
    parser.add_argument("--dry-run", action="store_true", help="List sources and totals without writing")
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        parser.error(f"Output already exists; choose a new --output: {output}")

    sources = []
    skipped = []
    for root in sorted(args.input_dir.resolve().glob(args.pattern)):
        info_path = root / "meta/info.json"
        if not info_path.is_file() or root.resolve() == output:
            continue
        info = json.loads(info_path.read_text())
        if info.get("total_episodes", 0) == 0 and info.get("total_frames", 0) == 0:
            skipped.append(str(root))
            continue
        if not info.get("total_episodes") or not info.get("total_frames"):
            raise ValueError(f"Inconsistent episode/frame counts: {root}")
        if info.get("codebase_version") != "v3.0":
            raise ValueError(f"Expected LeRobot v3.0: {root}")
        if output.is_relative_to(root.resolve()) or root.resolve().is_relative_to(output):
            raise ValueError(f"Source and output must not contain each other: {root}")
        sources.append({"root": str(root), "episodes": info["total_episodes"],
                        "frames": info["total_frames"],
                        "capture_failure": (root / "meta/capture_failure.json").exists()})
    if not sources:
        parser.error("No non-empty capture datasets found")
    manifest = {"repo_id": args.repo_id, "output": str(output), "sources": sources,
                "skipped_empty": skipped, "total_episodes": sum(s["episodes"] for s in sources),
                "total_frames": sum(s["frames"] for s in sources)}
    print(json.dumps(manifest, indent=2), flush=True)
    if args.dry_run:
        return

    os.environ["HF_HUB_OFFLINE"] = "1"
    os.environ["HF_DATASETS_OFFLINE"] = "1"
    from lerobot.datasets.aggregate import aggregate_datasets

    # Official aggregation remaps episode/frame/task indices and video shard references.
    # Keeping source video shards avoids re-encoding and cross-source timestamp offsets.
    aggregate_datasets(repo_ids=[f"local/{Path(s['root']).name}" for s in sources],
                       roots=[Path(s["root"]) for s in sources],
                       aggr_repo_id=args.repo_id, aggr_root=output,
                       concatenate_videos=False, concatenate_data=False)
    info = json.loads((output / "meta/info.json").read_text())
    for key in ("total_episodes", "total_frames"):
        if info[key] != manifest[key]:
            raise ValueError(f"Merged {key} mismatch: {info[key]} != {manifest[key]}")
    # Preserve an explicit mapping back to the original recordings and their configs.
    offset = 0
    for source in sources:
        source["merged_episode_start"] = offset
        offset += source["episodes"]
        source["merged_episode_end_exclusive"] = offset
    (output / "meta/merge_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"Merged {offset} episodes / {info['total_frames']} frames into {output}", flush=True)


if __name__ == "__main__":
    main()
