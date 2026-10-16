"""Bidirectional conversion using version-isolated official readers and writers."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import uuid


def convert(source, destination, python_v21, python_v3, repo_id, work_parent=None):
    source, destination = Path(source).resolve(), Path(destination).resolve()
    if destination.exists():
        raise FileExistsError(f"Destination already exists: {destination}")
    if destination == source or source in destination.parents:
        raise ValueError("Destination cannot be inside source")
    info = json.loads((source / "meta/info.json").read_text())
    version = info["codebase_version"]
    if version not in ("v2.1", "v3.0"):
        raise ValueError(f"Unsupported source format: {version}")
    target_version = "v2.1" if version == "v3.0" else "v3.0"
    interpreters = {"v2.1": python_v21, "v3.0": python_v3}
    worker = Path(__file__).with_name("conversion_worker.py")
    env = os.environ.copy()
    env.pop("PYTHONPATH", None)  # never leak one version into the other interpreter
    env.update(HF_HUB_OFFLINE="1", HF_DATASETS_OFFLINE="1")

    def call(v, *args):
        subprocess.run([interpreters[v], str(worker), *map(str, args), "--version", v], check=True, env=env)

    # Probe both environments before exporting many gigabytes of images.
    call(version, "probe")
    call(target_version, "probe")
    destination.parent.mkdir(parents=True, exist_ok=True)
    staging = destination.with_name(destination.name + ".converting-" + uuid.uuid4().hex[:8])
    try:
        with tempfile.TemporaryDirectory(prefix="lerobot-bridge-", dir=work_parent) as work:
            call(version, "export", "--root", source, "--bridge", work, "--repo-id", repo_id)
            call(target_version, "import", "--root", staging, "--bridge", work, "--repo-id", repo_id)
            call(target_version, "verify", "--root", staging, "--bridge", work, "--repo-id", repo_id)
            for name in ("capture_config.json", "environment.txt"):
                origin = source / "meta" / name
                if origin.exists():
                    shutil.copy2(origin, staging / "meta" / name)
            provenance = {"source": str(source), "source_version": version, "target_version": target_version,
                          "method": "Official decoder -> lossless NPZ bridge -> official writer",
                          "numeric_features": "Exact comparison passed against source parquet values",
                          "video": "Re-encoded; not bitwise/pixel lossless. See conversion_validation.json",
                          "task_indices": "May be reassigned; task strings and frame assignments preserved",
                          "scope": "RGB HWC video/image and fixed-shape numeric features; no subtasks"}
            (staging / "meta/conversion.json").write_text(json.dumps(provenance, indent=2), encoding="utf-8")
        # Refuse an overwrite even if another process created the destination while converting.
        if destination.exists():
            raise FileExistsError(destination)
        staging.rename(destination)
    except BaseException:
        # Leave failed output for inspection, visibly incomplete; never rename it to destination.
        if staging.exists():
            (staging / "CONVERSION_FAILED.txt").write_text("Incomplete conversion. Do not use as training data.\n")
        raise
    return {"destination": str(destination), "format": target_version}


def main():
    parser = argparse.ArgumentParser(description="LeRobot v2.1 <-> v3.0, offline and non-destructive")
    parser.add_argument("--source", required=True)
    parser.add_argument("--destination", required=True)
    parser.add_argument("--python-v21", required=True, help="Python executable of the LeRobot v0.3.3 environment")
    parser.add_argument("--python-v3", required=True, help="Python executable of the LeRobot v0.4.4 environment")
    parser.add_argument("--repo-id", default="local/converted")
    parser.add_argument("--work-parent", help="Temporary storage parent; decoded RGB requires much more disk space")
    args = parser.parse_args()
    print(json.dumps(convert(args.source, args.destination, args.python_v21, args.python_v3,
                             args.repo_id, args.work_parent), indent=2))


if __name__ == "__main__":
    main()
