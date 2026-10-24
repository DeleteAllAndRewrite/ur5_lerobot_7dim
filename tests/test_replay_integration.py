"""Small real LeRobot/Rerun export smoke test, with mock devices only."""
from pathlib import Path
import subprocess
import sys

import yaml


def test_multi_episode_rrd_exports(tmp_path, monkeypatch):
    from ur5_capture.config import CaptureConfig
    from ur5_capture.record import record
    from ur5_capture.replay import main

    cfg = CaptureConfig.model_validate(yaml.safe_load(
        (Path(__file__).resolve().parents[1] / "configs/mock.yaml").read_text()))
    cfg.dataset.root = str(tmp_path / "mock")
    cfg.dataset.episode_seconds = 0.2
    cfg.synchronization.max_loop_lateness_s = 0.3
    result = record(cfg, unattended=True)
    for mode in ("separate", "continuous"):
        output = tmp_path / mode
        monkeypatch.setattr(sys, "argv", ["ur5-replay", "--root", result["root"],
                            "--episodes", "all", "--episode-mode", mode,
                            "--save", "--output-dir", str(output)])
        main()
        files = sorted(output.rglob("*.rrd"))
        assert len(files) == (2 if mode == "separate" else 1)
        for path in files:
            assert path.stat().st_size > 0
            subprocess.run([str(Path(sys.executable).with_name("rerun")), "rrd", "verify", str(path)],
                           check=True, capture_output=True)
