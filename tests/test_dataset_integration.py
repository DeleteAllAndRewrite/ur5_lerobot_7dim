"""Run in the v3 environment. Uses actual official writer and reader, no hardware."""
from pathlib import Path

import numpy as np
import yaml

from ur5_capture.config import CaptureConfig
from ur5_capture.record import record


def test_three_rgb_two_episodes_v3(tmp_path):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    cfg = CaptureConfig.model_validate(yaml.safe_load(
        (Path(__file__).resolve().parents[1] / "configs/mock.yaml").read_text()))
    cfg.dataset.root = str(tmp_path / "data")
    cfg.dataset.episode_seconds = 0.2
    cfg.synchronization.max_loop_lateness_s = 0.3
    result = record(cfg, unattended=True)
    assert result['saved_episodes'] == 2
    assert result['root'] == str(tmp_path / 'data_0000')
    assert cfg.dataset.root == str(tmp_path / 'data')
    ds = LeRobotDataset(cfg.dataset.repo_id, root=result['root'], video_backend='pyav')
    assert len(ds) == 12
    assert ds.meta.info['codebase_version'] == 'v3.0'
    for i in [0, 5, 6, 11]:
        row = ds[i]
        assert row['observation.state'].shape == (8,)
        assert float(row['observation.state'][6]) >= 0
        assert float(row['observation.state'][7]) <= 0
        assert row['action'].shape == (7,)
        assert float(row['action'][-1]) in (0.0, 1.0)
        assert int(row['episode_index']) == i // 6
        for c in cfg.cameras:
            assert row[f'observation.images.{c.name}'].shape == (3, 48, 64)
        assert row['task'] == cfg.dataset.task
    assert {float(ds[i]['action'][-1]) for i in range(len(ds))} == {0.0, 1.0}
    assert tuple(ds.meta.features['observation.state']['shape']) == (8,)
    assert ds.meta.features['observation.state']['names'][-2:] == ['left_finger_qpos_m', 'right_finger_qpos_m']
    assert tuple(ds.meta.features['action']['shape']) == (7,)
    assert ds.meta.features['action']['names'] == ['dx_m', 'dy_m', 'dz_m', 'drx_rad', 'dry_rad', 'drz_rad', 'grasp']
    # Compare raw float64 timestamps (the public tensor transform may use float32).
    raw = ds.hf_dataset.with_format(None)
    for episode in range(2):
        frame_numbers = np.array([raw[i]['complementary_info.camera_frame_numbers']
                                  for i in range(episode * 6, (episode + 1) * 6)])
        assert np.all(np.diff(frame_numbers, axis=0) > 0)

    original_info = (Path(result['root']) / 'meta/info.json').read_bytes()
    second = record(cfg, unattended=True)
    assert second['root'] == str(tmp_path / 'data_0001')
    assert second['saved_episodes'] == 2
    second_ds = LeRobotDataset(cfg.dataset.repo_id, root=second['root'], video_backend='pyav')
    assert len(second_ds) == 12
    assert (Path(result['root']) / 'meta/info.json').read_bytes() == original_info
