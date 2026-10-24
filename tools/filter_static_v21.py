"""Compress idle runs in a local v2.1 dataset, with synchronized lossless video selection."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
import json
from pathlib import Path
import shutil
import subprocess

import av
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
from scipy.spatial.transform import Rotation


def retained_indices(action, state, fps):
    """Keep motion, grasp transitions (+/-1 s), short pauses, and idle boundaries."""
    n = len(action)
    if not np.isfinite(action).all() or not np.isfinite(state).all():
        raise ValueError('Nonfinite action/state')
    rot = Rotation.from_rotvec(state[:, 3:6])
    quiet = np.max(np.abs(action[:, :6]), axis=1) <= 1e-8
    quiet[0] = False
    quiet[1:] &= np.linalg.norm(np.diff(state[:, :3], axis=0), axis=1) <= 0.0001
    quiet[1:] &= (rot[1:] * rot[:-1].inv()).magnitude() <= 0.001
    quiet[1:] &= np.max(np.abs(np.diff(state[:, 6:], axis=0)), axis=1) <= 1e-6
    changes = np.flatnonzero(np.diff(action[:, 6]) != 0) + 1
    for i in changes:
        quiet[max(0, i - fps):min(n, i + fps + 1)] = False
    keep = np.ones(n, dtype=bool)
    start = 0
    while start < n:
        if not quiet[start]:
            start += 1
            continue
        end = start + 1
        while end < n and quiet[end]:
            # Bound accumulated drift so slow motion cannot be compressed away.
            if np.linalg.norm(state[end, :3] - state[start, :3]) > 0.0005:
                break
            if (rot[end] * rot[start].inv()).magnitude() > 0.003:
                break
            end += 1
        if end - start >= int(np.ceil(0.5 * fps)):
            keep[start + 3:end - 3] = False
        start = end
    return np.flatnonzero(keep)


def ranges(indices):
    groups = np.split(indices, np.flatnonzero(np.diff(indices) != 1) + 1)
    return [[int(g[0]), int(g[-1])] for g in groups if len(g)]


def video_job(src, dst, selected, fps):
    from lerobot.datasets.compute_stats import sample_indices, auto_downsample_height_width, get_feature_stats
    dst.parent.mkdir(parents=True, exist_ok=True)
    expr = '+'.join(f'between(n,{a},{b})' for a, b in ranges(selected))
    script = dst.with_suffix('.filter')
    script.write_text(f"select='{expr}',setpts=N/({fps}*TB)")
    subprocess.run(['ffmpeg', '-nostdin', '-v', 'error', '-n', '-i', str(src),
                    '-filter_script:v', str(script), '-an', '-c:v', 'libx264', '-crf', '0',
                    '-preset', 'veryfast', '-threads', '2', '-r', str(fps), str(dst)], check=True)
    script.unlink()
    samples = set(sample_indices(len(selected)))
    images = []
    count = 0
    with av.open(str(dst)) as container:
        for i, frame in enumerate(container.decode(video=0)):
            if abs(float(frame.time) - i / fps) > 1e-4:
                raise ValueError(f'Bad video timestamp: {dst} frame {i}')
            if i in samples:
                images.append(auto_downsample_height_width(frame.to_ndarray(format='rgb24').transpose(2, 0, 1)))
            count += 1
    if count != len(selected):
        raise ValueError(f'Video count mismatch: {dst}: {count} != {len(selected)}')
    stats = get_feature_stats(np.stack(images), axis=(0, 2, 3), keepdims=True)
    return {k: v if k == 'count' else np.squeeze(v / 255., axis=0) for k, v in stats.items()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', required=True, type=Path)
    parser.add_argument('--apply', action='store_true', help='Replace root only after validation, retaining a backup')
    args = parser.parse_args()
    root = args.root.resolve()
    info = json.loads((root / 'meta/info.json').read_text())
    if info['codebase_version'] != 'v2.1':
        raise ValueError('Expected v2.1')
    if (root / 'meta/static_filter.json').exists():
        raise ValueError('Already filtered; restore the backup before changing the rule')
    episodes = [json.loads(s) for s in (root / 'meta/episodes.jsonl').read_text().splitlines()]
    plans = []
    for ep in episodes:
        i = ep['episode_index']
        rel = info['data_path'].format(episode_chunk=i // info['chunks_size'], episode_index=i)
        table = pq.read_table(root / rel)
        if len(table) != ep['length']:
            raise ValueError('Episode length mismatch')
        selected = retained_indices(np.array(table['action'].to_pylist()),
                                    np.array(table['observation.state'].to_pylist()), info['fps'])
        plans.append((ep, rel, selected))
    before = sum(ep['length'] for ep, _, _ in plans)
    after = sum(len(s) for _, _, s in plans)
    if before != info['total_frames']:
        raise ValueError('Total length mismatch')
    print(json.dumps(dict(before=before, after=after, removed=before-after)), flush=True)
    if not args.apply:
        return
    from lerobot.datasets.compute_stats import compute_episode_stats
    from lerobot.datasets.utils import serialize_dict
    tag = datetime.now().strftime('%Y%m%d-%H%M%S')
    staging = root.with_name(root.name + '.filtering-' + tag)
    backup = root.with_name(root.name + '.backup-' + tag)
    staging.mkdir()
    (staging / 'meta').mkdir()
    shutil.copy2(root / 'meta/tasks.jsonl', staging / 'meta/tasks.jsonl')
    videos = [k for k, v in info['features'].items() if v['dtype'] == 'video']
    offset = 0
    report = {'source': str(root), 'backup': str(backup), 'before': before, 'after': after,
              'removed': before-after, 'rule': 'zero arm action <=1e-8; adjacent TCP <=0.1mm/0.001rad; run drift <=0.5mm/0.003rad; fingers <=1e-6m; protect grasp changes +/-1s; idle >=0.5s; retain first/last 3 idle frames',
              'video': 'H264 CRF0, synchronized frame selection, all output frames decoded and timestamps checked',
              'timestamps': 'timestamp reset to frame_index/fps; original hardware/host timestamps preserved',
              'episodes': []}
    for ep, rel, selected in plans:
        table = pq.read_table(root / rel).take(pa.array(selected))
        for key, values in [('frame_index', np.arange(len(selected))),
                            ('index', np.arange(offset, offset + len(selected))),
                            ('timestamp', np.arange(len(selected)) / info['fps'])]:
            pos = table.schema.get_field_index(key)
            table = table.set_column(pos, table.schema.field(pos), pa.array(values, type=table.schema.field(pos).type))
        dst = staging / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        pq.write_table(table, dst)
        if not pq.read_table(dst).equals(table):
            raise ValueError('Parquet round trip mismatch')
        stats = compute_episode_stats({k: np.array(table[k].to_pylist()) for k in table.column_names}, info['features'])
        i = ep['episode_index']
        def job(key):
            path = info['video_path'].format(episode_chunk=i // info['chunks_size'], episode_index=i, video_key=key)
            return video_job(root / path, staging / path, selected, info['fps'])
        with ThreadPoolExecutor(max_workers=3) as pool:
            stats.update(zip(videos, pool.map(job, videos)))
        with (staging / 'meta/episodes_stats.jsonl').open('a') as f:
            f.write(json.dumps({'episode_index': i, 'stats': serialize_dict(stats)}) + '\n')
        with (staging / 'meta/episodes.jsonl').open('a') as f:
            f.write(json.dumps({**ep, 'length': len(selected)}) + '\n')
        report['episodes'].append({'episode_index': i, 'before': ep['length'], 'after': len(selected),
                                   'source_frame_ranges_inclusive': ranges(selected)})
        offset += len(selected)
        print(f'Episode {i}: {ep["length"]} -> {len(selected)}; three videos verified', flush=True)
    info['total_frames'] = offset
    (staging / 'meta/info.json').write_text(json.dumps(info, indent=2) + '\n')
    (staging / 'meta/static_filter.json').write_text(json.dumps(report, indent=2) + '\n')
    # Official reader verifies indexing and video lookup at episode boundaries.
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    ds = LeRobotDataset('local/filtered', root=staging, video_backend='pyav')
    assert len(ds) == after
    for ep in ds.meta.episodes.values():
        start = int(ds.episode_data_index['from'][ep['episode_index']])
        for index in (start, start + ep['length'] - 1):
            frame = ds[index]
            for key in videos:
                h, w, c = info['features'][key]['shape']
                assert tuple(frame[key].shape) == (c, h, w)
    del ds
    if backup.exists():
        raise FileExistsError(backup)
    root.rename(backup)
    try:
        staging.rename(root)
    except BaseException:
        backup.rename(root)
        raise
    print(json.dumps({'root': str(root), 'backup': str(backup), 'removed': before-after}), flush=True)


if __name__ == '__main__':
    main()
