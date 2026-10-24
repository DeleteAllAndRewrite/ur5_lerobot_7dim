"""Remove static pauses from a local v2.1 dataset: the idle start of every episode,
plus every later pause (including the idle tail) of at least --min-pause seconds.

Parquet rows (action, observation.state, complementary_info, ...) and every camera
video drop the same frames, so all modalities stay aligned.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import shutil
import sys

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
from scipy.spatial.transform import Rotation


def first_motion(action, state, action_tol=1e-8, pos_tol=5e-4, rot_tol=3e-3, finger_tol=None):
    """Index of the first frame where the arm is commanded or moves; None if never."""
    if not np.isfinite(action).all() or not np.isfinite(state).all():
        raise ValueError('Nonfinite action/state')
    moving = np.max(np.abs(action[:, :6]), axis=1) > action_tol
    moving |= action[:, 6] != action[0, 6]
    moving |= np.linalg.norm(state[:, :3] - state[0, :3], axis=1) > pos_tol
    # Rotation-vector difference is a cheap bound; the arm is far from the pi wrap at rest.
    moving |= np.linalg.norm(state[:, 3:6] - state[0, 3:6], axis=1) > rot_tol
    if finger_tol is not None:
        moving |= np.max(np.abs(state[:, 6:] - state[0, 6:]), axis=1) > finger_tol
    hits = np.flatnonzero(moving)
    return int(hits[0]) if len(hits) else None


def pause_runs(action, state, min_frames, start=0, action_tol=1e-8, step_pos=1e-4, step_rot=1e-3,
               step_finger=1e-6, pos_tol=5e-4, rot_tol=3e-3):
    """[begin, end) runs after `start` where nothing is commanded and nothing moves.

    A frame is quiet when the arm action is zero, the grasp command is unchanged and
    TCP/fingers barely move since the previous frame. Runs are also cut once the TCP
    drifts past pos_tol/rot_tol, so slow motion is never mistaken for a pause.
    """
    rot = Rotation.from_rotvec(state[:, 3:6])
    quiet = np.zeros(len(action), dtype=bool)
    quiet[1:] = np.max(np.abs(action[1:, :6]), axis=1) <= action_tol
    quiet[1:] &= action[1:, 6] == action[:-1, 6]
    quiet[1:] &= np.linalg.norm(np.diff(state[:, :3], axis=0), axis=1) <= step_pos
    quiet[1:] &= (rot[1:] * rot[:-1].inv()).magnitude() <= step_rot
    quiet[1:] &= np.max(np.abs(np.diff(state[:, 6:], axis=0)), axis=1) <= step_finger
    runs = []
    i = start
    while i < len(quiet):
        if not quiet[i]:
            i += 1
            continue
        j = i + 1
        while (j < len(quiet) and quiet[j] and np.linalg.norm(state[j, :3] - state[i, :3]) <= pos_tol
               and (rot[j] * rot[i].inv()).magnitude() <= rot_tol):
            j += 1
        if j - i >= min_frames:
            runs.append((i, j))
        i = j
    return runs


def plan_episode(action, state, fps, min_pause, keep, finger_tol=None):
    """Frame indices to keep and the [begin, end) ranges removed."""
    start = first_motion(action, state, finger_tol=finger_tol)
    if start is None:
        return None, None
    n = len(action)
    removed = [(0, max(0, start - keep))]
    for begin, end in pause_runs(action, state, int(np.ceil(min_pause * fps)), start=start):
        # Keep `keep` frames on each side that touches motion; a tail pause has only one.
        removed.append((begin + keep, end if end == n else end - keep))
    removed = [(a, b) for a, b in removed if b > a]
    drop = np.zeros(n, dtype=bool)
    for a, b in removed:
        drop[a:b] = True
    return np.flatnonzero(~drop), [[int(a), int(b)] for a, b in removed]


def dir_size(path):
    return sum(f.stat().st_size for f in path.rglob('*') if f.is_file())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', required=True, type=Path, help='Source v2.1 dataset (never modified)')
    parser.add_argument('--output', type=Path, help='New dataset directory; omit for a dry run')
    parser.add_argument('--min-pause', type=float, default=1.0,
                        help='Remove pauses after the first motion lasting at least this many seconds '
                             '(default: 1.0); the idle start is always removed')
    parser.add_argument('--keep', type=int, default=0,
                        help='Static frames to keep next to motion at each removed pause (default: 0)')
    parser.add_argument('--finger-tol', type=float, default=None,
                        help='Also treat finger qpos changes above this (m) as motion; off by default '
                             'so the gripper auto-opening at startup is still trimmed')
    parser.add_argument('--workers', type=int, default=3, help='Parallel video encodes')
    args = parser.parse_args()
    root = args.root.resolve()
    info = json.loads((root / 'meta/info.json').read_text())
    if info['codebase_version'] != 'v2.1':
        parser.error('Expected a v2.1 dataset')
    if (root / 'meta/static_trim.json').exists():
        parser.error('Source was already trimmed')
    episodes = [json.loads(s) for s in (root / 'meta/episodes.jsonl').read_text().splitlines()]
    fps = info['fps']
    videos = [k for k, v in info['features'].items() if v['dtype'] == 'video']
    data_rel = lambda i: info['data_path'].format(episode_chunk=i // info['chunks_size'], episode_index=i)
    video_rel = lambda i, key: info['video_path'].format(
        episode_chunk=i // info['chunks_size'], episode_index=i, video_key=key)

    plans = []
    for ep in episodes:
        i = ep['episode_index']
        table = pq.read_table(root / data_rel(i), columns=['action', 'observation.state'])
        if len(table) != ep['length']:
            raise ValueError(f'Episode {i}: parquet has {len(table)} rows, episodes.jsonl says {ep["length"]}')
        selected, removed = plan_episode(np.array(table['action'].to_pylist()),
                                         np.array(table['observation.state'].to_pylist()), fps,
                                         args.min_pause, args.keep, finger_tol=args.finger_tol)
        if selected is None:
            raise ValueError(f'Episode {i} never moves; remove it instead of trimming')
        plans.append((ep, selected, removed))
    before = sum(ep['length'] for ep, _, _ in plans)
    after = sum(len(s) for _, s, _ in plans)
    if before != info['total_frames']:
        raise ValueError('Total frame count mismatch')
    for ep, s, removed in plans:
        lead = removed[0][1] if removed and removed[0][0] == 0 else 0
        dropped = ep['length'] - len(s)
        print(f'Episode {ep["episode_index"]}: drop {dropped:4d} / {ep["length"]} ({dropped / fps:.2f} s); '
              f'start {lead}, {len(removed) - (lead > 0)} later pauses')
    print(json.dumps(dict(before=before, after=after, removed=before - after)), flush=True)
    if args.output is None:
        print('Dry run; pass --output NEW_DIR to write the trimmed dataset')
        return

    output = args.output.resolve()
    staging = output.with_name(output.name + '.partial')
    if output.exists() or staging.exists():
        parser.error(f'Output exists; choose a new --output or remove: {output} / {staging}')
    if output.is_relative_to(root) or root.is_relative_to(output):
        parser.error('Source and output must not contain each other')
    # Lossless re-encoding keeps roughly the source size per retained frame.
    need = int(dir_size(root) * after / before * 1.05)
    free = shutil.disk_usage(output.parent if output.parent.exists() else Path.cwd()).free
    if free < need:
        parser.error(f'Need ~{need / 2**30:.1f} GiB at {output.parent}, only {free / 2**30:.1f} GiB free')

    # The idle-compression tool already does synchronized lossless cutting plus video stats.
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from filter_static_v21 import video_job
    from lerobot.datasets.compute_stats import compute_episode_stats
    from lerobot.datasets.utils import serialize_dict

    staging.mkdir(parents=True)
    shutil.copytree(root / 'meta', staging / 'meta',
                    ignore=shutil.ignore_patterns('info.json', 'episodes.jsonl', 'episodes_stats.jsonl'))
    report = {'source': str(root), 'before': before, 'after': after, 'removed': before - after,
              'rule': 'start: drop leading frames until arm action > 1e-8, grasp command changes, TCP '
                      'moves > 0.5 mm or rotvec > 0.003 rad from frame 0'
                      + (f', or fingers move > {args.finger_tol} m' if args.finger_tol is not None else '')
                      + f'; pauses >= {args.min_pause} s: zero arm action, unchanged grasp command, '
                      'per-frame TCP <= 0.1 mm/0.001 rad, fingers <= 1e-6 m, run drift <= 0.5 mm/0.003 rad',
              'min_pause_s': args.min_pause,
              'keep': args.keep,
              'video': 'H264 CRF0, same frames cut from every camera, all output frames decoded and counted',
              'timestamps': 'timestamp/frame_index/index reset; complementary_info times kept as recorded',
              'episodes': []}
    offset = 0
    for ep, selected, removed in plans:
        i = ep['episode_index']
        table = pq.read_table(root / data_rel(i)).take(pa.array(selected))
        for key, values in [('frame_index', np.arange(len(selected))),
                            ('index', np.arange(offset, offset + len(selected))),
                            ('timestamp', np.arange(len(selected)) / fps)]:
            pos = table.schema.get_field_index(key)
            field = table.schema.field(pos)
            table = table.set_column(pos, field, pa.array(values, type=field.type))
        dst = staging / data_rel(i)
        dst.parent.mkdir(parents=True, exist_ok=True)
        pq.write_table(table, dst)
        if not pq.read_table(dst).equals(table):
            raise ValueError(f'Parquet round trip mismatch: {dst}')
        stats = compute_episode_stats({k: np.array(table[k].to_pylist()) for k in table.column_names
                                       if k in info['features']}, info['features'])
        with ThreadPoolExecutor(max_workers=args.workers) as pool:
            jobs = {key: pool.submit(video_job, root / video_rel(i, key), staging / video_rel(i, key),
                                     selected, fps) for key in videos}
            stats.update({key: job.result() for key, job in jobs.items()})
        with (staging / 'meta/episodes_stats.jsonl').open('a') as f:
            f.write(json.dumps({'episode_index': i, 'stats': serialize_dict(stats)}) + '\n')
        with (staging / 'meta/episodes.jsonl').open('a') as f:
            f.write(json.dumps({**ep, 'length': len(selected)}) + '\n')
        report['episodes'].append({'episode_index': i, 'before': ep['length'], 'after': len(selected),
                                   'removed_source_ranges': removed})
        offset += len(selected)
        print(f'Episode {i}: {ep["length"]} -> {len(selected)}; {len(videos)} videos verified', flush=True)
    info['total_frames'] = offset
    (staging / 'meta/info.json').write_text(json.dumps(info, indent=4) + '\n')
    (staging / 'meta/static_trim.json').write_text(json.dumps(report, indent=2) + '\n')

    # Official reader checks indexing and video lookup at every episode boundary.
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    ds = LeRobotDataset('local/trimmed', root=staging, video_backend='pyav')
    assert len(ds) == after, (len(ds), after)
    for ep in ds.meta.episodes.values():
        first = int(ds.episode_data_index['from'][ep['episode_index']])
        for index in (first, first + ep['length'] - 1):
            frame = ds[index]
            for key in videos:
                h, w, c = info['features'][key]['shape']
                assert tuple(frame[key].shape) == (c, h, w)
    del ds
    staging.rename(output)
    print(json.dumps({'output': str(output), 'removed': before - after}), flush=True)


if __name__ == '__main__':
    main()
