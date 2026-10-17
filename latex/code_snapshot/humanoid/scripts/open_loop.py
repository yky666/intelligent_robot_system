#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Execute the team's fixed sequence, default dry run. No camera or feedback."""
import argparse
from pathlib import Path, PurePosixPath
import signal
import threading

from competition import Motion, Stopped, action_frames

ROOT = Path(__file__).resolve().parents[2]


def plan(config):
    steps = config['steps']
    if not steps or steps[0]['kind'] != 'stand' or steps[-1]['kind'] != 'release':
        raise ValueError('Sequence must start with stand and end with release')
    output = []
    for step in steps:
        kind, cycles = step['kind'], step['cycles']
        if kind not in ('stand', 'grab', 'forward', 'left', 'right', 'release'):
            raise ValueError('Unknown action kind: ' + str(kind))
        if isinstance(cycles, bool) or not isinstance(cycles, int) or cycles < 1:
            raise ValueError('cycles must be a positive integer')
        path = PurePosixPath(config['actions'][kind])
        if not path.is_absolute() or path.suffix != '.d6a':
            raise ValueError('Specify an absolute Pi .d6a path')
        output.extend([kind] * cycles)
    return output


def motion_config(config):
    # Motion's shared loader uses names without the .d6a extension.
    actions = {kind: str(PurePosixPath(path).with_suffix('')) for kind, path in config['actions'].items()}
    actions.update(approach=actions['grab'], align_left=actions['grab'], align_right=actions['grab'])
    # Include the initial stand in the shared executor's preflight file loading.
    actions['retry_reset'] = actions['stand']
    return {'actions': actions, 'action_dir': '/', 'hardware_backend': config['hardware_backend'],
            'route': [s for s in config['steps'] if s['kind'] in ('forward', 'left', 'right')],
            'obstacle_guard': {'enabled': False, 'mode': 'vision', 'poll_seconds': .05},
            'hold_arm_servos': [], 'hold_sensor_file': ''}


def run(sequence, motion):
    for index, kind in enumerate(sequence, 1):
        state = ('initializing' if kind == 'stand' else
                 ('grasping' if kind == 'grab' else ('releasing' if kind == 'release' else 'carrying')))
        print('[open-loop state]', state, 'step=%d/%d' % (index, len(sequence)), kind, flush=True)
        motion.action(kind)
    print('[open-loop state] done (sequence finished; grasp/release not verified)', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=str(ROOT / 'humanoid/config/open_loop.yaml'))
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--execute', action='store_true', help='send actual servo commands')
    mode.add_argument('--check-actions', action='store_true', help='only inspect files; no hardware imports')
    args = parser.parse_args()
    import yaml
    with open(args.config, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    sequence = plan(config)
    duration = 0
    for kind in sorted(set(sequence)):
        rows = action_frames(config['actions'][kind])
        seconds = sum(row[1] for row in rows) / 1000.
        duration += sequence.count(kind) * seconds
        print('[file]', kind, config['actions'][kind], 'frames=', len(rows), 'seconds=', seconds, flush=True)
    print('[plan] action calls=%d, frame-time total=%.2fs' % (len(sequence), duration), flush=True)
    if args.check_actions:
        return
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *unused: stop.set())
    signal.signal(signal.SIGTERM, lambda *unused: stop.set())
    motion = None
    try:
        motion = Motion(motion_config(config), stop, args.execute)
        if not args.execute:
            print('DRY RUN: no camera, hardware imports, servo commands or follower heartbeat.', flush=True)
        run(sequence, motion)
    except Stopped as exc:
        if motion:
            motion.halt()
        print('[open-loop state] stopped:', exc, flush=True)
    except Exception:
        if motion:
            motion.halt()
        print('[open-loop state] fault', flush=True)
        raise
    finally:
        stop.set()


if __name__ == '__main__':
    main()
