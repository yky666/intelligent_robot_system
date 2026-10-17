#!/usr/bin/env python3
"""Manual object placement -> stand -> grab -> bounded visual corridor trial."""
import argparse
from pathlib import Path
import signal
import threading
import time

from competition import Camera, Motion, Heartbeat, Stopped, detect, detect_details, action_frames
from debug_view import DebugView
from lane_control import observe
from task_feedback import roi_present
from task_feedback import TaskFeedback
from vision_guard import VisionGuard

ROOT = Path(__file__).resolve().parents[2]


def startup(config, motion, heartbeat, feedback):
    """No search/approach: the operator places the block at the grab position."""
    heartbeat.set('initializing')
    motion.action('stand')
    motion.wait(config['settle_seconds'])
    heartbeat.set('grasping')
    motion.action('grab')
    motion.wait(config['settle_seconds'])
    heartbeat.set('verifying_grasp')
    if not feedback.confirm('held'):
        raise RuntimeError('Initial grab not visually confirmed; no walking')
    heartbeat.set('grasp_success')


def decision(observation, lane):
    if not observation.get('valid') or observation.get('front'):
        return None
    error = observation['error'] + lane['heading_gain']*observation['heading']
    if abs(error) <= lane['center_tolerance']:
        return 'forward'
    return 'right' if error > 0 else 'left'


def run_trial(config, camera, motion, heartbeat, max_actions, timeout):
    lane = config['lane_following']
    vision = dict(config['vision'],min_area=config['feedback'].get('roi_min_area',.03),max_area=.95)
    held = config['feedback']['held_roi']
    last = time.monotonic()
    deadline = last + timeout
    stable = count = 0
    previous = None
    previous_error = None
    previous_action = None
    while count < max_actions and time.monotonic() < deadline:
        motion.check_safety()
        frame, stamp = camera.read_after(last)
        if frame is None:
            if time.monotonic()-last > config['camera_timeout']:
                raise RuntimeError('No fresh camera frame; trial stopped')
            motion.wait(.02)
            continue
        last = stamp
        if not roi_present(frame,held,vision,detect):
            raise RuntimeError('Carried red block not visible in held ROI; trial stopped')
        observation = observe(frame,lane,held)
        choice = decision(observation,lane)
        error = observation.get('error')
        view = getattr(motion,'debug_view',None)
        if view:
            view.update('TRIAL %d/%d choice=%s stable=%d error=%s' % (count,max_actions,choice,stable,error))
        stable = stable+1 if choice is not None and choice==previous else 0
        previous = choice
        if choice is None:
            heartbeat.set('trial_waiting_for_corridor') if heartbeat.state!='trial_waiting_for_corridor' else None
            motion.wait(.05)
            continue
        if stable < lane['stable_frames']:
            motion.wait(.05)
            continue
        if (previous_action in ('left','right') and previous_error is not None and
                abs(error) > abs(previous_error)+.02):
            raise RuntimeError('Correction increased centering error; inspect yaw action before retry')
        heartbeat.set('trial_'+choice)
        print('[trial]',count+1,'/',max_actions,'action=',choice,'error=',error,flush=True)
        motion.action(choice)
        count += 1
        previous_error,previous_action = error,choice
        motion.wait(lane['settle_seconds'])
        last = time.monotonic()  # discard every pre-action camera sample
        stable,previous = 0,None
    heartbeat.set('trial_complete' if count==max_actions else 'trial_timeout')
    print('[trial] finished with %d/%d actions; object remains held' % (count,max_actions),flush=True)
    return count


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',default=str(ROOT/'humanoid/config/lane_trial.yaml'))
    p.add_argument('--execute',action='store_true')
    p.add_argument('--debug',action='store_true')
    p.add_argument('--max-actions',type=int,default=3)
    p.add_argument('--timeout',type=float,default=20)
    p.add_argument('--already-held',action='store_true',help='skip stand/grab only when deliberately starting already loaded')
    a=p.parse_args()
    if not 1<=a.max_actions<=10 or not 1<=a.timeout<=60: p.error('max-actions 1..10; timeout 1..60 seconds')
    import yaml
    config=yaml.safe_load(Path(a.config).read_text())
    if config.get('trial_only') is not True: p.error('Use the explicitly bounded trial configuration')
    if not isinstance(config['feedback'].get('held_roi'),list): p.error('Trial requires a visible held ROI')
    config['heartbeat']['enabled']=False
    stop=threading.Event()
    signal.signal(signal.SIGINT,lambda *unused:stop.set())
    signal.signal(signal.SIGTERM,lambda *unused:stop.set())
    heartbeat=Heartbeat(config['heartbeat']); camera=None; motion=None
    try:
        camera=Camera(config['camera'],stop)
        motion=Motion(config,stop,a.execute)
        # Starting pose is already loaded; hold the exact known grab tail arms.
        tail=action_frames(Path(config['action_dir'])/(config['actions']['grab']+'.d6a'))[-1]
        motion.hold_values={i:tail[i+1] for i in config['hold_arm_servos']}
        if a.debug: motion.debug_view=DebugView(camera,config,heartbeat,motion,detect_details,Stopped)
        if a.execute: motion.guard=VisionGuard(config['obstacle_guard'],camera,config['vision'],config['feedback']['held_roi'])
        # Wait for camera warm-up, with a bounded deadline, before checking guard.
        deadline=time.monotonic()+config['camera_timeout']
        while camera.read_after(float('-inf'))[0] is None:
            if stop.wait(.02) or time.monotonic()>=deadline:
                raise RuntimeError('Camera unavailable: '+getattr(camera.cam,'last_error','check device path and camera owner'))
        if not a.already_held:
            feedback=TaskFeedback(dict(config['feedback'],camera_timeout=config['camera_timeout']),
                                  config['vision'],camera,motion,detect)
            startup(config,motion,heartbeat,feedback)
        run_trial(config,camera,motion,heartbeat,a.max_actions,a.timeout)
    except Stopped:
        print('[trial] stopped by operator')
    finally:
        if motion: motion.halt()
        stop.set()
        if camera: camera.close()
        if a.debug:
            import cv2
            cv2.destroyAllWindows()
        heartbeat.close()


if __name__=='__main__': main()
