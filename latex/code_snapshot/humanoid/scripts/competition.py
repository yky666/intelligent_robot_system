#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TonyPi: fresh vision -> grasp -> carry route -> release.

Single synchronous action executor prevents vision/motion command races.
Direct d6a frame execution propagates errors (some SDK action helpers swallow them).
Python >= 3.5. No ROS requirement on the Raspberry Pi.
"""
from __future__ import print_function
import argparse
import importlib
import json
import os
from pathlib import Path
import signal
import socket
import sqlite3
import sys
import threading
import time
import uuid

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'turtlebot/follow2/multi_robot_following/src'))
from competition_core.mission import Approach, validate_route
from task_feedback import DistanceGuard, TaskFeedback, validate_feedback
from vision_guard import VisionGuard, floor_obstacle
from debug_view import DebugView
from color_mask import threshold
import lane_control


class Stopped(RuntimeError):
    pass


def action_frames(path):
    path = Path(path)
    if not path.is_file():
        raise RuntimeError('Missing action file: ' + str(path))
    connection = sqlite3.connect('file:' + path.resolve().as_posix() + '?mode=ro', uri=True)
    try:
        rows = connection.execute('SELECT * FROM ActionGroup ORDER BY "Index"').fetchall()
        if not rows or any(len(row) not in (18, 20) or not isinstance(row[1], int) or not 20 <= row[1] <= 30000 or
                           any(not isinstance(v, int) or not 0 <= v <= 1000 for v in row[2:]) for row in rows):
            raise ValueError('Invalid 16/18-channel TonyPi d6a: ' + str(path))
        return rows
    finally:
        connection.close()


class Heartbeat(object):
    def __init__(self, config):
        self.config = config
        self.state = 'idle'
        self.stop = threading.Event()
        self.sequence = 0
        self.run_id = str(uuid.uuid4())
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.worker = threading.Thread(target=self.run)
        self.worker.daemon = True
        self.worker.start()

    def run(self):
        while not self.stop.is_set():
            if self.config['enabled']:
                data = dict(session=self.config['session'], state=self.state,
                            sequence=self.sequence, run_id=self.run_id)
                try:
                    self.sock.sendto(json.dumps(data).encode('utf-8'), (self.config['host'], self.config['port']))
                except OSError as exc:
                    print('[heartbeat]', exc, file=sys.stderr)
            self.sequence += 1
            self.stop.wait(0.2)

    def set(self, state):
        self.state = state
        print('[state]', state)

    def close(self):
        self.stop.set()
        self.worker.join(1)
        self.sock.close()


class Motion(object):
    def __init__(self, config, stop, execute):
        self.config, self.stop, self.execute = config, stop, execute
        self.frames = {}
        self.hold_values = {}
        self.guard = DistanceGuard(config['obstacle_guard']) if execute and config['obstacle_guard'].get('mode', 'distance') == 'distance' else None
        names = set(validate_route(config))
        if config['actions'].get('retry_reset'):
            names.add(config['actions']['retry_reset'])
        if config['actions'].get('stand'):
            names.add(config['actions']['stand'])
        for name in names:
            if execute:
                self.frames[name] = action_frames(Path(config['action_dir']) / (name + '.d6a'))
        self.controller = self.old_board = None
        backend = config.get('hardware_backend', 'auto')
        if backend not in ('auto', 'legacy', 'controller'):
            raise ValueError('hardware_backend must be auto, legacy or controller')
        if execute:
            for directory in ('/home/pi/TonyPi', '/home/pi/TonyPi/HiwonderSDK',
                              '/home/pi/TonyPi/HiwonderSDK/build/lib'):
                sys.path.insert(0, directory)
            # Only import hardware drivers after all preflight checks succeed.
            if backend == 'legacy':
                self.old_board = importlib.import_module('hiwonder.Board')
            else:
                try:
                    sdk = importlib.import_module('hiwonder.ros_robot_controller_sdk')
                    controller = importlib.import_module('hiwonder.Controller')
                except ImportError:
                    if backend == 'controller':
                        raise
                    self.old_board = importlib.import_module('hiwonder.Board')
                else:
                    self.controller = controller.Controller(sdk.Board())

    def check_safety(self):
        if self.stop.is_set():
            raise Stopped('stop requested')
        view = getattr(self, 'debug_view', None)
        if view:
            view.poll()
        guard = getattr(self, 'guard', None)
        if guard:
            guard.check()

    def wait(self, seconds):
        deadline = time.monotonic() + seconds
        while True:
            self.check_safety()
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return
            interval = self.config.get('obstacle_guard', {}).get('poll_seconds', .05)
            if self.stop.wait(min(interval, remaining)):
                raise Stopped('stop requested')

    def halt(self):
        """Stop interpolating bus servos where SDK supports it; retain torque."""
        if not self.execute:
            return
        try:
            if self.controller:
                self.controller.board.bus_servo_stop(list(range(1, 17)))
            elif hasattr(self.old_board, 'stopBusServo'):
                for servo in range(1, 17):
                    self.old_board.stopBusServo(servo)
            else:
                print('[stop] SDK lacks servo stop; last commanded frame may finish')
        except Exception as exc:
            print('[stop] Servo stop failed:', exc)

    def head(self):
        self.current_kind, self.current_action = 'head', 'head positioning'
        if isinstance(getattr(self, 'guard', None), VisionGuard):
            self.guard.phase = 'head'
        self.check_safety()
        if not self.execute:
            return
        head = dict(self.config['head'])
        # Phase-specific pitch is calibrated in PWM microseconds, not bus units.
        phase = getattr(self, 'head_phase', 'pickup')
        head['tilt'] = head.get(phase + '_tilt', head['tilt'])
        commands = [(head['tilt_id'], head['tilt'])]
        if not head.get('preserve_pan', False):
            commands.insert(0, (head['pan_id'], head['pan']))
        for servo, pulse in commands:
            if not 500 <= pulse <= 2500:
                raise ValueError('PWM head pulse outside 500..2500')
        for servo, pulse in commands:
            if self.controller:
                self.controller.set_pwm_servo_pulse(servo, pulse, 500)
            else:
                self.old_board.setPWMServoPulse(servo, pulse, 500)
        self.wait(0.6)

    def action(self, kind):
        self.current_kind, self.current_action = kind, self.config['actions'][kind]
        if isinstance(getattr(self, 'guard', None), VisionGuard):
            self.guard.phase = kind
        self.check_safety()
        name = self.config['actions'][kind]
        print('[action]', kind, name)
        if not self.execute:
            self.wait(0.01)
            return
        for frame in self.frames[name]:
            self.check_safety()
            if kind in ('forward', 'left', 'right'):
                check_hold(self.config)
            # Official TonyPi 1.20 diagram identifies 16 body bus servos.
            # Some local d6a files also contain 17/18 columns; keep them in the
            # parser but do not send commands to unverified extra bus channels.
            for servo, pulse in enumerate(frame[2:18], 1):
                if kind in ('forward', 'left', 'right'):
                    pulse = self.hold_values.get(servo, pulse)
                if self.controller:
                    self.controller.set_bus_servo_pulse(servo, pulse, frame[1])
                else:
                    self.old_board.setBusServoPulse(servo, pulse, frame[1])
            self.wait(frame[1] / 1000.0)
        if kind == 'grab':
            last = self.frames[name][-1]
            self.hold_values = {servo: last[servo + 1] for servo in self.config.get('hold_arm_servos', [])}
        elif kind in ('release', 'retry_reset'):
            self.hold_values = {}
        # Keep the final pose. Never insert a generic stand action while carrying.


class Camera(object):
    """Continuously drain camera so actions cannot leave old frames queued."""
    def __init__(self, source, stop):
        import cv2
        self.cv2 = cv2
        self.stop = stop
        self.lock = threading.Lock()
        self.frame, self.stamp = None, None
        if isinstance(source,dict):
            if source.get('type')!='http_snapshot':
                raise ValueError('Unsupported camera source type')
            from snapshot_camera import SnapshotCapture
            self.cam=SnapshotCapture(source,cv2)
        elif source == 'auto':
            try:
                module = importlib.import_module('hiwonder.Camera')
            except ImportError:
                self.cam = cv2.VideoCapture(0)
            else:
                self.cam = module.Camera()
                self.cam.camera_open()
        else:
            self.cam = cv2.VideoCapture(int(source) if str(source).isdigit() else source)
        self.worker = threading.Thread(target=self.run)
        self.worker.daemon = True
        self.worker.start()

    def run(self):
        while not self.stop.is_set():
            ok, frame = self.cam.read()
            if ok and frame is not None:
                with self.lock:
                    self.frame, self.stamp = frame.copy(), time.monotonic()
            else:
                self.stop.wait(0.02)

    def read_after(self, stamp):
        with self.lock:
            if self.stamp is None or self.stamp <= stamp:
                return None, None
            return self.frame.copy(), self.stamp

    def close(self):
        self.stop.set()
        # Close to unblock a native capture read before joining.
        if hasattr(self.cam, 'camera_close'):
            self.cam.camera_close()
        else:
            self.cam.release()
        self.worker.join(1)


def detect_details(frame, config):
    import cv2
    import numpy as np
    image = cv2.resize(frame, (320, 240))
    space = config.get('color_space', 'LAB').upper()
    if space not in ('LAB', 'HSV'):
        raise ValueError('color_space must be LAB or HSV')
    converted = cv2.cvtColor(cv2.GaussianBlur(image, (3, 3), 0),
                             cv2.COLOR_BGR2HSV if space == 'HSV' else cv2.COLOR_BGR2LAB)
    suffix = space.lower()
    mask = threshold(converted, config)
    kernel = np.ones((3, 3), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    valid = [c for c in contours if config['min_area'] <= cv2.contourArea(c) / (320 * 240.0) <= config['max_area']]
    if not valid:
        return None, mask, []
    boxes = []
    for candidate in valid:
        x, y, w, h = cv2.boundingRect(candidate)
        boxes.append({'bbox': [x / 320.0, y / 240.0, w / 320.0, h / 240.0],
                      'area': cv2.contourArea(candidate) / (320 * 240.0)})
    contour = max(valid, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(contour)
    return ((x + w / 2.0) / 320, (y + h) / 240.0, cv2.contourArea(contour) / (320 * 240.0)), mask, boxes


def detect(frame, config):
    # Preserve the decision API; boxes are only extra metadata for rendering.
    observation, mask, unused = detect_details(frame, config)
    return observation, mask


def check_hold(config):
    path = config.get('hold_sensor_file')
    if path:
        if not os.path.isfile(path) or not 0 <= time.time() - os.path.getmtime(path) <= config['hold_sensor_timeout']:
            raise RuntimeError('Grasp sensor missing/stale; holding at current pose')
        if Path(path).read_text().strip() != 'held':
            raise RuntimeError('Object dropped/not held; return to drop location before resuming')


def carry(config, motion, heartbeat, feedback):
    heartbeat.set('carrying')
    lane = config.get('lane_following', {})
    endpoint_confirmed = False
    if lane.get('enabled') and not getattr(feedback, 'simulate', False):
        motion.head_phase = 'carry'
        motion.head()
        endpoint_confirmed = lane_control.run(lane, motion, feedback, heartbeat)
    for step in ([] if lane.get('enabled') and not getattr(feedback, 'simulate', False) else config['route']):
        if step['kind'] == 'wait':
            remaining = float(step['seconds'])
            while remaining > 0:
                check_hold(config)
                duration = min(0.1, remaining)
                motion.wait(duration)
                remaining -= duration
        else:
            for unused in range(step['cycles']):
                check_hold(config)
                motion.action(step['kind'])
                motion.wait(config['settle_seconds'])
                if config['feedback'].get('verify_carry', True):
                    heartbeat.set('checking_hold')
                    if not feedback.confirm('held'):
                        raise RuntimeError('Holding confirmation lost; stop for object recovery')
                    heartbeat.set('carrying')
    check_hold(config)
    heartbeat.set('checking_placement')
    if not (endpoint_confirmed and lane.get('release_at_endpoint', False)) and not feedback.confirm('placement_ready'):
        raise RuntimeError('Placement area not confirmed; keep object held')
    heartbeat.set('checking_hold')
    if not feedback.confirm('held'):
        raise RuntimeError('Object no longer held before release')
    heartbeat.set('releasing')
    motion.action('release')
    motion.wait(config['settle_seconds'])
    heartbeat.set('verifying_release')
    if not feedback.confirm('released'):
        raise RuntimeError('Release not confirmed; task remains incomplete')
    heartbeat.set('done')


def manual_start_and_carry(config, motion, heartbeat, feedback):
    """Operator positions block; stand/grab once, then the complete route."""
    heartbeat.set('initializing')
    motion.action('stand')
    motion.wait(config['settle_seconds'])
    heartbeat.set('grasping')
    motion.action('grab')
    motion.wait(config['settle_seconds'])
    heartbeat.set('verifying_grasp')
    if not feedback.confirm('held'):
        heartbeat.set('grasp_failed')
        raise RuntimeError('Manual-start grab not confirmed; reposition object before restarting')
    heartbeat.set('grasp_success')
    carry(config, motion, heartbeat, feedback)


def grasp_and_carry(config, motion, heartbeat, approach, feedback):
    for attempt in range(1, config['grasp_attempts'] + 1):
        heartbeat.set('searching')
        approach()  # Re-detect and align after every reset; never blindly re-grab.
        heartbeat.set('grasping')
        print('[grasp attempt]', attempt, '/', config['grasp_attempts'])
        view = getattr(motion, 'debug_view', None)
        if view:
            view.update('Grasp attempt %d/%d' % (attempt, config['grasp_attempts']))
        motion.action('grab')
        motion.wait(config['settle_seconds'])
        heartbeat.set('verifying_grasp')
        if feedback.confirm('held'):
            heartbeat.set('grasp_success')
            carry(config, motion, heartbeat, feedback)
            return
        heartbeat.set('grasp_failed')
        if attempt == config['grasp_attempts']:
            raise RuntimeError('Grasp attempts exhausted; route not started')
        heartbeat.set('retrying')
        motion.action('retry_reset')
        motion.wait(config['settle_seconds'])


def approach_object(config, motion, camera, heartbeat, debug=False, simulate=False):
    planner = Approach(config['vision'])
    if simulate:
        for unused in range(config['vision']['stable_frames']):
            decision = planner.decide((config['vision']['center_x'], .95, .1))
        if decision != 'grab':
            raise RuntimeError('simulation did not reach grab state')
        return
    motion.head()
    deadline = time.monotonic() + config['search_timeout']
    last, actions = time.monotonic(), 0
    while not motion.stop.is_set():
        motion.check_safety()
        now = time.monotonic()
        if now >= deadline:
            raise RuntimeError('Object search/approach timed out')
        frame, stamp = camera.read_after(last)
        if frame is None:
            if now - last > config['camera_timeout']:
                raise RuntimeError('Camera frames missing/stale')
            motion.wait(.02)
            continue
        last = stamp
        observation, mask = detect(frame, config['vision'])
        decision = planner.decide(observation)
        if getattr(motion, 'debug_view', None):
            motion.debug_view.update('Decision=%s stable=%d/%d' % (decision, planner.ready, config['vision']['stable_frames']))
        if decision == 'grab':
            return
        if decision != 'wait':
            actions += 1
            if actions > config['approach_action_limit']:
                raise RuntimeError('Approach action limit exceeded')
            motion.action(decision)
            motion.wait(config['settle_seconds'])
            last = time.monotonic()
            planner.ready, planner.last = 0, None
    raise Stopped('stop requested')


def preview_vision(config, motion, camera, debug):
    last, printed = time.monotonic(), 0
    while not motion.stop.is_set():
        motion.check_safety()
        frame, stamp = camera.read_after(last)
        if frame is None:
            if time.monotonic() - last > config['camera_timeout']:
                raise RuntimeError('Camera frames missing/stale')
            motion.wait(.02)
            continue
        last = stamp
        observation, mask = detect(frame, config['vision'])
        if stamp - printed > .5:
            print('[%s observation x,bottom,area]' % config['vision'].get('target_color', 'target'), observation)
            guard = config['obstacle_guard']
            if guard.get('mode') == 'vision' and isinstance(guard.get('roi'), list):
                try:
                    blocked, occupancy, unknown = floor_obstacle(frame, guard, config['vision'],
                                                                 config['feedback'].get('held_roi'), 'forward')
                    print('[camera corridor blocked / occupancy]', blocked, round(occupancy, 3))
                except RuntimeError as exc:
                    print('[camera corridor]', exc)
            printed = stamp


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=str(ROOT / 'humanoid/config/competition.yaml'))
    parser.add_argument('--execute', action='store_true', help='send servo commands after calibration checks')
    parser.add_argument('--simulate', action='store_true', help='synthetic vision and printed route, no camera/hardware')
    parser.add_argument('--debug', action='store_true')
    parser.add_argument('--vision-only', action='store_true', help='preview target and corridor recognition without running actions')
    parser.add_argument('--check-actions', action='store_true', help='validate configured Pi action files without importing hardware SDK')
    args = parser.parse_args()
    if args.vision_only and (args.execute or args.simulate):
        parser.error('--vision-only cannot be combined with --execute or --simulate')
    import yaml
    with open(args.config, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    validate_route(config)
    startup_mode = config.get('startup_mode', 'automatic')
    if startup_mode not in ('automatic', 'manual'):
        parser.error('startup_mode must be automatic or manual')
    if startup_mode == 'manual' and not config['actions'].get('stand'):
        parser.error('manual startup requires actions.stand')
    if any(not isinstance(i, int) or not 1 <= i <= 16 for i in config.get('hold_arm_servos', [])):
        parser.error('hold_arm_servos must contain bus servo IDs 1..16')
    if args.check_actions:
        names = set(validate_route(config))
        if startup_mode == 'manual':
            names.add(config['actions']['stand'])
        if config['actions'].get('retry_reset'):
            names.add(config['actions']['retry_reset'])
        for name in sorted(names):
            rows = action_frames(Path(config['action_dir']) / (name + '.d6a'))
            print(name, 'frames=', len(rows), 'seconds=', sum(row[1] for row in rows) / 1000.0)
        return
    if args.execute and args.simulate:
        parser.error('--simulate cannot be combined with --execute')
    if args.execute and config['hardware_calibrated'] is not True:
        parser.error('Record/test right turn and release, calibrate route, then set hardware_calibrated: true')
    if args.execute:
        try:
            validate_feedback(config)
            if config.get('lane_following', {}).get('enabled'):
                lane_control.validate(config['lane_following'])
            if config['obstacle_guard'].get('mode', 'distance') == 'distance':
                DistanceGuard(config['obstacle_guard']).check()
        except (ValueError, RuntimeError) as exc:
            parser.error(str(exc))
    if not args.execute:
        config['heartbeat']['enabled'] = False  # dry runs must not launch the real followers
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *unused: stop.set())
    signal.signal(signal.SIGTERM, lambda *unused: stop.set())
    motion = Motion(config, stop, args.execute)
    heartbeat = Heartbeat(config['heartbeat'])
    camera = None
    try:
        if not args.simulate:
            camera = Camera(config['camera'], stop)
            if args.debug:
                motion.debug_view = DebugView(camera, config, heartbeat, motion, detect_details, Stopped)
        if args.vision_only:
            heartbeat.set('vision_preview')
            preview_vision(config, motion, camera, args.debug)
            return
        if args.execute and config['obstacle_guard'].get('mode') == 'vision':
            deadline = time.monotonic() + config['camera_timeout']
            while camera.read_after(float('-inf'))[0] is None:
                if time.monotonic() >= deadline or stop.wait(.02):
                    raise RuntimeError('Camera unavailable before first motion')
            motion.guard = VisionGuard(config['obstacle_guard'], camera, config['vision'], config['feedback']['held_roi'])
            motion.check_safety()
        motion.head()
        feedback_config = dict(config['feedback'], camera_timeout=config['camera_timeout'])
        feedback = TaskFeedback(feedback_config, config['vision'], camera, motion, detect, args.simulate)
        if startup_mode == 'manual':
            manual_start_and_carry(config, motion, heartbeat, feedback)
        else:
            approach = lambda: approach_object(config, motion, camera, heartbeat, args.debug, args.simulate)
            grasp_and_carry(config, motion, heartbeat, approach, feedback)
        # Keep done heartbeat alive while Turtlebot finishes its trailing goal.
        if args.execute:
            print('Route done; keep this process running until all followers stop. Ctrl+C exits.')
            while not stop.wait(0.2):
                if getattr(motion, 'debug_view', None):
                    motion.debug_view.poll()
    except Stopped as exc:
        motion.halt()
        heartbeat.set('stopped')
        print(exc)
    except Exception:
        motion.halt()
        heartbeat.set('fault')
        time.sleep(0.25)
        raise
    finally:
        stop.set()
        if camera:
            camera.close()
        if args.debug:
            import cv2
            cv2.destroyAllWindows()
        heartbeat.close()


if __name__ == '__main__':
    main()
