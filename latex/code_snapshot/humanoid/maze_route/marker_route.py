#!/usr/bin/env python3
"""Marker route for TonyPi: walk to marker A, turn left; to B, turn right;
walk to C and stop. While walking, the target marker is kept in the middle of
the image with a turn whenever it drifts off centre.

Actions are the copies in ./actions (base actions with grab_final_v2 hands):
    catch_ball_go            one walking step (+ WALK_PAUSE_S pause after it)
    catch_ball_turn_left/right   turning, at corners and for centring
    grab_final_v2            grab in place at the start (skip with --no-grab)
    put_down                 put the cube down at C (skip with --no-release)

Head: neutral (PWM pitch 1500, pan 1482) for the whole run, set once at start.

Desktop (default): a window shows the camera, the markers and the state.
    S start / resume    P pause after the current action    Q quit
    [ ] head pitch -/+ 20 (while not running)
Headless (no DISPLAY, or --headless): starts after --start-delay seconds.
At a corner the robot keeps turning until the next marker is in view, then walks.
Every run logs to logs/route-<time>/: route.log, decisions.csv, snapshots/.

    python3 marker_route.py [--no-grab] [--dry-run] [--turn-cm 40] [--stop-cm 40]
"""
import argparse
import csv
import glob
import json
import logging
import math
import os
import queue
import signal
import sqlite3
import subprocess
import sys
import threading
import time
from contextlib import closing
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, '/home/pi/TonyPi/Functions')            # fallback for robust_camera
sys.path.insert(0, str(HERE))                              # bundled robust_camera.py first
import markers  # noqa: E402

ACTIONS = HERE / 'actions'
CALIB = '/home/pi/TonyPi/Functions/CameraCalibration/calibration_param.npz'
ARMS = (6, 7, 8, 14, 15, 16)
SERVOS = range(1, 17)
BRIDGE_MS = 300          # first frame of an action is stretched to this when the pose jumps
BRIDGE_JUMP = 25         # ...if any servo has to move more than this
WALK_PAUSE_S = 0.2       # buffer between two walking steps
CORNER_PAUSE_S = 0.3     # buffer before, between and after the turns at a corner
SETTLE_S = 0.25          # wait after an action before trusting the camera
LOOK_FRAMES = 3          # fresh frames looked at per decision
ROUTE = [('A', 'left'), ('B', 'right'), ('C', 'stop')]
log = logging.getLogger('route')


# ---------------------------------------------------------------- servos
def serial_holders():
    out = []
    for proc in glob.glob('/proc/[0-9]*'):
        if int(proc[6:]) == os.getpid():
            continue
        try:
            for fd in os.listdir(proc + '/fd'):
                if os.readlink('%s/fd/%s' % (proc, fd)) == '/dev/ttyAMA0':
                    with open(proc + '/cmdline', 'rb') as f:
                        out.append((int(proc[6:]), f.read().replace(b'\0', b' ').decode(errors='replace').strip()))
                    break
        except OSError:
            continue
    return out


def take_serial():
    """Free the servo bus: close the factory GUI, then stop tonypi.service.
    The GUI runs `systemctl restart tonypi` when it exits, so the service is
    stopped after the GUI is gone, which also cancels that restart."""
    gui = [pid for pid, cmd in serial_holders() if 'TonyPi_PC_Software/main.py' in cmd]
    for pid in gui:
        log.warning('closing the factory GUI (PID %d)', pid)
        os.kill(pid, signal.SIGTERM)
    deadline = time.monotonic() + 5
    while gui and any(os.path.exists('/proc/%d' % pid) for pid in gui) and time.monotonic() < deadline:
        time.sleep(0.1)
    state = subprocess.run(['systemctl', 'is-active', 'tonypi.service'], capture_output=True, text=True).stdout.strip()
    if gui or state in ('active', 'activating', 'reloading'):
        log.warning('stopping tonypi.service (was %s)', state)
        subprocess.run(['sudo', '-n', 'systemctl', 'stop', 'tonypi.service'], timeout=20, check=False)
    deadline = time.monotonic() + 5
    while serial_holders() and time.monotonic() < deadline:
        time.sleep(0.2)
    left = serial_holders()
    if left:
        raise SystemExit('servo bus still in use: %s' % '; '.join('PID %d %s' % h for h in left))


class Servos:
    """Plays d6a frames; arms locked; stretches the first frame when the pose jumps."""

    def __init__(self, fake=False):
        self.fake, self.pose = fake, None
        if not fake:
            take_serial()
            import hiwonder.ros_robot_controller_sdk as rrc
            self.board = rrc.Board()
            self.board.enable_reception(True)
            time.sleep(0.2)
            try:
                self.pose = {i: int(self.read(i)) for i in SERVOS}
            except Exception as exc:
                log.warning('could not read the current pose (%s); first frame will be stretched', exc)

    def read(self, sid):
        r = queue.Queue()
        threading.Thread(target=lambda: r.put(self.board.bus_servo_read_position(sid)), daemon=True).start()
        v = r.get(timeout=0.6)
        return v[0] if isinstance(v, (list, tuple)) else v

    def head(self, pitch, pan, ms=500):
        log.info('head PWM pitch %d pan %d%s', pitch, pan, ' (no servos: not sent)' if self.fake else '')
        if not self.fake:
            self.board.pwm_servo_set_position(ms / 1000, [[1, pitch], [2, pan]])
        time.sleep(ms / 1000)

    def play(self, name, lock, stop_ev):
        with closing(sqlite3.connect('file:%s?mode=ro' % (ACTIONS / (name + '.d6a')), uri=True)) as db:
            frames = db.execute('select * from ActionGroup order by "Index"').fetchall()
        for k, f in enumerate(frames):
            if stop_ev.is_set():
                log.warning('%s interrupted after frame %d/%d', name, k, len(frames))
                return False
            target = {i: lock.get(i, f[1 + i]) for i in SERVOS}
            ms = f[1]
            if k == 0 and (self.pose is None or max(abs(target[i] - self.pose[i]) for i in SERVOS) > BRIDGE_JUMP):
                ms = max(ms, BRIDGE_MS)
            if not self.fake:
                self.board.bus_servo_set_position(ms / 1000, [[i, target[i]] for i in SERVOS])
            self.pose = target
            time.sleep(ms / 1000)
        return True


# ---------------------------------------------------------------- vision
class Eyes:
    """Camera thread: latest frame + marker detections, with a sequence number."""

    def __init__(self):
        import robust_camera
        p = np.load(CALIB)
        self.det = markers.MarkerDetector(p['mtx_array'], p['dist_array'])
        self.cam = robust_camera.Camera()
        self.cam.camera_open()
        self.lock = threading.Lock()
        self.frame, self.found, self.seq = None, {}, 0
        self.last_stats = (0, 0)        # (frames with the target, frames looked at) of the last look()
        self.quit = threading.Event()
        threading.Thread(target=self.loop, daemon=True).start()

    def loop(self):
        last = None
        while not self.quit.is_set():
            ok, img = self.cam.read()
            if not ok or img is last:
                time.sleep(0.01)
                continue
            last = img
            found = self.det.detect(img)
            with self.lock:
                self.frame, self.found, self.seq = img, found, self.seq + 1

    def latest(self):
        with self.lock:
            return self.frame, self.found, self.seq

    def look(self, target, n=LOOK_FRAMES, timeout=2.0):
        """Median measurement of `target` over n fresh frames, or None."""
        _, _, seq0 = self.latest()
        seen, frames, end = [], 0, time.monotonic() + timeout
        while frames < n and time.monotonic() < end:
            _, found, seq = self.latest()
            if seq > seq0:
                seq0, frames = seq, frames + 1
                if target in found:
                    seen.append(found[target])
            time.sleep(0.01)
        self.last_stats = (len(seen), frames)
        if frames < n:
            return 'no-camera'
        if len(seen) * 2 <= n:          # need a majority of the frames
            return None
        return dict(dist_cm=float(np.median([m['dist_cm'] for m in seen])),
                    bearing_deg=float(np.median([m['bearing_deg'] for m in seen])),
                    yaw_deg=float(np.median([m['yaw_deg'] for m in seen if m['yaw_deg'] is not None] or [0])),
                    seen=len(seen), of=frames)

    def close(self):
        self.quit.set()
        self.cam.camera_close()


# ---------------------------------------------------------------- route
class Route(threading.Thread):
    def __init__(self, a, servos, eyes, run_dir, interactive):
        super().__init__(daemon=True)
        self.a, self.servos, self.eyes, self.run_dir = a, servos, eyes, run_dir
        self.interactive = interactive     # a window to press S in; headless runs end instead of pausing
        self.go = threading.Event()        # set = running, clear = paused
        self.stop_ev = threading.Event()   # quit
        self.status = 'waiting for start'
        self.phase = 0
        self.lock_arms = {}
        self.snaps = 0
        # One catch_ball_turn turns a fixed, fairly large angle. Turning only
        # helps when the error is larger than about half of it, otherwise the
        # marker just ends up on the other side. The step is learnt online.
        self.turn_steps = [a.turn_step_deg]
        # Heading drift per walking step (+ = robot creeps left), learnt while a
        # marker is in view and used to correct blind walking.
        self.drift = []
        f = (run_dir / 'decisions.csv').open('w', newline='', encoding='utf-8')
        self.csv_file, self.csv = f, csv.writer(f)
        self.csv.writerow(['time', 'phase', 'target', 'seen', 'dist_cm', 'bearing_deg', 'yaw_deg', 'action', 'note'])

    def set_status(self, s):
        self.status = s
        log.info(s)

    def record(self, target, obs, action, note=''):
        o = obs if isinstance(obs, dict) else dict(zip(('seen', 'of'), self.eyes.last_stats))
        r = lambda v: '' if v is None else round(v, 1)
        self.csv.writerow([time.strftime('%H:%M:%S'), self.phase, target, '%d/%d' % (o.get('seen', 0), o.get('of', 0)),
                           r(o.get('dist_cm')), r(o.get('bearing_deg')), r(o.get('yaw_deg')), action, note])
        self.csv_file.flush()
        frame, found, _ = self.eyes.latest()
        if frame is not None and self.snaps < self.a.max_snapshots:
            vis = overlay(frame.copy(), found, target, '%s -> %s %s' % (target, action, note))
            cv2.imwrite(str(self.run_dir / 'snapshots' / ('%04d_%s_%s.jpg' % (self.snaps, target, action))), vis,
                        [cv2.IMWRITE_JPEG_QUALITY, 80])
            self.snaps += 1

    def pause(self, why):
        """Window: wait for S. Headless: nobody can resume, so end the run."""
        if self.interactive:
            self.set_status(why + '; paused, press S to retry')
            self.go.clear()
            return self.wait_running()
        self.set_status(why + '; headless, stopping')
        self.stop_ev.set()
        return False

    def wait_running(self):
        while not self.go.is_set():
            if self.stop_ev.wait(0.1):
                return False
        return not self.stop_ev.is_set()

    def act(self, name, count=1):
        for _ in range(count):
            if not self.wait_running():
                return False
            if self.a.dry_run:
                log.info('[dry-run] would play %s', name)
                time.sleep(0.5)
            elif not self.servos.play(name, self.lock_arms, self.stop_ev):
                return False
        time.sleep(SETTLE_S)
        return True

    def run(self):
        try:
            self._run()
        except Exception:
            log.exception('route crashed')
            self.status = 'crashed, see route.log'
        finally:
            self.csv_file.close()

    def _run(self):
        a = self.a
        if not self.wait_running():
            return
        if a.grab:
            self.set_status('grabbing (grab_final_v2)')
            if not self.act('grab_final_v2'):
                return
        # hands as at the end of grab_final_v2: same values the action copies carry
        with closing(sqlite3.connect('file:%s?mode=ro' % (ACTIONS / 'grab_final_v2.d6a'), uri=True)) as db:
            last = db.execute('select * from ActionGroup order by "Index" desc limit 1').fetchone()
        self.lock_arms = {i: last[1 + i] for i in ARMS}
        log.info('arms locked at %s', self.lock_arms)

        for self.phase, (target, then) in enumerate(ROUTE):
            steps = turns = misses = blind = 0
            progress = []            # distance to the target before each walking step
            seen_once = False
            prev_walk_obs = None     # marker seen right before the last walking step
            last_seen = None         # latest measurement of the target
            creep = 0.0              # predicted heading error while walking blind
            limit = a.turn_cm if then != 'stop' else a.stop_cm
            self.set_status('phase %d: walking to marker %s (%s at %.0f cm)' % (self.phase + 1, target, then, limit))
            while True:
                if self.stop_ev.is_set():
                    return
                obs = self.eyes.look(target)
                if obs == 'no-camera':
                    # The USB camera drops out and comes back by itself: stand
                    # still and wait for it rather than walking blind.
                    self.record(target, None, 'wait', 'no camera')
                    t0 = time.monotonic()
                    while self.eyes.look(target, n=1, timeout=1) == 'no-camera':
                        if self.stop_ev.is_set():
                            return
                        if int(time.monotonic() - t0) % 5 == 0:
                            self.set_status('no camera frames for %.0f s; standing still' % (time.monotonic() - t0))
                        if time.monotonic() - t0 > self.a.camera_wait:
                            if not self.pause('no camera for %.0f s' % self.a.camera_wait):
                                return
                            t0 = time.monotonic()
                    log.info('camera back after %.1f s', time.monotonic() - t0)
                    continue
                if obs and prev_walk_obs and abs(obs['bearing_deg']) < 15 and abs(prev_walk_obs['bearing_deg']) < 15:
                    self.drift.append(obs['bearing_deg'] - prev_walk_obs['bearing_deg'])
                prev_walk_obs = None
                # Close up, the marker slides out of the bottom of the image (head at
                # neutral: below ~37 cm). Lost right next to the goal, centred = arrived.
                arrived_blind = (obs is None and last_seen is not None and
                                 last_seen['dist_cm'] <= limit + a.lost_near_cm and
                                 abs(last_seen['bearing_deg']) <= max(a.centre_deg, 15))
                if arrived_blind:
                    log.info('%s lost at the goal (last seen %.0f cm, %+.0f deg): treating as reached',
                             target, last_seen['dist_cm'], last_seen['bearing_deg'])
                    obs = dict(last_seen, dist_cm=min(last_seen['dist_cm'], limit), lost_close=True)
                if obs:
                    seen_once, misses = True, 0
                    if not obs.get('lost_close'):
                        last_seen = obs
                    if obs['dist_cm'] <= limit:
                        if then == 'stop':
                            self.record(target, obs, 'stop', 'reached')
                            self.set_status('reached marker C at %.0f cm: stopped' % obs['dist_cm'])
                            if a.release:
                                time.sleep(CORNER_PAUSE_S)
                                self.set_status('putting the cube down (put_down)')
                                self.lock_arms = {}          # put_down moves the hands itself
                                if not self.act('put_down'):
                                    return
                                self.record(target, None, 'put_down', 'done')
                                self.set_status('reached marker C, cube put down: finished')
                            return
                        self.record(target, obs, 'corner', 'turn %s' % then)
                        if not self.corner(then, ROUTE[self.phase + 1][0], obs):
                            return
                        break
                    step = float(np.median(self.turn_steps[-5:]))
                    need = max(a.centre_deg, 0.6 * step)
                    if abs(obs['bearing_deg']) > need:
                        side = 'right' if obs['bearing_deg'] > 0 else 'left'
                        turns += 1
                        self.record(target, obs, 'turn_' + side,
                                    'centre %.1f deg (> %.1f, step ~%.0f)' % (obs['bearing_deg'], need, step))
                        if turns > a.max_centre_turns:
                            turns = 0
                            if not self.pause('too many centring turns for %s' % target):
                                return
                            continue
                        if not self.act('catch_ball_turn_' + side):
                            return
                        after = self.eyes.look(target)
                        if isinstance(after, dict):
                            moved = abs(after['bearing_deg'] - obs['bearing_deg'])
                            if 3 < moved < 60:
                                self.turn_steps.append(moved)
                                log.info('one turn ~ %.1f deg (median %.1f)', moved, np.median(self.turn_steps[-5:]))
                        continue
                    turns = 0
                    action = 'walk'
                else:
                    misses += 1
                    if seen_once and misses > a.max_misses:
                        self.record(target, None, 'pause', 'marker lost')
                        misses = 0
                        if not self.pause('marker %s lost' % target):
                            return
                        continue
                    action = 'walk-blind'
                    blind += 1
                    if len(self.drift) >= 3:
                        step = float(np.median(self.turn_steps[-5:]))
                        creep += float(np.median(self.drift[-15:]))
                        if abs(creep) > 0.6 * step:
                            side = 'right' if creep > 0 else 'left'
                            self.record(target, None, 'turn_' + side, 'blind drift correction %.1f deg' % creep)
                            if not self.act('catch_ball_turn_' + side):
                                return
                            creep -= step if creep > 0 else -step
                    if not seen_once and blind > a.max_blind_steps:
                        self.record(target, None, 'pause', 'blind limit')
                        blind = 0
                        if not self.pause('%d steps without seeing %s' % (a.max_blind_steps, target)):
                            return
                        continue
                steps += 1
                # Stalled? With the marker in view, the distance must shrink by
                # STALL_CM within STALL_STEPS walking steps (turns do not count).
                if isinstance(obs, dict):
                    progress.append(obs['dist_cm'])
                stalled = (len(progress) > a.stall_steps and
                           progress[-a.stall_steps - 1] - min(progress[-a.stall_steps:]) < a.stall_cm)
                if stalled or steps > a.max_steps:
                    why = ('no progress to %s: %.0f cm in %d steps' % (target, progress[-a.stall_steps - 1] - min(progress[-a.stall_steps:]), a.stall_steps)
                           if stalled else '%d steps without reaching %s' % (a.max_steps, target))
                    self.record(target, obs, 'pause', why)
                    steps, progress = 0, []
                    if not self.pause(why):
                        return
                    continue
                self.record(target, obs, action)
                if not self.act('catch_ball_go'):
                    return
                prev_walk_obs = obs if isinstance(obs, dict) else None
                time.sleep(WALK_PAUSE_S)

    def corner(self, side, nxt, at=None):
        """Keep turning (300 ms buffers) until the next marker is in view; only
        then walk on. Centring continues in the walk loop."""
        while True:
            for k in range(1, self.a.max_corner_turns + 1):
                self.set_status('corner: turn %s %d, looking for %s' % (side, k, nxt))
                time.sleep(CORNER_PAUSE_S)            # buffer before each corner turn
                if not self.act('catch_ball_turn_' + side):
                    return False
                time.sleep(CORNER_PAUSE_S)            # and after it, before looking
                obs = self.eyes.look(nxt)
                t0 = time.monotonic()
                while obs == 'no-camera':             # do not keep turning blind
                    if self.stop_ev.is_set():
                        return False
                    self.set_status('corner: no camera frames for %.0f s; standing still' % (time.monotonic() - t0))
                    if time.monotonic() - t0 > self.a.camera_wait:
                        break
                    obs = self.eyes.look(nxt)
                if isinstance(obs, dict):
                    self.record(nxt, obs, 'found', 'after %d turn(s)' % k)
                    return True
                self.record(nxt, None, 'turn_' + side, 'corner turn %d, %s not in view' % (k, nxt))
            self.record(nxt, None, 'pause', 'next marker not found')
            if not self.pause('marker %s not found after %d turns' % (nxt, self.a.max_corner_turns)):
                return False      # resumed: keep turning and looking


# ---------------------------------------------------------------- display
def overlay(img, found, target, status):
    h, w = img.shape[:2]
    markers.draw(img, found, target)
    cv2.line(img, (w // 2, 0), (w // 2, h), (255, 255, 255), 1)
    cv2.rectangle(img, (0, 0), (w, 24), (0, 0, 0), -1)
    cv2.putText(img, status[:80], (6, 17), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
    return img


def setup_logging(run_dir):
    fmt = logging.Formatter('%(asctime)s.%(msecs)03d %(levelname)-7s %(name)s: %(message)s', '%H:%M:%S')
    root = logging.getLogger()
    root.setLevel(logging.INFO)
    for h in (logging.FileHandler(str(run_dir / 'route.log'), encoding='utf-8'), logging.StreamHandler()):
        h.setFormatter(fmt)
        root.addHandler(h)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--no-grab', dest='grab', action='store_false',
                   help='skip grab_final_v2 (by default the robot grabs in place, then walks)')
    p.add_argument('--no-release', dest='release', action='store_false',
                   help='stay holding the cube at C (by default it is put down with put_down)')
    p.add_argument('--dry-run', action='store_true', help='look and decide, do not walk or turn (head is still set)')
    p.add_argument('--no-servos', action='store_true', help='do not touch the servo bus at all (implies --dry-run)')
    p.add_argument('--headless', action='store_true', help='no window even if a desktop is available')
    p.add_argument('--start-delay', type=float, default=3, help='headless: seconds before starting')
    p.add_argument('--turn-cm', type=float, default=40, help='turn at a corner marker this close (cm)')
    p.add_argument('--stop-cm', type=float, default=40,
                   help='stop at the last marker this close (cm); with the head at neutral the marker '
                        'leaves the image below ~37 cm')
    p.add_argument('--lost-near-cm', type=float, default=12,
                   help='a marker lost within this much of its goal distance counts as reached')
    p.add_argument('--centre-deg', type=float, default=8, help='never re-centre for less than this bearing')
    p.add_argument('--turn-step-deg', type=float, default=20,
                   help='initial guess of one catch_ball_turn (refined while running)')
    # Head stays at neutral for the whole run (walking and looking): pitch at the
    # PWM mid point, pan at the servo_config centre. Set once, never moved.
    p.add_argument('--head-pitch', type=int, default=1500, help='PWM servo 1, neutral 1500 (bigger = head up)')
    p.add_argument('--head-pan', type=int, default=1482, help='PWM servo 2, servo_config centre')
    p.add_argument('--max-steps', type=int, default=150, help='walking steps per leg, last-resort limit')
    p.add_argument('--stall-steps', type=int, default=20, help='walking steps over which progress is checked')
    p.add_argument('--stall-cm', type=float, default=5, help='pause if the target got less than this much closer')
    p.add_argument('--max-misses', type=int, default=2, help='blind steps after losing a seen marker')
    p.add_argument('--max-centre-turns', type=int, default=4)
    p.add_argument('--max-corner-turns', type=int, default=12, help='turns at a corner before giving up (~360 deg)')
    p.add_argument('--max-blind-steps', type=int, default=25, help='steps before a marker must be seen')
    p.add_argument('--max-snapshots', type=int, default=500)
    p.add_argument('--camera-wait', type=float, default=60, help='seconds to wait for a lost camera')
    a = p.parse_args()

    run_dir = HERE / 'logs' / ('route-' + time.strftime('%Y%m%d-%H%M%S'))
    (run_dir / 'snapshots').mkdir(parents=True)
    setup_logging(run_dir)
    show = not a.headless and bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))
    log.info('start %s', json.dumps(vars(a)))
    log.info('mode: %s, logs in %s', 'window' if show else 'headless', run_dir)

    a.dry_run = a.dry_run or a.no_servos
    servos = Servos(fake=a.no_servos)
    pitch = a.head_pitch
    servos.head(pitch, a.head_pan)
    eyes = Eyes()
    route = Route(a, servos, eyes, run_dir, interactive=show)
    route.start()

    def quit_now(*_):
        route.stop_ev.set()
        route.go.set()
    signal.signal(signal.SIGTERM, quit_now)
    signal.signal(signal.SIGINT, quit_now)

    if not show:
        log.info('headless: starting in %.0f s (Ctrl-C to quit)', a.start_delay)
        if not route.stop_ev.wait(a.start_delay):
            route.go.set()
    try:
        while route.is_alive():
            if show:
                frame, found, _ = eyes.latest()
                if frame is not None:
                    state = ('RUN ' if route.go.is_set() else 'PAUSED ') + route.status
                    vis = overlay(frame.copy(), found, ROUTE[min(route.phase, 2)][0], state)
                    hint = 'S start  P pause  Q quit  [ ] head %d' % pitch
                    cv2.putText(vis, hint, (6, vis.shape[0] - 8), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 255), 1)
                    cv2.imshow('marker route', vis)
                k = cv2.waitKey(30) & 0xFF
                if k in (ord('q'), 27):
                    log.info('quit key')
                    quit_now()
                elif k == ord('s'):
                    log.info('start / resume key')
                    route.go.set()
                elif k == ord('p'):
                    log.info('pause key (after the current action)')
                    route.go.clear()
                elif k in (ord('['), ord(']')) and not route.go.is_set():
                    pitch += 20 if k == ord(']') else -20
                    servos.head(pitch, a.head_pan, 200)
            else:
                route.join(0.5)
    finally:
        route.stop_ev.set()
        route.go.set()
        route.join(5)
        eyes.close()
        if show:
            cv2.destroyAllWindows()
        log.info('finished: %s', route.status)


if __name__ == '__main__':
    main()
