"""Fresh, phase-specific feedback and distance guard; no hardware imports."""
import json
import math
from pathlib import Path
import time


class FeedbackUnavailable(RuntimeError):
    pass


class CollisionBlocked(RuntimeError):
    pass


def read_sample(path, max_age, after=None, now=None):
    now = time.time() if now is None else now
    try:
        data = json.loads(Path(path).read_text(encoding='utf-8'))
        stamp = data['timestamp']
        if isinstance(stamp, bool) or not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
            raise ValueError('invalid timestamp')
        if not 0 <= now - stamp <= max_age or (after is not None and stamp <= after):
            raise ValueError('stale/future/pre-action sample')
        return data
    except (OSError, ValueError, TypeError, KeyError) as exc:
        raise FeedbackUnavailable('Missing/invalid/stale feedback at %s: %s' % (path, exc))


class DistanceGuard:
    def __init__(self, config):
        self.config = config

    def check(self):
        try:
            data = read_sample(self.config['file'], self.config['max_age'])
            if data.get('valid', True) is not True:
                raise ValueError('sensor reports invalid sample')
            distance = data['distance_m']
            if isinstance(distance, bool) or not isinstance(distance, (float, int)) or not math.isfinite(distance) or distance <= 0:
                raise ValueError('invalid distance')
            if distance <= self.config['stop_distance_m']:
                raise CollisionBlocked('Obstacle too close: %.3f m; motion blocked' % distance)
        except (FeedbackUnavailable, KeyError, ValueError) as exc:
            raise CollisionBlocked('Distance feedback unavailable; motion blocked: %s' % exc)


def roi_present(frame, roi, vision, detect):
    height, width = frame.shape[:2]
    x0, y0, x1, y1 = roi
    crop = frame[int(y0 * height):int(y1 * height), int(x0 * width):int(x1 * width)]
    if crop.size == 0:
        raise FeedbackUnavailable('ROI is empty for this camera frame')
    return detect(crop, vision)[0] is not None


class TaskFeedback:
    def __init__(self, config, vision, camera, motion, detect, simulate=False):
        self.config = config
        self.vision = dict(vision, min_area=config.get('roi_min_area', .03),
                           max_area=config.get('roi_max_area', .95))
        self.camera, self.motion, self.detect = camera, motion, detect
        self.simulate = simulate

    def confirm(self, event):
        """Require consecutive DISTINCT samples captured after entering this phase.

        False means valid observations failed to confirm, not missing sensing.
        Release needs both empty hands and blue block in placement ROI.
        """
        if self.simulate:
            print('[feedback simulated]', event)
            return True
        spec = self.config[event]
        view = getattr(self.motion, 'debug_view', None)
        if view:
            view.update('Feedback %s: waiting for NEW samples 0/%d' % (event, spec['stable_samples']))
        after_wall, last_camera = time.time(), time.monotonic()
        deadline = last_camera + spec['timeout']
        consecutive, valid = 0, 0
        last_sample = after_wall
        while time.monotonic() < deadline:
            self.motion.check_safety()
            if spec['mode'] == 'file':
                try:
                    sample = read_sample(spec['file'], spec['max_age'], after=last_sample)
                except FeedbackUnavailable:
                    self.motion.wait(.02)
                    continue
                if sample.get('state') not in ('held', 'not_held', 'released', 'not_released',
                                                'placement_ready', 'not_ready'):
                    self.motion.wait(.02)
                    continue
                last_sample = sample['timestamp']
                positive = sample.get('state') == event
            else:
                frame, stamp = self.camera.read_after(last_camera)
                if frame is None:
                    if time.monotonic() - last_camera > self.config['camera_timeout']:
                        raise FeedbackUnavailable('No fresh camera frames during ' + event)
                    self.motion.wait(.02)
                    continue
                last_camera = stamp
                held = roi_present(frame, self.config['held_roi'], self.vision, self.detect)
                positive = held if event == 'held' else (
                    not held and roi_present(frame, self.config['placed_roi'], self.vision, self.detect))
            valid += 1
            consecutive = consecutive + 1 if positive else 0
            view = getattr(self.motion, 'debug_view', None)
            if view:
                view.update('Feedback %s: %s %d/%d' % (event, 'yes' if positive else 'no',
                                                       consecutive, spec['stable_samples']))
            if consecutive >= spec['stable_samples']:
                print('[feedback confirmed]', event)
                return True
            self.motion.wait(.02)
        if not valid:
            raise FeedbackUnavailable('No valid post-action feedback during ' + event)
        if spec['mode'] == 'file' and time.time() - last_sample > spec['max_age']:
            raise FeedbackUnavailable('Feedback stopped updating during ' + event)
        return False


def validate_feedback(config):
    feedback = config['feedback']
    low, high = feedback.get('roi_min_area', .03), feedback.get('roi_max_area', .95)
    if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in (low, high)) or not 0 < low < high <= 1:
        raise ValueError('Invalid ROI contour area fractions')
    for event in ('held', 'released', 'placement_ready'):
        spec = feedback[event]
        if spec['mode'] not in ('vision', 'file') or (event == 'placement_ready' and spec['mode'] != 'file'):
            raise ValueError('Unsupported feedback mode for ' + event)
        for key in ('timeout', 'max_age'):
            if not isinstance(spec[key], (int, float)) or not math.isfinite(spec[key]) or spec[key] <= 0:
                raise ValueError('Invalid ' + event + '.' + key)
        if isinstance(spec['stable_samples'], bool) or not isinstance(spec['stable_samples'], int) or spec['stable_samples'] <= 0:
            raise ValueError('stable_samples must be a positive integer')
        if spec['mode'] == 'file' and not spec['file']:
            raise ValueError('Feedback file required for ' + event)
    if any(feedback[e]['mode'] == 'vision' for e in ('held', 'released')):
        for key in ('held_roi', 'placed_roi'):
            roi = feedback[key]
            if not isinstance(roi, list) or len(roi) != 4 or not all(
                    isinstance(v, (int, float)) and math.isfinite(v) and 0 <= v <= 1 for v in roi):
                raise ValueError('Calibrate normalized [x0,y0,x1,y1] ' + key)
            if roi[0] >= roi[2] or roi[1] >= roi[3]:
                raise ValueError('Invalid ROI bounds: ' + key)
        held, placed = feedback['held_roi'], feedback['placed_roi']
        if min(held[2], placed[2]) > max(held[0], placed[0]) and min(held[3], placed[3]) > max(held[1], placed[1]):
            raise ValueError('Held and placed ROIs must not overlap')
    attempts = config['grasp_attempts']
    if isinstance(attempts, bool) or not isinstance(attempts, int) or not 1 <= attempts <= 5:
        raise ValueError('grasp_attempts must be 1..5')
    if not config['actions'].get('retry_reset'):
        raise ValueError('retry_reset action required')
    guard = config['obstacle_guard']
    mode = guard.get('mode', 'distance')
    if guard.get('enabled') is not True or mode not in ('vision', 'distance'):
        raise ValueError('Hardware execution requires vision or distance obstacle guard')
    if mode == 'distance' and not guard.get('file'):
        raise ValueError('Distance mode requires a live sensor file')
    keys = ('max_age', 'poll_seconds') + (('stop_distance_m',) if mode == 'distance' else ())
    for key in keys:
        if not isinstance(guard[key], (int, float)) or not math.isfinite(guard[key]) or guard[key] <= 0:
            raise ValueError('Invalid obstacle guard ' + key)
    if guard['poll_seconds'] > .1:
        raise ValueError('Guard polling must be <= 0.1 seconds')
    if mode == 'vision':
        for key in ('roi', 'interaction_roi'):
            roi = guard.get(key)
            if key == 'interaction_roi' and roi is None:
                continue
            if not isinstance(roi, list) or len(roi) != 4 or not all(
                    isinstance(v, (int, float)) and math.isfinite(v) and 0 <= v <= 1 for v in roi
                    ) or roi[0] >= roi[2] or roi[1] >= roi[3]:
                raise ValueError('Calibrate camera obstacle ' + key)
        for key in ('floor_lower_hsv', 'floor_upper_hsv'):
            values = guard.get(key)
            if not isinstance(values, list) or len(values) != 3 or any(
                    isinstance(v, bool) or not isinstance(v, int) or not 0 <= v <= limit
                    for v, limit in zip(values, (179, 255, 255))):
                raise ValueError('Calibrate ' + key)
        if any(a > b for a, b in zip(guard['floor_lower_hsv'], guard['floor_upper_hsv'])):
            raise ValueError('Floor HSV lower exceeds upper')
        for key in ('contour_fraction', 'occupancy_fraction', 'min_visible_fraction',
                    'edge_fraction', 'edge_contour_height'):
            value = guard[key]
            if not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 < value <= 1:
                raise ValueError('Invalid camera guard ' + key)
