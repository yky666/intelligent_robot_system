"""Humanoid task decisions; all coordinates normalized to image size."""
from .planning import finite


class Approach(object):
    def __init__(self, config):
        self.config = config
        self.ready = 0
        self.last = None

    def decide(self, observation):
        if observation is None:
            self.ready = 0
            self.last = None
            return 'wait'
        x, bottom, area = observation
        if not all(finite(v) and 0 <= v <= 1 for v in observation):
            self.ready = 0
            self.last = None
            return 'wait'
        error = x - self.config['center_x']
        if abs(error) > self.config['x_tolerance']:
            self.ready = 0
            self.last = None
            return 'align_right' if error > 0 else 'align_left'
        if bottom < self.config['grab_bottom'] or area < self.config['grab_area']:
            self.ready = 0
            self.last = None
            return 'approach'
        if self.last is not None and abs(x - self.last[0]) + abs(bottom - self.last[1]) > self.config['stability_tolerance']:
            self.ready = 0
        self.last = observation
        self.ready += 1
        return 'grab' if self.ready >= self.config['stable_frames'] else 'wait'


def validate_route(config):
    actions = config['actions']
    required = set(['grab', 'release', 'approach', 'align_left', 'align_right'])
    route = config['route']
    if not route or not all(step['kind'] in ('forward', 'left', 'right', 'wait') for step in route):
        raise ValueError('route supports forward/left/right/wait only')
    for step in route:
        if step['kind'] == 'wait':
            if not finite(float(step['seconds'])) or step['seconds'] < 0:
                raise ValueError('wait seconds must be finite and nonnegative')
        else:
            count = step['cycles']
            if isinstance(count, bool) or not isinstance(count, int) or count <= 0:
                raise ValueError('cycles must be positive integers')
            required.add(step['kind'])
    if not {'left', 'right'}.issubset(set(step['kind'] for step in route)):
        raise ValueError('competition requires both left and right turns')
    for name in required:
        if not actions.get(name):
            raise ValueError('missing action mapping: ' + name)
    return sorted(set(actions[name] for name in required))
