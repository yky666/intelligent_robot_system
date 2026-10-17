"""Small ROS message adapters; geometry stays in independently tested modules."""
from .depth import finite
from .motion import Grid, yaw
from .motion import fresh
import math


class OdometryState(object):
    def __init__(self):
        self.stamp = self.received = None
        self.speed = self.angular = 0.
        self.forward = self.lateral = self.yaw_rate = 0.

    def update(self, msg, frame, base, now, wall, timeout):
        stamp = msg.header.stamp.to_sec()
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        values = (v.x, v.y, v.z, w.x, w.y, w.z)
        if (not same_frame(msg.header.frame_id, frame) or not same_frame(msg.child_frame_id, base) or
                not fresh(stamp, now, timeout) or (self.stamp is not None and stamp <= self.stamp) or
                not all(finite(x) for x in values)):
            # Malformed messages must not leave a formerly fresh velocity usable.
            self.received = None
            return False
        self.stamp, self.received = stamp, wall
        self.speed, self.angular = math.hypot(v.x, v.y), abs(w.z)
        self.forward, self.lateral, self.yaw_rate = v.x, v.y, w.z
        return True

    def valid(self, now, wall, timeout):
        return fresh(self.stamp, now, timeout) and fresh(self.received, wall, timeout)


def same_frame(left, right):
    return bool(left) and left.lstrip('/') == right.lstrip('/')


def grid_message(msg, frame):
    if not same_frame(msg.header.frame_id, frame):
        raise ValueError('costmap_frame_mismatch')
    p, q = msg.info.origin.position, msg.info.origin.orientation
    return Grid(msg.data, msg.info.width, msg.info.height, msg.info.resolution, (p.x, p.y, yaw(q)))


def transform_pose(transform):
    t, q = transform.transform.translation, transform.transform.rotation
    values = (t.x, t.y, t.z, q.x, q.y, q.z, q.w)
    if not all(finite(v) for v in values) or abs(sum(v*v for v in values[3:]) - 1.) > .1:
        raise ValueError('invalid_base_transform')
    return t.x, t.y, yaw(q)
