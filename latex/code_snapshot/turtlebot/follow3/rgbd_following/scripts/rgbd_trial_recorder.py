#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Passive, timestamped JSONL evidence. Never publishes motion commands."""
from __future__ import division
import json
import hashlib
import math
import os
import threading
import time
import rospy
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, String
from rgbd_following_core.motion import fresh, yaw
from rgbd_following_core.ros_helpers import same_frame, OdometryState
from rgbd_following_core.settings import validate


class Recorder(object):
    def __init__(self):
        self.c = validate(rospy.get_param('~settings'))
        self.frame = rospy.get_param('~odom_frame', 'odom')
        self.base = rospy.get_param('~base_frame', 'base_footprint')
        self.clock = getattr(time, 'monotonic', time.time)
        self.lock, self.data = threading.RLock(), {}
        self.odom = OdometryState()
        self.scenario = rospy.get_param('~scenario')
        path = os.path.abspath(os.path.expanduser(rospy.get_param('~output')))
        self.file = os.fdopen(os.open(path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644), 'w')
        self.write(dict(kind='metadata', schema=1, scenario=self.scenario, settings=self.c,
                        settings_sha256=hashlib.sha256(json.dumps(self.c,sort_keys=True).encode('utf-8')).hexdigest(),
                        revision=rospy.get_param('~revision','unspecified'), ros_distro=os.environ.get('ROS_DISTRO','unknown'),
                        odom_frame=self.frame, base_frame=self.base,
                        note='Costmap clearance is an estimate, not physical collision ground truth.'))
        self.subs = []
        for topic, typ in [('odom', Odometry), ('target_pose', PoseStamped), ('target_valid', Bool),
                           ('mission', String), ('follow_status', String), ('gate_status', String),
                           ('cmd_vel_observed', Twist), ('move_base/local_costmap/costmap', OccupancyGrid)]:
            self.subs.append(rospy.Subscriber(topic, typ, self.receive, callback_args=topic, queue_size=1))
        rospy.on_shutdown(self.close)
        self.timer = rospy.Timer(rospy.Duration(.1), self.tick)

    def write(self, item):
        self.file.write(json.dumps(item, allow_nan=False, sort_keys=True)+'\n')
        self.file.flush()

    def receive(self, msg, topic):
        with self.lock:
            self.data[topic] = (msg, self.clock())

    def message(self, topic, wall, timeout):
        msg, received = self.data.get(topic, (None, None))
        return msg if fresh(received, wall, timeout) else None

    def tick(self, unused):
        with self.lock:
            if self.file.closed:
                return
            now, wall = rospy.Time.now().to_sec(), self.clock()
            row = dict(kind='sample', t=now, wall=wall, scenario=self.scenario)
            for name in ('mission', 'follow_status', 'gate_status'):
                msg = self.message(name, wall, self.c['mission_timeout'])
                row[name] = msg.data if msg is not None else None
            msg = self.message('target_valid', wall, self.c['target_timeout'])
            row['target_valid'] = bool(msg.data) if msg is not None else False
            odom = self.message('odom', wall, self.c['odom_timeout'])
            if odom is not None:
                # Ignore duplicate sampling of the same odometry message.
                if self.odom.stamp != odom.header.stamp.to_sec():
                    self.odom.update(odom, self.frame, self.base, now, self.data['odom'][1], self.c['odom_timeout'])
            row['odom_fresh'] = self.odom.valid(now, wall, self.c['odom_timeout']) and odom is not None
            row['speed'] = self.odom.speed if row['odom_fresh'] else None
            row['angular_speed'] = self.odom.angular if row['odom_fresh'] else None
            row['target_distance_m'] = row['euclidean_gap_error_m'] = row['costmap_clearance_m'] = None
            target = self.message('target_pose', wall, self.c['target_timeout'])
            valid_target = (target is not None and same_frame(target.header.frame_id, self.frame) and
                            fresh(target.header.stamp.to_sec(), now, self.c['target_timeout']))
            row['target_valid'] = row['target_valid'] and valid_target
            grid = self.message('move_base/local_costmap/costmap', wall, self.c['grid_timeout'])
            row['grid_fresh'] = (grid is not None and same_frame(grid.header.frame_id, self.frame) and
                                 fresh(grid.header.stamp.to_sec(), now, self.c['grid_timeout']))
            if row['odom_fresh']:
                p = odom.pose.pose.position
                if row['target_valid']:
                    t = target.pose.position
                    row['target_distance_m'] = math.hypot(t.x-p.x, t.y-p.y)
                    row['euclidean_gap_error_m'] = abs(row['target_distance_m']-self.c['follow_distance'])
                if row['grid_fresh']:
                    origin, res = grid.info.origin, grid.info.resolution
                    angle = yaw(origin.orientation)
                    c, s = math.cos(angle), math.sin(angle)
                    distances = []
                    for i, value in enumerate(grid.data):
                        if value == 100:
                            x, y = (i % grid.info.width+.5)*res, (i // grid.info.width+.5)*res
                            distances.append(math.hypot(origin.position.x+c*x-s*y-p.x,
                                                        origin.position.y+s*x+c*y-p.y)-self.c['footprint_radius']-res*math.sqrt(2)/2)
                    row['costmap_clearance_m'] = min(distances) if distances else None
            cmd = self.message('cmd_vel_observed', wall, .3)
            row['command_linear'] = cmd.linear.x if cmd is not None else None
            row['command_angular'] = cmd.angular.z if cmd is not None else None
            try:
                self.write(row)
            except ValueError:
                self.write(dict(kind='invalid_sample', t=now, reason='nonfinite_telemetry'))

    def close(self):
        with self.lock:
            if not self.file.closed:
                self.file.close()


if __name__ == '__main__':
    rospy.init_node('rgbd_trial_recorder')
    Recorder()
    rospy.spin()
