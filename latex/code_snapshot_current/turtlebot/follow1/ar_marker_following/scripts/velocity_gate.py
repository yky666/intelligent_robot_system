#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Only publisher on the navigation input of the base velocity mux."""
import math
import threading
import time
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from competition_core.planning import clamp, fresh, finite, scan_clear


class Gate(object):
    def __init__(self):
        self.lock = threading.RLock()
        self.clock = getattr(time, 'monotonic', time.time)
        self.command = Twist()
        self.command_stamp = self.allowed_stamp = self.scan_stamp = None
        self.allowed = False
        self.estop = False
        self.scan = None
        self.max_v = rospy.get_param('~max_linear', 0.15)
        self.max_w = rospy.get_param('~max_angular', 0.5)
        self.clearance = rospy.get_param('~stop_distance', 0.5)
        self.require_scan = rospy.get_param('~require_scan', True)
        self.pub = rospy.Publisher('cmd_vel_safe', Twist, queue_size=1)
        self.subs = [rospy.Subscriber('cmd_vel_raw', Twist, self.cmd, queue_size=1),
                     rospy.Subscriber('motion_allowed', Bool, self.permission, queue_size=1),
                     rospy.Subscriber('scan', LaserScan, self.laser, queue_size=1),
                     rospy.Subscriber('/competition/estop', Bool, self.emergency, queue_size=1)]
        rospy.on_shutdown(lambda: self.pub.publish(Twist()))
        # Wall-clock worker continues to zero output if ROS /clock is paused.
        self.worker = threading.Thread(target=self.run)
        self.worker.daemon = True
        self.worker.start()

    def cmd(self, msg):
        with self.lock:
            self.command, self.command_stamp = msg, self.clock()

    def permission(self, msg):
        with self.lock:
            self.allowed, self.allowed_stamp = msg.data, self.clock()

    def emergency(self, msg):
        with self.lock:
            self.estop = msg.data
            if self.estop:
                self.pub.publish(Twist())

    def laser(self, msg):
        with self.lock:
            # Require fresh sensor timestamp as well as callback receipt.
            if fresh(msg.header.stamp.to_sec(), rospy.Time.now().to_sec(), 0.5):
                self.scan, self.scan_stamp = msg, self.clock()

    def output(self):
        now = self.clock()
        out = Twist()
        v, w = self.command.linear.x, self.command.angular.z
        if self.estop or not self.allowed or not fresh(self.allowed_stamp, now, 0.4) or not fresh(self.command_stamp, now, 0.4):
            return out
        if not finite(v) or not finite(w) or v < 0:  # no blind reverse with front laser only
            return out
        if self.require_scan:
            scan = self.scan
            # Wider sector during rotation; the full footprint stays in costmaps.
            sector = math.pi / 2 if abs(w) > 0.05 else math.pi / 4
            if scan is None or not fresh(self.scan_stamp, now, 0.5) or not scan_clear(
                    scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min,
                    scan.range_max, self.clearance, sector):
                return out
        out.linear.x = clamp(v, self.max_v)
        out.angular.z = clamp(w, self.max_w)
        return out

    def run(self):
        while not rospy.is_shutdown():
            with self.lock:
                self.pub.publish(self.output())
            time.sleep(0.05)


if __name__ == '__main__':
    rospy.init_node('velocity_gate')
    Gate()
    rospy.spin()
