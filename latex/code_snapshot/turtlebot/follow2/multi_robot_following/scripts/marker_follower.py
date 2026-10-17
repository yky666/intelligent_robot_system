#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Optional back-mounted AR direct follower for follow1 integration.

    This controller stops for front obstacles; it does not plan around them.
    Use map_follower for the group's method 2.
    """
import math
import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from ar_track_alvar_msgs.msg import AlvarMarkers
from competition_core.planning import clamp, fresh, finite


class MarkerFollower(object):
    def __init__(self):
        self.id = rospy.get_param('~marker_id', 1)
        self.base = rospy.get_param('~base_frame', 'base_footprint')
        self.gap = rospy.get_param('~follow_distance', 1.0)
        self.stamp = self.mission_stamp = None
        self.target = None
        self.state = 'idle'
        self.estop = False
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.pub = rospy.Publisher('cmd_vel_raw', Twist, queue_size=1)
        self.allow = rospy.Publisher('motion_allowed', Bool, queue_size=1)
        self.subs = [rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.markers, queue_size=1),
                     rospy.Subscriber('/competition/mission', String, self.mission, queue_size=1),
                     rospy.Subscriber('/competition/estop', Bool, self.emergency, queue_size=1)]
        self.timer = rospy.Timer(rospy.Duration(0.05), self.tick)

    def emergency(self, msg):
        self.estop = msg.data

    def mission(self, msg):
        self.state = msg.data
        self.mission_stamp = rospy.Time.now().to_sec()

    def markers(self, msg):
        for marker in msg.markers:
            if marker.id != self.id:
                continue
            pose = marker.pose
            if not pose.header.frame_id:
                pose.header = marker.header if marker.header.frame_id else msg.header
            stamp = pose.header.stamp.to_sec()
            if stamp <= 0 or not fresh(stamp, rospy.Time.now().to_sec(), 0.5):
                return
            try:
                p = self.buffer.transform(pose, self.base, rospy.Duration(0.05)).pose.position
                if all(finite(v) for v in (p.x, p.y)) and p.x > 0:
                    self.target, self.stamp = (p.x, p.y), stamp
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass
            return

    def tick(self, event):
        now = rospy.Time.now().to_sec()
        good = not self.estop and self.state in ('carrying', 'releasing', 'done') and fresh(self.stamp, now, 0.6) and fresh(self.mission_stamp, now, 1.5)
        out = Twist()
        if good:
            x, y = self.target
            angle = math.atan2(y, x)
            if x > self.gap and abs(angle) < 0.4:
                out.linear.x = min(0.12, 0.4 * (x - self.gap))
            out.angular.z = clamp(1.2 * angle, 0.45) if x > 0.65 else 0.0
        self.allow.publish(good)
        self.pub.publish(out)


if __name__ == '__main__':
    rospy.init_node('marker_follower')
    MarkerFollower()
    rospy.spin()
