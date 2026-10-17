#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Relay predecessor AMCL pose in the common map, reject bad covariance."""
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from competition_core.planning import fresh, localization_ok


class Relay(object):
    def __init__(self):
        self.frame = rospy.get_param('~map_frame', 'map')
        self.pub = rospy.Publisher('target_pose', PoseStamped, queue_size=1)
        self.sub = rospy.Subscriber('leader_pose', PoseWithCovarianceStamped, self.cb, queue_size=1)

    def cb(self, msg):
        if msg.header.frame_id.lstrip('/') != self.frame.lstrip('/'):
            rospy.logwarn_throttle(2, 'Leader pose is not in the shared map; relay refused')
            return
        if not fresh(msg.header.stamp.to_sec(), rospy.Time.now().to_sec(), 0.8):
            return
        if not localization_ok(msg.pose.covariance, 0.25, 0.35):
            return
        out = PoseStamped(header=msg.header, pose=msg.pose.pose)
        self.pub.publish(out)


if __name__ == '__main__':
    rospy.init_node('pose_relay')
    Relay()
    rospy.spin()
