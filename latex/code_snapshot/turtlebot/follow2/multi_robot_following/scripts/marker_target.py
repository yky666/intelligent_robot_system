#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Transform a fixed-ID AR measurement into map at the measurement time."""
import copy
import rospy
import tf2_ros
import tf2_geometry_msgs  # registers PoseStamped transform support
from geometry_msgs.msg import PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
from competition_core.planning import fresh, finite


class MarkerTarget(object):
    def __init__(self):
        self.frame = rospy.get_param('~map_frame', 'map')
        self.marker_id = rospy.get_param('~marker_id', 1)
        self.timeout = rospy.get_param('~measurement_timeout', 0.5)
        self.max_range = rospy.get_param('~max_range', 4.0)
        self.buffer = tf2_ros.Buffer(rospy.Duration(10))
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.pub = rospy.Publisher('target_pose', PoseStamped, queue_size=1)
        self.sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.on_markers, queue_size=1)

    def on_markers(self, msg):
        now = rospy.Time.now().to_sec()
        for marker in msg.markers:
            if marker.id != self.marker_id:
                continue
            pose = copy.deepcopy(marker.pose)
            if not pose.header.frame_id:
                # ALVAR Kinetic puts the header on AlvarMarker, not its nested pose.
                pose.header = copy.deepcopy(marker.header if marker.header.frame_id else msg.header)
            if pose.header.stamp.to_sec() <= 0 or not fresh(pose.header.stamp.to_sec(), now, self.timeout):
                return
            xyz = pose.pose.position
            if not all(finite(v) for v in (xyz.x, xyz.y, xyz.z)):
                return
            if xyz.x * xyz.x + xyz.y * xyz.y + xyz.z * xyz.z > self.max_range ** 2:
                return
            try:
                out = self.buffer.transform(pose, self.frame, rospy.Duration(0.08))
                out.header.stamp = pose.header.stamp  # never make old data fresh
                self.pub.publish(out)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
                rospy.logwarn_throttle(2, 'target TF unavailable: %s' % exc)
            return


if __name__ == '__main__':
    rospy.init_node('marker_target')
    MarkerTarget()
    rospy.spin()
