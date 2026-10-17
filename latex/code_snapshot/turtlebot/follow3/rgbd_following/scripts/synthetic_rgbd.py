#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Synthetic input for probe training only. Does not simulate robot perception."""
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo


def main():
    rospy.init_node('synthetic_rgbd')
    bridge = CvBridge()
    rgb_pub = rospy.Publisher('rgb', Image, queue_size=1)
    depth_pub = rospy.Publisher('depth', Image, queue_size=1)
    info_pub = rospy.Publisher('camera_info', CameraInfo, queue_size=1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        mode = rospy.get_param('~mode', 'good')
        if mode not in ('good', 'invalid', 'mixed', 'stale', 'frame_mismatch', 'pause'):
            rospy.logwarn_throttle(2, 'Unknown mode: ' + str(mode))
            rate.sleep()
            continue
        if mode == 'pause':
            rate.sleep()
            continue
        rgb = np.zeros((240, 320, 3), dtype=np.uint8)
        rgb[84:156, 128:192] = (0, 200, 0)
        depth = np.full((240, 320), 2.5, dtype=np.float32)
        depth[84:156, 128:192] = 1.2
        if mode == 'invalid':
            depth[:] = float('nan')
        elif mode == 'mixed':
            depth[:, 160:] = 2.5
        stamp = rospy.Time.now()
        if mode == 'stale':
            stamp = rospy.Time.from_sec(max(0, stamp.to_sec() - 2.0))
        rgb_msg = bridge.cv2_to_imgmsg(rgb, encoding='bgr8')
        depth_msg = bridge.cv2_to_imgmsg(depth, encoding='32FC1')
        info = CameraInfo()
        info.width, info.height = 320, 240
        info.distortion_model = 'plumb_bob'
        info.D = [0.] * 5
        info.K = [250., 0., 160., 0., 250., 120., 0., 0., 1.]
        info.R = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
        info.P = [250., 0., 160., 0., 0., 250., 120., 0., 0., 0., 1., 0.]
        for msg in (rgb_msg, depth_msg, info):
            msg.header.stamp = stamp
            msg.header.frame_id = 'synthetic_rgb_optical_frame'
        if mode == 'frame_mismatch':
            depth_msg.header.frame_id = 'unregistered_depth_optical_frame'
        rgb_pub.publish(rgb_msg)
        depth_pub.publish(depth_msg)
        info_pub.publish(info)
        rate.sleep()


if __name__ == '__main__':
    main()
