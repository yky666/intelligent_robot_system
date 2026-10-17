#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Inspect synchronized rectified RGB + registered depth, never publish velocity."""
import json
import threading
import time

import cv2
import message_filters
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from rgbd_following_core.depth import check_pair, finite, roi_bounds, summarize


class Probe(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.lock = threading.RLock()
        self.clock = getattr(time, 'monotonic', time.time)
        self.last_receipt = None
        self.last_stamp = None
        self.last_pair = None
        self.slop = float(rospy.get_param('~sync_slop', 0.05))
        self.max_age = float(rospy.get_param('~max_age', 0.5))
        self.roi = rospy.get_param('~roi', [0.4, 0.35, 0.6, 0.65])
        self.stride = int(rospy.get_param('~sample_stride', 2))
        self.depth_args = dict(
            min_depth=float(rospy.get_param('~min_depth', 0.4)),
            max_depth=float(rospy.get_param('~max_depth', 4.0)),
            min_samples=int(rospy.get_param('~min_samples', 30)),
            min_ratio=float(rospy.get_param('~min_valid_ratio', 0.5)),
            max_spread=float(rospy.get_param('~max_depth_spread', 0.25)))
        if not all(finite(v) and v > 0 for v in (self.slop, self.max_age)) or self.stride < 1:
            raise ValueError('sync_slop, max_age and sample_stride must be positive')
        roi_bounds(self.roi, 640, 480)
        summarize([], '32FC1', **self.depth_args)
        self.status = rospy.Publisher('~status', String, queue_size=1)
        self.preview = rospy.Publisher('~preview', Image, queue_size=1)
        # Synchronize calibration too: a different camera's calibration must not
        # silently be accepted. Driver camera_info is expected at image cadence.
        self.inputs = [message_filters.Subscriber('rgb', Image, queue_size=1, buff_size=2**24),
                       message_filters.Subscriber('registered_depth', Image, queue_size=1, buff_size=2**24),
                       message_filters.Subscriber('camera_info', CameraInfo, queue_size=1)]
        self.sync = message_filters.ApproximateTimeSynchronizer(self.inputs, 5, self.slop)
        self.sync.registerCallback(self.callback)
        self.worker = threading.Thread(target=self.watchdog)
        self.worker.daemon = True
        self.worker.start()

    def report(self, reason, **fields):
        fields.update(reason=reason, motion_enabled=False, target_identified=False)
        self.status.publish(String(data=json.dumps(fields, sort_keys=True, allow_nan=False)))

    def callback(self, rgb, depth, info):
        with self.lock:
            try:
                stamps = [m.header.stamp.to_sec() for m in (rgb, depth, info)]
                check_pair(stamps, rospy.Time.now().to_sec(), self.max_age, self.slop,
                           [m.header.frame_id for m in (rgb, depth, info)],
                           [(m.width, m.height) for m in (rgb, depth, info)], depth.encoding)
                if self.last_pair is not None and any(s <= p for s, p in zip(stamps, self.last_pair)):
                    raise ValueError('repeated_or_out_of_order_input')
                # Rectified RGB and depth must use this projection matrix.
                if (not all(finite(v) for v in info.P) or
                        info.P[0] <= 0 or info.P[5] <= 0 or
                        not 0 <= info.P[2] < rgb.width or not 0 <= info.P[6] < rgb.height):
                    raise ValueError('invalid_camera_calibration')
                array = self.bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough')
                view = self.bridge.imgmsg_to_cv2(rgb, desired_encoding='bgr8').copy()
                if array.shape != (depth.height, depth.width) or view.shape[:2] != array.shape:
                    raise ValueError('decoded_image_size_mismatch')
                x0, y0, x1, y1 = roi_bounds(self.roi, rgb.width, rgb.height)
                values = array[y0:y1:self.stride, x0:x1:self.stride].ravel()
                result = summarize(values, depth.encoding, **self.depth_args)
                self.last_pair = stamps
                self.last_stamp = min(stamps)
                self.last_receipt = self.clock()
                reason = result.pop('reason')
                self.report(reason, stamp=self.last_stamp, frame=rgb.header.frame_id,
                            sync_delta_s=max(stamps) - min(stamps), encoding=depth.encoding,
                            roi_pixels=[x0, y0, x1, y1], **result)
                color = (0, 255, 0) if result['usable'] else (0, 0, 255)
                cv2.rectangle(view, (x0, y0), (x1 - 1, y1 - 1), color, 2)
                label = '%s z=%s valid=%.2f' % (reason, result['median_m'], result['valid_ratio'])
                cv2.putText(view, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                out = self.bridge.cv2_to_imgmsg(view, encoding='bgr8')
                out.header = rgb.header
                self.preview.publish(out)
            except (ValueError, TypeError, CvBridgeError, cv2.error) as exc:
                self.report(str(exc), usable=False)

    def watchdog(self):
        while not rospy.is_shutdown():
            with self.lock:
                now = rospy.Time.now().to_sec()
                if self.last_receipt is None:
                    self.report('waiting_for_synchronized_rgb_depth_info', usable=False)
                elif (not 0 <= self.clock() - self.last_receipt <= self.max_age or
                      not 0 <= now - self.last_stamp <= self.max_age):
                    self.report('input_timeout', usable=False)
            time.sleep(0.2)


if __name__ == '__main__':
    rospy.init_node('rgbd_probe')
    Probe()
    rospy.spin()
