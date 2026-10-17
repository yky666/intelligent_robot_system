#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Camera-frame enrollment preview only. No velocity, target or navigation output."""
import json
import threading
import time
import cv2
import numpy as np
import rospy
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from rgbd_following_core.depth import check_pair
from rgbd_following_core.settings import validate
from rgbd_following_core.vision import detect, intrinsics, metric_depth


class Preview(object):
    def __init__(self):
        self.config = validate(rospy.get_param('~settings'))
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest = None
        self.previous = None
        self.clock = getattr(time, 'monotonic', time.time)
        self.image = rospy.Publisher('~preview', Image, queue_size=1)
        self.status = rospy.Publisher('~status', String, queue_size=1)
        self.inputs = [message_filters.Subscriber('rgb', Image, queue_size=1, buff_size=2**24),
                       message_filters.Subscriber('registered_depth', Image, queue_size=1, buff_size=2**24),
                       message_filters.Subscriber('camera_info', CameraInfo, queue_size=1)]
        self.sync = message_filters.ApproximateTimeSynchronizer(self.inputs, 5, self.config['sync_slop'])
        self.sync.registerCallback(self.receive)
        self.worker = threading.Thread(target=self.run)
        self.worker.daemon = True
        self.worker.start()

    def receive(self, rgb, depth, info):
        with self.lock:
            self.latest = (rgb, depth, info, self.clock())

    def run(self):
        while not rospy.is_shutdown():
            self.render()
            time.sleep(.2)

    def render(self):
        with self.lock:
            sample = self.latest
        view = np.zeros((480, 640, 3), dtype=np.uint8)
        result = dict(motion_enabled=False, target_identified=False, candidate_count=0)
        try:
            if sample is None:
                raise ValueError('waiting_for_rgbd')
            rgb, depth, info, receipt = sample
            cfg = self.config
            stamps = [m.header.stamp.to_sec() for m in (rgb, depth, info)]
            check_pair(stamps, rospy.Time.now().to_sec(), cfg['target_timeout'], cfg['sync_slop'],
                       [m.header.frame_id for m in (rgb, depth, info)],
                       [(m.width, m.height) for m in (rgb, depth, info)], depth.encoding)
            if not 0 <= self.clock()-receipt <= cfg['target_timeout']:
                raise ValueError('input_timeout')
            if self.previous is not None and any(s <= p for s,p in zip(stamps,self.previous)):
                raise ValueError('waiting_for_new_frame')
            self.previous = stamps
            view = self.bridge.imgmsg_to_cv2(rgb, 'bgr8').copy()
            d = metric_depth(self.bridge.imgmsg_to_cv2(depth, 'passthrough'), depth.encoding)
            started = self.clock()
            candidates, unused = detect(view, d, intrinsics(info.P, rgb.width, rgb.height), cfg['perception'])
            if not 0 <= rospy.Time.now().to_sec()-min(stamps) <= cfg['target_timeout']:
                raise ValueError('processing_too_slow')
            result.update(reason='unvalidated_candidates', candidate_count=len(candidates),
                          scores=[c['identity_score'] for c in candidates],
                          depths_m=[c['depth'] for c in candidates],
                          processing_ms=(self.clock()-started)*1000)
            for candidate in candidates:
                x,y,w,h = candidate['bbox']
                cv2.rectangle(view, (x,y), (x+w,y+h), (0,200,255), 2)
                cv2.putText(view, 'candidate %.2fm' % candidate['depth'], (x,max(42,y-8)),
                            cv2.FONT_HERSHEY_SIMPLEX,.45,(0,200,255),1)
        except (ValueError, TypeError, CvBridgeError, cv2.error) as exc:
            result['reason'] = str(exc)
            view[:] = 0
        cv2.putText(view, 'UNVALIDATED - MOTION DISABLED', (8,20), cv2.FONT_HERSHEY_SIMPLEX,.5,(0,200,255),1)
        cv2.putText(view, result['reason'], (8,view.shape[0]-12), cv2.FONT_HERSHEY_SIMPLEX,.45,(0,200,255),1)
        msg = self.bridge.cv2_to_imgmsg(view, 'bgr8')
        msg.header.stamp = rospy.Time.now()
        self.image.publish(msg)
        self.status.publish(String(data=json.dumps(result)))


if __name__ == '__main__':
    rospy.init_node('rgbd_identity_preview')
    Preview()
    rospy.spin()
