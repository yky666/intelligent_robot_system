#!/usr/bin/env python
"""Real ROS/Kinetic/cv_bridge/TF/VoxelLayer/DWA, synthetic RGB-D and kinematics.

Exercises transport and navigation startup plus loss stopping; NOT Gazebo,
hardware, or evidence of complete-course obstacle avoidance.
"""
import math
import threading
import time
import unittest
import numpy as np
import rospy
import rostest
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped, Twist, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Bool
from dynamic_reconfigure.msg import Config


class NavigationSmoke(unittest.TestCase):
    def test_transport_navigation_and_loss_stop(self):
        bridge = CvBridge()
        pubs = dict((k,rospy.Publisher('/smoke/input/'+k,t,queue_size=1)) for k,t in
                    [('rgb',Image),('depth',Image),('info',CameraInfo),('odom',Odometry)])
        mission = rospy.Publisher('/competition/mission',String,queue_size=1)
        estop = rospy.Publisher('/competition/estop',Bool,queue_size=1)
        tf = tf2_ros.TransformBroadcaster()
        static = tf2_ros.StaticTransformBroadcaster()
        mount = TransformStamped()
        mount.header.frame_id, mount.child_frame_id = 'base_footprint','rgb_optical'
        mount.transform.translation.z = .5
        q = mount.transform.rotation
        q.x,q.y,q.z,q.w = -.5,.5,-.5,.5
        static.sendTransform(mount)
        seen = dict(valid=False, grid=False, positive=False, command=None, status='', gate='', grid_counts={}, dwa=None)
        def valid(m): seen['valid'] = m.data
        def grid(m):
            seen['grid'] = len(m.data)>0
            seen['grid_counts'] = dict(unknown=m.data.count(-1),free=m.data.count(0),occupied=m.data.count(100))
        def command(m):
            seen['command'] = (time.time(),m.linear.x,m.angular.z)
            seen['positive'] = seen['positive'] or m.linear.x>0
        def status(m): seen['status'] = m.data
        def gate(m): seen['gate'] = m.data
        def dwa(m): seen['dwa'] = dict((p.name,p.value) for p in m.doubles)
        subs = [rospy.Subscriber('/smoke/target_valid',Bool,valid),
                rospy.Subscriber('/smoke/move_base/local_costmap/costmap',OccupancyGrid,grid),
                rospy.Subscriber('/smoke/cmd_vel_preview',Twist,command),
                rospy.Subscriber('/smoke/follow_status',String,status),
                rospy.Subscriber('/smoke/gate_status',String,gate),
                rospy.Subscriber('/smoke/move_base/DWAPlannerROS/parameter_updates',Config,dwa)]
        x,y,heading = 0.,0.,0.
        start, last = time.time(), time.time()
        fault_since = None
        while time.time()-start < 80 and not rospy.is_shutdown():
            wall = time.time()
            dt,last = min(.15,wall-last),wall
            v,w = (seen['command'][1:] if seen['command'] and wall-seen['command'][0]<.3 else (0.,0.))
            x += v*math.cos(heading)*dt
            y += v*math.sin(heading)*dt
            heading += w*dt
            # Pre-match fixture: provide actual depth observations around the
            # stationary base before carrying starts. This is input generation,
            # not a recovery behavior: the controller still emits zero in idle.
            elapsed = wall-start
            if seen['dwa'] is not None:
                for key,value in [('max_trans_vel',.12),('min_trans_vel',.015),('max_rot_vel',.4),('min_rot_vel',.03)]:
                    self.assertIn(key,seen['dwa'])
                    self.assertAlmostEqual(seen['dwa'][key],value)
            if elapsed < 6.:
                x,y,heading,v,w = 0.,0.,2*math.pi*elapsed/6.,0.,2*math.pi/6.
            elif elapsed < 7.:
                x,y,heading,v,w = 0.,0.,0.,0.,0.
            stamp = rospy.Time.now()
            base = TransformStamped()
            base.header.frame_id,base.child_frame_id,base.header.stamp = 'odom','base_footprint',stamp
            base.transform.translation.x,base.transform.translation.y = x,y
            base.transform.rotation.z,base.transform.rotation.w = math.sin(heading/2),math.cos(heading/2)
            tf.sendTransform(base)
            odom = Odometry()
            odom.header,odom.child_frame_id = base.header,base.child_frame_id
            odom.pose.pose.position.x,odom.pose.pose.position.y = x,y
            odom.pose.pose.orientation = base.transform.rotation
            odom.twist.twist.linear.x,odom.twist.twist.angular.z = v,w
            pubs['odom'].publish(odom)
            mission.publish(String(data='carrying' if elapsed>=7. else 'idle'))
            estop.publish(Bool(data=False))
            info = CameraInfo()
            info.width,info.height = 320,240
            info.P = [250.,0.,160.,0.,0.,250.,120.,0.,0.,0.,1.,0.]
            info.K = [250.,0.,160.,0.,250.,120.,0.,0.,1.]
            rgb = np.zeros((240,320,3),np.uint8)
            depth = np.full((240,320),4.,np.float32)
            for row in range(121,240):
                depth[row,:] = min(4.,.5*250/(row-120))
            target_x = 1.2+min(max(0.,elapsed-9)*.10,2.5)
            dx,dy = target_x-x,-y
            forward = math.cos(heading)*dx+math.sin(heading)*dy
            lateral = -math.sin(heading)*dx+math.cos(heading)*dy
            if forward>.4 and fault_since is None:
                center = int(160-250*lateral/forward)
                half = max(3,int(.12*250/forward))
                x0,x1 = max(0,center-half),min(320,center+half)
                if x0<x1:
                    rgb[120-half:120+half,x0:x1] = (0,220,0)
                    depth[120-half:120+half,x0:x1] = forward
            messages = [bridge.cv2_to_imgmsg(rgb,'bgr8'),bridge.cv2_to_imgmsg(depth,'32FC1'),info]
            for topic,msg in zip(('rgb','depth','info'),messages):
                msg.header.frame_id,msg.header.stamp = 'rgb_optical',stamp
                pubs[topic].publish(msg)
            if seen['positive'] and x>.05 and fault_since is None:
                fault_since = wall
            if fault_since is not None and wall-fault_since>1.:
                self.assertFalse(seen['valid'])
                self.assertIsNotNone(seen['command'])
                self.assertEqual(seen['command'][1:],(0.,0.))
                self.assertTrue(seen['grid'])
                self.assertIsNotNone(seen['dwa'])
                published = dict(rospy.get_published_topics())
                self.assertNotIn('/smoke/cmd_vel_safe',published)
                self.assertNotIn('/cmd_vel_mux/input/navi',published)
                return
            time.sleep(.08)
        self.fail('No successful navigation/loss cycle: %r, robot x=%.3f' % (seen,x))


if __name__ == '__main__':
    rospy.init_node('navigation_smoke_test')
    rostest.rosrun('rgbd_following','navigation_smoke',NavigationSmoke)
