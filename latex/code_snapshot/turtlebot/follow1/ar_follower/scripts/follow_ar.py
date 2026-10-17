#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class FollowAR(object):
    def __init__(self):
        rospy.init_node('follow_ar')

        # 参数：可以在 launch 文件里改
        self.target_id = rospy.get_param('~target_id', 1)          # AR 标签 ID
        self.target_dist = rospy.get_param('~target_dist', 0.4)    # 保持距离，单位米
        self.Kp_linear = rospy.get_param('~Kp_linear', 0.5)        # 距离比例
        self.Kp_angular = rospy.get_param('~Kp_angular', 1.0)      # 角度比例
        self.max_linear = rospy.get_param('~max_linear', 0.3)      # 最大前进速度
        self.max_angular = rospy.get_param('~max_angular', 1.0)    # 最大转向速度
        self.min_obstacle_dist = rospy.get_param('~min_obstacle_dist', 0.25)  # 避障距离

        self.marker_x = None
        self.marker_z = None
        self.obstacle_dist = 999.0

        # 发布速度到 Turtlebot2 的 mux
        self.cmd_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)

        # 订阅 AR 标签和激光雷达
        self.ar_sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.ar_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # 10Hz 控制循环
        self.timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)

        rospy.loginfo("FollowAR started. target_id=%d, target_dist=%.2f" % (self.target_id, self.target_dist))

    def ar_callback(self, msg):
        self.marker_x = None
        self.marker_z = None
        for marker in msg.markers:
            if marker.id == self.target_id:
                self.marker_x = marker.pose.pose.position.x
                self.marker_z = marker.pose.pose.position.z
                break

    def scan_callback(self, msg):
        # 只取正前方左右 30 度
        center = len(msg.ranges) // 2
        range_30 = int(30.0 / 180.0 * len(msg.ranges))
        min_d = 999.0
        for i in range(center - range_30, center + range_30 + 1):
            if 0 <= i < len(msg.ranges):
                d = msg.ranges[i]
                if math.isinf(d) or math.isnan(d):
                    continue
                if d < min_d:
                    min_d = d
        self.obstacle_dist = min_d

    def control_loop(self, event):
        cmd = Twist()

        # 1. 避障优先
        if self.obstacle_dist < self.min_obstacle_dist:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5   # 左转避障，如果实际向右转更安全，改成 -0.5
            self.cmd_pub.publish(cmd)
            rospy.logwarn_throttle(1, "Obstacle detected! dist=%.2f" % self.obstacle_dist)
            return

        # 2. 没看到 AR 标签，停止
        if self.marker_x is None or self.marker_z is None:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return

        # 3. 距离控制：z 是前方距离，target_dist 是希望保持的距离
        error_z = self.marker_z - self.target_dist
        cmd.linear.x = self.Kp_linear * error_z
        cmd.linear.x = max(min(cmd.linear.x, self.max_linear), -self.max_linear)

        # 4. 角度控制：x 是左右偏移，正右负左
        # ROS 中 angular.z 正为左转，负为右转，所以这里取负号
        cmd.angular.z = -self.Kp_angular * self.marker_x
        cmd.angular.z = max(min(cmd.angular.z, self.max_angular), -self.max_angular)

        # 5. 太近就不后退，只停止
        if self.marker_z < 0.4:
            cmd.linear.x = 0.0

        self.cmd_pub.publish(cmd)

if __name__ == '__main__':
    try:
        FollowAR()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
