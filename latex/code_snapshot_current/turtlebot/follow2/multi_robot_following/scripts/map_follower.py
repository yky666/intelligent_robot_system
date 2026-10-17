#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Measured breadcrumb target -> move_base; no synthetic leader odometry."""
import math
import threading
import rospy
import actionlib
import tf2_ros
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from competition_core.planning import Trail, distance, fresh, finite, grid_clear, localization_ok


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Follower(object):
    def __init__(self):
        self.lock = threading.RLock()
        self.frame = rospy.get_param('~map_frame', 'map')
        self.base = rospy.get_param('~base_frame', 'base_footprint')
        self.gap = rospy.get_param('~follow_distance', 0.9)
        self.min_gap = rospy.get_param('~minimum_distance', 0.65)
        self.target_timeout = rospy.get_param('~target_timeout', 0.8)
        self.goal_interval = rospy.get_param('~goal_interval', 1.0)
        self.goal_shift = rospy.get_param('~goal_shift', 0.25)
        self.radius = rospy.get_param('~footprint_radius', 0.25)
        self.tf_timeout = rospy.get_param('~tf_timeout', 0.8)
        self.finish_tolerance = rospy.get_param('~finish_tolerance', 0.2)
        if self.gap <= self.min_gap or self.min_gap <= self.radius:
            raise ValueError('follow_distance > minimum_distance > footprint_radius required')
        self.trail = Trail(jump_limit=rospy.get_param('~target_jump_limit', 0.8))
        self.target_stamp = self.loc_stamp = self.mission_stamp = None
        self.target = None
        self.grid = None
        self.grid_stamp = None
        self.mission = 'idle'
        self.estop = False
        self.goal = None
        self.goal_time = 0
        self.finished = False
        self.failed = False
        self.loc_good = False
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.allowed = rospy.Publisher('motion_allowed', Bool, queue_size=1)
        self.status = rospy.Publisher('follow_status', String, queue_size=1, latch=True)
        self.subs = [
            rospy.Subscriber('target_pose', PoseStamped, self.on_target, queue_size=1),
            rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.on_loc, queue_size=1),
            rospy.Subscriber('move_base/global_costmap/costmap', OccupancyGrid, self.on_grid, queue_size=1),
            rospy.Subscriber('/competition/mission', String, self.on_mission, queue_size=1),
            rospy.Subscriber('/competition/estop', Bool, self.on_estop, queue_size=1)]
        rospy.on_shutdown(self.stop)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.tick)

    def on_target(self, msg):
        with self.lock:
            stamp = msg.header.stamp.to_sec()
            if msg.header.frame_id.lstrip('/') != self.frame.lstrip('/') or not fresh(stamp, rospy.Time.now().to_sec(), self.target_timeout):
                return
            if self.target_stamp is not None and stamp <= self.target_stamp:
                return
            p = msg.pose.position
            if self.trail.add(p.x, p.y):
                self.target = (p.x, p.y)
                self.target_stamp = stamp
            else:
                rospy.logwarn_throttle(2, 'Target jumped; rejecting measurement, waiting for reacquisition')

    def on_loc(self, msg):
        with self.lock:
            self.loc_good = msg.header.frame_id.lstrip('/') == self.frame.lstrip('/') and localization_ok(msg.pose.covariance, 0.25, 0.35)
            self.loc_stamp = msg.header.stamp.to_sec()

    def on_grid(self, msg):
        with self.lock:
            self.grid = msg
            self.grid_stamp = rospy.Time.now().to_sec()

    def on_mission(self, msg):
        with self.lock:
            self.mission = msg.data
            self.mission_stamp = rospy.Time.now().to_sec()

    def on_estop(self, msg):
        with self.lock:
            self.estop = msg.data

    def stop(self):
        self.allowed.publish(False)
        if self.goal is not None:
            self.client.cancel_goal()
            self.goal = None

    def block(self, reason):
        self.stop()
        self.status.publish(reason)

    def tick(self, event):
        with self.lock:
            self.step()

    def step(self):
        now = rospy.Time.now().to_sec()
        # Publisher emits this every cycle; velocity gate has a separate watchdog.
        if self.finished or self.failed:
            return self.block('finished' if self.finished else 'navigation_failed_reset_required')
        if self.estop or self.mission not in ('carrying', 'releasing', 'done') or not fresh(self.mission_stamp, now, 1.5):
            return self.block('mission_stopped')
        if not fresh(self.target_stamp, now, self.target_timeout):
            return self.block('target_lost_hold_position')
        if not self.loc_good or not fresh(self.loc_stamp, now, 2.0):
            return self.block('localization_invalid')
        if self.grid is None or not fresh(self.grid_stamp, now, 3.0):
            return self.block('costmap_missing_or_stale')
        try:
            transform = self.buffer.lookup_transform(self.frame, self.base, rospy.Time(0), rospy.Duration(0.03))
            if not fresh(transform.header.stamp.to_sec(), now, self.tf_timeout):
                return self.block('base_tf_stale')
            t = transform.transform.translation
            robot = (t.x, t.y)
            if not all(finite(v) for v in robot):
                return self.block('base_tf_invalid')
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return self.block('base_tf_missing')
        if distance(robot, self.target) < self.min_gap:
            return self.block('leader_too_close')
        point = self.trail.behind(self.gap)
        if point is None:
            return self.block('waiting_for_leader_trail')
        if self.mission == 'done' and distance(robot, point) <= self.finish_tolerance:
            self.finished = True
            return self.block('finished')
        grid = self.grid
        origin = grid.info.origin
        if grid.header.frame_id.lstrip('/') != self.frame.lstrip('/') or not grid_clear(
                grid.data, grid.info.width, grid.info.height, grid.info.resolution,
                origin.position.x, origin.position.y, yaw(origin.orientation),
                point[0], point[1], self.radius):
            return self.block('follow_goal_blocked')
        # Do not send repeated goals near the current goal (avoids preemption oscillation).
        state = self.client.get_state() if self.goal is not None else None
        if state in (4, 5, 9):  # ABORTED, REJECTED, LOST
            self.failed = True
            return self.block('navigation_failed_reset_required')
        if not self.client.wait_for_server(rospy.Duration(0.01)):
            return self.block('move_base_unavailable')
        if self.goal is None or (now - self.goal_time >= self.goal_interval and distance(self.goal, point) >= self.goal_shift):
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = self.frame
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = point[0]
            goal.target_pose.pose.position.y = point[1]
            goal.target_pose.pose.orientation.z = math.sin(point[2] / 2)
            goal.target_pose.pose.orientation.w = math.cos(point[2] / 2)
            self.client.send_goal(goal)
            self.goal = point
            self.goal_time = now
        self.allowed.publish(True)
        self.status.publish('following')


if __name__ == '__main__':
    rospy.init_node('map_follower')
    Follower()
    rospy.spin()
