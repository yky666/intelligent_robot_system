#!/usr/bin/env python
"""Real Kinetic VoxelLayer regression for the production observation sources.

Synthetic, isolated TF and clouds; no robot, camera, goal or motion publisher.
Tests clearing evidence, not RGB-D transport or complete-course navigation.
"""
import math
import time
import unittest

import rospy
import rostest
import tf2_ros
from costmap_2d.msg import VoxelGrid
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


NS = '/clearing_regression'
SENSOR_HEIGHT = 0.30


class ClearingObservations(unittest.TestCase):
    def setUp(self):
        self.voxels = None
        self.map = None
        self.global_map = None
        self.mark = rospy.Publisher(NS + '/obstacle_cloud', PointCloud2, queue_size=1)
        self.clear = rospy.Publisher(NS + '/clearing_cloud', PointCloud2, queue_size=1)
        self.subscribers = [
            rospy.Subscriber(NS + '/move_base/local_costmap/obstacles/voxel_grid',
                             VoxelGrid, self.receive_voxels),
            rospy.Subscriber(NS + '/move_base/local_costmap/costmap',
                             OccupancyGrid, self.receive_map),
            rospy.Subscriber(NS + '/move_base/global_costmap/costmap',
                             OccupancyGrid, self.receive_global_map)]
        self.static = tf2_ros.StaticTransformBroadcaster()
        self.dynamic = tf2_ros.TransformBroadcaster()
        self.base = TransformStamped()
        self.base.header.frame_id = 'clearing_regression_odom'
        self.base.child_frame_id = 'clearing_regression_base'
        self.base.transform.rotation.w = 1.0
        sensor = TransformStamped()
        sensor.header.stamp = rospy.Time.now()
        sensor.header.frame_id = self.base.child_frame_id
        sensor.child_frame_id = 'clearing_regression_sensor'
        sensor.transform.translation.z = SENSOR_HEIGHT
        sensor.transform.rotation.w = 1.0
        self.static.sendTransform(sensor)
        # Kinetic Costmap2DROS checks the latest base-pose timestamp. A fully
        # static odom->base chain gives time zero and fails that freshness check.
        self.tf_timer = rospy.Timer(rospy.Duration(0.03), self.publish_base)
        self.addCleanup(self.tf_timer.shutdown)
        deadline = time.time() + 20.0
        while not rospy.is_shutdown() and time.time() < deadline:
            # Both costmaps share one process and may share one TCP transport.
            if self.mark.get_num_connections() >= 1 and self.clear.get_num_connections() >= 1:
                return
            time.sleep(0.1)
        self.fail('Both production costmaps must subscribe to both isolated clouds')

    def publish_base(self, event):
        self.base.header.stamp = rospy.Time.now()
        self.dynamic.sendTransform(self.base)

    def receive_voxels(self, message):
        self.voxels = message

    def receive_map(self, message):
        self.map = message

    def receive_global_map(self, message):
        self.global_map = message

    def publish(self, marking, clearing):
        header = Header(stamp=rospy.Time.now(), frame_id='clearing_regression_sensor')
        # Fixtures are written in the fixed/world frame. The actual messages
        # use a raised sensor frame, exercising the observation-origin TF.
        def cloud(points):
            return point_cloud2.create_cloud_xyz32(header, [
                (p[0], p[1], p[2] - SENSOR_HEIGHT) for p in points])
        if marking is not None:
            self.mark.publish(cloud(marking))
        self.clear.publish(cloud(clearing))

    def voxel_state(self, point):
        grid = self.voxels
        if grid is None:
            return None
        x = int(math.floor((point[0] - grid.origin.x) / grid.resolutions.x))
        y = int(math.floor((point[1] - grid.origin.y) / grid.resolutions.y))
        z = int(math.floor((point[2] - grid.origin.z) / grid.resolutions.z))
        if not (0 <= x < grid.size_x and 0 <= y < grid.size_y and 0 <= z < grid.size_z):
            return None
        column = grid.data[y * grid.size_x + x]
        # voxel_grid::VoxelGrid stores unknown in low bits, marking in high
        # bits. 11=occupied, 01=unknown, 00=observed free.
        return ((column >> z) & 1) | (((column >> (z + 16)) & 1) << 1)

    def map_value(self, point, global_map=False):
        grid = self.global_map if global_map else self.map
        if grid is None:
            return None
        x = int(math.floor((point[0] - grid.info.origin.position.x) / grid.info.resolution))
        y = int(math.floor((point[1] - grid.info.origin.position.y) / grid.info.resolution))
        if not (0 <= x < grid.info.width and 0 <= y < grid.info.height):
            return None
        return grid.data[y * grid.info.width + x]

    def await_state(self, label, point, state, marking, clearing, timeout=7.0):
        deadline = time.time() + timeout
        # Require several published maps, rather than accepting an old sample
        # immediately after changing the source observations.
        since = rospy.Time.now()
        while not rospy.is_shutdown() and time.time() < deadline:
            self.publish(marking, clearing)
            fresh = (self.voxels is not None and self.map is not None and self.global_map is not None and
                     self.voxels.header.stamp > since + rospy.Duration(0.4) and
                     self.map.header.stamp > since + rospy.Duration(0.4) and
                     self.global_map.header.stamp > since + rospy.Duration(0.4))
            value = self.map_value(point)
            global_value = self.map_value(point, global_map=True)
            expected_map = (value == global_value == 100 if state == 3 else value == global_value == 0)
            if fresh and self.voxel_state(point) == state and expected_map:
                rospy.loginfo('%s: voxel=%s occupancy=%s', label, state, value)
                return
            time.sleep(0.05)
        self.fail('%s: voxel=%r occupancy=%r (expected %s)' %
                  (label, self.voxel_state(point), self.map_value(point), state))

    def remain_marked(self, label, point, marking, clearing, duration=1.5):
        deadline = time.time() + duration
        while not rospy.is_shutdown() and time.time() < deadline:
            self.publish(marking, clearing)
            self.assertEqual(self.voxel_state(point), 3, label)
            self.assertEqual(self.map_value(point), 100, label)
            self.assertEqual(self.map_value(point, global_map=True), 100, label)
            time.sleep(0.05)

    def test_valid_rays_clear_without_fabricating_free_space(self):
        for scope in ('local_costmap', 'global_costmap'):
            config = rospy.get_param(NS + '/move_base/' + scope + '/obstacles')
            self.assertEqual(set(config['observation_sources'].split()), set(['depth', 'clearing']))
            self.assertTrue(config['depth']['marking'])
            self.assertFalse(config['depth']['clearing'])
            self.assertFalse(config['clearing']['marking'])
            self.assertTrue(config['clearing']['clearing'])

        low = (1.275, 0.025, 0.55)
        far_x = 4.3  # Outside the old 4m cloud limit, inside the new 4.5m limit.
        far = (far_x, low[1] * far_x / low[0],
               SENSOR_HEIGHT + (low[2] - SENSOR_HEIGHT) * far_x / low[0])
        self.await_state('initial obstacle', low, 3, [low], [])
        nan = float('nan')
        self.remain_marked('invalid depths must not clear', low, [], [(nan, nan, nan)])
        self.remain_marked('no observations must not clear', low, [], [])
        near_x = 0.7
        near = (near_x, low[1] * near_x / low[0],
                SENSOR_HEIGHT + (low[2] - SENSOR_HEIGHT) * near_x / low[0])
        self.remain_marked('foreground endpoint cannot clear behind itself', low, [], [near])
        self.await_state('4.3m background clears old voxel', low, 0, [], [far])
        self.assertNotEqual(self.voxel_state(far), 3, 'clearing-only endpoint must not mark')

        high = (1.275, 0.025, 0.85)
        high_x = 3.4
        above = (high_x, high[1] * high_x / high[0],
                 SENSOR_HEIGHT + (high[2] - SENSOR_HEIGHT) * high_x / high[0])
        self.assertGreater(above[2], 1.5)
        # A small background patch models multiple camera pixels. Kinetic's
        # clipped integer 3-D Bresenham ray can visit the adjacent height bin
        # for a single ideal line, so do not make this regression depend on a
        # sub-voxel coincidence. Every endpoint remains above the marking cap.
        high_patch = [(above[0], above[1], above[2] + dz)
                      for dz in (-0.12, -0.08, -0.04, 0.0, 0.04, 0.08, 0.12)]
        self.assertTrue(all(p[2] > 1.5 for p in high_patch))
        self.await_state('high obstacle', high, 3, [high], [])
        ground = (high_x, high[1] * high_x / high[0], 0.0)
        self.remain_marked('ground ray cannot erase high obstacle', high, [], [ground])
        self.await_state('endpoints above 1.5m clear old voxel', high, 0, [], high_patch)

        self.await_state('real obstacle reappears', low, 3, [low], [])
        self.await_state('marking wins over crossing clearing ray', low, 3, [low], [far])
        self.remain_marked('real obstacle remains occupied', low, [low], [far])
        # observation_persistence=0 retains the most recent cloud. Publishing
        # clearing alone must not make this fixture look clear while a stale
        # marking observation keeps re-painting the old obstacle each cycle.
        self.remain_marked('cached marking observation is still applied', low, None, [far])
        self.await_state('empty fresh marking replaces cached obstacle', low, 0, [], [far])


if __name__ == '__main__':
    rospy.init_node('clearing_observations_test')
    rostest.rosrun('rgbd_following', 'clearing_observations', ClearingObservations)
