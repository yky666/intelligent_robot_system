# AR Marker Turtlebot2 Following Experiment Report

## 1. Experiment background

This experiment studies a direct visual-following method for a Turtlebot2.
The target robot carries an ALVAR AR marker on its rear side. The following
Turtlebot2 uses a Kinect RGB camera to detect the marker and estimates the
target's relative position from the AR observation. The follower then
generates linear and angular velocity commands from the relative distance and
lateral offset.

Unlike a fixed-marker waypoint method, this approach does not require a set
of AR markers to be placed throughout the whole course, and it does not
require the follower to know the target robot's global map pose. Its main
requirement is that the target marker remains visible to the camera.

## 2. Objectives

The objectives were:

1. Launch `ar_track_alvar` in ROS Kinetic and obtain AR marker observations.
2. Select a target marker by ID from `/ar_pose_marker`.
3. Use the target distance and lateral offset to control Turtlebot2 motion.
4. Stop when the target marker is not visible.
5. Add a simple front laser check for local stop-and-turn behavior.
6. Record the real launch file and Python node used in the virtual machine so
   the experiment can be reproduced from the repository.

## 3. Hardware and software

| Item | Configuration |
|---|---|
| Following platform | Turtlebot2 Kobuki |
| Target | TonyPi or another robot carrying an AR marker |
| Camera | Kinect RGB camera |
| Range sensor | Turtlebot2 laser scan on `/scan` |
| Operating system | Ubuntu 16.04 |
| ROS | ROS Kinetic |
| Python | Python 2.7 |
| Main detection package | `ar_track_alvar` |
| Experiment package | `ar_follower` |

The package source is stored at
`turtlebot/follow1/ar_follower/`.

## 4. System architecture

The experiment consists of two ROS nodes:

1. `ar_track_alvar` receives the RGB image and camera calibration data, then
   publishes detected markers on `/ar_pose_marker`.
2. `follow_ar.py` selects the configured marker ID, reads `/scan`, computes a
   velocity command, and publishes it to
   `/cmd_vel_mux/input/teleop`.

The actual package layout is:

```text
turtlebot/follow1/ar_follower/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── ar_track.launch
└── scripts/
    └── follow_ar.py
```

## 5. Experimental commands

The VM command history confirmed the following launch sequence:

```bash
roslaunch ar_follower ar_track.launch
rosrun ar_follower follow_ar.py
```

The first command starts the AR detector. The second command starts the
closed-loop follower. Before running them, the Turtlebot2 base, Kinect driver,
and laser driver must already be running and the required ROS environment must
be sourced.

The package can be copied into a catkin workspace with:

```bash
mkdir -p ~/catkin_ws/src
cp -r turtlebot/follow1/ar_follower ~/catkin_ws/src/
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 6. AR detection parameters

The launch file is:

```text
turtlebot/follow1/ar_follower/launch/ar_track.launch
```

Its default parameters are:

| Parameter | Default |
|---|---:|
| `marker_size` | `14.0` |
| `max_new_marker_error` | `0.08` |
| `max_track_error` | `0.2` |
| `cam_image_topic` | `/camera/rgb/image_color` |
| `cam_info_topic` | `/camera/rgb/camera_info` |
| `output_frame` | `/camera_rgb_optical_frame` |

`marker_size` must match the physical marker used in the experiment. If the
camera publishes different topic names, the launch arguments can be
overridden, for example:

```bash
roslaunch ar_follower ar_track.launch \
  cam_image_topic:=/camera/rgb/image_raw \
  cam_info_topic:=/camera/rgb/camera_info
```

The detector output can be checked with:

```bash
rostopic echo /ar_pose_marker
```

The target marker ID must appear in the message before the follower can
generate forward motion.

## 7. Following controller

The control node is:

```text
turtlebot/follow1/ar_follower/scripts/follow_ar.py
```

The controller uses the target marker position:

- `marker.pose.pose.position.z` is treated as the forward distance.
- `marker.pose.pose.position.x` is treated as the lateral offset.
- The distance error is `marker_z - target_dist`.
- The lateral error is converted to angular velocity with a proportional
  gain.

The main control equations are:

```text
linear.x  = Kp_linear  * (marker_z - target_dist)
angular.z = -Kp_angular * marker_x
```

Both commands are limited by configurable maximum values. When the marker is
closer than `0.4 m`, the implementation prevents backward motion by setting
the linear velocity to zero.

The default controller parameters are:

| Parameter | Default | Meaning |
|---|---:|---|
| `~target_id` | `1` | Marker ID to follow |
| `~target_dist` | `0.4` m | Desired following distance |
| `~Kp_linear` | `0.5` | Linear proportional gain |
| `~Kp_angular` | `1.0` | Angular proportional gain |
| `~max_linear` | `0.3` m/s | Linear speed limit |
| `~max_angular` | `1.0` rad/s | Angular speed limit |
| `~min_obstacle_dist` | `0.25` m | Front obstacle threshold |

For a different marker ID:

```bash
rosrun ar_follower follow_ar.py _target_id:=2
```

## 8. Local obstacle response

The node subscribes to `/scan` and examines the front 60-degree sector. It
ignores infinite and NaN readings and keeps the smallest valid range. If that
range is below `min_obstacle_dist`, the command is changed to:

```text
linear.x  = 0.0
angular.z = 0.5
```

This behavior is a local stop-and-turn response, not map-based obstacle
avoidance. It does not plan a route around the obstacle.

## 9. Experimental observations

The VM shell history confirms that the AR detector and direct follower were
started with the commands listed in Section 5. The repository now preserves
the launch file and Python node used by that experiment.

The available record does not include a complete measurement table for
tracking error, maximum following time, success rate, or obstacle-clearance
distance. Therefore this report does not claim a quantitative acceptance
result. For a formal comparison, future runs should record:

| Metric | Suggested measurement |
|---|---|
| Marker detection rate | Visible frames with target ID / total frames |
| Distance error | Mean and maximum absolute error |
| Lateral error | Mean and maximum absolute offset |
| Lost-target response | Time from marker loss to zero linear velocity |
| Following success | Completed trials / total trials |
| Obstacle response | Minimum observed range and stop latency |

## 10. Troubleshooting

### No marker message

Check that the Kinect driver is running, the image and camera-info topics
match the launch arguments, and the physical marker is printed at the size
given by `marker_size`.

### Marker is detected but the robot does not move

Check the marker ID, `/scan`, and the final velocity topic:

```bash
rostopic echo /ar_pose_marker
rostopic echo /scan
rostopic echo /cmd_vel_mux/input/teleop
```

Also make sure that another node is not competing for the same velocity
input.

### Distance is inaccurate

Measure the physical marker and update `marker_size`. A mismatch changes the
estimated relative distance.

### Oscillating steering

Reduce `~Kp_angular` or `~max_angular`. If the sign of the turn is reversed
because of the camera installation, verify the coordinate convention before
changing the sign in the controller.

## 11. Safety limitations

The extracted script is a compact teaching experiment and should be treated
as a direct velocity controller:

- It publishes directly to `/cmd_vel_mux/input/teleop`.
- It has no sensor-freshness timeout for `/scan`.
- Before the first valid scan, the obstacle distance is initialized to
  `999.0`.
- On a close obstacle, it stops forward motion but continues to command an
  angular velocity of `0.5`.
- It does not use the follow2 `velocity_gate.py`.
- It does not use AMCL, `move_base`, a costmap, or a mission-state bridge.
- It should not run simultaneously with another follower or navigation
  controller on the same robot.

An operator should remain next to the robot and be ready to stop the base
during all real-hardware tests.

## 12. Conclusion

The experiment demonstrates a compact AR-marker-based direct-following
pipeline for Turtlebot2. The approach is easy to understand and reproduce:
the camera detects a known marker, the follower converts relative position
into bounded velocity commands, and loss of the marker causes the robot to
stop.

Its main advantage is that it follows the target using relative visual
geometry without requiring a shared map. Its main limitations are dependence
on marker visibility, sensitivity to marker-size calibration, and the lack of
full navigation and sensor watchdog functions. The implementation is
therefore suitable as the repository's follow1 experiment record, while the
more complete map-navigation and velocity-gating functions should remain
documented under follow2.

## 13. Future improvements

Future work can add:

1. A sensor-freshness timeout and explicit emergency-stop input.
2. Low-pass filtering of AR pose measurements.
3. A separate safe velocity gate before the base input.
4. A proper local planner or `move_base` integration for obstacle avoidance.
5. Recorded rosbag trials and quantitative acceptance metrics.
6. Namespaced topics and TF frames for multi-robot deployment.
