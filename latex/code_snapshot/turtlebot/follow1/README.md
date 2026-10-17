# Follow1: AR marker direct following

This directory records the first Turtlebot2 following experiment. The
experiment used a Turtlebot2 with a Kinect RGB camera to detect an ALVAR AR
marker attached to the target robot, then generated a velocity command from
the marker's relative position.

The code copied from the ROS virtual machine is the complete `ar_follower`
package:

```text
ar_follower/
├── CMakeLists.txt
├── package.xml
├── launch/ar_track.launch
└── scripts/follow_ar.py
```

## Experiment commands

After placing the package in a catkin workspace and sourcing ROS:

```bash
roslaunch ar_follower ar_track.launch
rosrun ar_follower follow_ar.py
```

The command history used in the experiment was exactly the two commands
above. The launch file starts `ar_track_alvar` and remaps the camera topics.
The Python node subscribes to `/ar_pose_marker` and `/scan`, then publishes to
`/cmd_vel_mux/input/teleop`.

## Main parameters

`ar_track.launch` defaults:

| Parameter | Default | Meaning |
|---|---:|---|
| `marker_size` | `14.0` | Physical marker size used by ALVAR |
| `max_new_marker_error` | `0.08` | Maximum error for a new marker |
| `max_track_error` | `0.2` | Maximum tracking error |
| `cam_image_topic` | `/camera/rgb/image_color` | RGB image topic |
| `cam_info_topic` | `/camera/rgb/camera_info` | Camera calibration topic |
| `output_frame` | `/camera_rgb_optical_frame` | AR output frame |

`follow_ar.py` private parameters can be changed at launch time:

```bash
rosrun ar_follower follow_ar.py \
  _target_id:=1 \
  _target_dist:=0.4 \
  _Kp_linear:=0.5 \
  _Kp_angular:=1.0 \
  _max_linear:=0.3 \
  _max_angular:=1.0 \
  _min_obstacle_dist:=0.25
```

The default target marker ID is `1`, the desired distance is `0.4 m`, the
maximum linear speed is `0.3 m/s`, and the maximum angular speed is
`1.0 rad/s`.

## Control method

The controller uses the marker position reported by
`ar_track_alvar_msgs/AlvarMarkers`:

- `marker.pose.pose.position.z` is used as the forward distance.
- `marker.pose.pose.position.x` is used as the lateral error.
- Linear velocity is proportional to the distance error.
- Angular velocity is proportional to the lateral error.
- If the target marker is not visible, the robot commands zero velocity.

The laser scan is sampled over the front 60-degree sector. If the minimum
valid range is below `min_obstacle_dist`, the node sets linear velocity to
zero and commands a slow left turn to try to clear the obstacle.

## Limitations

This is the code used for the direct-following experiment, not the more
complete map-navigation implementation under `turtlebot/follow2`. It does not
use `velocity_gate.py`, `move_base`, AMCL, a mission-state bridge, or a
sensor-freshness watchdog.

In particular:

- The output topic is hard-coded as `/cmd_vel_mux/input/teleop`.
- If no laser scan has arrived yet, the initial obstacle distance is `999.0`.
- A stale or missing scan is not detected by a timeout.
- When an obstacle is close, the node stops forward motion but still commands
  angular velocity `0.5`.
- The node has no active obstacle-avoidance planner; it only performs this
  local stop-and-turn behavior.

Test at low speed with an operator ready to stop the robot. Do not run this
node together with another follower or navigation node that publishes to the
same velocity input.

## Experiment report

The detailed experiment summary is available at:

[AR marker Turtlebot2 following experiment report](../../docs/ar_marker_follow_experiment_summary.md)
