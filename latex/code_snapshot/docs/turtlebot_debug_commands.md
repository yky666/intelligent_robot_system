# Turtlebot 底盘、键盘遥控和建图调试命令

2026-10-11 最新复查：小组已实机验证底盘和键盘遥控命令可用；虚拟机 `192.168.118.129` 已有 `/mobile_base`、`/turtlebot_teleop_keyboard` 和 `/slam_gmapping`。`turtlebot_navigation` 与 `turtlebot_rviz_launchers` 均能找到。下面优先使用原有 TurtleBot 课程建图与 RViz 入口，每项长期运行的命令使用单独终端。

## 每个终端先加载环境

```bash
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT_BASE=kobuki
export TURTLEBOT_STACKS=hexagons
export TURTLEBOT_3D_SENSOR=kinect
export TURTLEBOT_SIMULATION=false
export TURTLEBOT_BATTERY=None
export TURTLEBOT_SERIAL_PORT=/dev/kobuki
```

`TURTLEBOT_3D_SENSOR=kinect` 必须在 source 之后设置，因为课程工作空间默认环境会设成 asus_xtion_pro。`TURTLEBOT_BATTERY=None` 跳过虚拟机没有的笔记本电池接口。单机直接使用默认本机 ROS master；多机联调另按实际网络配置 ROS_IP/ROS_MASTER_URI。

## 终端 1：ROS master

```bash
roscore
```

已有 roscore 时不要重复启动。

## 终端 2：底盘和机器人 TF

先确认底盘通电，USB 在 VMware 可移动设备菜单中连接到虚拟机，再检查：

```bash
lsusb
ls -l /dev/kobuki /dev/ttyUSB* /dev/serial/by-id/*
```

使用默认的 /dev/kobuki 链接启动：

```bash
roslaunch turtlebot_bringup minimal.launch
```

若没有 /dev/kobuki，但已确认底盘实际是 /dev/ttyUSB0，则使用：

```bash
roslaunch turtlebot_bringup minimal.launch serialport:=/dev/ttyUSB0
```

有多个串口时按设备 ID 确认，不直接猜 ttyUSB0。若连 ttyUSB* 都没有，需要先解决底盘 USB 透传；仅有 Kinect 并不能驱动底盘或提供里程计。

## 终端 3：原有课程建图（包含 Kinect 和深度转扫描）

```bash
roslaunch turtlebot_navigation gmapping_demo.launch 3d_sensor:=kinect
```

该入口自动启动 Kinect、depthimage_to_laserscan、GMapping 和 move_base，但不包含 RViz。不要再单独启动 `3dsensor.launch`、OpenNI 驱动或其他 mapping.launch，否则会重复占用相机或启动多个 GMapping。当前已运行 `/slam_gmapping` 时，请在原建图终端 Ctrl+C 结束后再切换；保留底盘和键盘终端。此阶段仅使用键盘探索，不发送 `2D Nav Goal`。

## 终端 4：键盘遥控

```bash
roslaunch turtlebot_teleop keyboard_teleop.launch
```

在该终端按显示的键位操作，先降低速度并确认方向；k 停止，Ctrl+C 退出。这个节点输出到 /cmd_vel_mux/input/teleop。建图时不同时启动方式二跟随导航，避免两个控制源干扰。

## 终端 5：自动打开配置好的 RViz 窗口

```bash
roslaunch turtlebot_rviz_launchers view_navigation.launch
```

在虚拟机桌面终端执行，会打开已有导航配置的 RViz 窗口。普通 SSH 会话没有图形显示环境时，请改在虚拟机桌面运行此命令。Fixed Frame 设为 `map`，Map 使用 `/map`，LaserScan 使用 `/scan`。遥控走遍场地，观察地图逐步扩展。

### PR #2 的 path_planning 入口

[PR #2](https://github.com/yky666/intelligent_robot_system/pull/2) 已合并，源码见 [slam/README.md](../slam/README.md)。它的 `roslaunch path_planning mapping.launch` 仅启动深度转扫描和 GMapping，相机和 RViz 分别启动；并不会自动打开窗口。

目前虚拟机 `~/catkin_ws/src/path_planning` 还是原课程版本，仅有 C++ 导航程序，没有这个 mapping.launch。PR 的 `slam/lab3.sh sync` 会拒绝覆盖它；不要直接照搬 PR 命令。当前优先用上面的原有 `turtlebot_navigation`，无需替换旧包。若后续使用 PR 版本，可按其 README 设置独立 `LAB3_CATKIN_WS` 编译。

此前提供的 `multi_robot_following mapping.launch` 也是有效的轻量建图入口，只启动 GMapping，需要另外启动相机和 RViz。三种建图入口任选一种即可，保存的实际场地地图都可供方式二跟随使用。

## 终端 6：检查和保存

逐条执行，hz 类命令需要 Ctrl+C 结束后再运行下一条：

```bash
rostopic hz /odom
rostopic hz /scan
rostopic hz /camera/rgb/image_raw
rostopic echo -n 1 /camera/rgb/camera_info
rosrun tf tf_echo odom base_footprint
```

在虚拟机桌面终端运行 `rviz`，Fixed Frame 设置为 map，添加 Map（/map）、LaserScan（/scan）、TF 和 RobotModel。必要时用 rqt_image_view 检查 /camera/rgb/image_raw。

另开终端保存实际场地地图：

```bash
mkdir -p ~/catkin_ws/maps
rosrun map_server map_saver -f ~/catkin_ws/maps/competition
ls -l ~/catkin_ws/maps/competition.yaml ~/catkin_ws/maps/competition.pgm
```

## 建图完成后切换方式二跟随

停止键盘遥控和 `gmapping_demo.launch`（同时结束其 move_base 和相机），保留 roscore、底盘和 RViz。重新启动完整 Kinect 驱动供 AR 识别使用，然后启动方式二跟随，分别保持两个终端运行：

```bash
roslaunch turtlebot_bringup 3dsensor.launch 3d_sensor:=kinect scan_topic:=/scan
# 另一个终端：
roslaunch multi_robot_following vm_follow2.launch \
  map_file:=$HOME/catkin_ws/maps/competition.yaml marker_id:=1
```

在 RViz 用 2D Pose Estimate 设置当前车初始位置，初始位姿话题为 /initialpose。AR 标签 ID、尺寸、相机话题按实物调整。此时仍需有效任务状态才会行进；当前 TonyPi 开环程序不发送心跳，操作员调试任务状态的方法见 [虚拟机跟随说明](vm_follow2.md)。不要让 gmapping 和 AMCL 同时发布 map→odom。
