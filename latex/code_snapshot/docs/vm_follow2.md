# ROS Kinetic 虚拟机：方式二建图避障跟随

底盘、Kinect、键盘遥控、建图及保存地图的逐终端命令见 [Turtlebot 调试命令](turtlebot_debug_commands.md)。小组已实机验证底盘和键盘遥控，最新复查已看到对应 ROS 节点；原有 turtlebot_navigation 与 turtlebot_rviz_launchers 均可找到。

虚拟机：`ssh haiden@192.168.118.129`。密码仅用于 SSH 登录，不保存在仓库。系统为 Ubuntu 16.04、ROS Kinetic、Python 2.7。独立功能包部署在 `/home/haiden/catkin_ws/src/multi_robot_following`，源码对应仓库 `turtlebot/follow2/multi_robot_following`，不覆盖原来的课程功能包。

包内包括：gmapping 建图入口、固定地图上的 AMCL 定位、move_base/DWA 避障导航、前车地图轨迹跟随、AR 观测到地图的转换，以及输出速度检查。目标、定位、TF、代价地图、任务状态和扫描数据缺失/过期时保持停止。默认最大前进速度 0.15 m/s，不盲目倒车。

团队已确认本车直接跟随 TonyPi 背部 AR 标记；`vm_follow2.launch` 默认使用 `target_source:=marker` 并启动 AR 检测。因此完成建图后主要启动命令为 `roslaunch multi_robot_following vm_follow2.launch map_file:=$HOME/catkin_ws/maps/competition.yaml marker_id:=1`。标签 ID 和尺寸以实际打印的 ALVAR 标记为准。

## 工作空间

```bash
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rospack find multi_robot_following
# 修改包后重新编译
cd ~/catkin_ws
catkin_make -j2
source devel/setup.bash
```

原 `~/.bashrc` 已包含课程 rocon、kobuki、turtlebot 和 catkin_ws 的环境加载，无需重复添加。比赛节点使用 Python 2.7，不切换虚拟机系统 Python。不要同时启动课程里直接发布速度的跟随程序和本包的速度输出。

## 先建图

优先使用原有课程入口 `roslaunch turtlebot_navigation gmapping_demo.launch 3d_sensor:=kinect`，再用 `roslaunch turtlebot_rviz_launchers view_navigation.launch` 打开配置好的 RViz。前者已包含相机、深度转扫描、GMapping 和 move_base，不要重复启动其他建图或相机驱动。详细切换步骤见调试命令文档。也可使用下面的独立 GMapping 入口，此时另行启动相机驱动，确认 `/scan`、`/odom` 以及 `odom → base_footprint → 传感器` 的 TF 正常。

```bash
roslaunch multi_robot_following mapping.launch
# 在建图阶段按课程方式遥控走遍场地，然后另开终端保存地图
mkdir -p ~/catkin_ws/maps
rosrun map_server map_saver -f ~/catkin_ws/maps/competition
```

结束 gmapping 和遥控后才启动 AMCL，避免同时发布 map→odom。实际扫描话题或 TF 名称不同，使用 `scan_topic`、`base_frame`、`odom_frame` 参数指定。本包不启动底盘/相机驱动，不自动猜测设备。

## 跟随前一台 Turtlebot

两台机器人必须在同一 ROS 通信网络、同一张地图和同一地图原点中定位。前车定位话题必须唯一，例如 `/tb1/amcl_pose`；当前车驱动使用全局 `/scan`、`/odom` 和 `base_footprint` 时：

```bash
roslaunch multi_robot_following vm_follow2.launch \
  map_file:=$HOME/catkin_ws/maps/competition.yaml \
  target_source:=pose leader_pose_topic:=/tb1/amcl_pose
```

在 RViz 选择 Fixed Frame `map`，用 2D Pose Estimate 设置当前车初始位置，话题为 `/initialpose`。显示 Map `/map`、LaserScan `/scan`、TF、目标 `/target_pose`、move_base 路径与代价地图。输出经过速度检查后发送到 `/cmd_vel_mux/input/navi`，应与课程底盘的导航输入一致；不同话题通过 `cmd_vel_topic` 指定。

跟随逻辑记录前车的真实地图轨迹，在其路径上取落后约 0.9 米的点作为导航目标，move_base 根据地图和实时扫描规划绕障；不会仅用机器人动作次数估算前车位移。多台车必须配置不同命名空间及 TF，详见 README 的多机说明。

## 直接观测人形机器人的标记

如果当前车直接看 TonyPi 的背部标记，改用另一目标输入，仍通过地图导航避障：

```bash
roslaunch multi_robot_following vm_follow2.launch \
  map_file:=$HOME/catkin_ws/maps/competition.yaml \
  target_source:=marker start_ar_tracker:=true marker_id:=1
```

AR 检测使用 ALVAR 标记，相机内参和 optical frame 必须正常。普通 QR 二维码不能直接作为 ALVAR 标记使用。目标输入仅启动一种，不能把 pose 和 marker 同时作为同一跟随器输入。

## 任务状态与网络

跟随器只有持续收到 `/competition/mission` 的 carrying/releasing/done 阶段才允许行进，且仍需目标、定位、TF、扫描等其他条件全部有效。正式闭环版可通过 `mission_bridge.launch` 接收 TonyPi UDP 心跳。当前人形机器人 `open_loop.py` 不发送心跳，因此仅启动导航和跟随 launch 会保持停止。

现场独立调试时可由操作员在终端持续发布任务状态（停止该命令后状态超时即停止）：

```bash
rostopic pub -r 10 /competition/mission std_msgs/String "data: 'carrying'"
# 紧急停止
rostopic pub -1 /competition/estop std_msgs/Bool "data: true"
```

本次 SSH 地址 `192.168.118.129` 是 VMware NAT 地址，虚拟机目前只有 ens33。该地址可供 PC 访问，但不能据此认为 TonyPi 或其他 Turtlebot 能反向访问虚拟机。多机调试需给虚拟机增加连接机器人共同局域网的桥接网卡，按实际局域网地址设置 `ROS_IP` 和共同的 `ROS_MASTER_URI`；可保留 NAT 网卡供联网。不要把所有主机的 ROS_IP 都写成 192.168.118.129。

## 本次验证

新增包已与原来的五个课程功能包一起通过 `catkin_make -j2`。Kinetic 上 Python 2.7 语法检查通过；建图、AR 跟随、地图位姿跟随三个入口的 launch 解析通过。已修复空命名空间的 Kinetic 兼容问题。

使用独立的本机 ROS master（端口 11339）启动跟随器和速度检查节点，在没有目标、定位、任务状态和扫描输入时观测到连续零速度、motion_allowed=false、follow_status=mission_stopped。测试未启动真实底盘或向机器人速度接口发送指令。全项目本地 163 项测试通过。隔离检查日志保存在虚拟机 `~/catkin_ws/.follow2-qa`，构建日志为 `~/catkin_ws/follow2-build.log`。

早期 USB/master 检查仅代表当时设备状态。小组随后已验证底盘、键盘遥控可用，最新复查也看到对应节点；原有课程建图与 RViz 入口的 `roslaunch --nodes` 解析通过。本次复查未重新启动真实驱动、未打开 GUI 或发送运动目标，实际地图质量、定位和避障仍需现场确认。
