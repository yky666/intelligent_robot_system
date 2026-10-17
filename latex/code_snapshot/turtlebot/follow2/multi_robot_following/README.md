# multi_robot_following：方式二地图避障跟随

团队 ROS Kinetic 虚拟机部署路径：`~/catkin_ws/src/multi_robot_following`。

```bash
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
# 先启动课程的底盘、相机和扫描驱动，再建图
roslaunch multi_robot_following mapping.launch
# 另开终端保存实际场地地图
mkdir -p ~/catkin_ws/maps
rosrun map_server map_saver -f ~/catkin_ws/maps/competition
# 结束建图和遥控后，用 AR 标记作为跟随目标
roslaunch multi_robot_following vm_follow2.launch \
  map_file:=$HOME/catkin_ws/maps/competition.yaml marker_id:=1
```

默认使用不带前缀的单车 TF 和 `/scan`；标签需是 ALVAR AR 标记。设置 RViz 初始位姿后，只有持续有效的目标、定位、TF、代价地图、扫描及任务状态全部满足，才允许输出导航速度。当前 TonyPi 开环入口不发送任务心跳，单独启动上述 launch 会保持停止。

完整网络、任务状态、调试和多机配置见仓库 [虚拟机部署说明](https://github.com/yky666/intelligent_robot_system/blob/main/docs/vm_follow2.md)。本包已在 Ubuntu 16.04 / ROS Kinetic 下编译并做隔离启动检查，实机运动仍需底盘和传感器接入后验证。
