# 方式四前车轨迹串行跟随

前车有有效地图定位时，后车可以订阅前车 AMCL 位姿，将带间隔的历史轨迹点送入后车 move_base：

```bash
roslaunch multi_robot_following navigation.launch robot_ns:=tb4 \
  map_file:=$HOME/intelligent_robot_system/maps/competition.yaml
roslaunch multi_robot_following follow_map.launch robot_ns:=tb4 \
  target_source:=pose leader_pose_topic:=/tb3/amcl_pose follow_distance:=0.9
```

前后车地图原点一致、各车独立 map→odom→base TF；前车的 AMCL 应有 `localization_refresh.py` 定期请求静止扫描更新。仅有前车 odom 时不能直接当 map pose 使用。

后车记录前车实测轨迹，等前车积累足够间隔再出发；前车位姿过时、跳变、协方差异常、后车定位异常均停止。初始目标观测距离应让后车在前车路径后方，不能要求跟随算法穿过前车到达路径。

如队友使用前车背贴标记，则可改用 `target_source:=marker` 并设置独立 marker_id。各车的速度输入只能有一个控制源，失去目标后先原地重新识别。
