# 现场操作手册

## 赛前完成

1. 确认 `ROS_DISTRO` 与 Pi 的 Python、SDK 版本；不要猜虚拟机版本。
2. 补齐并验证右转和放下动作，记录实测数据到 [validation.md](validation.md)。普通步态与夹持动作的衔接需要带真实物体测试。
3. 用固定头部姿态标定蓝色 HSV 阈值（也可选 LAB）、画面底边抓取阈值、面积阈值及持物/放置 ROI。低光照和比赛光照各测试一次。
4. 场地摆好障碍后建图；地图 yaml/pgm 复制到所有需要加载地图的主机。
5. 所有设备联网校时；每车独立 TF 和命名空间，分别 AMCL 初始化。单车环境先用根 README 的无命名空间配置。
6. 检查地图包含通道，脚掌和双臂摆动不碰障碍。TonyPi 当前路线为标定后的动作序列，没有实现在线场地建图或自动新障碍绕行。
7. 整队终点区应足够长，允许后车仍保持间隔。地图 follow_distance 不是“车头到车尾净空”，需扣除两个机器人本体尺寸后判断。

## 每次开始前

```bash
git pull --ff-only
python3 tools/validate_project.py
# 在 Pi，用真实 ActionGroups 路径检查是否缺少动作，不会打开硬件驱动
python3 humanoid/scripts/competition.py --check-actions
```

检查动作只验证格式和文件存在，不能代替上位机实测。若 `--check-actions` 缺少动作，先补齐实际动作再运行，不要修改配置绕过文件检查。

ROS 端先启动 master、每车底盘/传感器、navigation、AR tracker、follow_map、单个 mission_bridge。Pi 尚未 carrying 时所有后车应保持不动。

```bash
rostopic list
rostopic hz /tb2/scan
rostopic hz /tb2/amcl_pose
rostopic echo /tb2/target_pose
rosrun tf tf_echo map tb2/base_footprint
rostopic info /tb2/cmd_vel_mux/input/navi
```

导航输入应只有门控发布。其他 mux 输入、直接底盘速度发布者也要检查，不能只查一个话题。RViz 的地图/初始位姿/导航话题分别设置为对应机器人的 namespaced 话题。

## 正式运行

当前硬件无测距传感器，裁判发令前确认相机通道检测已标定、抱物后仍看得清前方；若加装测距并切换模式，再启动对应适配器。发令后在 Pi 运行 `competition.py --execute`。程序识别蓝色、接近并抓取，持物反馈成功后才发布 carrying。失败时有限次数复位并重新识别。后车积累足够前车轨迹后逐台启动；没有可靠目标时不自行巡游搜索。终点进入 checking_placement 时按[反馈指南](grasp_safety_placement.md)提供放置许可。

Pi 完成放下后继续发 done，不立即退出。每车达到带间隔的终点跟随点后，其 follow_status 应为 finished。观察整个队列已在终点区域且停稳后，结束 Pi 进程和 ROS 节点。

## 常见停车原因

| 状态 | 检查和处理 |
|---|---|
| `mission_stopped` | Pi 是否真实执行、心跳 enabled/IP/session、桥是否只运行一个 |
| `waiting_for_leader_trail` | 前车尚未走出配置间隔，正常等待；初始摆位应在前车后方 |
| `target_lost_hold_position` | 标记被挡、目标话题不更新、ID 错误；原地恢复可靠观测 |
| `localization_invalid` | AMCL 初始位姿、协方差、scan、nomotion_update 服务 |
| `base_tf_missing` / `base_tf_stale` | TF 前缀和时间同步，底盘里程计是否刷新 |
| `costmap_missing_or_stale` | move_base/costmap 未启动、地图路径或扫描问题 |
| `follow_goal_blocked` | 跟随点未知/障碍/膨胀区域；检查地图、定位和间隔 |
| `leader_too_close` | 两车太近，等待前车移开；后车不会盲目倒车 |
| `navigation_failed_reset_required` | 规划失败/卡住，排除故障并按裁判规则重启 follower |

若 follow_status 为 following 但底盘仍不动，查看 `/cmd_vel_raw`、门控输出以及有效扫描；前方障碍、扫描全 Inf 或过时会被独立门控阻止。禁止为了继续跑而启动遥控代替自主跟随。

## 故障与规则

目标丢失须原地重新捕获，实验手册规定超过 1 分钟未重新捕获则该车失败。本代码不会移动搜索，也没有代替裁判计分。大幅定位重置需要清除旧轨迹再重启，不能带着旧 map 轨迹继续。

物体掉落时须返回掉落点重新抱取后继续；本版本没有自动掉落点回溯。搬运动作后持物视觉/传感器反馈失败会停车；后续如何返回由你们实测补充，并按比赛规则执行。初始抓取失败的有限次数重抓已实现，但相机遮挡或断流不触发盲目重抓。抓取动作执行完成本身不代表持物成功。

课程要求每项最多两次完整重试；比赛中人工扶、推、提机器人会影响成绩。故障重置向裁判申请，不用人工调整代替正常自主运行。
