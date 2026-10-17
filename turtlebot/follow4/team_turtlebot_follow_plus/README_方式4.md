# TurtleBot2 跟随方式4功能包

本包用于方式4车队中二号车和三号车的 AR 标签跟随。系统为 ROS Kinetic/C++：`ar_track_alvar` 根据 Kinect 图像检测标签，`depthimage_to_laserscan` 将深度图转换为 `/scan`，本包的跟随节点根据指定标签位姿和扫描数据生成 TurtleBot2 速度指令。

车队中一号车通过点云视觉跟踪 TonyPi，不使用本包的 AR 标签跟随节点；一号车尾部贴 ID 0，二号车尾部贴 ID 1，三号车尾部不贴标签。二号车跟随 ID 0，三号车跟随 ID 1。AR 标签黑色方框边长为 14 cm。

## 安装与编译

将本目录放入 `~/catkin_ws/src/`，在 ROS Kinetic 环境中编译：

```bash
source /opt/ros/kinetic/setup.bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

运行前需确认 TurtleBot2 底盘、Kinect、AR 检测及 `/scan` 转换节点均已启动，且 `/ar_pose_marker` 与 `/scan` 持续更新。报告的运行流程列出了各终端命令。

## 启动跟随节点

二号车执行：

```bash
roslaunch team_turtlebot_follow_plus follow_ar.launch \
  marker_id:=0 marker_size:=14.0 desired_distance:=0.40 \
  max_linear_speed:=0.35 max_angular_speed:=0.25 \
  avoid_distance:=0.38 stop_distance:=0.20 side_clearance:=0.40 \
  avoid_release_margin:=0.20 obstacle_turn_speed:=0.20 \
  marker_timeout:=0.8 lost_search_timeout:=16 search_angular_speed:=0.20
```

三号车使用相同参数，将 `marker_id:=0` 改为 `marker_id:=1`。如果使用 launch 文件默认值，`marker_id` 默认是 0，其余默认值与上列参数一致。

## 参数说明

| 参数 | 默认值 | 含义 |
| --- | ---: | --- |
| `marker_id` | `0` | 要跟随的 AR 标签编号。 |
| `marker_size` | `14.0 cm` | 标签黑色方框边长，用于 AR 距离估计。 |
| `desired_distance` | `0.40 m` | Kinect 到标签的目标距离；程序不倒车，标签近于该距离时停止前进。 |
| `max_linear_speed` | `0.35 m/s` | 最大前进速度上限。 |
| `max_angular_speed` | `0.25 rad/s` | 正常跟随和转向的角速度上限。 |
| `avoid_distance` | `0.38 m` | 前方障碍进入该距离后开始减速并选择扫描较空的一侧。 |
| `stop_distance` | `0.20 m` | 前方障碍达到该距离后停止前进；侧向净空足够时可转向。 |
| `side_clearance` | `0.40 m` | 选定转向侧所需的最小扫描净空；两侧均不足时停车。 |
| `avoid_release_margin` | `0.20 m` | 障碍距离超过避障距离加该余量后退出避障状态。 |
| `obstacle_turn_speed` | `0.20 rad/s` | 达到停车距离后向选定侧转向时的角速度上限。 |
| `marker_timeout` | `0.8 s` | 标签多久没有新检测结果后视为丢失并开始搜索。 |
| `lost_search_timeout` | `16 s` | 目标搜索的最长时间；超时仍未找到则停车。 |
| `search_angular_speed` | `0.20 rad/s` | 标签丢失后原地搜索的角速度，不超过 `max_angular_speed`。 |

## 节点行为与测试范围

跟随节点以 10 Hz 运行，根据标签距离误差调节线速度、根据标签方位调节角速度。扫描数据分为前方及左右侧区域；进入避障状态后保持选定转向方向，扫描超时则发布零速度。目标首次出现前保持停止，已检测目标丢失后原地搜索，搜索超时后停止。

三车联调中，直行时队列间距稳定且未发生车辆间碰撞；转弯采用圆弧轨迹，终点按顺序停车。本次未专门布置障碍物，因而没有完成专门避障验收。实测中围栏以及长时间停留在近处的前车曾触发误避障，后续需调整参数并复测。目标未发生丢失，重捕获逻辑没有实际触发记录。

Kinect 生成的 `/scan` 视野有限，不能覆盖视野外或深度图缺测区域；本次测试结果不代表复杂障碍场景下的避障能力。
