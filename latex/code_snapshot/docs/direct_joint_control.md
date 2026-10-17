# 根据官方教程直接控制关节

已核对用户提供的[官方上位机与动作编程教程](https://docs.hiwonder.com/projects/TonyPi/en/latest/docs/9.upper_computer_action_editing_course.html)，包括其 [TonyPi 1.20 界面图](https://docs.hiwonder.com/projects/TonyPi/en/latest/_static/media/15.upper_computer/1.1/image3.png)。该图能够确定16个机体舵机的安装位置与 ID，支持直接关节位置控制。

## 编号对应位置

左右按机器人自身方向表示，图为正面视图，所以图片右边是机器人左侧。

| 安装位置，从靠躯干到末端 | 机器人左侧 ID | 机器人右侧 ID |
|---|---|---|
| 手臂靠躯干的肩部轴 | 8 | 16 |
| 手臂中间轴 | 7 | 15 |
| 肘部轴 | 6 | 14 |
| 腿部靠躯干的髋部轴 | 5 | 13 |
| 腿部上端轴 | 4 | 12 |
| 膝部轴 | 3 | 11 |
| 踝部轴 | 2 | 10 |
| 足部轴 | 1 | 9 |

这些是安装位置名称，不是已经建立的 URDF/DH 坐标定义。图片没有充分给出每轴的 yaw/pitch/roll 正方向、装配零位或机械限位。`joints.yaml` 故意不填写这些实机标定数据，避免把位置值500直接当作所有关节的零度。

头部 PWM 舵机为另一套编号和接口，不在这16个机体总线舵机之中。本工具只控制总线舵机。当前动作文件中的17/18列保留用于解析和追溯，比赛执行器只向已确认的1～16机体通道发送指令，不把额外列当作头部 PWM。

## 直接位置与角度控制

[官方 Controller](https://github.com/Hiwonder/TonyPi/blob/main/HiwonderSDK/hiwonder/Controller.py) 提供 `set_bus_servo_pulse(id, position, time_ms)`；[底层 SDK](https://github.com/Hiwonder/TonyPi/blob/main/HiwonderSDK/hiwonder/ros_robot_controller_sdk.py) 提供 `bus_servo_set_position(time_s, [[id, position], ...])`，允许同一包控制多个关节。

本项目入口为 `humanoid/scripts/joint_control.py`。默认只打印指令，不打开串口，不运行硬件：

```bash
python3 humanoid/scripts/joint_control.py --joint left_arm_elbow --position 574 --duration-ms 1000
# 使用数字 ID 也可以
python3 humanoid/scripts/joint_control.py --joint 6 --position 574 --duration-ms 1000
```

以上位置只是演示指令格式，不代表适用于当前任意姿态。实际执行前，复制 `humanoid/config/joints.yaml` 为 `joints.local.yaml`，对要控制的关节填写：

- `zero`：你们定义关节零度时的位置刻度。
- `direction`：位置增加对应角度增加时为1，反之为-1。
- `min` / `max`：实机确认的可用位置区间，而非直接照抄舵机全行程。
- `hardware_verified`：确认该配置适用于本机后才设为 true。

LX-824HV 官方规格为位置0～1000对应舵机轴0～240°，因此默认 `degrees_per_unit: 0.24`。若实际舵机型号不同，先核对规格。[LX-824HV 规格](https://www.hiwonder.com/products/lx-824hv)。

关节角度转换：`q_deg = direction * (position - zero) * degrees_per_unit`。示例调用在完成标定后使用：

```bash
python3 humanoid/scripts/joint_control.py --config joints.local.yaml \
  --joint left_arm_elbow --angle 5 --duration-ms 1000
# 确认目标位置、当前姿态和限位后，添加 --execute 才发送硬件指令
```

Python 接口支持一次下发多关节位置：

```python
import yaml
from joint_control import JointControl

with open('joints.local.yaml') as f:
    config = yaml.safe_load(f)
control = JointControl(config, execute=False)
command = control.move_joint_group({'left_arm_elbow': 574, 'right_arm_elbow': 392}, 1000)
print(command)
```

将 `humanoid/scripts` 加入 Python 模块路径或从该目录运行。`hardware_backend: controller` 使用 `ros_robot_controller_sdk.Board`，以秒为时间单位，批量发送关节目标；`hardware_backend: legacy` 使用旧版 `hiwonder.Board.setBusServoPulse`，以毫秒为时间单位，逐个发送关节目标。两者均仅在实际发送时初始化。2026-10-10 检查的团队机器人官方动作程序采用旧版 Board（115200 波特率），本项目配置选择 legacy；不能仅因新版 SDK 文件存在而切换协议。旧版逐个发送不保证所有关节严格同时启动。

## 角速度控制的范围

当前 SDK 是有限目标位置加运动时间接口，没有在该接口中发现独立的关节速度设定或力矩设定命令。工具的 `move_joint_at_speed` 表示有限行程的期望平均角速度：

`duration_ms = 1000 * abs(target_deg - start_deg) / speed_deg_s`

实际按量化后的位置差计算时间，允许20～9999 ms。超出时间范围时拒绝指令，不默默限幅改变用户要求的速度。速度必须为正的幅值，转向由目标与起点决定；没有位置变化时不发送命令。

```bash
python3 humanoid/scripts/joint_control.py --config joints.local.yaml \
  --joint left_arm_elbow --start-angle 0 --angle 5 --speed 5
```

`--start-angle` 必须是实际已知起点，工具不会假装已经读取到这个角度；此模式不是闭环恒速控制。负载、舵机速度上限与内部插值会影响实际结果。需要更平滑的速度/加速度轨迹时可在上层生成分段位置目标，但反馈周期和内部控制仍需实测。

## 动作组解析与调用

官方教程4.1与4.3介绍的是一组按时间排列的目标姿态，编辑器单帧时间范围20～9999 ms，动作保存到 `/home/pi/TonyPi/ActionGroups`。4.6展示通过 `ActionGroupControlDemo.py` 调用动作；动作编号字典是手机 App 的动作名称映射，与关节 ID 不同。

网页4.6末尾出现 `/home/pi/ArmPi_mini/ActionGroups`，与同页 TonyPi 的路径不一致；本项目依据 TonyPi 的路径和 SDK 默认路径使用 `/home/pi/TonyPi/ActionGroups`。

本地 `.d6a` 已确认是 SQLite，主表包含 Index、Time、Servo1…Servo18。可以直接读出帧并导出CSV，不涉及硬件：

```bash
python3 tools/inspect_action.py humanoid/ActionGroup/grab.d6a
python3 tools/inspect_action.py humanoid/ActionGroup/grab.d6a --csv grab_frames.csv
```

CSV 文件若已存在，工具拒绝覆盖。文件没有关节零位、正方向、夹持力和语义标签，不能仅凭其数值自动建立完整机器人运动学。

直接控制工具与比赛程序应二选一控制串口和关节，不能同时启动两个控制器。腿部单关节控制会改变双足平衡，需先通过受支撑条件下的小范围标定确认姿态，再用于步态编排。


500 中位、URDF 坐标及基于动作先验的标定见 [关节标定](joint_calibration.md)。
