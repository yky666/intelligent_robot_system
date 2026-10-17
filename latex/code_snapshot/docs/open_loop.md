# 开环固定动作版本

入口 `humanoid/scripts/open_loop.py`。2026-10-12 更新为：stand 一次 → grab 一次 → grab_go_forward 十次 → grab_turn_left 十次 → grab_go_forward 十次 → grab_turn_right 十次 → grab_go_forward 十次 → release 一次，共 53 次动作调用，帧时长合计 44.75 秒。初始站立使用新机器原装 stand；动作次数与顺序不变。

直走及左右转使用 `/home/pi/tonypi_web_control/action`（单数）中的新生成文件，其下半身源自当前机器人 go_forward、turn_left_fast、turn_right_fast，手臂固定在实际 grab 尾帧；详见 [生成规则和源文件](../humanoid/ActionGroup/carry_generated/README.md)。抓取与放置仍使用原 `actions`（复数）目录的 grab、release。执行前读取并检查所有文件的 SQLite 帧格式；不打开摄像头、不定位物块、不确认抓取/放置、不重抓、不检测障碍、不调整头部，也不发送 Turtlebot 心跳。所有已确认身体关节 1–16 按文件原值发送，不额外锁定手臂。按源步态各帧时长等待，没有额外插入过渡帧或动作间停顿。

当前 Debian 12 新控制板机器必须使用 `runtime/open_loop.local.yaml`（hardware_backend 为 controller），不要直接使用共享配置中的旧 legacy 默认值。9 个历史网页动作保持在 `humanoid/ActionGroup/web_control_reference`；本次三个生成文件与源步态分别归档在 carry_generated 和 carry_gait_sources_20261012，附有 SHA256 清单。

```bash
cd /home/pi/intelligent_robot_system
# 文件检查
python3 humanoid/scripts/open_loop.py --config runtime/open_loop.local.yaml --check-actions
# 默认只打印顺序，不发送舵机指令
python3 humanoid/scripts/open_loop.py --config runtime/open_loop.local.yaml
# 实际运动：先让机器人对准物块，并确保整条行进路线可通行
python3 humanoid/scripts/open_loop.py --config runtime/open_loop.local.yaml --execute
```

正式运动前退出其他上位机、网页动作 worker 等舵机控制程序，避免同时控制串口。Ctrl+C 或 SIGTERM 请求停止，复用旧版 Board 的舵机停止接口；停止步态中间帧仍可能失衡。开环中的 carrying 和 done 仅表示执行阶段、指令序列完成，不代表抓取、转向或放置实际成功。

原闭环入口仍为 `competition.py`：寻找蓝色物块 → 对齐/接近并等待连续稳定识别 → 抓取 → 视觉确认抱持（失败则复位后重新识别，最多三次）→ 路线每个动作后检查抱持 → 等待放置区域确认 → 再查抱持 → 放置 → 确认手中无物且放置区域有蓝色物块 → 完成。闭环运动同时受图像障碍检查约束，反馈缺失或过期则停止；放置区域确认目前由外部/操作员信号提供。
