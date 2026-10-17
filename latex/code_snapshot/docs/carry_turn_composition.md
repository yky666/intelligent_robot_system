# 固定抱物手臂的组合转向动作

2026-10-10 从实机 `/home/pi/TonyPi/ActionGroups` 读取了 `grab`、`turn_left`、`turn_right` 和 `put_down`。实机 `grab` 与仓库文件逐字节一致。原转向文件保存在 `humanoid/ActionGroup/reference`，放置文件保存为 `humanoid/ActionGroup/put_down.d6a`。

新候选文件为 `humanoid/ActionGroup/candidates/grab_turn_left_composed.d6a` 和 `grab_turn_right_composed.d6a`。每一帧的腿部 ID 1–5、9–13 完整保留各自原转向轨迹，不通过镜像猜测右转。手臂 ID 6、7、8、14、15、16 固定为 `grab` 尾帧：574、921、477、392、79、596。额外 17/18 列保留抓取尾帧值，但比赛程序只发送已确认的 16 个身体关节。

原转向动作各 6 帧、1000 毫秒。新动作开头增加 600 毫秒的过渡目标帧，之后原 6 帧时长保持不变，总共 7 帧、1600 毫秒。过渡使用转向第一帧的腿部目标和固定手臂，减少从抓取站姿直接用 100 毫秒进入转向起始姿态的突变；不是基于实际关节反馈的规划。

重新生成时使用新输出路径，工具拒绝覆盖已有文件：

```bash
python3 tools/generate_action_candidates.py --source humanoid/ActionGroup/reference/turn_left.d6a --carry-pose humanoid/ActionGroup/grab.d6a --output /tmp/grab_turn_left_new.d6a --entry-ms 600
python3 tools/generate_action_candidates.py --source humanoid/ActionGroup/reference/turn_right.d6a --carry-pose humanoid/ActionGroup/grab.d6a --output /tmp/grab_turn_right_new.d6a --entry-ms 600
```

比赛配置 `left/right` 改为两个新候选，`release` 改为 `put_down`。`retry_reset` 仍使用原反向抓取候选，没有把正常放置动作直接视为失败抓取的复位动作。实际运行时的手臂锁定仍生效，且仅作用于抱物直走、左转、右转；放置时允许手臂按 `put_down` 变化。

固定手臂会改变原空手转向动作的重心与惯性，复制腿部动作不能保证实际转向角度、抱持稳定性或平衡。新动作仍是未验证候选，保持 `hardware_calibrated: false`；现场扶稳机器人先验证一次转向，再标定需要的循环次数。`put_down` 的起始姿态和抓取尾帧的连接也需要验证。原来的抱物转向和反向放置候选保留用于比较。
