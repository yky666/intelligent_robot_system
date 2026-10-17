# 2026-10-12 新 TonyPi 抱物步态

对应实机目录 `/home/pi/tonypi_web_control/action`（本次新建，单数）。抓取和放置仍使用之前 `actions` 目录的文件；动作顺序和重复次数不变。

| 生成文件 | 新机器人原装下半身源动作 | 帧数 | 原帧时长总计 |
|---|---|---|---|
| grab_go_forward.d6a（当前前进） | go_forward.d6a | 10 | 0.50 秒 |
| grab_go_forward_fast.d6a（保留备选） | go_forward_fast.d6a | 10 | 0.45 秒 |
| grab_turn_left.d6a | turn_left_fast.d6a | 5 | 0.90 秒 |
| grab_turn_right.d6a | turn_right_fast.d6a | 5 | 0.90 秒 |

源文件逐字节归档在 `../carry_gait_sources_20261012`。抓取源是实机 `/home/pi/tonypi_web_control/actions/grab.d6a`，已复制到新 action 目录供核对；生成使用其尾帧，不复制抓取过程。手臂 6、7、8、14、15、16 固定为 574、921、477、426、79、523。腿部 1–5、9–13 的每帧目标与原始步态完全相同。

步态文件包含 18 路，grab 包含 16 路；输出仅包含已确认的 16 路身体舵机，不构造额外 17/18 通道目标。使用 `--entry-ms 0`，不增加过渡帧，保持每次步态原帧数和时长。加载物块后的稳定性和转角尚未实测；这些文件是生成动作，不是已验证动作。

重建示例（输出必须是尚不存在的新文件）：

```bash
python3 tools/generate_action_candidates.py \
  --source humanoid/ActionGroup/carry_gait_sources_20261012/go_forward.d6a \
  --carry-pose humanoid/ActionGroup/carry_gait_sources_20261012/grab.d6a \
  --entry-ms 0 --output /tmp/grab_go_forward.d6a
```

`manifest.json` 记录源文件/结果哈希、固定姿态、帧数和时长。新控制板机器运行时使用 `--config runtime/open_loop.local.yaml`，该本机配置选择 controller；闭环本机配置也已改用相同 grab 和三个生成步态，手臂保持值与抓取源一致。
