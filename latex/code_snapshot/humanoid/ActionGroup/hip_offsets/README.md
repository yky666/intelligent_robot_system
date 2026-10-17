# 上腿部 ±20 刻度偏置候选

按现有 TonyPi 映射，上腿部配对为 ID4 / ID12。这里修改每一帧这两个身体总线舵机的目标刻度，不改其他关节、帧顺序或时长。20 是舵机刻度，不是20°。左右使用相反符号以保持镜像关系；哪组实际减轻前倾须观察实机，文件生成本身不能证明重心更稳定。

两套候选分开存放，文件名都为原名加 `_offset.d6a`：

| 目录 | ID4 | ID12 |
|---|---|---|
| hip4_minus20 | 原值−20 | 原值+20 |
| hip4_plus20 | 原值+20 | 原值−20 |

每套包含 stand、grab、grab_go_forward、grab_go_forward_fast、grab_turn_left、grab_turn_right、release、put_down、grab_release、go_forward_one_small_step、left_move、right_move，共12个文件。抓取源使用当前抱物步态对应的实际 grab 归档，stand/接近/横移源自此次连接恢复后读取的原装文件。原文件完整保留，运行配置未切换；两个 manifest 记录源/输出 SHA256、偏置、帧数与时长。程序越界即拒绝，不截断到量程边缘，也不重复叠加 *_offset 文件。

此次已补充这台机器 stand、接近及横移动作的原始归档，位于 ../hip_offset_sources_20261012。部署时两套副本位于 `/home/pi/tonypi_web_control/hip4_minus20` 和 `/home/pi/tonypi_web_control/hip4_plus20`；不要让偏置 grab 与不同符号的步态混用。下面的工具命令用于未来重新生成其他参数，不会自动执行。

## 在机器人上生成原装站立副本

下面命令仅生成文件，不执行舵机动作。输出必须尚不存在：

```bash
cd /home/pi/intelligent_robot_system
python3 tools/generate_action_offsets.py \
  --source /home/pi/TonyPi/ActionGroups/stand.d6a \
  --left-delta -20 --output-dir /home/pi/tonypi_web_control/hip4_minus20_stand
# 比较方向时，另一个目录生成反向版：
python3 tools/generate_action_offsets.py \
  --source /home/pi/TonyPi/ActionGroups/stand.d6a \
  --left-delta 20 --output-dir /home/pi/tonypi_web_control/hip4_plus20_stand
```

任意单个动作可用 --source，多文件也可一起传入；--source-dir 会为目录内所有原始 .d6a 生成 *_offset 副本，跳过已有 *_offset 文件。输出目录不会覆盖任何已有同名结果或清单。

先比较静态 stand_offset 姿态确认方向，然后同一组参数用于 grab_offset 和各抱物步态。若前倾更明显，应回到原动作后比较另一组；不能因为每帧都有同一偏置就认定整个步态已稳定。整体姿态还受膝、踝、负载和脚底支撑影响。候选未做实机稳定性验证，未自动执行，未写入舵机硬件校准偏差。
