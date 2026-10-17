# 红色抓取与视觉道路居中

## 完整路线的人工起步

当前红色道路配置默认 `startup_mode: manual`。完整入口 competition.py 先执行 `/home/pi/TonyPi/ActionGroups/stand.d6a` 一次，再执行当前 grab 一次。机器人和物块的抓取位置由操作员事先摆好，程序跳过寻找、接近和抓取位置对齐；新鲜视觉确认持物后，进入完整路线：第一段居中 → 左转并对齐 → 第二段居中 → 右转并对齐 → 第三段居中 → 黄色雪糕筒停车确认 → 持物确认与放置 → 放置视觉确认。

人工起步抓取未确认时直接停止，等待操作员重新摆放后启动，不自动重抓。这里没有短程测试的3次道路动作上限，完整路线仍有道路总超时、转向周期上限和视觉失效停止。要恢复原自动搜索/接近起步，可将 startup_mode 改为 automatic。旧蓝色示例未指定这个字段，仍保持原自动起步行为。

启动命令仍为 `python3 humanoid/scripts/competition.py --config runtime/competition.local.yaml --execute --debug`，前提是完成正式路线所需标定；代码不会因为启用了人工起步而取消这些检查。--vision-only 依然只看画面，不站立、不抓取。--simulate 会包含 stand/grab，但后续仍演练固定参考路线，不代表真实道路视频验证。

尚未更新的实机本机配置，需要补入 `startup_mode: manual` 和 `actions.stand: /home/pi/TonyPi/ActionGroups/stand`；不要给这个映射加 .d6a 扩展名，因为共享执行器会自行添加。机器人离线时无法写入该本机配置，须待连接恢复后部署。

当前头部设置已按实际画面确认：PWM servo1 为 1200 微秒；servo2 保持当前值。配置 head.tilt、pickup_tilt、carry_tilt 均为 1200，head.preserve_pan=true，状态机不会发送 servo2 命令。本次按操作员明确指令实际发送了 servo1=1200，没有执行身体动作。道路挡板表面新增低饱和度要求，红色抱持物不能用其边缘替代灰色挡板；仍需用实际道路画面标定 held_roi 和可见道路区域。

## 按挡板场地视角更新

当前默认识别依据是灰色挡板下沿、竖向框架/支架及开口几何，终点模式改为 `yellow_cone`。地面胶带、黄黑条纹、地砖缝及其他地面标识不是路线、转角或终点识别标准。

边界候选必须同时有较长的竖向结构在其下沿附近结束，且下沿上方呈相对平滑的挡板表面；普通地面线条不满足这些支撑条件。前方挡板使用同样条件检测。路口还检查侧挡板的远端是否先于前方挡板结束，从而识别“近处仍能看到侧挡板，但其后方已经有开口”的情况。两侧都是灰色且没有纹理时，仅颜色阈值不够；这仍是可标定的结构启发式，不是训练完成的挡板语义分割模型。

此场地的障碍 guard 也已切换到支撑结构检查：接近的前方挡板下沿、进入身体前方区域的受支持支架和过近雪糕筒触发停止，地面颜色占用和地面孤立边缘不再触发它。此检查不输出米制距离，不能保证发现所有类型的障碍物。

雪糕筒检测使用黄色 HSV 加“直立、上窄下宽、上下中心接近、足够面积和高度”的轮廓筛选；黄色地面胶带不会仅凭颜色触发终点。只在完成左/右转后的最后一段，且已前进一定周期，才允许终点确认。目标足够居中且底部达到 `cone.stop_bottom` 后，先停止前进，连续确认并保持至少 1 秒，再执行持物和放置确认。出现黄色候选但仍很远不会直接结束。当前 stop_bottom=0.65 是图像坐标示例，需要用实际相机画面对应的安全停车距离标定。

默认不再把两侧边界消失当作终点；此旧模式仅作为 `boundaries_disappear` 可选策略保留。看不到挡板且也没有可靠雪糕筒时，保持等待并超时停止。调试窗口增加 YELLOW CONE 候选框；黄色道路边界线必须核对是否确实画在挡板下沿。

提供的五个视角用于设计结构线索；它们尚未作为本地图片文件进行算法回放，也不是机器人视频的实机验收。测试使用合成挡板结构、地面线/黄黑条纹和黄色锥形轮廓，验证干扰不能单独产生道路或终点证据。实际俯仰角、遮挡、曝光和支架形状仍需实机标定。

当前前进已切回由 `go_forward` 腿部生成的 `grab_go_forward`，保持原 grab 手臂尾帧。开环顺序不变。新增闭环参数源为 `humanoid/config/competition_vision_lane.yaml`；旧 competition.yaml 保留旧蓝色示例，不用于当前红色场地。

本机闭环入口仍是 competition.py，使用 `runtime/competition.local.yaml`。抓取阶段识别红色、对齐、接近、抓取和确认抱持；抓取成功后视觉道路控制接管原固定步数路线。HSV 同时使用 0–10 和 170–179 两段红色色相，避免漏掉色相跨零的红色。抓取、持物、放置、障碍图中的目标排除均复用同一颜色规则，调试标签显示 RED。

道路阶段每次只执行一个动作组周期，停稳后丢弃动作前画面，再识别下一帧。道路边界检测用 Canny/Hough 提取 ROI 中的左右收敛线，以近处中心偏差和远近中心差修正方向。灰色地面和障碍不能仅靠颜色区分；这里的边缘只是实验几何线索，地砖缝、阴影和非连续障碍可能造成误判，必须用实际画面校验黄色边界线确实对应道路边界。

状态顺序：第一段道路居中 → 前方挡板且左侧开口，连续确认后左转 → 看见并稳定对齐新道路 → 第二段居中 → 对称条件触发右转 → 稳定对齐第三段 → 第三段居中 → 黄色雪糕筒满足停车和稳定条件 → 持物确认 → 放置 → 手中无红块且放置 ROI 出现红块，确认完成。

终点只允许在完成两次转向、最后直行至少指定周期之后判断。前两段丢失边界不表示到终点；相机失效、遮挡、图像质量不足或看见前方挡墙均不会触发这个条件。未校准的“无边界”也可能是漏检，故当前仅交付可视化调试方案，不声称终点识别已经可靠。

转弯采用已生成抱物左/右转步态，至少转过实测的最少周期后，再用新道路居中和方向稳定结果确认完成，并有最大周期限制。**仅凭这些单目边界不能保证精确 90°**；需在地面实际量角标定 turn_min_cycles/turn_max_cycles，若需要角度误差保证，还需 IMU 航向或视觉定位标记反馈。配置中的 3/12 是待标定示例，不是实测 90°数据。居中修正也使用同一转向周期，若单周期幅度过大，需另录较小幅度抱物转向动作后再标定。

## 相机与头部 pitch

头部 pitch 是 PWM 舵机 1，单位微秒（500–2500），与身体总线舵机中位 500 的刻度不同。`head_control.py` 可单独直接发送头部命令；默认只打印，必须带 --execute 才运动。不要与其他舵机控制程序同时使用。

```bash
cd /home/pi/intelligent_robot_system
# 打印指令，不动舵机：
python3 humanoid/scripts/head_control.py --servo 1 --pulse 1200
# 操作员观察方向，在中位附近小步测试，不要直接跳到量程端点：
python3 humanoid/scripts/head_control.py --servo 1 --pulse 1200 --backend controller --execute
```


## 调试与启用

```bash
cd /home/pi/intelligent_robot_system
# VNC 桌面终端：红色框、目标 mask、黄色道路边界和状态
python3 humanoid/scripts/competition.py --config runtime/competition.local.yaml --vision-only --debug
python3 humanoid/scripts/competition.py --config runtime/competition.local.yaml --check-actions
# 开环已恢复 go_forward 下半身，无道路闭环：
python3 humanoid/scripts/open_loop.py --config runtime/open_loop.local.yaml --check-actions
```

现场分别标定红色阈值、held/placed ROI、障碍 guard ROI、道路几何与转角触发区域、抱物转向步数和头部角度。灰色地面通道检查目前同时使用颜色和边缘，不能代替测距。仅在这些检查通过后设置 lane_following.calibrated 和 hardware_calibrated 为 true，再运行：

```bash
python3 humanoid/scripts/competition.py --config runtime/competition.local.yaml --execute --debug
```

本机配置默认不允许执行；--vision-only 不会移动头部或身体。模拟模式仍演练固定参考路线与抓放状态，不代表完成真实道路检测验证。普通 SSH 不会弹出 OpenCV 窗口，需要 VNC 桌面或图形转发。
