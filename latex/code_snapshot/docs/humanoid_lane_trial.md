# 人工摆放、站立抓取后的真机短程闭环测试

## HW-9EBBA609 相机共享输入

此机器的 tonypi_web_control/server.py 网页服务占用 /dev/video0，并提供 `http://127.0.0.1:9040/api/camera` JPEG 图像。本机 competition.local.yaml 和 lane_trial.local.yaml 改用下面的输入，不再重复打开 USB 相机：

```yaml
camera:
  type: http_snapshot
  url: http://127.0.0.1:9040/api/camera
  timeout: 0.8
  max_fps: 10
```

接口关闭/503、不能解码或反复返回同一张 JPEG 时，不刷新摄像头帧时间；原有超时和持物/道路检查仍有效。网页摄像头服务应保持运行，但不要同时通过网页或其他上位机发送身体动作。USB 直读模式仍支持 auto/设备路径，只有相机没有其他占用进程时使用。

入口为 `humanoid/scripts/lane_trial.py`，配置为 `runtime/lane_trial.local.yaml`。明确带 --execute 才发送实际身体指令。默认先执行原装 stand 一次，再直接执行 grab 一次，视觉确认已持物后进入短程道路居中。物块和机器人位置由操作员事先摆好，不执行搜索、接近或抓取位置自动对齐。抓取确认失败则停止，不盲目重抓或走路。不执行放置和整圈转角状态机，不移动头部，保持当前 servo1=1200 与 servo2 位置；抱物手臂固定在实际 grab 尾帧。没有把正式比赛配置的 hardware_calibrated/lane calibrated 改为 true。

测试使用新鲜红块持物观察和挡板边界观察，每次道路决策连续稳定后仅执行一个周期，再重新识别。最多执行指定次数（默认3，最多10），超时默认20秒（最多60秒）。既没有可靠道路，也没有前进；前方挡板、相机失效、持物丢失或修正后横向偏差增大时停止。不是绕过缺失视觉的强制行走开关。

本机测试 ROI 根据当前截图设为上方道路区域 [0.04,0.10,0.96,0.55]，held_roi 为下方 [0,0.57,0.92,1]。这是操作员试运行参数，不是标定完成的数据；窗口应核对黄色边界确实对应左右挡板。为了适应截图中偏紫的灰色挡板，表面最大饱和度暂设120，真实红色物块仍被排除。曝光/白平衡和实际板面颜色不同需调整参数。

先退出旧 competition 预览，以及会占用相机或同时发送舵机指令的 LAB_Tool、上位机或遥控程序。把红块放在 grab 可直接抓取的位置，机器人放在道路起点，操作员在旁观察。默认 stand 会改变身体和手臂姿态，因此不要已经抱着物块启动这个默认流程。在 VNC 桌面终端：

```bash
cd /home/pi/intelligent_robot_system
python3 humanoid/scripts/lane_trial.py --config runtime/lane_trial.local.yaml --execute --debug --max-actions 3 --timeout 20
```

每个动作前终端打印 `[trial] ... action=... error=...`；窗口显示 TRIAL 进度与新鲜度。q/Esc 或 Ctrl+C 停止。停止保留抱物姿态，不自动插入 stand 或 release。若显示 waiting_for_corridor 并以0次动作结束，说明当前图像仍没有可信道路，需核对边界/ROI，不能用开环前进替代这一判断。

--max-actions 仅计抓取成功后的道路动作，默认3次不包含初始 stand 和 grab。若明确希望从已经抱物的姿态继续，可额外加 --already-held，此时跳过 stand/grab。道路“自动校正”是每个动作后重新识别并修正方向，不是自动完成所有参数标定。

去掉 --execute 时只运行同一视觉决策并打印动作，不发送舵机命令。此测试不运行 Turtlebot 心跳、不执行90°转角、不代表正式比赛整条路线已通过验收。
