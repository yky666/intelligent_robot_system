# competition 比赛入口与可视化

比赛入口是 `humanoid/scripts/competition.py`，配置为 `humanoid/config/competition.yaml`。原 `grab_walking.py` 保留作课程参考，不作为比赛入口。

执行链已接入原始动作和候选动作：蓝色识别 → 对准/接近 → 稳定条件触发 `grab` → 新画面持物确认 → 抱物前进/左转/右转 → 终点放置许可 → 再确认持物 → `grab_release` → 放置结果确认 → done。初始抓取失败时 `retry_reset` 后重新识别对准，最多 `grasp_attempts` 次。图像通道异常或画面过期停止。

当前路线仍按标定动作次数推进；终点许可来自操作员或外部检测器，尚未识别终点标记。右转、放下、复位仍须实机验证；持物/放置/通道 ROI 仍须标定。单目通道检测不保证所有障碍可见，掉落点自动返回未实现，详见[反馈指南](grasp_safety_placement.md)。

## 打开窗口

只调识别和 ROI，不运行动作：

```powershell
python humanoid/scripts/competition.py --vision-only --debug
```

Pi 上比赛并可视化（先完成动作及 ROI 验证）：

```bash
python3 humanoid/scripts/competition.py --execute --debug
```

没有 `--execute` 的运行不会发送舵机命令。完整流程模拟用 `--simulate`，不打开实际摄像头。`--debug` 是窗口开关；不加此参数时保留终端日志，适用于无图形环境。

## 窗口内容

- **TonyPi competition - camera**：摄像头原画面、符合颜色/面积条件的蓝色候选框、中心点和 `BLUE target area=...` 标签。
- 左上角显示 `STATE`、`ACTION`、当前决策/稳定帧/抓取尝试或反馈确认进度，及画面年龄、通道检测结果。识别面积是比例，不是模型置信度；检测没有将色块分类成精确物体类别。
- 绿色 held 框、黄色 placed 框、红色 corridor 框、紫色 interaction 框对应配置中的检测区域。白线标出横向目标中心和抓取底边阈值；未标定区域不会绘制。
- **TonyPi blue mask**：蓝色分割掩膜。
- **TonyPi non-floor mask**：配置通道 ROI 后显示非地面掩膜。边缘也参与停车判定，所以非地面掩膜为空并不必然说明通道通过。

显示在主控制线程刷新，约 10 Hz。动作组执行等待、持物确认、终点等待及放置确认期间仍刷新窗口，不另开线程控制舵机。识别和确认使用原摄像头的新帧，绘制框不会改写用于控制的图像。

鼠标点击原画面打印该像素 HSV 和归一化坐标，方便标定。按 `Q`、`Esc` 或关闭主窗口停止程序；实机模式走正常停车流程，尝试停止总线舵机运动并保留力矩，不卸载。另用 Ctrl+C 也可停止。

## 图形环境

在 Pi 的桌面/VNC 图形终端运行窗口模式。纯 SSH 无 DISPLAY 时使用图形转发或切换到桌面，不能仅靠 `--debug` 创建远端桌面。OpenCV 需要 GUI 支持；`opencv-python-headless` 只用于离线测试。若原装 `cv2` 能运行 `imshow`，保留原装环境，不必为开窗口更换 SDK。

尚未对 Pi 实际 GUI/相机/舵机进行验收；离线测试验证了真实轮廓框缩放、绘制不改变原图、窗口退出传播停车以及动作等待期间刷新。
