# 以 500 中位生成动作，状态机调用动作组

比赛项目采用：已有动作 → 离线生成对称候选动作 → 上位机检查/试运行 → 部署 d6a → competition 状态机调用。相对中位角度用于编辑动作，不要求实测角度 CSV，也不要求 URDF 或机械零位拟合。`measurements.local.csv` 仅用于以后需要几何坐标标定时的可选工具。

## 镜像规则

定义刻度坐标中的相对中位角 `theta = (p - 500) * 0.24°`，正方向是刻度增加方向。此定义不宣称左右关节的几何正方向相同。按官网配对 `1↔9 ... 8↔16`，镜像时交换左右，并将相对中位角取负：

```text
theta_left_new  = -theta_right_old
theta_right_new = -theta_left_old
p_left_new     = 1000 - p_right_old
p_right_new    = 1000 - p_left_old
```

镜像直接用整数刻度计算，避免先换成角度再四舍五入的误差。每帧时长、顺序及未选中通道保持原样；18 通道文件的 17/18 也原样保留，生成器不将它们解释成头部关节。

## 生成抱物右转

使用 `--scope legs` 镜像腿部，原样保留手臂抱物姿态。下面的新文件名不会与已有候选冲突：

```powershell
python tools/generate_action_candidates.py --source humanoid/ActionGroup/grab_turn_left.d6a --output humanoid/ActionGroup/candidates/grab_turn_right_mirrored.d6a --scope legs
python tools/inspect_action.py humanoid/ActionGroup/candidates/grab_turn_right_mirrored.d6a
```

输出已存在时拒绝覆盖。可选 `--scope arms` 仅镜像手臂、`--scope body` 镜像全部 16 个身体通道。已有 `candidates/grab_turn_right.d6a` 使用相同腿部规则生成，不需要为使用新接口而重复生成。无参数运行仍生成原来两种候选，原文件已存在时不会覆盖。

左右安装偏差和负载可能破坏理想对称性，因此结果是候选动作。先在上位机检查动作，再做受支撑条件下的试运行和调整；通过后复制到 Pi 动作目录，并在本机 competition 配置中填入对应动作名。例如 `right: grab_turn_right_mirrored`，不含 `.d6a` 后缀。

放下物体是另一个动作语义，不能由左右镜像生成。现有 `grab_release.d6a` 是倒序抓取的初稿，仍需检查接地姿态、过渡和放手过程。

## competition 的反馈范围

competition 保持串行的动作组执行：视觉识别 → 横向对准/小步接近 → 稳定满足抓取条件 → 抓取动作 → 抱物路线 → 放下动作。运行时读取 d6a 帧并执行，不调用相对角度编辑工具。

当前视觉反馈用于对准和触发抓取；每次接近动作停止后重新读取画面，并重新累计稳定帧。视觉条件不满足时不会仅按等待时间开始抓取。

抓取动作结束后须通过新的持物反馈，才进入路线状态；失败时有限次数复位并重新识别对准。放下后须确认手中无物块且放置区域有物块，才发布完成。路线直走/转弯仍按动作重复次数推进，终点放置许可由操作员或外部适配器提供，尚未实现终点标记识别。详见[抓取、防碰撞与放置](grasp_safety_placement.md)。
