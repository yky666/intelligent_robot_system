# TonyPi 关节坐标与标定

当前项目仅为生成对称动作使用相对 500 中位坐标，按 [动作镜像工作流](action_mirroring.md) 操作即可，无需本页的测量 CSV。以下是对齐真实机械角度或模型时的可选标定流程。

500 可以定义为**舵机中位参考零位**。官方编辑器明确将回中位置标为 500，并提供独立的舵机偏差设置。它是否也是“膝盖伸直为 0°”或某个 URDF 的 q=0，取决于舵盘安装、偏差和模型约定。

来源：[官方动作编辑教程](https://docs.hiwonder.com/projects/TonyPi/en/latest/docs/9.upper_computer_action_editing_course.html)、[LX-824HV 规格](https://www.hiwonder.com/products/lx-824hv)。0..1000 对应标称 240°，先使用 0.24°/刻度；实际型号和装配仍需核实。下面所有分析离线运行，不调用硬件。

## 现有动作提供的证据

已检索比赛目录、课程资料目录、VMware 共享目录，未找到 URDF/Xacro 或 STL/DAE 模型。也检查了 [Hiwonder/TonyPi](https://github.com/Hiwonder/TonyPi/tree/dc452eaac9991516600860f706f4b889f3305d1d) 此提交的完整文件树，未找到这些模型。其他型号的模型不能直接当作 TonyPi 标定模型。

解析原始三个动作，不包含生成的候选动作。左右名称按机器人自身视角；配对来自官方舵机图，不是通过相关性自动识别。

| 左/右 ID | 部位 | grab 第一帧左/右刻度 | 全 grab 镜像残差 RMSE（刻度） |
|---|---|---|---|
| 1 / 9 | 足端 | 500 / 500 | 9.143 |
| 2 / 10 | 踝部 | 390 / 610 | 11.045 |
| 3 / 11 | 膝部 | 500 / 500 | 14.332 |
| 4 / 12 | 上腿部 | 600 / 400 | 11.916 |
| 5 / 13 | 腿根部 | 500 / 500 | 8.402 |
| 6 / 14 | 肘部 | 575 / 425 | 28.737 |
| 7 / 15 | 臂中部 | 800 / 200 | 20.401 |
| 8 / 16 | 臂根部 | 725 / 275 | 58.900 |

残差定义为 `sqrt(mean((p_left + p_right - 1000)^2))`。第一帧所有配对和为 1000，支持围绕 500 的镜像先验。但后续姿态并非严格镜像：抱物直走中 8/16 之和为 1060..1089，左转为 1062..1094。因此 `p_right=1000-p_left` 只能作为对称动作的初始猜测，不能作为所有动作的硬约束，也不能从它确认关节在世界坐标系中的正方向。

抓取中 6 号仅变化 571..575，而 7 号变化 800..921、8 号变化 477..726。几乎不变化的关节不具备足够的标定激励；动作名字也不能给出精确角度。观察到的最小/最大刻度只表示这些动作经过的范围，不是安全限位。

完整数值：[joint_action_analysis.json](joint_action_analysis.json)。重新分析（Windows PowerShell 使用 `python`，单行命令）：

```powershell
python tools/analyze_joint_mapping.py actions humanoid/ActionGroup/grab.d6a humanoid/ActionGroup/grab_go_forward.d6a humanoid/ActionGroup/grab_turn_left.d6a
```

## 模型和刻度映射分别标定

控制配置的映射为：

`q_deg = direction * 0.24 * (position - zero)`

- 中位参考坐标可先选择 `zero=500`；例如 574 比中位偏移 17.76°。实际关节角度还需要确定 direction，以及所选机械零姿态。
- URDF 的 `origin xyz/rpy` 定义 q=0 时关节坐标系相对父连杆的位置与方向，`axis xyz` 定义关节局部坐标系的转轴。需要连杆尺寸、转轴位置和安装朝向；q 用弧度。参见 [URDF joint 数据结构](https://github.com/ros/urdfdom_headers/blob/master/include/urdf_model/joint.h)。
- 即使取得匹配的 URDF，也需要建立关节名称到舵机 ID 的对应关系和刻度转换；模型不自动证明实机的装配偏差。改变模型零姿态时，控制配置的 zero 也应按同一约定调整。

只有刻度轨迹，没有姿态观测，无法唯一恢复连杆长度、轴的位置、正方向和机械零位。可以在定义好的镜像坐标约定下提出符号先验；定义每个关节局部轴的正方向后再验证。不能仅凭左右刻度相反，断言两个 URDF 轴的 direction 必须相反。

若取得匹配模型并测得多个姿态的手/脚位置或关节角度，可最小化正运动学预测与测量的误差，估计零位偏差和符号；模型尺寸未知时还需额外测量。单目图像有尺度、遮挡和投影歧义，现有三个动作也不能保证所有参数都可辨识。

## 现在可用的实测拟合工具

先选定零姿态和正方向，例如在固定观察视角中规定肘屈曲为正。机器人受支撑、停止其他动作控制时，记录同一关节至少两个不同姿态，建议三个以上并留一个独立姿态验证。记录当时真实刻度、测量角度和舵机偏差设置；标定后不要改变偏差设置而继续使用旧结果。

建立 UTF-8 CSV，列为 `joint,position,angle_deg`。以下仅是**数学演示数据，不是 TonyPi 实测值**：

首次使用先复制空模板，再用文本编辑器或 Excel 填写实际测量值（保存为 UTF-8 CSV）。每个关节至少两行不同姿态，推荐三个以上；角度不能用刻度公式生成，否则不能提供独立标定信息。

```powershell
Copy-Item humanoid/config/measurements.template.csv measurements.local.csv
notepad measurements.local.csv
```

如果 `measurements.local.csv` 已有测量记录，请直接打开编辑，不要再次复制覆盖。`*.local.csv` 已加入 Git 忽略规则。

```csv
joint,position,angle_deg
left_arm_elbow,420,24
left_arm_elbow,520,0
left_arm_elbow,620,-24
```

```powershell
python tools/analyze_joint_mapping.py fit measurements.local.csv
```

此示例会得到 `zero=520, direction=-1`。工具固定标称 0.24°/刻度，分别拟合正负方向的零位，返回角度残差及另一方向的残差，便于评估。它不估计舵机增益、齿隙或非线性；若残差明显，先检查角度定义、测量、型号和安装，不能直接接受结果。方向残差接近时，应补充跨度更大的有效测量。

演示数据已保存为独立文件，可直接验证工具运行：

```powershell
python tools/analyze_joint_mapping.py fit humanoid/config/measurements.demo.csv
```

演示输出不代表机器人实际标定结果，不可据此修改硬件配置。

输出始终是候选参数，`hardware_verified=false`；不改写控制配置，不推断安全限位。确认后将 zero/direction 填入自己的 `joints.local.yaml`，另行实测 min/max，并完成原控制接口要求的硬件确认。项目公共配置保持未标定状态，避免把 500 的先验误当作完成实机标定。
