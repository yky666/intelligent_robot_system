# 智能机器人实验报告（LaTeX 协作目录）

入口为 **main.tex**，编译器使用 **XeLaTeX**。本次更新为第五大组第一小组审阅后的报告；第一小组负责 **TonyPi 人形机器人与方式二建图避障跟随**。封面记录杨开元、林楗棋、曹淇涵、刘小凡的主要分工，分工正文采用常规字重，第2章保留详细协作说明。

## 查看与编译

- [已编译报告](report-draft.pdf)：80页，含实景照片、比赛视频截图、状态机和源码分析图。
- [主文件](main.tex)、[封面](cover.tex)、[公共样式](preamble.tex)、[封面信息](metadata.tex)。
- 将完整 latex 目录上传到 Overleaf，主文件选 main.tex，编译器选 XeLaTeX。fonts/ 随附中文字体，figures/ 提供图形。
- 本地运行 build.cmd，或执行 `latexmk -xelatex -interaction=nonstopmode -halt-on-error main.tex`。至少编译两次更新目录及交叉引用。
- 不提交 build/、编译辅助文件、缓存、完整视频及 bag。

## 各组编辑位置

| 内容 | 源文件 | 图形目录 |
|---|---|---|
| 第一小组：TonyPi | sections/10_humanoid.tex、11_humanoid_analysis.tex、12_humanoid_contracts.tex | figures/humanoid/；状态机见 figures/*.tex |
| 第一小组：方式二 | sections/30_follow2.tex、31_follow2_analysis.tex、32_follow2_contracts.tex | figures/follow2/、figures/analysis/、figures/enriched/ |
| 方式一 | sections/20_follow1.tex | figures/follow1/；原有独立稿 follow1/ar_marker_report.tex 保留 |
| 方式三 | sections/40_follow3.tex | figures/follow3/ |
| 方式四 | sections/50_follow4.tex | figures/follow4/ |
| 组内协作 | sections/02_division.tex；封面摘要在 cover.tex | 无 |
| 实验与视频证据 | sections/60_experiments.tex、61_video_evidence.tex | figures/site/、figures/video/ |
| 引用与实现依据 | sections/80_references.tex、90_reproduction.tex、91_code_analysis.tex、92_parameter_snapshot.tex | data/、code_snapshot/、code_snapshot_current/ |

各组独立维护对应章节和图形；公共入口、样式和跨模块接口协商调整。不要用某位贡献者在仓库 README 中的“我们组”推断全队组号，也不要将视频画面顺序自动等同于方式编号。

## 图形与证据

- 状态机、节点和守卫图为可编辑 TikZ，科研曲线以矢量 PDF 为主。
- 比赛视频只提交引用帧；时间戳和原片摘要见 data/video_evidence.json，裁剪框见 data/enriched_figures.json，完整原始帧同时保留。
- 车辆与方式身份未知，正文按“近端跟随车／后续车辆”描述。
- 动作时长来自真实 d6a；投票、时效、采样与终点边界图为源码或明确输入的解析说明，不作为实测性能。
- code_snapshot/ 保留原分析依据；code_snapshot_current/ 保留43a7771复核文件，差异见 data/github_alignment.json。最新门控功能包归属在正文说明。
- 未采集到的结果保持为未测；视频不单独证明完整终点放置或末车停稳。

## 协作与同步

从最新 main 创建各组报告分支，只修改对应章节与图形，通过 Pull Request 合并。其他小组原有独立文件和图片目录保留。新增图表需有前后解释，实验结果关联代码版本、配置和原始证据。
