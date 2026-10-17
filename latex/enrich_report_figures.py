from pathlib import Path
import sqlite3,json,sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle,Circle
from PIL import Image
P=Path(__file__).resolve().parent;F=P/'figures/enriched';F.mkdir(exist_ok=True)
plt.rcParams.update({'font.family':'sans-serif','font.sans-serif':['Microsoft YaHei','DejaVu Sans'],'axes.unicode_minus':False,'font.size':12,'axes.spines.top':False,'axes.spines.right':False,'pdf.fonttype':42})
N='#24445C';T='#247F86';G='#B28C46';R='#B96651';M='#63727F'
def save(name,fig):
 fig.tight_layout(pad=1.3)
 fig.savefig(F/(name+'.pdf'),bbox_inches='tight',pad_inches=.08)
 fig.savefig(F/(name+'.png'),bbox_inches='tight',pad_inches=.08,dpi=160)
 plt.close(fig)
# Actual database duration, not observed physical execution time.
names=['grab_final_v2','catch_ball_go','catch_ball_turn_left','catch_ball_turn_right','put_down'];labels=['抱起物块','抱物直行','抱物左转','抱物右转','放置物块'];stats=[]
for name in names:
 conn=sqlite3.connect('file:'+(P/'code_snapshot/humanoid/maze_route/actions'/(name+'.d6a')).as_posix()+'?mode=ro',uri=True)
 rows=conn.execute('SELECT * FROM ActionGroup ORDER BY "Index"').fetchall();conn.close()
 times=np.array([row[1] for row in rows],float)/1000
 stats.append({'action':name,'frames':len(rows),'duration_s':float(times.sum()),'first_frame_s':float(times[0]),'max_frame_s':float(times.max())})
fig,ax=plt.subplots(figsize=(8.6,3.6));y=np.arange(5)
a=np.array([x['first_frame_s'] for x in stats]);b=np.array([x['duration_s'] for x in stats])-a
ax.barh(y,a,color=G,label='数据库首帧');ax.barh(y,b,left=a,color=T,label='其余动作帧')
for yy,row in zip(y,stats):ax.text(row['duration_s']+.1,yy,f"{row['duration_s']:.2f} s / {row['frames']} 帧",va='center',fontsize=11)
ax.set_yticks(y,labels);ax.invert_yaxis();ax.set_xlabel('动作数据库累计时长（s）');ax.set_xlim(0,max(a+b)*1.45);ax.legend(frameon=False,loc='lower right');ax.grid(axis='x',alpha=.15);ax.set_axisbelow(True);save('action_time_budget',fig)
# Exhaustive combinations of three visible-frame flags, no probability model.
patterns=np.array([[int(x) for x in f'{i:03b}'] for i in range(8)])
fig,(ax,bx)=plt.subplots(1,2,figsize=(9.2,3.2),gridspec_kw={'width_ratios':[2.8,1]})
ax.imshow(patterns.T,cmap=matplotlib.colors.ListedColormap(['#EDF1F4',T]),vmin=0,vmax=1,aspect='auto')
ax.set_xticks(range(8),[''.join(map(str,x)) for x in patterns]);ax.set_yticks(range(3),['新帧 1','新帧 2','新帧 3']);ax.set_xlabel('三帧目标可见组合（1 为可见）')
for i,row in enumerate(patterns):ax.text(i,3.05,'采用' if row.sum()>=2 else '丢失',ha='center',color=T if row.sum()>=2 else R,fontsize=11)
ax.set_ylim(3.5,-.5);ax.set_title('新帧齐全：至少两帧可见才采用目标',color=N)
bx.axis('off');bx.text(0,.77,'新帧不足 3 个',weight='bold',color=R,fontsize=13);bx.text(0,.5,'进入相机等待\n不参与多数投票',linespacing=1.8,fontsize=12);save('observation_vote',fig)
# Source thresholds, not measured stop latency.
labels=['速度命令／运动许可','扫描','目标观测／底盘 TF','任务状态','自身定位','代价地图'];vals=[.4,.5,.8,1.5,2.,3.]
fig,ax=plt.subplots(figsize=(9,3.4));y=np.arange(len(vals));ax.barh(y,vals,color=[R,R,T,N,N,M],height=.55)
for i,v in enumerate(vals):ax.text(v+.045,i,f'{v:g} s',va='center',fontsize=12)
ax.set_yticks(y,labels);ax.invert_yaxis();ax.set_xlim(0,3.5);ax.set_xlabel('允许的信息年龄上限（s）');ax.grid(axis='x',alpha=.15);ax.set_axisbelow(True);save('freshness_budget',fig)
# Call actual Trail.add on a declared analytic path.
sys.path.insert(0,str(P/'code_snapshot/turtlebot/follow2/multi_robot_following/src'))
from competition_core.planning import Trail
path=np.vstack([np.column_stack([np.zeros(101),np.linspace(0,1,101)]),np.column_stack([np.linspace(0,1,101)[1:],np.ones(100)]),np.column_stack([np.ones(100),np.linspace(1,2,101)[1:]])])
fig,axs=plt.subplots(1,3,figsize=(10,3.2));sample_stats=[]
for ax,s in zip(axs,[.04,.08,.25]):
 trail=Trail(spacing=s)
 for p in path:trail.add(*p)
 q=np.array(trail.points);ax.plot(path[:,0],path[:,1],color='#CDD5DC',lw=3);ax.plot(q[:,0],q[:,1],'o-',color=T,ms=3,lw=1)
 ax.set_title(f'采样间距 {s:g} m：{len(q)} 点',fontsize=12,color=N);ax.set_aspect('equal');ax.set(xlabel='x（m）',ylabel='y（m）');ax.grid(alpha=.15);sample_stats.append({'spacing_m':s,'retained_points':len(q)})
save('trail_sampling',fig)
# One-dimensional terminal threshold counterexample; not a recorded run.
fig,ax=plt.subplots(figsize=(8.8,2.8));ax.axhline(0,color=M,lw=1);ax.axvspan(.04,.44,color=T,alpha=.12);ax.axvspan(-.25,.25,color=G,alpha=.10)
ax.scatter([0,.24],[0,0],s=[110,110],c=[N,T],zorder=5)
ax.annotate('机器人／旧目标 0 m',(0,0),xytext=(-.2,.52),arrowprops={'arrowstyle':'-','color':N},fontsize=12)
ax.annotate('最新回退目标 0.24 m',(.24,0),xytext=(.17,-.58),arrowprops={'arrowstyle':'-','color':T},fontsize=12)
ax.annotate('',(0,.23),(.24,.23),arrowprops={'arrowstyle':'<->','color':N});ax.text(.12,.31,'0.24 m < 0.25 m 刷新位移门限',ha='center',fontsize=11)
ax.text(.31,.65,'青色区：距最新目标 ≤ 0.20 m',ha='center',color=T,fontsize=11)
ax.set_xlim(-.3,.55);ax.set_ylim(-.8,.95);ax.set_yticks([]);ax.set_xlabel('简化同一直线路径坐标（m）');ax.spines['left'].set_visible(False);save('terminal_threshold',fig)
# Crops preserve original files and record pixel boxes.
crops={'figures/site/corridor.jpg':((40,140,1230,1230),'figures/site/corridor_focus.jpg'),'figures/video/competition_184s.jpg':((70,70,850,465),'figures/video/competition_184s_focus.jpg'),'figures/video/competition_076s.jpg':((225,70,895,520),'figures/video/competition_076s_focus.jpg'),'figures/video/competition_133s.jpg':((150,35,890,520),'figures/video/competition_133s_focus.jpg'),'figures/video/competition_165s.jpg':((100,40,900,530),'figures/video/competition_165s_focus.jpg'),'figures/video/competition_196s.jpg':((100,30,900,530),'figures/video/competition_196s_focus.jpg'),'figures/video/competition_218s.jpg':((70,30,930,530),'figures/video/competition_218s_focus.jpg')}
for src,(box,dst) in crops.items():Image.open(P/src).crop(box).save(P/dst,quality=96)
(P/'data/enriched_figures.json').write_text(json.dumps({'source':'repository code and original video; no empirical performance estimates','actions':stats,'trail_sampling':sample_stats,'crops':crops},ensure_ascii=False,indent=2),'utf-8')
print(json.dumps({'actions':stats,'new_analytic_figures':5,'crops':len(crops)},ensure_ascii=True))
