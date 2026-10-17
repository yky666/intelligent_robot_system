from pathlib import Path
import sys,ast,json,sqlite3,math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch,FancyArrowPatch,Rectangle,Circle,Polygon
ROOT=Path(__file__).resolve().parent
REPO=ROOT/'code_snapshot' if (ROOT/'code_snapshot').exists() else ROOT.parent
FIG=ROOT/'figures/analysis'
plt.rcParams.update({'font.family':'sans-serif','font.sans-serif':['Microsoft YaHei','DejaVu Sans'],'axes.unicode_minus':False,'font.size':11,'axes.spines.top':False,'axes.spines.right':False,'pdf.fonttype':42,'savefig.facecolor':'white'})
NAVY='#24445C';TEAL='#247F86';GOLD='#B28C46';RUST='#B96651';GRAY='#63727F'
def save(name,fig):
 fig.savefig(FIG/(name+'.pdf'),bbox_inches='tight',pad_inches=.18)
 fig.savefig(FIG/(name+'.png'),bbox_inches='tight',pad_inches=.18,dpi=160)
 plt.close(fig)
def axgrid(ax):
    ax.grid(alpha=.16); ax.set_axisbelow(True)
def diagram(name,nodes,edges,figsize=(10,5)):
    fig,ax=plt.subplots(figsize=figsize)
    for key,text,x,y,w,h,col in nodes:
        ax.add_patch(FancyBboxPatch((x-w/2,y-h/2),w,h,boxstyle='round,pad=.03,rounding_size=.1',ec=col,fc='#F7F9FA',lw=1.2))
        ax.text(x,y,text,ha='center',va='center',color=col,fontsize=10)
    nd={n[0]:n for n in nodes}
    for a,b,label in edges:
        x1,y1,w1,h1=nd[a][2:6]; x2,y2,w2,h2=nd[b][2:6]
        dx,dy=x2-x1,y2-y1
        ratio=max(abs(dx)/(w1/2+.1),abs(dy)/(h1/2+.1))
        sx,sy=x1+dx/ratio,y1+dy/ratio
        ratio=max(abs(dx)/(w2/2+.1),abs(dy)/(h2/2+.1))
        ex,ey=x2-dx/ratio,y2-dy/ratio
        ax.add_patch(FancyArrowPatch((sx,sy),(ex,ey),arrowstyle='-|>',mutation_scale=11,lw=1,color=GRAY,connectionstyle=('arc3,rad=.18' if any(e[0]==b and e[1]==a for e in edges) else 'arc3,rad=0')))
        if label: ax.text((sx+ex)/2,(sy+ey)/2+(.28 if a < b else -.28) if any(e[0]==b and e[1]==a for e in edges) else (sy+ey)/2+.12,label,ha='center',fontsize=8,color=GRAY,bbox=dict(fc='white',ec='none',pad=.8))
    xs=[n[2] for n in nodes]; ys=[n[3] for n in nodes]
    ax.set_xlim(min(xs)-1.5,max(xs)+1.5);ax.set_ylim(min(ys)-.8,max(ys)+.9);ax.axis('off'); save(name,fig)

diagram('humanoid_main_fsm',[('s','等待启动\nS / 自动延时',0,4,2.2,.9,NAVY),('g','抱起并锁臂\ngrab_final_v2',3,4,2.2,.9,TEAL),('a','走向 A\n观测 / 对中 / 前进',6,4,2.4,.9,NAVY),('l','左转寻找 B\n最多 12 次尝试',6,2,2.4,.9,TEAL),('b','走向 B\n新阶段计数复位',3,2,2.4,.9,NAVY),('r','右转寻找 C\n先看见 C 才继续',0,2,2.4,.9,TEAL),('c','走向 C\n到达距离条件',0,0,2.4,.9,NAVY),('p','解除锁臂 / 放置\nput_down',3,0,2.4,.9,TEAL),('d','结束线程\n保留日志',6,0,2.4,.9,NAVY)],[('s','g','启动'),('g','a','动作结束'),('a','l','到达 A'),('l','b','看到 B'),('b','r','到达 B'),('r','c','看到 C'),('c','p','到达 C'),('p','d','动作结束')],(12,7))
diagram('humanoid_leg_fsm',[('o','获取 3 个新帧\n目标观测取中位数',0,4,2.6,.9,NAVY),('v','目标有效？',3.5,4,2.2,.9,TEAL),('arr','距离达到阈值？',7,4,2.6,.9,NAVY),('cam','无新帧：原地等待\n超时暂停 / 结束',0,2,2.6,.9,RUST),('blind','无标记：受限盲行\n漂移补偿与计数限制',3.5,2,2.8,.9,GOLD),('align','方位误差超过门限？\nmax(8°, 0.6 × 转角估计)',7,2,3,.9,NAVY),('turn','执行一次纠偏\n再观测转角',7,0,2.5,.9,TEAL),('walk','执行一步直行\n更新推进与偏航记录',3.5,0,2.7,.9,TEAL),('next','阶段结束\n进入转角或放置',10.5,4,2.3,.9,TEAL)],[('o','v','有新帧'),('o','cam','无相机'),('v','arr','可见'),('v','blind','不可见'),('arr','next','达到'),('arr','align','未达到'),('align','turn','偏差大'),('align','walk','容差内'),('blind','walk','未超限制')],(14,7))
diagram('pause_fsm',[('run','运行中\ngo = set',0,2,2.3,1,NAVY),('pause','交互暂停\ngo = clear',4,2,2.6,1,RUST),('exit','无头模式退出\nstop_ev = set',4,0,2.8,1,RUST),('end','退出 / 异常\n停止后续动作与关闭资源',0,0,3,1,NAVY)],[('run','pause','P / 失败守卫'),('pause','run','S 恢复'),('run','exit','失败且无窗口'),('run','end','Q / 信号 / 异常')],(10,4.5))
diagram('corner_fsm',[('arr','到达 A 或 B\n确定固定转向方向',0,2,2.8,1,NAVY),('turn','等待 → 转一次 → 等待\n0.3 s / 动作 / 0.3 s',4,2,3,1,TEAL),('see','检查下一目标码\nB 或 C',8,2,2.7,1,NAVY),('ok','返回主循环\n下一段继续对中',8,0,2.8,1,TEAL),('lim','尚未看见：重试\n12 次上限后暂停',4,0,2.8,1,RUST),('cam','无新帧时等待\n不继续转动',0,0,2.8,1,RUST)],[('arr','turn','corner'),('turn','see','新观测'),('see','ok','可见'),('see','lim','未见'),('lim','turn','剩余尝试'),('see','cam','相机断帧')],(13,4.5))
# Pull exact custom marker bit patterns by parsing source constants only.
node=ast.parse((REPO/'humanoid/maze_route/markers.py').read_text('utf-8'))
patterns={}
for n in node.body:
 if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='TEMPLATES' for t in n.targets):patterns=ast.literal_eval(n.value)
fig,axs=plt.subplots(1,3,figsize=(10,3.5))
for ax,(name,bits) in zip(axs,patterns.items()):
 M=np.zeros((9,9));M[2:7,2:7]=np.array(list(bits),int).reshape(5,5)
 ax.imshow(M,cmap='gray',vmin=0,vmax=1,interpolation='nearest');ax.set_title(name+'：'+{'A':'左转路口','B':'右转路口','C':'停止放置'}[name],color=NAVY);ax.set_xticks([]);ax.set_yticks([])
save('marker_patterns',fig)
# Minimum rotational Hamming distances from the actual templates.
arr={n:np.array(list(v),int).reshape(5,5) for n,v in patterns.items()};M=np.zeros((3,3),int)
for i,a in enumerate(arr):
 for j,b in enumerate(arr):M[i,j]=min(np.count_nonzero(arr[a]!=np.rot90(arr[b],r)) for r in range(4))
fig,ax=plt.subplots(figsize=(5.8,3.5));ax.imshow(M,cmap='Blues');ax.set_xticks(range(3),list(arr));ax.set_yticks(range(3),list(arr))
for i in range(3):
 for j in range(3):ax.text(j,i,str(M[i,j]),ha='center',va='center',color='white' if M[i,j]>3 else NAVY,fontsize=16)
ax.set_title('四种旋转下的最小数据位差（源码模板）',color=NAVY);save('hamming',fig)
# Real current action database statistics.
res=[]
fig,axs=plt.subplots(2,2,figsize=(11,7.2))
for ax,name in zip(axs.ravel(),['grab_final_v2','catch_ball_go','catch_ball_turn_left','catch_ball_turn_right']):
 con=sqlite3.connect('file:'+(REPO/'humanoid/maze_route/actions'/(name+'.d6a')).resolve().as_posix()+'?mode=ro',uri=True);rows=con.execute('select * from ActionGroup order by "Index"').fetchall();con.close();A=np.array(rows)
 ax.imshow(A[:,2:18].T,origin='lower',aspect='auto',cmap='cividis',vmin=0,vmax=1000);ax.set(title=name,xlabel='动作帧序号',ylabel='身体通道');ax.set_yticks([0,5,7,13,15],[1,6,8,14,16]);res.append({'name':name,'frames':len(rows),'seconds':float(A[:,1].sum()/1000),'arm_tail':[int(A[-1,i+1]) for i in [6,7,8,14,15,16]]})
fig.tight_layout(pad=2);save('current_actions',fig)
# Current control equations, clearly analytic not physical measurements.
fig,axs=plt.subplots(1,2,figsize=(10,3.8));t=np.linspace(0,50,300);axs[0].plot(t,np.maximum(8,.6*t),color=TEAL,lw=2);axs[0].set(xlabel='近期单次转角估计（度）',ylabel='纠偏触发门限（度）',title='自适应死区：max(8°, 0.6θ)');axs[0].grid(alpha=.2)
h=np.linspace(0,25,200);axs[1].plot(h,np.sqrt(40**2+h*h),color=NAVY,lw=2);axs[1].axhline(40,color=GRAY,ls='--');axs[1].set(xlabel='镜头与码中心高度差（cm）',ylabel='地面水平距离 40 cm 时的斜距（cm）',title='PnP 斜距与水平距离的区别');axs[1].grid(alpha=.2);save('control_geometry',fig)
fig,ax=plt.subplots(figsize=(9,3.8));xx=np.linspace(20,85,240);yy=np.linspace(-30,30,200);X,Y=np.meshgrid(xx,yy);Z=np.where(X<=40,0,np.where(abs(Y)>18,1,2));from matplotlib.colors import ListedColormap
ax.pcolormesh(xx,yy,Z,cmap=ListedColormap([GOLD,TEAL,NAVY]),shading='auto');ax.axvline(40,color='white',ls='--');ax.axhline(18,color='white',ls='--');ax.axhline(-18,color='white',ls='--');ax.text(24,0,'到达\n转角 / 放置',color='white');ax.text(54,23,'对中转向',color='white');ax.text(54,0,'直行一步',color='white');ax.set(xlabel='观测斜距（cm）',ylabel='目标方位角（度）',title='有效观测条件下的决策区域（假定近期转角 30°）');save('decision_region',fig)
# Copy current Trail and grid logic through a pure-core import.
sys.path.insert(0,str(REPO/'turtlebot/follow2/multi_robot_following/src'))
from competition_core.planning import Trail,grid_clear,fresh,scan_clear
P=np.array([[0,0],[0,.5],[0,1],[0,1.5],[-.5,1.5],[-1,1.5],[-1.5,1.5],[-1.5,2],[-1.5,2.5]])
t=Trail(spacing=.08)
for p in P:t.add(*p)
fig,ax=plt.subplots(figsize=(7.5,4.2));ax.plot(P[:,0],P[:,1],'o-',color=NAVY,label='给定目标折线')
for gap,col in [(.9,TEAL),(1.5,GOLD),(2.1,RUST)]:
 q=t.behind(gap);ax.scatter(q[0],q[1],s=80,c=col,label='回退 '+str(gap)+' m')
ax.set_aspect('equal');ax.grid(alpha=.2);ax.set(xlabel='x（m）',ylabel='y（m）',title='实际 Trail.behind 的目标返回点');ax.legend(fontsize=9,loc='center left',bbox_to_anchor=(1, .5));save('trail_actual',fig)
diagram('follow2_nodes',[('ar','AR 检测\nar_pose_marker',0,3,2.4,1,NAVY),('tf','目标适配\nmarker_target + TF',3.5,3,2.8,1,TEAL),('f','地图跟随器\nmap_follower',7,3,2.5,1,NAVY),('nav','规划器\nmove_base / DWA',10.5,3,2.6,1,TEAL),('mi','任务桥 / 定位\nmission + AMCL',3.5,0.5,2.8,1,NAVY),('ga','独立速度门控\nvelocity_gate',7,.5,2.8,1,RUST),('base','底盘导航输入\ncmd_vel_mux',10.5,.5,2.6,1,NAVY)],[('ar','tf','测量时刻'),('tf','f','target_pose'),('f','nav','导航目标'),('mi','f','有效性'),('f','ga','许可'),('nav','ga','原始速度'),('ga','base','安全速度')],(14,5.5))
diagram('follow2_guards',[('m','任务、目标有效',0,3,2.7,.9,NAVY),('l','定位、地图、TF 有效',3.7,3,3,.9,TEAL),('p','间距与轨迹长度\n目标足迹可行',7.4,3,2.8,1,NAVY),('n','发送目标与许可\n导航未失败',7.4,1,2.8,1,TEAL),('g','命令、许可、扫描新鲜\n扇区距离与速度合法',3.7,1,3,1,RUST),('o','唯一安全速度出口',0,1,2.8,1,NAVY)],[('m','l','通过'),('l','p','通过'),('p','n','通过'),('n','g','Twist + allowed'),('g','o','通过，否则零速')],(12,4.7))
fig,axs=plt.subplots(1,2,figsize=(10,3.7));age=np.linspace(-.2,1.1,200)
for i,(tau,col) in enumerate([(.4,NAVY),(.5,TEAL),(.8,GOLD)]):axs[0].step(age,((age>=0)&(age<=tau))+i*1.4,where='mid',color=col,label=str(tau)+' s')
axs[0].set(xlabel='now − stamp（s）',ylabel='有效域（分行显示）',title='实际 fresh 函数的时间边界');axs[0].legend(fontsize=9)
w=np.linspace(.55,1.5,200)
for r,col in [(.25,NAVY),(.3,TEAL),(.35,GOLD)]:axs[1].plot(w,np.maximum(0,w-2*r),color=col,label='半径 '+str(r)+' m')
axs[1].set(xlabel='净通道宽度（m）',ylabel='圆心可行横向余量（m）',title='未计定位误差的几何通行边界');axs[1].legend(fontsize=9);axs[1].grid(alpha=.2);save('fresh_clearance',fig)
(ROOT/'data/source_analysis.json').write_text(json.dumps({'actions':res,'hamming':M.tolist(),'trail':P.tolist(),'type':'source-derived action data and synthetic function analysis'},ensure_ascii=False,indent=2),'utf-8')
print('new figures',len(list(FIG.glob('*.pdf'))))
