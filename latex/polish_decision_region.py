from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
plt.rcParams.update({'font.family':'sans-serif','font.sans-serif':['Microsoft YaHei','DejaVu Sans'],'axes.unicode_minus':False,'font.size':12,'pdf.fonttype':42,'axes.spines.top':False,'axes.spines.right':False})
fig,ax=plt.subplots(figsize=(9,4.0))
for xy,w,h,c in [((20,-30),20,60,'#B28C46'),((40,-18),45,36,'#24445C'),((40,18),45,12,'#247F86'),((40,-30),45,12,'#247F86')]:ax.add_patch(Rectangle(xy,w,h,facecolor=c,edgecolor='none'))
for y in [-18,18]:ax.plot([40,85],[y,y],color='white',ls='--',lw=1.3)
ax.axvline(40,color='white',ls='--',lw=1.3)
ax.text(30,0,'到达条件满足\n转角／放置',color='white',ha='center',va='center',fontsize=13)
ax.text(62.5,0,'直行一步',color='white',ha='center',va='center',fontsize=14)
for y in [-24,24]:ax.text(62.5,y,'对中转向',color='white',ha='center',va='center',fontsize=12)
ax.set(xlim=(20,85),ylim=(-30,30),xlabel='观测斜距（cm）',ylabel='目标方位角（度）',title='有效观测下的决策区域（近期转角估计 30°）')
ax.set_yticks([-30,-18,0,18,30]);fig.tight_layout()
for ext in ['pdf','png']:fig.savefig('figures/analysis/decision_region.'+ext,bbox_inches='tight',dpi=180)
