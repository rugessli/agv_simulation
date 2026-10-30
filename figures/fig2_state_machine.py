import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Ellipse, FancyArrowPatch
fig,ax=plt.subplots(figsize=(12,8.2)); ax.set_xlim(0,12); ax.set_ylim(0,8.3); ax.axis('off')
BLUE='#2e86c1'; GREEN='#27ae60'; PURPLE='#8e44ad'; ORANGE='#e67e22'; RED='#c0392b'; TEAL='#16a085'
def ell(x,y,t,s,c): 
    ax.add_patch(Ellipse((x,y),2.5,1.05,facecolor=c,edgecolor='none'))
    ax.text(x,y+0.13,t,ha='center',va='center',color='white',fontsize=12,fontweight='bold'); ax.text(x,y-0.2,s,ha='center',va='center',color='white',fontsize=8.5)
def box(x,y,t,s,c,s2=None,w=2.5,h=0.95):
    ax.add_patch(FancyBboxPatch((x-w/2,y-h/2),w,h,boxstyle='round,pad=0.03',facecolor=c,edgecolor='none'))
    ax.text(x,y+0.2,t,ha='center',va='center',color='white',fontsize=12,fontweight='bold'); ax.text(x,y-0.1,s,ha='center',va='center',color='white',fontsize=8.5)
    if s2: ax.text(x,y-0.32,s2,ha='center',va='center',color='white',fontsize=7.5)
def arr(p,q,c='#2c3e50',ls='-',rad=0,lab=None,lp=None,lc='#333'):
    ax.add_patch(FancyArrowPatch(p,q,arrowstyle='-|>',mutation_scale=16,color=c,lw=1.6,ls=ls,connectionstyle=f'arc3,rad={rad}'))
    if lab: ax.text(*lp,lab,fontsize=8,color=lc,ha='center',va='center')
ell(1.6,7.0,'CRUISE','v = 1.5 m/s',BLUE)
box(5.2,7.0,'PRIORITY & OFFSET','Pᵢ = α·cᵢ + β·mᵢ + γ·rᵢ + ε·IDᵢ',PURPLE,'λᵢ = Pⱼ/(Pᵢ+Pⱼ),  δᵢ = max(δ_sym, δ_min+λᵢΔ)',w=3.0,h=1.0)
ell(9.4,7.0,'APPROACH','v = 0.8 m/s, offset arc',BLUE)
ell(9.4,4.6,'OFFSET','v = 0.3 m/s',GREEN)
ell(9.4,2.3,'CREEP','v = 0.1 m/s',GREEN)
box(5.2,4.6,'HOLD','v = 0 (partner offset incomplete)',RED,'eⱼ > 0.05 m  and  dᵢⱼ ≤ d_hold(eⱼ)',w=3.0)
box(5.2,2.3,'PASSING','monitor clearance ≥ d_min',TEAL)
ell(1.6,2.3,'RECOVER','return to centreline',BLUE)
arr((2.85,7.0),(3.7,7.0),lab='opposite AGV\nwithin d_plan = 11 m',lp=(3.27,7.75))
arr((6.7,7.0),(8.15,7.0),lab='peer state\nexchanged',lp=(7.42,7.5))
arr((9.4,6.47),(9.4,5.13),lab='dᵢⱼ ≤ d_detect = 3 m',lp=(10.55,5.8))
arr((9.4,4.07),(9.4,2.83),lab='dᵢⱼ ≤ d_clear = 0.5 m',lp=(10.55,3.45))
arr((8.15,4.75),(6.7,4.75),c=RED,lab='partner not yet\nin offset lane',lp=(7.42,5.2),lc=RED)
arr((6.7,4.45),(8.15,4.45),c=RED,ls='--',lab='eⱼ ≤ 0.05 m',lp=(7.42,4.12),lc=RED)
arr((8.15,2.3),(6.45,2.3),lab='bodies alongside',lp=(7.3,2.6))
arr((3.95,2.3),(2.85,2.3),lab='passed by > 1 m',lp=(3.4,2.65))
arr((1.6,2.83),(1.6,6.47),lab='θ → 0, v → 1.5 m/s',lp=(0.75,4.6))
ax.text(0.75,4.6,'',fontsize=8)
ax.text(6.0,0.9,'Constraints in every moving state: |ω| ≤ v/R_min (R_min = 2.6 m); heading bound keeps the rotated body inside the walls;\n'
        'lateral target rate-limited to 0.35 m/s; footprint-overlap check reduces speed before any contact.',ha='center',fontsize=8.5,color='#333',
        bbox=dict(boxstyle='round,pad=0.4',facecolor='#f4f6f7',edgecolor='#bbb'))
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
ax.legend(handles=[Line2D([0],[0],color='#2c3e50',lw=1.6,label='Normal transition'),Line2D([0],[0],color=RED,lw=1.6,label='Hold transition'),
                   Line2D([0],[0],color=RED,lw=1.6,ls='--',label='Hold release'),Patch(color=BLUE,label='Speed state'),Patch(color=GREEN,label='Offset state'),
                   Patch(color=PURPLE,label='Computation'),Patch(color=RED,label='Hold state')],loc='lower right',bbox_to_anchor=(1.0,0.0),fontsize=8,frameon=True)
fig.tight_layout(); fig.savefig('output/paper_figures/fig2_state_machine.png',dpi=150); print('ok')
