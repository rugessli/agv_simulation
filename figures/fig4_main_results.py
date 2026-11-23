import sys, pandas as pd, numpy as np, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
from matplotlib.patches import Patch
csv = sys.argv[1] if len(sys.argv)>1 else 'output/results.csv'
out = sys.argv[2] if len(sys.argv)>2 else 'output/paper_figures/fig4_main_results.png'
df=pd.read_csv(csv)
A=['DLTO','SW','ORCA','FLC_ORCA']; LAB={'FLC_ORCA':'FLC-ORCA'}
COL={'DLTO':'#e67e22','SW':'#7f8c8d','ORCA':'#8e44ad','FLC_ORCA':'#16a085'}
W=[2.8,3.2,3.6]; x=np.arange(3); bw=0.18
fig,axs=plt.subplots(2,3,figsize=(18,11.5))
def bars(ax,sub,metric,err=False,skip_sw=False):
    for k,a in enumerate(A):
        if skip_sw and a=='SW': continue
        g=sub[sub.algorithm==a].groupby('corridor_width')[metric]
        m=[g.mean().get(w,np.nan) for w in W]; s=[g.std().get(w,0) for w in W]
        ax.bar(x+(k-1.5)*bw,m,bw,color=COL[a],edgecolor='#333',lw=0.6,
               yerr=s if err else None,capsize=3,error_kw=dict(lw=0.8,ecolor='#444'))
    ax.set_xticks(x); ax.set_xticklabels([f'{w} m' for w in W]); ax.set_xlabel('Corridor Width')
    ax.grid(axis='y',alpha=0.3)
ax=axs[0,0]; bars(ax,df,'DRR'); ax.set_ylim(0,1.18); ax.set_ylabel('DRR'); ax.set_title('(a) DRR — Deadlock Resolution Rate',fontweight='bold')
ax=axs[0,1]; bars(ax,df,'ART'); sw=df[df.algorithm=='SW'].ART.mean()
ax.axhline(sw,ls='--',color='grey',lw=0.8); ax.text(2.45,sw+1.5,'SW\ntimeout',fontsize=7,color='grey')
ax.set_ylim(0,max(130,sw*1.2)); ax.set_ylabel('Seconds'); ax.set_title('(b) ART — Avg. Resolution Time (s)',fontweight='bold')
ax=axs[0,2]; bars(ax,df,'ST'); ax.set_ylim(0,50); ax.set_ylabel('Missions / 10 min'); ax.set_title('(c) ST — System Throughput (missions / 10 min)',fontweight='bold')
for ax,(pp,t) in zip(axs[1],[('unloaded','(d) MCD — Unloaded Payload (m)'),('mixed','(e) MCD — Mixed Payload (m)'),('loaded','(f) MCD — Loaded Payload (m)')]):
    sub=df[df.payload_profile==pp]; bars(ax,sub,'MCD',err=True,skip_sw=True)
    ax.axhline(0.25,ls='--',color='red',lw=0.9); ax.text(2.62,0.25,'d_min',color='red',fontsize=7,va='center')
    for i,w in enumerate(W):
        d=sub[(sub.algorithm=='DLTO')&(sub.corridor_width==w)].MCD; o=sub[(sub.algorithm=='ORCA')&(sub.corridor_width==w)].MCD
        diff=d.mean()-o.mean()
        if abs(diff)>=0.0005:
            ax.text(i-1.5*bw,d.mean()+d.std()+0.02,f'{diff:+.3f}m',color='#c0392b',fontsize=8,fontweight='bold',ha='center')
    ax.set_ylim(0,0.98); ax.set_ylabel('Min. Clearance Distance (m)'); ax.set_title(t,fontweight='bold')
fig.legend(handles=[Patch(facecolor=COL[a],edgecolor='#333',label=LAB.get(a,a)) for a in A],loc='lower center',ncol=4,fontsize=11)
n=len(df)//4
fig.tight_layout(rect=(0,0.05,1,1)); fig.savefig(out,dpi=150); print('saved',out)
