import sys, math, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt, matplotlib.patches as mp
sys.path.insert(0,sys.argv[1] if len(sys.argv)>1 else '.'); import simulation as S
from config import AGV_LENGTH as L, AGV_WIDTH as Wd
out=sys.argv[2] if len(sys.argv)>2 else 'output/paper_figures/fig3_snapshots.png'; CW=3.2
ALG=[('DLTO','DLTO','#e67e22'),('SW','SW','#c0392b'),('ORCA','ORCA','#8e44ad'),('FLC_ORCA','FLC-\nORCA','#16a085')]
TIMES=[(8.8,'Approaching'),(12.8,'Offset\nInitiation'),(18.2,'Crossing'),(23.5,'Post-crossing')]
fig=plt.figure(figsize=(16,7.6)); fig.patch.set_facecolor('white')
gs=fig.add_gridspec(5,5,width_ratios=[0.55,1,1,1,1],height_ratios=[0.42,1,1,1,1],wspace=0.04,hspace=0.08,left=0.01,right=0.99,top=0.98,bottom=0.07)
for j,(t,ph) in enumerate(TIMES):
    ax=fig.add_subplot(gs[0,j+1]); ax.axis('off'); hl=(j==2)
    ax.add_patch(mp.FancyBboxPatch((0.01,0.05),0.98,0.9,boxstyle='square,pad=0',facecolor='#ffeb3b' if hl else '#dddddd',edgecolor='#999',transform=ax.transAxes))
    ax.text(0.5,0.5,f't = {t} s{" ★" if hl else ""}\n({ph.replace(chr(10)," ")})',ha='center',va='center',fontsize=13,fontweight='bold',color='#7a5c00' if hl else '#222',transform=ax.transAxes)
for i,(alg,lab,col) in enumerate(ALG):
    r=S.run_scenario(n_agv=2,algorithm=alg,seed=1,corridor_width=CW,payload_profile='mixed',record_trajectory=True)
    traj=r[1]
    axl=fig.add_subplot(gs[i+1,0]); axl.axis('off')
    axl.add_patch(mp.Rectangle((0.03,0.03),0.94,0.94,facecolor=col,edgecolor='#333',transform=axl.transAxes))
    axl.text(0.5,0.5,lab,ha='center',va='center',color='white',fontsize=17,fontweight='bold',transform=axl.transAxes)
    for j,(t,_) in enumerate(TIMES):
        ax=fig.add_subplot(gs[i+1,j+1]); st=min(traj,key=lambda e:abs(e[0]-t))[1]
        ax.set_facecolor('#f5f5f5'); ax.set_xlim(12.5,27.5); ax.set_ylim(-CW/2-0.15,CW/2+0.15); ax.set_aspect('equal')
        for y in (CW/2,-CW/2): ax.axhline(y,color='#333',lw=2)
        ax.axhline(0,ls='--',lw=0.6,color='#aaa')
        for (x,y,th,state,arr,cargo,ty) in st:
            if arr: continue
            c,s_=math.cos(th),math.sin(th)
            pts=[(x+lx*c-ly*s_, y+lx*s_+ly*c) for lx,ly in ((L/2,Wd/2),(L/2,-Wd/2),(-L/2,-Wd/2),(-L/2,Wd/2))]
            ax.add_patch(mp.Polygon(pts,closed=True,facecolor='#e74c3c' if cargo else '#3498db',edgecolor='#222',lw=1.2,alpha=0.95))
            ax.annotate('',xy=(x+0.9*c,y+0.9*s_),xytext=(x,y),arrowprops=dict(arrowstyle='-|>',color='white',lw=1.2))
        if alg=='SW' and j>0: ax.text(20,-CW/2+0.25,'deadlock (MCD = 0)',ha='center',fontsize=8,color='#c0392b')
        ax.set_xticks([]); ax.set_yticks([])
        if j==2:
            for sp in ax.spines.values(): sp.set_edgecolor('#b8860b'); sp.set_linewidth(2.5)
fig.text(0.5,0.02,'■ Red = loaded AGV (higher priority)     ■ Blue = unloaded AGV (lower priority, larger lateral offset)     Corridor width 3.2 m, n_agv = 2, mixed payload, seed 1',ha='center',fontsize=11,color='#333')
fig.savefig(out,dpi=150); print('saved',out)
