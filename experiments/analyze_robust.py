import pandas as pd, numpy as np, json, sys
from scipy.stats import wilcoxon
f = sys.argv[1] if len(sys.argv) > 1 else 'results/robustness.csv'
d=pd.read_csv(f); key=['corridor_width','n_agv','payload_profile','seed']
s=d.groupby(['condition','algorithm']).agg(DRR=('DRR','mean'),ART=('ART','mean'),ST=('ST','mean'),MCD=('MCD','mean'),
   MCDmin=('MCD','min'),below=('MCD',lambda x:int((x<0.25).sum())),contact=('MCD',lambda x:int((x<=1e-9).sum())),wall=('wall_clearance','min'))
print(s.round(3).to_string())
res={}
for c in sorted(d.condition.unique()):
    X=d[d.condition==c]; D=X[X.algorithm=='DLTO'].set_index(key); O=X[X.algorithm=='ORCA'].set_index(key); F=X[X.algorithm=='FLC_ORCA'].set_index(key)
    def pc(sel,B):
        a=D.MCD[sel]; b=B.MCD.loc[a.index]; df=a-b; return 100*(a.mean()/b.mean()-1), wilcoxon(df[df.abs()>1e-12]).pvalue
    L=D.index.get_level_values
    m=L(2)=='mixed'
    res[c]=dict(mixO=pc(m,O),mixF=pc(m,F),m32=pc(m&(L(0)==3.2),O),m36=pc(m&(L(0)==3.6),O))
    print(c,{k:(round(v[0],1),f"{v[1]:.0e}") for k,v in res[c].items()})
b=d[d.MCD<0.25]; print(b.groupby(['condition','algorithm','corridor_width']).size().to_string())
