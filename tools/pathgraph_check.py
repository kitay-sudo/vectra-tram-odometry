import json, math, glob, numpy as np
src=open('tools/mgrs_check.py').read(); ns={"math":math}
exec(src[src.index('def utm'):src.index('pts=')], ns); utm=ns['utm']
pg={}
for name in ['щукинская - таллинская','таллинская - щукинская']:
    d=json.load(open(f'_incoming/pathgraph/{name}.json',encoding='utf-8'))
    print(name,'top keys',list(d.keys()), {k:(len(v) if isinstance(v,list) else v) for k,v in d.items()})
    P=np.array([[p['x'],p['y'],p['z']] for p in d['points']])
    seg=np.linalg.norm(np.diff(P[:,:2],axis=0),axis=1)
    print('  n',len(P),'x',P[:,0].min(),P[:,0].max(),'y',P[:,1].min(),P[:,1].max(),'z',P[:,2].min(),P[:,2].max(),'len m',seg.sum(),'step med',np.median(seg),'max',seg.max())
    print('  extra point keys', set().union(*[set(p.keys()) for p in d['points'][:50]]))
    pg[name]=P
allP=np.vstack(list(pg.values()))
# compare GNSS master & rover fixes of a few bags (UTM - 37UCB origin 300000, 6100000)
res=[]
for f in sorted(glob.glob('analysis/cache/3*.npz'))[:60]:
    a=np.load(f,allow_pickle=True)
    for ant in ('mfix','rfix'):
        m=a[ant]
        if not len(m): continue
        m=m[(np.isfinite(m[:,2]))&(m[:,2]>1)&(m[:,5]>=2)][::25]  # RTK-ish status 2
        for row in m[:200]:
            z,E,N=utm(row[2],row[3]); x=E-300000; y=N-6100000
            dd=np.hypot(allP[:,0]-x, allP[:,1]-y); i=dd.argmin()
            res.append((ant,dd[i],row[4]-allP[i,2]))
res=np.array([(0 if r[0]=='mfix' else 1, r[1], r[2]) for r in res])
for k,nm in ((0,'master'),(1,'rover')):
    r=res[res[:,0]==k]
    print(nm,'n',len(r),'horiz dist to pathgraph: med',np.median(r[:,1]),'p95',np.percentile(r[:,1],95),' alt - z_pg: med',np.median(r[:,2]),'p5..p95',np.percentile(r[:,2],5),np.percentile(r[:,2],95))
