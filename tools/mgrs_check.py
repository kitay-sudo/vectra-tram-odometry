import numpy as np, glob, math
lat=[];lon=[];alt=[]
for f in sorted(glob.glob('analysis/cache/*.npz'))[:200]:
    try: a=np.load(f, allow_pickle=True)
    except Exception: continue
    if 'mfix' in a.files and len(a['mfix']):
        m=a['mfix']; lat+=list(m[:,2]); lon+=list(m[:,3]); alt+=list(m[:,4])
lat=np.array(lat);lon=np.array(lon)
ok=np.isfinite(lat)&(np.abs(lat)>1)
lat,lon=lat[ok],lon[ok]
print('lat',lat.min(),lat.max(),'lon',lon.min(),lon.max(), 'alt', np.nanmin(alt), np.nanmax(alt))
# UTM (Krüger) zone from lon
def utm(lat,lon):
    a=6378137.0; f=1/298.257223563; k0=0.9996
    zone=int((lon+180)//6)+1; lon0=math.radians((zone-1)*6-180+3)
    n=f/(2-f); A=a/(1+n)*(1+n*n/4+n**4/64)
    al=[n/2-2*n*n/3+5*n**3/16, 13*n*n/48-3*n**3/5, 61*n**3/240]
    phi=math.radians(lat); lam=math.radians(lon)-lon0
    e=math.sqrt(f*(2-f))
    t=math.sinh(math.atanh(math.sin(phi))-e*math.atanh(e*math.sin(phi)))
    xi=math.atan2(t,math.cos(lam)); eta=math.atanh(math.sin(lam)/math.sqrt(1+t*t))
    E=500000+k0*A*(eta+sum(al[j]*math.cos(2*(j+1)*xi)*math.sinh(2*(j+1)*eta) for j in range(3)))
    N=k0*A*(xi+sum(al[j]*math.sin(2*(j+1)*xi)*math.cosh(2*(j+1)*eta) for j in range(3)))
    return zone,E,N
pts=[utm(la,lo) for la,lo in zip(lat[::50],lon[::50])]
Z=set(p[0] for p in pts); E=np.array([p[1] for p in pts]); N=np.array([p[2] for p in pts])
print('zones',Z,'E',E.min(),E.max(),'N',N.min(),N.max())
print('100km squares E',set((E//1e5).astype(int)),'N',set((N//1e5).astype(int)))
print('MGRS in-square E',(E%1e5).min(),(E%1e5).max(),'N',(N%1e5).min(),(N%1e5).max())
