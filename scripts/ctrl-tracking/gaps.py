import re,sys,math,collections
pts=sorted((int(m.group(1)),*map(float,m.group(2,3,4))) for l in open(sys.argv[1],errors='replace') for m in [re.search(r'raw constellation sample .*?t=(\d+) pos=\(([-\d.]+),([-\d.]+),([-\d.]+)\)',l)] if m)
span=(pts[-1][0]-pts[0][0])/1e9
spd=[]
for i in range(5,len(pts)):
    s=(pts[i][0]-pts[i-5][0])/1e9
    if 0<s<0.1: spd.append((pts[i][0],math.dist(pts[i][1:],pts[i-5][1:])/s))
fast_time=sum(1 for _,v in spd if v>0.8)
gaps=collections.defaultdict(list)
for i in range(1,len(pts)):
    dt=(pts[i][0]-pts[i-1][0])/1e6
    if dt>60:
        j=i-1
        while j>0 and (pts[i-1][0]-pts[j][0])/1e6<50: j-=1
        s=(pts[i-1][0]-pts[j][0])/1e9
        v=math.dist(pts[i-1][1:],pts[j][1:])/s if s>0 else 0
        k='slow<0.3' if v<0.3 else 'mid' if v<0.8 else 'fast>0.8'
        gaps[k].append((dt,math.dist(pts[i][1:],pts[i-1][1:])*100))
# jitter at rest: residual vs 5-sample mean when speed<0.05
res=[]
for i in range(2,len(pts)-2):
    w=pts[i-2:i+3]
    if (w[-1][0]-w[0][0])/1e6>80: continue
    v=math.dist(w[-1][1:],w[0][1:])/((w[-1][0]-w[0][0])/1e9)
    if v<0.05:
        c=[sum(p[k] for p in w)/5 for k in (1,2,3)]; res.append(math.dist(pts[i][1:],c)*1000)
res.sort()
print(f"{sys.argv[2]}: {len(pts)/span:.0f} samples/s over {span:.0f}s; fast-motion samples {fast_time}")
for k in ['slow<0.3','mid','fast>0.8']:
    L=gaps[k]
    if L: print(f"  gaps after {k:9s}: {len(L):3d} ({60*len(L)/span:.1f}/min) median {sorted(x for x,_ in L)[len(L)//2]:.0f} ms, jump {sorted(y for _,y in L)[len(L)//2]:.1f} cm")
if fast_time: print(f"  fast-motion gaps per 100 fast samples: {100*len(gaps['fast>0.8'])/fast_time:.1f}")
if res: print(f"  rest jitter mm p50 {res[len(res)//2]:.2f} p90 {res[int(len(res)*.9)]:.2f} (n={len(res)})")
