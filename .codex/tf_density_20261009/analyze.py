import csv,json,sys
from pathlib import Path
import numpy as np
root=Path(sys.argv[1]);rows=list(csv.DictReader((root/'events.csv').open()));mo=[r for r in rows if r['kind']=='mo'];maps=[r for r in rows if r['kind']=='map'];scans=[r for r in rows if r['kind']=='merged'];graphs=[r for r in rows if r['kind']=='graph'];stalls=[]
if mo:
 start=last=mo[0]
 for r in mo[1:]:
  if r['stamp']!=last['stamp']:
   duration=float(r['wall'])-float(start['wall'])
   if duration>.3:
    inside=[m for m in maps if float(start['wall'])<=float(m['wall'])<=float(r['wall'])]
    stalls.append({'sim':float(start['ros']),'duration':duration,'map_inside':len(inside)})
   start=r
  last=r
bins=[]
for t in range(0,900,100):
 part=[r for r in mo if t<=float(r['ros'])<t+100]
 if part:bins.append({'sim_bin':t,'max_lag':max(float(r['ros'])-float(r['stamp'])+1 for r in part),'stamp_jump_max':max([float(y['stamp'])-float(x['stamp']) for x,y in zip(part,part[1:])],default=0)})
result={'tf_count':len(mo),'map_count':len(maps),'merged_count':len(scans),'merged_max_stamp_gap':max(np.diff([float(r['stamp']) for r in scans]),default=0),'graph_vertices_final':int(graphs[-1]['extra'])-3 if graphs else None,'max_lag':max([float(r['ros'])-float(r['stamp'])+1 for r in mo],default=0),'bins':bins,'stalls_over_0_3_count':len(stalls),'stalls_over_0_3_with_map':sum(s['map_inside']>0 for s in stalls),'max_free_area':max([int(r['extra'].split()[1])*.05*.05 for r in maps],default=0)}
if (root/'final_map.npz').exists():
 m=np.load(root/'final_map.npz');d=m['data'];result['final_free_area']=float(np.count_nonzero(d==0)*m['res']**2);result['final_known_area']=float(np.count_nonzero(d>=0)*m['res']**2)
(root/'analysis.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
