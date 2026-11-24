#!/usr/bin/env python3
"""Independent Gazebo-only test oracle; NEVER imported by the mission node."""
import argparse,json,subprocess,math
from pathlib import Path

def evaluate(data):
    targets={'red':(2.35,1.2),'green':(2.35,0.),'blue':(2.35,-1.2)};results={}
    poses={p.get('name'):p for p in data.get('pose',[])}
    for color,(tx,ty) in targets.items():
        p=poses.get(color+'_cube')
        if p is None:results[color]={'delivered':False,'error':'missing model pose'};continue
        pos=p.get('position',{});q=p.get('orientation',{});x,y,z,w=[q.get(k,0. if k!='w' else 1.) for k in ['x','y','z','w']]
        rot=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
             [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
             [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
        corners=[]
        for dx in [-.09,.09]:
            for dy in [-.09,.09]:
                for dz in [-.09,.09]:
                    v=[dx,dy,dz];corners.append([pos.get(k,0)+sum(rot[i][j]*v[j] for j in range(3)) for i,k in enumerate(['x','y','z'])])
        inside=all(tx-.3<=c[0]<=tx+.3 and ty-.3<=c[1]<=ty+.3 for c in corners)
        results[color]={'delivered':inside,'center':pos,'orientation':q}
    return {'delivered_count':sum(v['delivered'] for v in results.values()),'cubes':results}

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',default='/tmp/cube_evaluation.json');args=parser.parse_args()
    r=subprocess.run(['gz','topic','-t','/world/competition/dynamic_pose/info','-e','-n','1','--json-output'],capture_output=True,text=True,timeout=15,check=True)
    report=evaluate(json.loads(r.stdout));Path(args.output).write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
if __name__=='__main__':main()
