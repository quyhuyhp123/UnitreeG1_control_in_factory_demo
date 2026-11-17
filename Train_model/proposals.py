"""Offline Factory V2 manifest projections for HUMAN REVIEW, not AI detections."""
import json,itertools
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent

def propose(source,classes):
    metadata=Path(source).parent/'metadata.json'
    if not metadata.exists():raise ValueError('Ảnh phải là rgb.png cạnh metadata.json từ camera G1')
    meta=json.loads(metadata.read_text());K=np.array(meta['intrinsics_K']);T=np.linalg.inv(np.array(meta['T_world_optical']))
    w,h=meta['resolution_wh'];groups={}
    manifest=ROOT.parent/'assets/factory/factory_layout_v2/object_manifest.json'
    for obj in json.loads(manifest.read_text())['objects']:
        name=obj['semantic_class']
        if name not in classes or name not in ['person','machine','table','chair']:continue
        lo,hi=np.array(obj['world_bounding_box_m']);key=(name,obj.get('assembly_id') or obj['object_id'])
        if key in groups:groups[key]=(np.minimum(groups[key][0],lo),np.maximum(groups[key][1],hi))
        else:groups[key]=(lo,hi)
    boxes=[]
    for (name,_),(lo,hi) in groups.items():
        points=np.array(list(itertools.product(*zip(lo,hi))))
        optical=points@T[:3,:3].T+T[:3,3]
        if np.any(optical[:,2]<=.1):continue
        uv=optical@K.T;uv=uv[:,:2]/uv[:,2:3]
        x1,y1=np.maximum(uv.min(axis=0),[0,0]);x2,y2=np.minimum(uv.max(axis=0),[w,h])
        if x2-x1>=3 and y2-y1>=3:boxes.append([classes.index(name),float(x1),float(y1),float(x2),float(y2)])
    return boxes
