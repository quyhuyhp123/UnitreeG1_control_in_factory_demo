"""Narrow Factory V2 inner-island machines and widen the green navigation lanes.

Run with Isaac Sim's Python (needs pxr):
    scripts/isaac_python.sh scripts/resize_factory_machines.py [--width 1.9]

What it does (idempotent; always derived from the unchanged source USD):
  * Each of the 8 island machines (Machine_Inner_*) is narrowed along X from
    2.6 m to --width, keeping its operator-facing side fixed, so the outer
    corridors gain the difference. Visual Xform, Blender proxy and the
    PhysicsColliders cube are all edited as opinions in the physics wrapper
    layer; factory_layout_v2.usd (hash-pinned source) is untouched.
  * configs/navigation_map_v2.json gets a green lane that reaches the wall-side
    machines and one red forbidden rectangle per island machine + operator,
    computed from the composed collider and worker bounds.
"""
from pathlib import Path
import argparse,json,sys
ROOT=Path(__file__).resolve().parents[1]
FACTORY=ROOT/'assets/factory/factory_layout_v2'
INNER=[f'Machine_Inner_{side}_{i:02d}' for side in ('Left','Right') for i in range(1,5)]
ORIGINAL_WIDTH_M=2.6


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--width',type=float,default=1.9,help='new island machine width along X (m)')
    args,_=parser.parse_known_args()
    if not 1.2<=args.width<=ORIGINAL_WIDTH_M: raise ValueError('width must be within [1.2, 2.6] m')
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True})
    try:
        from pxr import Usd,UsdGeom,Gf,Sdf
        source=Usd.Stage.Open(str(FACTORY/'factory_layout_v2.usd'))
        physics_path=FACTORY/'factory_layout_v2_physics.usd'
        stage=Usd.Stage.Open(str(physics_path)); stage.SetEditTarget(stage.GetRootLayer())
        scale=args.width/ORIGINAL_WIDTH_M; record=[]
        for name in INNER:
            proxy=source.GetPrimAtPath('/root/COL_'+name)
            tx=proxy.GetAttribute('xformOp:translate').Get()
            bounds=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','proxy','render']).ComputeWorldBound(proxy).ComputeAlignedRange()
            width=bounds.GetMax()[0]-bounds.GetMin()[0]
            if abs(width-ORIGINAL_WIDTH_M)>.01: raise RuntimeError(f'{name}: unexpected source width {width}')
            # Left island operators stand on +X, right island operators on -X.
            edge=tx[0]+ORIGINAL_WIDTH_M/2 if '_Left_' in name else tx[0]-ORIGINAL_WIDTH_M/2
            for path in ('/FactoryV2/'+name,'/FactoryV2/COL_'+name):
                prim=stage.GetPrimAtPath(path)
                original=source.GetPrimAtPath(path.replace('/FactoryV2','/root'))
                t=original.GetAttribute('xformOp:translate').Get(); s=original.GetAttribute('xformOp:scale').Get()
                prim.GetAttribute('xformOp:translate').Set(type(t)(edge+(t[0]-edge)*scale,t[1],t[2]))
                prim.GetAttribute('xformOp:scale').Set(type(s)(s[0]*scale,s[1],s[2]))
            cube=stage.GetPrimAtPath('/FactoryV2/PhysicsColliders/COL_'+name)
            t=cube.GetAttribute('xformOp:translate').Get(); s=cube.GetAttribute('xformOp:scale').Get()
            cube.GetAttribute('xformOp:translate').Set(type(t)(edge+(tx[0]-edge)*scale,t[1],t[2]))
            cube.GetAttribute('xformOp:scale').Set(type(s)(args.width,s[1],s[2]))
            record.append({'machine':name,'fixed_operator_edge_x_m':edge,'width_before_m':ORIGINAL_WIDTH_M,'width_after_m':args.width})
        layer=stage.GetRootLayer()
        data=dict(layer.customLayerData); data['island_machine_width_m']=args.width; layer.customLayerData=data
        layer.Save()

        # Recompose from disk and measure what the robot will actually meet.
        stage=Usd.Stage.Open(str(physics_path))
        cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','proxy','render'])
        def box(path):
            r=cache.ComputeWorldBound(stage.GetPrimAtPath(path)).ComputeAlignedRange()
            return [list(r.GetMin())[:2],list(r.GetMax())[:2]]
        def union(*boxes):
            return [[min(b[0][i] for b in boxes) for i in (0,1)],[max(b[1][i] for b in boxes) for i in (0,1)]]
        forbidden=[]
        for name in INNER:
            operator=name.replace('Machine','Operator')
            lo,hi=union(box('/FactoryV2/PhysicsColliders/COL_'+name),box('/FactoryV2/PhysicsColliders/COL_'+operator),
                        box('/FactoryV2/Workers/'+operator))
            lo=[round(v,3) for v in lo]; hi=[round(v,3) for v in hi]
            forbidden.append({'id':name,'polygon':[[lo[0],lo[1]],[hi[0],lo[1]],[hi[0],hi[1]],[lo[0],hi[1]]]})
        outer_left=round(max(box(f'/FactoryV2/PhysicsColliders/COL_Machine_Outer_Left_{i:02d}')[1][0] for i in (1,2,3)),3)
        outer_right=round(min(box(f'/FactoryV2/PhysicsColliders/COL_Machine_Outer_Right_{i:02d}')[0][0] for i in (1,2,3)),3)
        nav_path=ROOT/'configs/navigation_map_v2.json'; nav=json.loads(nav_path.read_text())
        old=nav['allowed_lanes'][0]['polygon']
        top=max(p[1] for p in old); bottom=min(p[1] for p in old); entry_top=sorted({p[1] for p in old})[1]
        nav['allowed_lanes']=[{'name':'Connected_Green_Corridors','polygon':[[-10.0,bottom],[outer_right,bottom],[outer_right,top],
                               [outer_left,top],[outer_left,entry_top],[-10.0,entry_top]]}]
        nav['forbidden_zones']=forbidden
        # Hand-tuned detours for the old 1.6 m corridors are replaced by the
        # generic obstacle bypass in navigation/multi_route_controller.py.
        nav.pop('local_detours',None); nav.pop('map_2_chair_bypass',None)
        nav['lane_note']=(f'Island machines narrowed to {args.width} m (scripts/resize_factory_machines.py). '
                          'Forbidden zones hug each island machine + operator; the green lane reaches the wall-side machines.')
        nav_path.write_text(json.dumps(nav,indent=2)+'\n')
        # Keep the polygon-with-holes source in step: build_factory_v2_physics.py
        # regenerates navigation_map_v2.json from it, and route checks read it.
        areas_path=FACTORY/'navigation_allowed_areas.json'; areas=json.loads(areas_path.read_text())
        outer=nav['allowed_lanes'][0]['polygon']; holes=[f['polygon'] for f in forbidden]
        areas['allowed_polygons']=[{'id':'Connected_Green_Corridors','outer':outer,'holes':holes}]
        areas['forbidden_polygons']=[{'id':f['id'],'polygon':f['polygon']} for f in forbidden]
        areas['navigation_boundaries']['loops']=[outer]+holes
        for item in areas['static_obstacles']:
            if item['id'].startswith('COL_Machine_Inner_'):
                lo,hi=box('/FactoryV2/PhysicsColliders/'+item['id'])
                item['bounds'][0][:2]=[round(v,3) for v in lo]; item['bounds'][1][:2]=[round(v,3) for v in hi]
        areas['validation']['note']=f'Lanes/islands updated for {args.width} m island machines; validated by scripts/check_route_geometry.py.'
        areas_path.write_text(json.dumps(areas,indent=2)+'\n')
        (FACTORY/'machine_resize.json').write_text(json.dumps({'island_machine_width_m':args.width,'machines':record,
            'allowed_lane_x_m':[outer_left,outer_right],'forbidden_zones':forbidden},indent=2)+'\n')
        print('MACHINE_RESIZE_OK '+json.dumps({'width':args.width,'lane_x':[outer_left,outer_right]}),flush=True)
    finally:
        app.close()


if __name__=='__main__': main()
