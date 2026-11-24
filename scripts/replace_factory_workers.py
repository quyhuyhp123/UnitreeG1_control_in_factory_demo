"""Swap Factory V2's blocky workers and desk chairs for the models YOLO recognises.

Pure OpenUSD authoring: never starts Isaac Sim, never moves a robot.

What changes
  * every `Operator_*` (standing at a machine) and `Person_*` (sitting at a desk)
    visual is switched off and a realistic worker model is placed there;
  * every `Chair_*` visual is switched off and an office chair is placed there.
What does not change
  * `factory_layout_v2.usd` (the Blender export) is not touched, so its hash and
    every navigation/route file keyed to it stay valid;
  * walls, floor, machines, tables, markings, lights, colliders, spawn marker.

How: the swap lives in one small override layer,
`factory_layout_v2_workers.usda`, which is added as a sublayer of
`factory_layout_v2_physics.usd` (the file the simulation loads).

Run with the USD libraries that ship with Isaac Sim (no simulator is launched):
    ./scripts/usd_python.sh scripts/replace_factory_workers.py            # apply
    ./scripts/usd_python.sh scripts/replace_factory_workers.py --revert   # undo
"""
from pathlib import Path
import argparse,hashlib,json,math,re,shutil,time
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'assets/factory/factory_layout_v2'
PHYSICS=OUT/'factory_layout_v2_physics.usd'; SOURCE=OUT/'factory_layout_v2.usd'
LAYER=OUT/'factory_layout_v2_workers.usda'; SUBLAYER='./factory_layout_v2_workers.usda'
WORKER='./props/worker/working.usda'  # Faces -Y with elbows bent, feet at Z=0.
CHAIR='./props/chair/SM_Chair.usd'                            # faces +X
OLD=re.compile(r'^((?:Operator_(?:Inner|Outer)_(?:Left|Right)_\d+)|(?:Person_\d+)|(?:Chair_\d+))(?:_(.+))?$')
SIDE_STEP_M=.75          # a standing figure cannot sit: it stands beside the chair, facing the desk
DESK_CLEARANCE_M=.03


def main():
    from pxr import Gf,Sdf,Usd,UsdGeom,UsdPhysics
    parser=argparse.ArgumentParser(); parser.add_argument('--revert',action='store_true'); args=parser.parse_args()
    source_hash=hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    backup=OUT/'backups'/('factory_layout_v2_physics_'+time.strftime('%Y%m%d_%H%M%S')+'.usd'); backup.parent.mkdir(exist_ok=True)
    shutil.copy2(PHYSICS,backup)
    root_layer=Sdf.Layer.FindOrOpen(str(PHYSICS))
    # Always read the factory without the swap layer, so re-running is idempotent.
    paths=[p for p in root_layer.subLayerPaths if p!=SUBLAYER]
    if list(root_layer.subLayerPaths)!=paths: root_layer.subLayerPaths=paths
    if args.revert:
        root_layer.Save(); print('REVERTED '+json.dumps({'physics_usd':str(PHYSICS),'backup':str(backup),'layer_kept_on_disk':str(LAYER)})); return

    stage=Usd.Stage.Open(root_layer); root=stage.GetDefaultPrim()
    cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render','proxy'],False,True)
    def box(prim):
        r=cache.ComputeWorldBound(prim).ComputeAlignedRange(); return list(r.GetMin()),list(r.GetMax())
    def centre(prim):
        low,high=box(prim); return [(a+b)/2 for a,b in zip(low,high)]
    def union(prims):
        boxes=[box(p) for p in prims]
        return [min(b[0][i] for b in boxes) for i in range(3)],[max(b[1][i] for b in boxes) for i in range(3)]
    groups={}
    for child in root.GetChildren():
        match=OLD.match(child.GetName())
        if match: groups.setdefault(match.group(1),{})[match.group(2) or 'root']=child
    if not groups: raise RuntimeError('No Operator_/Person_/Chair_ prims found: wrong file?')
    def facing(parts):
        """Unit axis the old figure looked along: forearms sit in front of the torso."""
        torso=centre(parts['Torso']); arms=[centre(p) for name,p in parts.items() if name.startswith('Forearm')]
        dx=sum(a[0] for a in arms)/len(arms)-torso[0]; dy=sum(a[1] for a in arms)/len(arms)-torso[1]
        return (math.copysign(1.,dx),0.) if abs(dx)>abs(dy) else (0.,math.copysign(1.,dy))
    def worker_yaw(direction): return (math.degrees(math.atan2(direction[1],direction[0]))+90.)%360   # the model faces -Y at yaw 0

    workers=[]; chairs=[]
    for name,parts in sorted(groups.items()):
        low,high=union(parts.values())
        if name.startswith('Operator_'):
            direction=facing(parts); torso=centre(parts['Torso'])
            workers.append({'name':name,'was':'standing at a machine','position':[torso[0],torso[1],0.],'yaw_deg':worker_yaw(direction),
                            'facing':direction,'old_bounds':[low,high]})
        elif name.startswith('Person_'):
            direction=facing(parts); number=name.split('_')[1]
            chair_low,chair_high=union(groups['Chair_'+number].values()); chair_x=(chair_low[0]+chair_high[0])/2
            desk_low,_=box(root.GetChild('Table_'+number)); monitor=root.GetChild(f'Table_{number}_Monitor')
            # Step to the side away from the monitor so the screen stays visible from the aisle.
            side=-1. if (monitor and centre(monitor)[0]>chair_x) else 1.
            # Facing +Y towards the desk: boot tips reach 0.225 m in front of the model origin.
            y=desk_low[1]-DESK_CLEARANCE_M-.225 if direction==(0.,1.) else (low[1]+high[1])/2
            workers.append({'name':name,'was':'sitting at a desk','position':[chair_x+side*SIDE_STEP_M,y,0.],'yaw_deg':worker_yaw(direction),
                            'facing':direction,'old_bounds':[low,high],'note':'stands beside the chair; the model has no sitting pose'})
        else:
            number=name.split('_')[1]; person=groups.get('Person_'+number)
            direction=facing(person) if person else (0.,1.)
            yaw=math.degrees(math.atan2(direction[1],direction[0]))%360            # the chair faces +X at yaw 0
            # The chair mesh is centred 0.04 m behind its origin; keep the old chair's footprint centre.
            cx,cy=(low[0]+high[0])/2,(low[1]+high[1])/2
            chairs.append({'name':name,'position':[cx+.04*direction[0],cy+.04*direction[1],0.],'yaw_deg':yaw,'facing':direction,'old_bounds':[low,high]})
    colliders_before=sorted((str(p.GetPath()),box(p)) for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI))

    # ---- author the override layer ------------------------------------------------
    if LAYER.exists(): LAYER.unlink()
    layer=Sdf.Layer.CreateNew(str(LAYER)); edit=Usd.Stage.Open(layer)
    UsdGeom.SetStageUpAxis(edit,'Z'); UsdGeom.SetStageMetersPerUnit(edit,1)
    layer.comment='Factory V2 worker/chair swap. Generated by scripts/replace_factory_workers.py; delete the sublayer entry in factory_layout_v2_physics.usd to undo.'
    base=edit.OverridePrim(root.GetPath()); edit.SetDefaultPrim(base)
    hidden=[]
    for parts in groups.values():
        for prim in parts.values():
            edit.OverridePrim(prim.GetPath()).SetActive(False); hidden.append(prim.GetName())
    def place(scope,item,asset):
        xform=UsdGeom.Xform.Define(edit,root.GetPath().AppendPath(f'{scope}/{item["name"]}'))
        xform.AddTranslateOp().Set(Gf.Vec3d(*item['position'])); xform.AddRotateZOp().Set(float(item['yaw_deg']))
        xform.GetPrim().CreateAttribute('factory:replaces',Sdf.ValueTypeNames.String).Set(item['name'])
        model=edit.DefinePrim(xform.GetPath().AppendChild('model'),'Xform'); model.GetReferences().AddReference(asset)
        item['prim']=str(xform.GetPath()); return model
    UsdGeom.Scope.Define(edit,root.GetPath().AppendChild('Workers')); UsdGeom.Scope.Define(edit,root.GetPath().AppendChild('OfficeChairs'))
    for item in workers: place('Workers',item,WORKER)
    for item in chairs:
        model=place('OfficeChairs',item,CHAIR)
        # The asset ships with a mesh collider. The factory already has COL_Chair_* colliders, so keep physics exactly as it was.
        mesh=edit.OverridePrim(model.GetPath().AppendChild('SM_Chair'))
        mesh.CreateAttribute('physics:collisionEnabled',Sdf.ValueTypeNames.Bool).Set(False)
    layer.Save()
    root_layer.subLayerPaths.insert(0,SUBLAYER); root_layer.Save()

    # ---- verify the composed result (still no simulator) ---------------------------
    check=Usd.Stage.Open(str(PHYSICS)); top=check.GetDefaultPrim()
    cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render','proxy'],False,True)
    def world_box(prim):
        r=cache.ComputeWorldBound(prim).ComputeAlignedRange(); return [round(v,3) for v in r.GetMin()],[round(v,3) for v in r.GetMax()]
    still=[n for n in hidden if top.GetChild(n).IsActive()]
    for item in workers+chairs:
        prim=check.GetPrimAtPath(item['prim']); meshes=[p for p in Usd.PrimRange(prim) if p.IsA(UsdGeom.Mesh)]
        item['meshes']=len(meshes); item['new_bounds']=world_box(prim)
        item['position']=[round(v,3) for v in item['position']]; item['old_bounds']=[[round(v,3) for v in b] for b in item['old_bounds']]
    colliders_after=sorted((str(p.GetPath()),box_) for p,box_ in ((p,(list(cache.ComputeWorldBound(p).ComputeAlignedRange().GetMin()),
        list(cache.ComputeWorldBound(p).ComputeAlignedRange().GetMax()))) for p in check.Traverse()
        if p.HasAPI(UsdPhysics.CollisionAPI) and UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get() is not False))
    same=len(colliders_before)==len(colliders_after) and all(a[0]==b[0] and all(abs(x-y)<1e-9 for u,v in zip(a[1],b[1]) for x,y in zip(u,v))
                                                              for a,b in zip(colliders_before,colliders_after))
    untouched=[c.GetName() for c in top.GetChildren() if c.IsActive() and not OLD.match(c.GetName()) and c.GetName() not in ('Workers','OfficeChairs')]
    report={'status':'applied','date':time.strftime('%Y-%m-%d %H:%M:%S'),'physics_usd':str(PHYSICS.relative_to(ROOT)),'override_layer':str(LAYER.relative_to(ROOT)),
        'backup_of_physics_usd':str(backup.relative_to(ROOT)),'worker_asset':WORKER,'chair_asset':CHAIR,
        'asset_origin':'NVIDIA Isaac Sim 6.1 asset library: People/Characters/male_adult_construction_05_new and Environments/Office/Props/SM_Chair',
        'source_usd_sha256_before':source_hash,'source_usd_sha256_after':hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
        'old_visual_prims_switched_off':len(hidden),'old_visual_prims_still_active':still,'workers_placed':len(workers),'chairs_placed':len(chairs),
        'enabled_colliders_before':len(colliders_before),'enabled_colliders_after':len(colliders_after),'colliders_identical':same,
        'other_top_level_prims_untouched':len(untouched),'workers':workers,'chairs':chairs,
        'not_verified_here':'Rendering, YOLO detection on the new figures, FPS and VRAM: these need a simulation run, which was not started.'}
    ok=(report['source_usd_sha256_before']==report['source_usd_sha256_after'] and not still and same and all(i['meshes']>0 for i in workers+chairs))
    report['checks_passed']=ok
    (ROOT/'docs/factory_v2_worker_replacement.json').write_text(json.dumps(report,indent=2))
    print('WORKERS_REPLACED '+json.dumps({k:report[k] for k in ('checks_passed','old_visual_prims_switched_off','workers_placed','chairs_placed',
        'enabled_colliders_before','enabled_colliders_after','colliders_identical','other_top_level_prims_untouched')}))
    if not ok: raise SystemExit(1)


if __name__=='__main__': main()
