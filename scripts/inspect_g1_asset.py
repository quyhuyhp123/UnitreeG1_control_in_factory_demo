from pathlib import Path
import json,math
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':640,'height':480,'renderer':'RaytracedLighting','multi_gpu':False})
from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema
root=Path(r'D:\unitree_g1')
path=root/'assets/robot/unitree_model/G1/29dof/usd/g1_29dof_rev_1_0/g1_29dof_rev_1_0.usd'
stage=Usd.Stage.Open(str(path))
cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render','proxy','guide'],False,True)
result={'default':str(stage.GetDefaultPrim().GetPath()),'up':str(UsdGeom.GetStageUpAxis(stage)),'meters':UsdGeom.GetStageMetersPerUnit(stage),'used_layers':[l.realPath for l in stage.GetUsedLayers()],'roots':[],'bodies':[],'joints':[],'colliders':[],'sensors':[]}
for layer in stage.GetUsedLayers():
    if layer.realPath and 'base.usd' not in layer.realPath:
        (root/'logs'/ (Path(layer.realPath).stem+'.usda')).write_text(layer.ExportToString(),encoding='utf-8')
result['prim_schema_inventory']=[{'path':str(p.GetPath()),'type':p.GetTypeName(),'active':p.IsActive(),'schemas':p.GetAppliedSchemas()} for p in stage.TraverseAll()]
for p in stage.Traverse(Usd.TraverseInstanceProxies()):
    if p.HasAPI(UsdPhysics.ArticulationRootAPI): result['roots'].append(str(p.GetPath()))
    if p.HasAPI(UsdPhysics.RigidBodyAPI):
        api=UsdPhysics.MassAPI(p)
        result['bodies'].append({'path':str(p.GetPath()),'mass':api.GetMassAttr().Get(),'inertia':list(api.GetDiagonalInertiaAttr().Get()) if api.GetDiagonalInertiaAttr().Get() is not None else None,'kinematic':UsdPhysics.RigidBodyAPI(p).GetKinematicEnabledAttr().Get(),'matrix':list(map(list,UsdGeom.XformCache().GetLocalToWorldTransform(p)))})
    if p.IsA(UsdPhysics.Joint):
        j=UsdPhysics.Joint(p)
        row={'name':p.GetName(),'path':str(p.GetPath()),'type':p.GetTypeName(),'body0':[str(x) for x in j.GetBody0Rel().GetTargets()],'body1':[str(x) for x in j.GetBody1Rel().GetTargets()]}
        row['attrs']={a.GetName():str(a.Get()) for a in p.GetAttributes() if a.GetName().startswith(('physics:','drive:','physxJoint:','state:'))}
        result['joints'].append(row)
    if p.HasAPI(UsdPhysics.CollisionAPI):
        b=cache.ComputeWorldBound(p).ComputeAlignedRange()
        result['colliders'].append({'path':str(p.GetPath()),'enabled':UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get(),'bounds':[list(b.GetMin()),list(b.GetMax())],'approximation':str(UsdPhysics.MeshCollisionAPI(p).GetApproximationAttr().Get())})
    if p.GetTypeName() in ['Camera','IsaacRtxLidarSensor','IsaacRtxRadarSensor']: result['sensors'].append(str(p.GetPath()))
(root/'logs/g1_asset_inventory.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in result.items() if k not in ['bodies','joints','colliders']},indent=2),flush=True)
print('COUNTS',len(result['bodies']),len(result['joints']),len(result['colliders']),flush=True)
app.close()
