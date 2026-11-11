"""OpenUSD authoring in Blender's bundled pxr; never runs Isaac Sim."""
from pathlib import Path
import json,hashlib,math
from pxr import Usd,UsdGeom,UsdPhysics,UsdShade,Gf,Sdf
ROOT=Path(r'D:\unitree_g1'); OUT=ROOT/'assets/factory/factory_layout_v2'
source=OUT/'factory_layout_v2.usd'; src=Usd.Stage.Open(str(source))
assert UsdGeom.GetStageMetersPerUnit(src)==1 and UsdGeom.GetStageUpAxis(src)=='Z'
manifest=json.loads((OUT/'object_manifest.json').read_text()); nav=json.loads((OUT/'navigation_allowed_areas.json').read_text())
dest=OUT/'factory_layout_v2_physics.usd'
stage=Usd.Stage.CreateNew(str(dest)) if not dest.exists() else Usd.Stage.Open(str(dest))
stage.GetRootLayer().Clear()
root=UsdGeom.Xform.Define(stage,'/FactoryV2'); stage.SetDefaultPrim(root.GetPrim())
UsdGeom.SetStageUpAxis(stage,'Z'); UsdGeom.SetStageMetersPerUnit(stage,1)
root.GetPrim().GetReferences().AddReference('./factory_layout_v2.usd',src.GetDefaultPrim().GetPath())
physics=UsdPhysics.Scene.Define(stage,'/FactoryV2/PhysicsScene'); physics.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1)); physics.CreateGravityMagnitudeAttr(9.81)
mat=UsdShade.Material.Define(stage,'/FactoryV2/PhysicsMaterials/Floor')
props=UsdPhysics.MaterialAPI.Apply(mat.GetPrim()); props.CreateStaticFrictionAttr(.8); props.CreateDynamicFrictionAttr(.6); props.CreateRestitutionAttr(0)
colliders=[]
for row in manifest['objects']:
    if not row['object_name'].startswith('COL_'): continue
    lo,hi=row['world_bounding_box_m']; dims=[hi[i]-lo[i] for i in range(3)]; center=[(hi[i]+lo[i])/2 for i in range(3)]
    name='Floor' if row['object_name']=='COL_Floor' else row['object_name']
    cube=UsdGeom.Cube.Define(stage,'/FactoryV2/PhysicsColliders/'+name); cube.CreateSizeAttr(1)
    cube.AddTranslateOp().Set(Gf.Vec3d(*center)); cube.AddScaleOp().Set(Gf.Vec3d(*dims))
    cube.CreateVisibilityAttr('invisible'); cube.CreatePurposeAttr('proxy')
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim()).CreateCollisionEnabledAttr(True)
    UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(mat,UsdShade.Tokens.weakerThanDescendants,'physics')
    cube.GetPrim().CreateAttribute('factory:semantic_class',Sdf.ValueTypeNames.String).Set(row['semantic_class'])
    if row['semantic_class']=='person': cube.GetPrim().CreateAttribute('navigation:kind',Sdf.ValueTypeNames.String).Set('human')
    colliders.append({'path':str(cube.GetPath()),'source_proxy':row['object_name'],'bounds':[lo,hi],'semantic_class':row['semantic_class']})
stage.GetRootLayer().Save()
base=json.loads((ROOT/'configs/factory_g1.json').read_text())
base.update(factory_id='v2',factory_usd=str(dest.relative_to(ROOT)).replace('\\','/'),floor_prim_relative='PhysicsColliders/Floor',
    navigation_map='configs/navigation_map_v2.json',patrol_user_config='configs/patrol_v2_user.json',scene_output='logs/generated_g1_factory_v2_base.usd',
    source_factory_usd=str(source.relative_to(ROOT)).replace('\\','/'),source_factory_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    standing_configuration_reference='configs/locomotion_mjlab.yaml: deploy default_joint_pos and existing FK height',
    navigation_map_verified=True,hide_source_object_ids=['Factory_Roof'],top_view={'height_m':45,'projection':'perspective','focal_length_mm':19,'horizontal_aperture':20.955,'vertical_aperture':13.97,'clipping_m':[.1,100]})
base['robot_spawn']={'marker_prefix':'Robot_Entry_Spawn','position_override_m':None,'heading_deg':nav['robot_spawn']['yaw_deg'],'position_from_marker_m':nav['robot_spawn']['position_m'],'clearance_radius_m':.67,'units':'meters; heading degrees about +Z; robot forward +X'}
base['entrance_apron']={'enabled':False}; base['head_camera']['enabled']=True
base['multi_route_enabled']=True
base['head_camera']['torso_offset_m']=[.1,0,.52]
keys=json.loads((ROOT/'configs/keyboard.yaml').read_text())['keyboard_mapping']
keys.update(SPACE='follow_camera',X='stop',F='follow_camera')
base['keyboard_overrides']['keyboard_mapping']=keys
base['camera'].update(mode='free',free_eye_m=[-12,-16,3],free_target_m=[-8,-13.5,.8],overview_eye_m=[22,-29,30],overview_target_m=[0,0,0])
base['note']='One V2 factory, one unchanged G1/ONNX, main viewport plus 320x180 live head-camera preview. Roof hidden visually at runtime; roof collision remains.'
(ROOT/'configs/factory_v2.json').write_text(json.dumps(base,indent=2),encoding='utf-8')
map_config={'factory_id':'v2','source_factory_sha256':base['source_factory_sha256'],'resolution_m':.1,'footprint':nav['footprint'],
    'allowed_lanes':[{'name':p['id'],'polygon':p['outer']} for p in nav['allowed_polygons']],
    'forbidden_zones':[{'polygon':h} for p in nav['allowed_polygons'] for h in p['holes']],
    'static_obstacles':[],'obstacle_height_range_m':[.06,1.9]}
(ROOT/'configs/navigation_map_v2.json').write_text(json.dumps(map_config,indent=2),encoding='utf-8')
(OUT/'physics_layer_manifest.json').write_text(json.dumps({'source_sha256':base['source_factory_sha256'],'colliders':colliders,'friction':[.8,.6],'spawn':nav['robot_spawn']},indent=2),encoding='utf-8')
print('V2_PHYSICS_BUILT',len(colliders),flush=True)
