"""Reproducible scenario obstacles. No sensor-based avoidance or route edits."""
from pathlib import Path
import random,json,math
ROOT=Path(__file__).resolve().parents[1]
FOLDER=ROOT/'assets/factory/factory_layout_v2/maps'

def layout(map_id):
    if map_id not in (1,2,3):raise ValueError('Map must be 1, 2 or 3')
    if map_id==1:return []
    rng=random.Random(5100+map_id);items=[]
    # Long, straight lane interiors; avoid spawn, doors and corner waypoints.
    for lane,x in [('shared_left',-6.985645933014354),('route_1_center',0.),('route_2_right',6.985645933014354)]:
        for j,kind in enumerate(['chair','person','box']):
            y=[-6.,1.,7.][j]+rng.uniform(-1.,1.)
            items.append({'name':f'{lane}_{kind}','kind':kind,'position_m':[x+rng.uniform(-.08,.08),y,0.],
                          'yaw_deg':rng.choice([0.,90.,180.,270.]),'lane':lane})
    return items

def build(map_id):
    from pxr import Usd,UsdGeom,UsdPhysics,Gf,Sdf
    FOLDER.mkdir(parents=True,exist_ok=True);path=FOLDER/f'map_{map_id}.usda'
    if path.exists():
        manifest=path.with_suffix('.json')
        if manifest.exists() and json.loads(manifest.read_text())['obstacles']==layout(map_id):return path
        raise FileExistsError(f'Existing map differs; preserve it and generate a new version: {path}')
    stage=Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageMetersPerUnit(stage,1);UsdGeom.SetStageUpAxis(stage,'Z')
    root=UsdGeom.Xform.Define(stage,'/FactoryVariant');stage.SetDefaultPrim(root.GetPrim())
    stage.DefinePrim('/FactoryVariant/Factory','Xform').GetReferences().AddReference('../factory_layout_v2_physics.usd')
    UsdGeom.Xform.Define(stage,'/FactoryVariant/Obstacles')
    for item in layout(map_id):
        prim=UsdGeom.Xform.Define(stage,'/FactoryVariant/Obstacles/'+item['name'])
        prim.AddTranslateOp().Set(Gf.Vec3d(*item['position_m']));prim.AddRotateZOp().Set(item['yaw_deg'])
        prim.GetPrim().CreateAttribute('navigation:routeTestObstacle',Sdf.ValueTypeNames.Bool).Set(True)
        prim.GetPrim().CreateAttribute('navigation:kind',Sdf.ValueTypeNames.String).Set('human' if item['kind']=='person' else 'static')
        if item['kind']!='box':
            asset='../props/chair/SM_Chair.usd' if item['kind']=='chair' else '../props/worker/standing.usda'
            model=stage.DefinePrim(str(prim.GetPath())+'/Visual','Xform');model.GetReferences().AddReference(asset)
            # Asset mesh collisions are superseded by a cheap explicit proxy.
            for child in Usd.PrimRange(model):
                if child.HasAPI(UsdPhysics.CollisionAPI):UsdPhysics.CollisionAPI(child).CreateCollisionEnabledAttr(False)
        dimensions={'chair':(.65,.65,1.1),'person':(.65,.55,1.85),'box':(.65,.65,.65)}[item['kind']]
        cube=UsdGeom.Cube.Define(stage,str(prim.GetPath())+'/Collider');cube.CreateSizeAttr(1.)
        cube.AddTranslateOp().Set(Gf.Vec3d(0,0,dimensions[2]/2));cube.AddScaleOp().Set(Gf.Vec3f(*dimensions))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        if item['kind']=='box':cube.CreateDisplayColorAttr([Gf.Vec3f(.55,.32,.12)])
        else:cube.CreateVisibilityAttr('invisible')
    stage.GetRootLayer().Save()
    manifest={'map_id':map_id,'seed':5100+map_id,'obstacles':layout(map_id),'base_factory':'../factory_layout_v2_physics.usd',
              'note':'Static collision test props. AI detection is not guaranteed; COCO has no generic box class.'}
    path.with_suffix('.json').write_text(json.dumps(manifest,indent=2))
    return path

def author_layer(layer,map_id):
    """Edit only opinions; no temporary Stage/reference recomposition for resets."""
    from pxr import Sdf,Gf
    if map_id not in (1,2,3):raise ValueError('Invalid map')
    root=Sdf.CreatePrimInLayer(layer,'/World/MapObstacles')
    def attribute(prim,name,type_name,value):
        attr=prim.attributes.get(name) or Sdf.AttributeSpec(prim,name,type_name)
        attr.default=value
    with Sdf.ChangeBlock():
        attribute(root,'visibility',Sdf.ValueTypeNames.Token,'invisible' if map_id==1 else 'inherited')
        for item in layout(map_id if map_id!=1 else 2):
            prim=Sdf.CreatePrimInLayer(layer,'/World/MapObstacles/'+item['name'])
            if map_id!=1:
                attribute(prim,'xformOp:translate',Sdf.ValueTypeNames.Double3,Gf.Vec3d(*item['position_m']))
                attribute(prim,'xformOp:rotateZ',Sdf.ValueTypeNames.Float,float(item['yaw_deg']))
            collider=Sdf.CreatePrimInLayer(layer,str(prim.path)+'/Collider')
            attribute(collider,'physics:collisionEnabled',Sdf.ValueTypeNames.Bool,map_id!=1)

def author(stage,map_id):
    from pxr import UsdGeom,Sdf
    if map_id not in (1,2,3):raise ValueError('Invalid map')
    path=FOLDER/'map_2.usda'
    if not path.exists():raise FileNotFoundError('Build map assets with scripts/build_factory_maps.py first')
    prim=stage.GetPrimAtPath('/World/MapObstacles')
    # One shared pool: both scenarios have the same nine prop types/names.
    # Material/mesh references stay resident. Map1 disables collisions AND
    # visibility, so hidden scenario props cannot block the original route.
    if not prim or not prim.GetAttribute('factory:residentMapPool').Get():
        if prim:stage.RemovePrim('/World/MapObstacles')
        prim=UsdGeom.Xform.Define(stage,'/World/MapObstacles').GetPrim()
        prim.GetReferences().AddReference(str(path),Sdf.Path('/FactoryVariant/Obstacles'))
        prim.CreateAttribute('factory:residentMapPool',Sdf.ValueTypeNames.Bool).Set(True)
    author_layer(stage.GetEditTarget().GetLayer(),map_id)
    return prim
