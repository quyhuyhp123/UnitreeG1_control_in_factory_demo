"""Scene builder extracted from the validated flat-test setup; no runtime motion."""
from pathlib import Path
import asyncio,json,math,re,shutil
from datetime import datetime
PROJECT_ROOT=Path(r"D:\unitree_g1")

def build_g1_scene(app,cfg,passive=False):
    source=PROJECT_ROOT/cfg['asset']
    output=PROJECT_ROOT/cfg['scene_output']
    import numpy as np
    import carb,omni.usd,omni.timeline
    from pxr import Usd,UsdGeom,UsdPhysics,UsdShade,UsdLux,PhysxSchema,Gf,Sdf
    from isaacsim.core.simulation_manager import SimulationManager
    from isaacsim.core.experimental.prims import Articulation
    from controllers.g1_joint_targets import G1JointTargets,as_numpy
    SimulationManager.switch_physics_engine('physx')
    SimulationManager.set_device('cpu')
    SimulationManager.enable_fabric(False)
    settings=carb.settings.get_settings()
    settings.set('/physics/suppressReadback',False)
    settings.set('/physics/updateToUsd',True)
    omni.usd.get_context().new_stage()
    stage=omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage,'Z')
    UsdGeom.SetStageMetersPerUnit(stage,1)
    stage.SetTimeCodesPerSecond(cfg['physics_hz'])
    world=UsdGeom.Xform.Define(stage,'/World')
    stage.SetDefaultPrim(world.GetPrim())
    robot=UsdGeom.Xform.Define(stage,'/World/G1').GetPrim()
    source_stage=Usd.Stage.Open(str(source))
    source_root=source_stage.GetDefaultPrim()
    if UsdGeom.GetStageMetersPerUnit(source_stage)!=1 or UsdGeom.GetStageUpAxis(source_stage)!='Z':
        raise RuntimeError('Asset units/up axis mismatch')
    robot.GetReferences().AddReference(str(source),source_root.GetPath())
    robot.GetVariantSets().GetVariantSet('Physics').SetVariantSelection('PhysX')
    robot.GetVariantSets().GetVariantSet('Sensor').SetVariantSelection('None')
    app.update()
    def robot_prims(): return list(Usd.PrimRange(robot,Usd.TraverseInstanceProxies()))
    prims=robot_prims()
    bodies=[p for p in prims if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    roots=[p for p in prims if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    joints=[p for p in prims if p.IsA(UsdPhysics.RevoluteJoint)]
    colliders=[p for p in prims if p.HasAPI(UsdPhysics.CollisionAPI) and UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get()]
    if len(roots)!=1 or len(bodies)!=30 or len(joints)!=29: raise RuntimeError('Unexpected G1 articulation structure')
    names=[p.GetName() for p in joints]
    expected=cfg['sdk_joint_order']
    if len(set(names))!=29 or set(names)!=set(expected): raise RuntimeError('Missing/extra/duplicate source joints')
    if any(p.IsA(UsdGeom.Camera) for p in prims): raise RuntimeError('Unexpected robot camera')
    for p in prims:
        if p.IsA(UsdPhysics.FixedJoint):
            joint=UsdPhysics.Joint(p)
            if not joint.GetBody0Rel().GetTargets() or not joint.GetBody1Rel().GetTargets():
                raise RuntimeError('Robot fixed to world')
    body_report=[]
    body_paths={p.GetPath() for p in bodies}
    added_proxies=[]
    for body in bodies:
        if any(c.GetPath().HasPrefix(body.GetPath()) for c in colliders): continue
        cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render','proxy','guide'],False,True)
        extent=cache.ComputeUntransformedBound(body).ComputeAlignedRange()
        size=extent.GetSize()
        center=extent.GetMidpoint()
        if any(not math.isfinite(v) or v<=0 or v>2 for v in size): raise RuntimeError('Cannot derive missing collider '+str(body.GetPath()))
        # A conservative local box is explicitly approximate; original
        # collision geometry on all other links is left untouched.
        proxy=UsdGeom.Cube.Define(stage,body.GetPath().AppendChild('ProjectCollisionProxy'))
        proxy.CreateSizeAttr(1)
        proxy.AddTranslateOp().Set(center)
        proxy.AddScaleOp().Set(Gf.Vec3f(size))
        proxy.CreatePurposeAttr('guide')
        proxy.CreateVisibilityAttr('invisible')
        UsdPhysics.CollisionAPI.Apply(proxy.GetPrim())
        # Supplemental visual-bound boxes can overlap neighboring housings.
        # Keep external/floor collision but exclude these approximate shapes
        # from self-collision; original Unitree shapes retain self-collision.
        UsdPhysics.FilteredPairsAPI.Apply(proxy.GetPrim()).CreateFilteredPairsRel().SetTargets(sorted(body_paths))
        colliders.append(proxy.GetPrim())
        added_proxies.append({'body':str(body.GetPath()),'proxy':str(proxy.GetPath()),'local_size_m':list(size),'local_center_m':list(center)})
    for p in bodies:
        rb=UsdPhysics.RigidBodyAPI(p)
        if rb.GetKinematicEnabledAttr().Get(): raise RuntimeError('Kinematic robot link')
        mass=UsdPhysics.MassAPI(p)
        value=mass.GetMassAttr().Get()
        inertia=list(mass.GetDiagonalInertiaAttr().Get())
        if not value or value<=0 or not all(math.isfinite(x) and x>0 for x in inertia): raise RuntimeError('Invalid mass/inertia '+str(p.GetPath()))
        if max(inertia)>sum(inertia)-max(inertia)+1e-6: raise RuntimeError('Nonphysical inertia '+str(p.GetPath()))
        owned=[c for c in colliders if c.GetPath().HasPrefix(p.GetPath())]
        if not owned: raise RuntimeError('Missing collision geometry for '+str(p.GetPath()))
        body_report.append({'path':str(p.GetPath()),'mass_kg':value,'inertia_diagonal_kg_m2':inertia,
                            'principal_axes_wxyz':str(mass.GetPrincipalAxesAttr().Get()),'center_of_mass_m':list(mass.GetCenterOfMassAttr().Get()),
                            'collider_paths':[str(c.GetPath()) for c in owned]})
    q_by_name={name:0.0 for name in names}
    for pattern,value in cfg['initial_joint_position_rules_rad'].items():
        for name in names:
            if re.fullmatch(pattern,name): q_by_name[name]=value
    def resolve(value,name):
        if isinstance(value,dict):
            matched=[v for k,v in value.items() if re.fullmatch(k,name)]
            if len(matched)!=1: raise RuntimeError('Ambiguous gain rule '+name)
            return matched[0]
        return value
    joint_report=[]
    joint_by_child={}
    for p in joints:
        j=UsdPhysics.RevoluteJoint(p)
        parent=j.GetBody0Rel().GetTargets()
        child=j.GetBody1Rel().GetTargets()
        if len(parent)!=1 or len(child)!=1 or parent[0] not in body_paths or child[0] not in body_paths:
            raise RuntimeError('Broken joint graph '+p.GetName())
        if child[0] in joint_by_child: raise RuntimeError('Multiple parent joints')
        joint_by_child[child[0]]=p
        name=p.GetName()
        lo,hi=[math.radians(v) for v in [j.GetLowerLimitAttr().Get(),j.GetUpperLimitAttr().Get()]]
        if not math.isfinite(lo+hi) or lo>=hi or not lo<=q_by_name[name]<=hi: raise RuntimeError('Invalid limits/pose '+name)
        groups=[g for g in cfg['actuator_groups'].values() if any(re.fullmatch(expr,name) for expr in g['joint_names_expr'])]
        if len(groups)!=1: raise RuntimeError('Actuator coverage '+name)
        group=groups[0]
        kp,kd=resolve(group['stiffness'],name),resolve(group['damping'],name)
        if passive: kp=kd=0.0
        drive=UsdPhysics.DriveAPI.Apply(p,'angular')
        drive.CreateTypeAttr('force')
        # USD angular gains are per degree; external config uses per radian.
        drive.CreateStiffnessAttr(kp*math.pi/180)
        drive.CreateDampingAttr(kd*math.pi/180)
        drive.CreateMaxForceAttr(group['effort_limit_sim'])
        drive.CreateTargetPositionAttr(math.degrees(q_by_name[name]))
        drive.CreateTargetVelocityAttr(0)
        p.GetAttribute('state:angular:physics:position').Set(math.degrees(q_by_name[name]))
        p.GetAttribute('state:angular:physics:velocity').Set(0)
        px=PhysxSchema.PhysxJointAPI.Apply(p)
        px.CreateMaxJointVelocityAttr(math.degrees(group['velocity_limit_sim']))
        px.CreateArmatureAttr(group['armature'])
        joint_report.append({'name':name,'path':str(p.GetPath()),'lower_rad':lo,'upper_rad':hi,'initial_rad':q_by_name[name],
                             'stiffness_Nm_rad':kp,'damping_Nm_s_rad':kd,'effort_limit_Nm':group['effort_limit_sim'],
                             'velocity_limit_rad_s':group['velocity_limit_sim'],'armature':group['armature']})
    root_path=roots[0].GetPath()
    if set(body_paths)-set(joint_by_child)!={root_path}: raise RuntimeError('Joint graph not a single free-base tree')
    articulation_api=PhysxSchema.PhysxArticulationAPI.Apply(roots[0])
    articulation_api.CreateEnabledSelfCollisionsAttr(cfg['self_collision'])
    articulation_api.CreateSolverPositionIterationCountAttr(32)
    articulation_api.CreateSolverVelocityIterationCountAttr(4)
    # One-time reset pose FK; runtime controller only sends joint targets.
    def frame(position,rotation):
        q=Gf.Quatd(float(rotation.GetReal()),Gf.Vec3d(rotation.GetImaginary()))
        matrix=Gf.Matrix4d().SetRotate(q)
        matrix.SetTranslateOnly(Gf.Vec3d(position))
        return matrix
    poses={root_path:Gf.Matrix4d(1)}
    remaining=dict(joint_by_child)
    while remaining:
        before=len(remaining)
        for child,p in list(remaining.items()):
            joint=UsdPhysics.RevoluteJoint(p)
            parent=joint.GetBody0Rel().GetTargets()[0]
            if parent not in poses: continue
            local0=frame(joint.GetLocalPos0Attr().Get(),joint.GetLocalRot0Attr().Get())
            local1=frame(joint.GetLocalPos1Attr().Get(),joint.GetLocalRot1Attr().Get())
            axis={'X':Gf.Vec3d(1,0,0),'Y':Gf.Vec3d(0,1,0),'Z':Gf.Vec3d(0,0,1)}[joint.GetAxisAttr().Get()]
            rotation=Gf.Matrix4d().SetRotate(Gf.Rotation(axis,math.degrees(q_by_name[p.GetName()])))
            poses[child]=local1.GetInverse()*rotation*local0*poses[parent]
            del remaining[child]
        if len(remaining)==before: raise RuntimeError('Disconnected/cyclic joint graph')
    for path,matrix in poses.items():
        xf=UsdGeom.Xformable(stage.GetPrimAtPath(path))
        xf.ClearXformOpOrder()
        xf.AddTransformOp(opSuffix='initialPose').Set(matrix)
    feet=[c for c in colliders if any(n in str(c.GetPath()) for n in ['left_ankle_roll_link/','right_ankle_roll_link/'])]
    if not feet: raise RuntimeError('Foot collision geometry missing')
    def bounds(p):
        cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render','proxy','guide'],False,True)
        value=cache.ComputeWorldBound(p).ComputeAlignedRange()
        return [list(value.GetMin()),list(value.GetMax())]
    lowest=min(bounds(p)[0][2] for p in feet)
    height=cfg['initial_sole_clearance_m']-lowest
    robot_xf=UsdGeom.Xformable(robot)
    robot_xf.ClearXformOpOrder()
    robot_xf.AddTranslateOp(opSuffix='spawn').Set(Gf.Vec3d(0,0,height))
    orientation=cfg['initial_base_orientation_wxyz']
    if orientation!=[1,0,0,0]: raise ValueError('This reset FK is validated for identity orientation only')
    initial_foot_bounds={str(p.GetPath()):bounds(p) for p in feet}
    floor=UsdGeom.Mesh.Define(stage,'/World/Floor')
    s=cfg['floor_size_m']/2
    floor.CreatePointsAttr([(-s,-s,-.1),(-s,-s,0),(-s,s,-.1),(-s,s,0),(s,-s,-.1),(s,-s,0),(s,s,-.1),(s,s,0)])
    floor.CreateFaceVertexCountsAttr([4]*6)
    floor.CreateFaceVertexIndicesAttr([0,4,6,2,1,3,7,5,0,1,5,4,2,6,7,3,0,2,3,1,4,5,7,6])
    floor.CreateSubdivisionSchemeAttr('none')
    UsdPhysics.CollisionAPI.Apply(floor.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(floor.GetPrim()).CreateApproximationAttr('boundingCube')
    material=UsdShade.Material.Define(stage,'/World/Materials/Floor')
    api=UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr(cfg['floor_static_friction'])
    api.CreateDynamicFrictionAttr(cfg['floor_dynamic_friction'])
    api.CreateRestitutionAttr(0)
    UsdShade.MaterialBindingAPI.Apply(floor.GetPrim()).Bind(material,UsdShade.Tokens.weakerThanDescendants,'physics')
    shader=UsdShade.Shader.Define(stage,'/World/Materials/Floor/Shader')
    shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.10,.25,.16))
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
    UsdShade.MaterialBindingAPI.Apply(floor.GetPrim()).Bind(material)
    ps=UsdPhysics.Scene.Define(stage,'/World/PhysicsScene')
    ps.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1))
    ps.CreateGravityMagnitudeAttr(cfg['gravity_m_s2'])
    px=PhysxSchema.PhysxSceneAPI.Apply(ps.GetPrim())
    px.CreateEnableGPUDynamicsAttr(False)
    px.CreateBroadphaseTypeAttr('MBP')
    px.CreateTimeStepsPerSecondAttr(cfg['physics_hz'])
    px.CreateEnableCCDAttr(True)
    px.CreateSolverTypeAttr('TGS')
    if sum(p.IsA(UsdPhysics.Scene) for p in stage.Traverse())!=1: raise RuntimeError('More than one physics scene')
    dome=UsdLux.DomeLight.Define(stage,'/World/Lighting/Dome')
    dome.CreateIntensityAttr(700)
    for i,(x,y) in enumerate([(1,-1),(-1,1)]):
        lamp=UsdLux.RectLight.Define(stage,f'/World/Lighting/Key_{i}')
        lamp.CreateIntensityAttr(5000)
        lamp.CreateNormalizeAttr(False)
        lamp.CreateWidthAttr(2)
        lamp.CreateHeightAttr(2)
        UsdGeom.Xformable(lamp).AddTranslateOp().Set(Gf.Vec3d(x,y,3))
    camera=UsdGeom.Camera.Define(stage,'/World/TestCamera')
    camera.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(2.3,-2.4,1.7),Gf.Vec3d(0,0,.75),Gf.Vec3d(0,0,1)).GetInverse())
    camera.CreateFocalLengthAttr(30)
    camera.CreateClippingRangeAttr(Gf.Vec2f(.05,100))
    stage.GetRootLayer().customLayerData={'cameraSettings':{'boundCamera':'/World/TestCamera'}}
    if output.exists():
        backup=PROJECT_ROOT/'logs/g1_backups'/datetime.now().strftime('%Y%m%d_%H%M%S')
        backup.mkdir(parents=True,exist_ok=True)
        shutil.copy2(output,backup/output.name)
    stage.GetRootLayer().Export(str(output))
    layer=Sdf.Layer.FindOrOpen(str(output))
    layer.GetPrimAtPath('/World/G1').referenceList.prependedItems=[Sdf.Reference('unitree_model/G1/29dof/usd/g1_29dof_rev_1_0/g1_29dof_rev_1_0.usd',source_root.GetPath())]
    layer.Save()
    from omni.kit.viewport.utility import get_active_viewport,capture_viewport_to_file
    viewport=get_active_viewport()
    viewport.camera_path='/World/TestCamera'
    viewport.set_texture_resolution((960,720))
    async def capture_wait(path):
        capture=capture_viewport_to_file(viewport,str(path))
        return await capture.wait_for_result()
    def capture(path):
        for _ in range(40): app.update()
        task=asyncio.ensure_future(capture_wait(path))
        for _ in range(200):
            app.update()
            if task.done(): break
        if not task.done(): raise RuntimeError('Viewport capture timeout')
        task.result()

    return {'stage':stage,'robot':robot,'root_path':str(root_path),'height':height,'camera':camera,'viewport':viewport,'capture':capture,'bodies':body_report,'joints':joint_report,'added_proxies':added_proxies,'foot_bounds':initial_foot_bounds}
