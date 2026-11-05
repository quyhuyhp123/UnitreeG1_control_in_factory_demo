"""Unitree G1 29-DOF, free-base flat-floor test on Isaac Sim 6.1.

No real-robot SDK, sockets, kinematic base, world anchor or pose-based motion.
Initial USD transforms are computed once by FK before physics starts.
"""
from pathlib import Path
import argparse,asyncio,hashlib,json,math,re,shutil,sys,traceback
from datetime import datetime

PROJECT_ROOT=Path(r'D:\unitree_g1')
sys.path.insert(0,str(PROJECT_ROOT))

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--gui',action='store_true')
    parser.add_argument('--passive',action='store_true',help='Disable joint PD; no balance controller in either mode')
    parser.add_argument('--exit-after-test',action='store_true')
    parser.add_argument('--no-capture',action='store_true')
    parser.add_argument('--updates',type=int)
    args,_=parser.parse_known_args()
    cfg=json.loads((PROJECT_ROOT/'configs/g1_robot.yaml').read_text(encoding='utf-8-sig'))
    report={'status':'starting','mode':'passive' if args.passive else 'joint_pd_only','checks':{}}
    report_file=PROJECT_ROOT/'logs/g1_articulation_report.json'
    report_file.write_text(json.dumps(report),encoding='utf-8')
    source=PROJECT_ROOT/cfg['asset']
    output=PROJECT_ROOT/cfg['scene_output']
    if not source.is_file(): raise FileNotFoundError(source)
    source_hash=hashlib.sha256(source.read_bytes()).hexdigest()
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':not args.gui,'width':960,'height':720,'renderer':'RaytracedLighting','multi_gpu':False,
                       'extra_args':['--/exts/isaacsim.physics.newton/auto_switch_on_startup=false',
                                     '--/exts/isaacsim.core.simulation_manager/default_engine=physx']})
    try:
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
            if args.passive: kp=kd=0.0
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
        if not args.no_capture: capture(PROJECT_ROOT/'logs/g1_initial_isaac.png')
        articulation=Articulation(str(root_path))
        SimulationManager.set_physics_dt(1/cfg['physics_hz'])
        timeline=omni.timeline.get_timeline_interface()
        timeline.play()
        app.update()
        if not articulation.is_physics_tensor_entity_valid(): raise RuntimeError('Articulation tensor view not initialized')
        adapter=G1JointTargets(articulation,expected)
        runtime_names=list(articulation.dof_names)
        if articulation.num_dofs!=29: raise RuntimeError('Runtime DOF count mismatch')
        mapping=[{'runtime_index':i,'name':name,'sdk_index':expected.index(name)} for i,name in enumerate(runtime_names)]
        (PROJECT_ROOT/'configs/g1_joint_indices.json').write_text(json.dumps(mapping,indent=2),encoding='utf-8')
        (PROJECT_ROOT/'docs/g1_joint_indices.md').write_text('# G1 joint indices\n\nRuntime order is discovered, never assumed to equal SDK/policy order.\n\n| Runtime index | Joint name | SDK index |\n|---:|---|---:|\n'+'\n'.join(f"| {r['runtime_index']} | {r['name']} | {r['sdk_index']} |" for r in mapping),encoding='utf-8')
        initial_feedback=adapter.read_feedback()
        q_expected=np.array([q_by_name[n] for n in runtime_names])
        initial_error=float(np.max(np.abs(np.array(initial_feedback['joint_positions_rad'])-q_expected)))
        (PROJECT_ROOT/'logs/g1_initial_feedback_debug.json').write_text(json.dumps({'feedback':initial_feedback,'expected_rad':q_expected.tolist(),'errors_rad':(np.array(initial_feedback['joint_positions_rad'])-q_expected).tolist()},indent=2),encoding='utf-8')
        if initial_error>.08: raise RuntimeError('Initial joint state/FK mismatch '+str(initial_error))
        if not args.passive: adapter.set_joint_targets([q_by_name[n] for n in expected],expected)
        rejected=0
        for values,order in [([float('nan')],[expected[0]]),([0,0],[expected[0],expected[0]]),([0],['not_a_joint']),([0,0],[expected[0]]),([100],[expected[0]])]:
            try: adapter.set_joint_targets(values,order)
            except ValueError: rejected+=1
        if rejected!=5: raise RuntimeError('Target adapter did not reject bad input')
        # Command a small real drive target; verify target readback, not base teleport.
        target_test={'mode':'skipped_passive'}
        if not args.passive:
            test_name='right_wrist_yaw_joint'
            goal=q_by_name[test_name]+.05
            adapter.set_joint_targets([goal],[test_name])
            target_value=float(as_numpy(articulation.get_dof_position_targets()).reshape(-1)[runtime_names.index(test_name)])
            if abs(target_value-goal)>1e-5: raise RuntimeError('Joint target API readback mismatch')
            target_test={'joint':test_name,'target_rad':goal,'readback_rad':target_value}
        feedback=[{'update':0,**initial_feedback}]
        updates=args.updates or cfg['test_updates']
        for i in range(updates):
            app.update()
            if i==59 and not args.passive:
                target_test['position_after_60_updates_rad']=adapter.read_feedback()['joint_positions_rad'][runtime_names.index('right_wrist_yaw_joint')]
                adapter.set_joint_targets([q_by_name[n] for n in expected],expected)
            if (i+1)%30==0: feedback.append({'update':i+1,**adapter.read_feedback()})
        timeline.pause()
        final_feedback=adapter.read_feedback()
        for sample in feedback+[final_feedback]:
            for key,value in sample.items():
                if key not in ['joint_names','update'] and not np.isfinite(value).all(): raise RuntimeError('Nonfinite feedback '+key)
        final_q=Gf.Quatd(final_feedback['base_orientation_wxyz'][0],Gf.Vec3d(*final_feedback['base_orientation_wxyz'][1:]))
        up=Gf.Rotation(final_q).TransformDir(Gf.Vec3d(0,0,1))
        tilt=math.degrees(math.acos(max(-1,min(1,up[2]))))
        fallen=tilt>45 or final_feedback['base_position_m'][2]<height*.55
        report.update({'status':'passed','asset':str(source),'asset_sha256':source_hash,'scene':str(output),
                       'physics_engine':str(SimulationManager.get_active_physics_engine()),'physics_hz':cfg['physics_hz'],
                       'articulation_root':str(root_path),'body_count':len(bodies),'dof_count':articulation.num_dofs,
                       'joint_names_runtime':runtime_names,'runtime_order_equals_sdk':runtime_names==expected,
                       'body_mass_sum_kg':sum(r['mass_kg'] for r in body_report),'bodies':body_report,'joints':joint_report,
                       'collider_count':len(colliders),'added_box_proxies':added_proxies,
                       'proxy_self_collision_note':'Added box proxies filter against all robot bodies; they still collide with floor/external bodies. Original Unitree shapes keep configured self-collision.',
                       'collider_approximations':sorted({str(UsdPhysics.MeshCollisionAPI(p).GetApproximationAttr().Get()) for p in colliders}),
                       'initial_base_height_m':height,'initial_foot_bounds_m':initial_foot_bounds,'initial_max_joint_error_rad':initial_error,
                       'initial_feedback':initial_feedback,'final_feedback':final_feedback,'final_tilt_deg':tilt,'fallen_during_test':fallen,
                       'interpretation':'Joint PD is not a balance/walking controller; falling does not by itself imply an asset error.',
                       'target_adapter_test':target_test,'invalid_target_cases_rejected':rejected,'timeline_seconds':timeline.get_current_time(),
                       'updates':updates,'self_collision':cfg['self_collision']})
        report['checks']={'one_free_base_articulation':True,'no_kinematic_links_or_world_anchors':True,'joint_count_names_unique':True,
                          'joint_graph_connected':True,'mass_and_inertia_positive_physical':True,'collision_on_each_body':True,
                          'initial_foot_clearance_calculated':True,'initial_joint_state_matches':True,'finite_feedback':True,
                          'named_joint_target_api':not args.passive,'source_unchanged':hashlib.sha256(source.read_bytes()).hexdigest()==source_hash}
        (PROJECT_ROOT/'logs/g1_feedback.json').write_text(json.dumps(feedback,indent=2),encoding='utf-8')
        report_file.write_text(json.dumps(report,indent=2),encoding='utf-8')
        print('G1_RESULT '+json.dumps({k:v for k,v in report.items() if k not in ['bodies','joints','initial_feedback','final_feedback','initial_foot_bounds_m']}),flush=True)
        if not args.no_capture:
            # Camera-only framing for the final state, including a fallen robot.
            base=final_feedback['base_position_m']
            target=Gf.Vec3d(base[0],base[1],max(.4,base[2]))
            camera.GetOrderedXformOps()[0].Set(Gf.Matrix4d().SetLookAt(target+Gf.Vec3d(2.4,-3,2.1),target,Gf.Vec3d(0,0,1)).GetInverse())
            camera.GetFocalLengthAttr().Set(25)
            capture(PROJECT_ROOT/'logs/g1_final_isaac.png')
        if args.gui and not args.exit_after_test:
            timeline.play()
            while app.is_running(): app.update()
    except BaseException:
        report['status']='failed'
        report['error']=traceback.format_exc()
        report_file.write_text(json.dumps(report,indent=2),encoding='utf-8')
        print(report['error'],flush=True)
        raise
    finally: app.close()

if __name__=='__main__': main()
