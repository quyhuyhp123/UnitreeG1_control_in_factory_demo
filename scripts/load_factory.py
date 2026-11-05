"""Isaac Sim 6.1 standalone: USD reference + static box colliders, no robot.

Run with scripts/run_factory.ps1. Source USD is never modified.
"""
from pathlib import Path
import argparse
import hashlib
import json
import shutil
import traceback
from datetime import datetime

PROJECT_ROOT = Path(r'D:\unitree_g1')

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--validate', action='store_true')
    parser.add_argument('--frames', type=int, default=0, help='GUI auto-close frames; 0 keeps GUI open')
    parser.add_argument('--config', type=Path, default=PROJECT_ROOT/'configs/factory_physics.json')
    args, kit_args = parser.parse_known_args()
    cfg = json.loads(args.config.read_text(encoding='utf-8-sig'))
    source = PROJECT_ROOT/cfg['source']
    output = PROJECT_ROOT/cfg['output']
    if not source.is_file(): raise FileNotFoundError(source)
    if output.resolve() == source.resolve(): raise ValueError('Output must differ from source')
    if not output.resolve().is_relative_to(PROJECT_ROOT): raise ValueError('Output outside project')
    if cfg['gravity_m_s2'] <= 0 or cfg['physics_hz'] <= 0: raise ValueError('Invalid gravity/timestep')
    friction = cfg['floor_material']
    if not 0 <= friction['dynamic_friction'] <= friction['static_friction'] or not 0 <= friction['restitution'] <= 1:
        raise ValueError('Invalid physics material')
    report_path = PROJECT_ROOT/'logs/factory_physics_result.json'
    report = {'status':'starting','isaac_sim_target':'6.1','source':str(source),'output':str(output),'checks':{}}
    source_hash = hashlib.sha256(source.read_bytes()).hexdigest()
    from isaacsim import SimulationApp
    app = SimulationApp({'headless':not args.gui,'width':cfg['render_width'],'height':cfg['render_height'],
                         'renderer':'RaytracedLighting','multi_gpu':False,'extra_args':[
                             '--/exts/isaacsim.physics.newton/auto_switch_on_startup=false',
                             '--/exts/isaacsim.core.simulation_manager/default_engine=physx']})
    try:
        import carb
        import omni.usd
        import omni.timeline
        import omni.physics.core
        from isaacsim.core.simulation_manager import SimulationManager
        from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, UsdLux, PhysxSchema, Gf, Sdf
        SimulationManager.switch_physics_engine('physx')
        report['physics_engine'] = str(SimulationManager.get_active_physics_engine())
        settings = carb.settings.get_settings()
        settings.set('/rtx/rendermode','RaytracedLighting')
        settings.set('/physics/updateToUsd',True)
        settings.set('/physics/updateVelocitiesToUsd',True)
        settings.set('/app/asyncRendering',False)
        omni.usd.get_context().new_stage()
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage,1.0)
        stage.SetTimeCodesPerSecond(cfg['physics_hz'])
        world = UsdGeom.Xform.Define(stage,'/World')
        stage.SetDefaultPrim(world.GetPrim())
        factory = UsdGeom.Xform.Define(stage,'/World/Factory').GetPrim()
        source_stage = Usd.Stage.Open(str(source))
        source_root = source_stage.GetDefaultPrim()
        if not source_root: raise RuntimeError('Source USD has no default prim')
        if UsdGeom.GetStageMetersPerUnit(source_stage) != 1 or UsdGeom.GetStageUpAxis(source_stage) != 'Z':
            raise RuntimeError('Source must be Z-up and meters; no implicit conversion')
        factory.GetReferences().AddReference(str(source),source_root.GetPath())
        for _ in range(10): app.update()
        cache = UsdGeom.XformCache()
        bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(),[UsdGeom.Tokens.default_,UsdGeom.Tokens.render],useExtentsHint=False)
        def bounds(prim):
            r=bbox.ComputeWorldBound(prim).ComputeAlignedRange()
            return [list(r.GetMin()),list(r.GetMax())]
        def category(prim):
            while prim and prim.GetPath()!=Sdf.Path('/World'):
                attr=prim.GetAttribute('userProperties:asset_type')
                if attr and attr.Get(): return str(attr.Get())
                prim=prim.GetParent()
            return None
        meshes=[p for p in Usd.PrimRange(factory) if p.IsA(UsdGeom.Mesh)]
        if not meshes: raise RuntimeError('Reference loaded no meshes')
        transforms={str(p.GetPath()):list(map(list,cache.GetLocalToWorldTransform(p))) for p in meshes}
        geometry_changes=[]
        for prim in meshes:
            suffix=prim.GetPath().MakeRelativePath(factory.GetPath())
            original=source_stage.GetPrimAtPath(source_root.GetPath().AppendPath(suffix))
            if not original or prim.GetName()!=original.GetName(): geometry_changes.append(str(prim.GetPath()))
            elif cache.GetLocalToWorldTransform(prim)!=cache.GetLocalToWorldTransform(original): geometry_changes.append(str(prim.GetPath()))
        if geometry_changes: raise RuntimeError('Reference changed names/transforms: '+str(geometry_changes))
        ps=UsdPhysics.Scene.Define(stage,'/World/PhysicsScene')
        ps.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1))
        ps.CreateGravityMagnitudeAttr(cfg['gravity_m_s2'])
        physx=PhysxSchema.PhysxSceneAPI.Apply(ps.GetPrim())
        physx.CreateTimeStepsPerSecondAttr(cfg['physics_hz'])
        physx.CreateEnableGPUDynamicsAttr(False)
        physx.CreateBroadphaseTypeAttr('MBP')
        physx.CreateSolverTypeAttr('TGS')
        physx.CreateEnableCCDAttr(True)
        material=UsdShade.Material.Define(stage,'/World/Materials/FloorPhysics')
        api=UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        api.CreateStaticFrictionAttr(friction['static_friction'])
        api.CreateDynamicFrictionAttr(friction['dynamic_friction'])
        api.CreateRestitutionAttr(friction['restitution'])
        colliders=[]
        floors=[]
        counts={}
        spawn_blockers=[]
        for prim in meshes:
            kind=category(prim)
            if kind not in cfg['collider_types']: continue
            UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('boundingCube')
            px=PhysxSchema.PhysxCollisionAPI.Apply(prim)
            px.CreateContactOffsetAttr(.005)
            px.CreateRestOffsetAttr(0.0)
            if prim.HasAPI(UsdPhysics.RigidBodyAPI): raise RuntimeError('Unexpected dynamic factory prim')
            bb=bounds(prim)
            colliders.append({'path':str(prim.GetPath()),'category':kind,'bounds_m':bb,'approximation':'boundingCube'})
            counts[kind]=counts.get(kind,0)+1
            if kind=='floor':
                floors.append(prim)
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(material,UsdShade.Tokens.weakerThanDescendants,'physics')
            elif (bb[0][0]<cfg['spawn_half_size_m'] and bb[1][0]>-cfg['spawn_half_size_m']
                  and bb[0][1]<cfg['spawn_half_size_m'] and bb[1][1]>-cfg['spawn_half_size_m']
                  and bb[0][2]<cfg['spawn_clear_height_m'] and bb[1][2]>0):
                spawn_blockers.append(str(prim.GetPath()))
        if len(floors)!=1: raise RuntimeError('Expected exactly one source floor')
        floor_bounds=bounds(floors[0])
        if abs(floor_bounds[1][2])>1e-5: raise RuntimeError('Floor top is not Z=0')
        if spawn_blockers: raise RuntimeError('Spawn blockers: '+str(spawn_blockers))
        for required in ['floor','wall','machine','table_part','shelf_part']:
            if counts.get(required,0)==0: raise RuntimeError('Missing colliders for '+required)
        # Blender watt-based lights are not a calibrated Isaac interior rig.
        # Author explicit USD lights below the roof, preserving source geometry.
        lighting=cfg['lighting']
        dome=UsdLux.DomeLight.Define(stage,'/World/Lighting/Ambient')
        dome.CreateIntensityAttr(lighting['dome_intensity'])
        dome.CreateColorAttr(Gf.Vec3f(.85,.91,1))
        for row,y in enumerate([-3.5,-1,1.5,4]):
            for column,x in enumerate([-2,2]):
                lamp=UsdLux.RectLight.Define(stage,f'/World/Lighting/Ceiling_{row}_{column}')
                lamp.CreateIntensityAttr(lighting['ceiling_intensity'])
                lamp.CreateNormalizeAttr(False)
                lamp.CreateWidthAttr(1.3)
                lamp.CreateHeightAttr(.35)
                lamp.CreateColorAttr(Gf.Vec3f(1,.96,.90))
                # USD light emits toward local -Z, directly down into the hall.
                UsdGeom.Xformable(lamp).AddTranslateOp().Set(Gf.Vec3d(x,y,lighting['ceiling_z_m']))
        stage.GetRootLayer().customLayerData={
            'cameraSettings':{'boundCamera':'/World/OverviewCamera'}}
        camera=UsdGeom.Camera.Define(stage,'/World/OverviewCamera')
        eye=Gf.Vec3d(*cfg['camera_eye_m'])
        target=Gf.Vec3d(*cfg['camera_target_m'])
        view=Gf.Matrix4d().SetLookAt(eye,target,Gf.Vec3d(0,0,1))
        camera.AddTransformOp().Set(view.GetInverse())
        camera.CreateFocalLengthAttr(16)
        camera.CreateHorizontalApertureAttr(24)
        camera.CreateClippingRangeAttr(Gf.Vec2f(.05,100))
        if args.gui:
            from omni.kit.viewport.utility import get_active_viewport
            viewport=get_active_viewport()
            if viewport: viewport.camera_path=str(camera.GetPath())
        report.update({'source_sha256':source_hash,'mesh_count':len(meshes),'collider_count':len(colliders),
                       'collider_categories':counts,'floor_bounds_m':floor_bounds,'gravity_direction':[0,0,-1],
                       'gravity_m_s2':ps.GetGravityMagnitudeAttr().Get(),'material':friction,
                       'spawn_blockers':spawn_blockers,'colliders':colliders})
        report['checks'].update({'reference_loaded':True,'names_and_world_transforms_unchanged':True,
                                'floor_top_z_zero':True,'single_floor_no_added_ground':True,'spawn_clear':True})
        # Save a composition layer (relative reference); do not flatten the factory.
        # Anonymous root uses absolute resolution; exported layer gets a relative arc.
        if output.exists():
            backup=PROJECT_ROOT/'logs/factory_physics_backups'/datetime.now().strftime('%Y%m%d_%H%M%S')
            backup.mkdir(parents=True,exist_ok=True)
            shutil.copy2(output,backup/output.name)
        stage.GetRootLayer().Export(str(output))
        layer=Sdf.Layer.FindOrOpen(str(output))
        spec=layer.GetPrimAtPath('/World/Factory')
        spec.referenceList.prependedItems=[Sdf.Reference('./factory.usd',source_root.GetPath())]
        layer.Save()
        # Reopen the actual saved scene, testing relative reference resolution.
        if not omni.usd.get_context().open_stage(str(output)): raise RuntimeError('Saved stage open failed')
        stage=omni.usd.get_context().get_stage()
        SimulationManager.set_device('cpu')
        SimulationManager.enable_fabric(False)
        settings.set('/physics/suppressReadback',False)
        settings.set('/physics/updateToUsd',True)
        for _ in range(15): app.update()
        if args.gui:
            from omni.kit.viewport.utility import get_active_viewport
            viewport=get_active_viewport()
            if viewport: viewport.camera_path='/World/OverviewCamera'
        if sum(p.IsA(UsdGeom.Mesh) for p in stage.Traverse())!=len(meshes): raise RuntimeError('Saved reference mesh mismatch')
        report['checks']['saved_scene_reopened']=True
        if args.validate:
            # Runtime-only root edits, after saving the clean scene. PhysX writes
            # transforms into the root: session-authored transforms would mask
            # that writeback with a stronger constant value. Never save the cube.
            with Usd.EditContext(stage,stage.GetRootLayer()):
                cube=UsdGeom.Cube.Define(stage,'/World/CollisionTestCube')
                cube.CreateSizeAttr(cfg['test_cube_size_m'])
                cube.AddTranslateOp().Set(Gf.Vec3d(0,0,cfg['test_cube_center_z_m']))
                UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
                UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
                UsdPhysics.MassAPI.Apply(cube.GetPrim()).CreateMassAttr(1)
                PhysxSchema.PhysxRigidBodyAPI.Apply(cube.GetPrim()).CreateEnableCCDAttr(True)
            # Flush USD notices before PhysX parses the new session-layer body.
            for _ in range(3): app.update()
            timeline=omni.timeline.get_timeline_interface()
            timeline.set_start_time(0)
            timeline.set_end_time(100)
            timeline.set_target_framerate(cfg['physics_hz'])
            timeline.set_time_codes_per_second(cfg['physics_hz'])
            timeline.set_looping(False)
            timeline.play()
            report['readback_settings']={k:settings.get(k) for k in ['/physics/updateToUsd','/physics/suppressReadback']}
            dt=1/cfg['physics_hz']
            samples=[]
            for i in range(cfg['test_steps']):
                app.update()
                xcache=UsdGeom.XformCache()
                z=float(xcache.GetLocalToWorldTransform(cube.GetPrim()).ExtractTranslation()[2])
                samples.append(z)
            sim_time=timeline.get_current_time()
            timeline.pause()
            expected=cfg['test_cube_size_m']/2
            bottom=[z-expected for z in samples]
            test={'steps':len(samples),'duration_s':sim_time,'start_center_z_m':cfg['test_cube_center_z_m'],
                  'final_center_z_m':samples[-1],'expected_rest_center_z_m':expected,'minimum_bottom_z_m':min(bottom),
                  'last_60_range_m':max(samples[-60:])-min(samples[-60:]),'center_z_samples_m':samples}
            fell=min(samples)<cfg['test_cube_center_z_m']-.2
            resting=abs(samples[-1]-expected)<.012 and test['last_60_range_m']<.003
            no_tunnel=min(bottom)>-.025
            report['drop_test']=test
            report['checks'].update({'gravity_moved_cube':fell,'cube_resting_on_floor':resting,'no_floor_tunneling':no_tunnel})
            if not all([fell,resting,no_tunnel]): raise RuntimeError('Drop test failed: '+str({k:v for k,v in test.items() if k!='center_z_samples_m'}))
            with Usd.EditContext(stage,stage.GetRootLayer()): stage.RemovePrim('/World/CollisionTestCube')
        if hashlib.sha256(source.read_bytes()).hexdigest()!=source_hash: raise RuntimeError('Source modified')
        report['checks']['source_unchanged']=True
        report['status']='passed' if args.validate else 'scene_loaded_not_drop_tested'
        report_path.write_text(json.dumps(report,indent=2),encoding='utf-8')
        print('FACTORY_PHYSICS_RESULT '+json.dumps({k:v for k,v in report.items() if k not in ['colliders','drop_test']}),flush=True)
        if args.gui:
            if args.frames:
                for _ in range(args.frames): app.update()
            else:
                while app.is_running(): app.update()
    except BaseException:
        report['status']='failed'
        report['error']=traceback.format_exc()
        print(report['error'],flush=True)
        report_path.write_text(json.dumps(report,indent=2),encoding='utf-8')
        raise
    finally:
        app.close()

if __name__=='__main__': main()
