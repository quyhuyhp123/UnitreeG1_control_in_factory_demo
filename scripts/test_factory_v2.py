"""Actual PhysX/G1 acceptance for selected factory; no walking pose updates."""
from pathlib import Path
import sys,json,math,time,argparse,statistics
ROOT=Path(r'D:\unitree_g1'); sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--factory',choices=['v1','v2'],default='v2'); parser.add_argument('--views-only',action='store_true'); args,_=parser.parse_known_args()
    from controllers.factory_selector import load_factory
    from run_factory_g1 import preflight
    cfg=load_factory(args.factory); preflight(cfg,False)
    cfg['multi_route_enabled']=False # This regression covers legacy A/B protection.
    from isaacsim import SimulationApp
    start=time.perf_counter(); report={'factory':args.factory,'checks':{},'status':'running'}; runtime=None
    app=SimulationApp({'headless':True,'width':960,'height':640,'renderer':'RaytracedLighting','anti_aliasing':1,'multi_gpu':False,'extra_args':['--/app/vulkan=false','--/exts/isaacsim.physics.newton/auto_switch_on_startup=false','--/exts/isaacsim.core.simulation_manager/default_engine=physx']})
    dest=ROOT/(f'docs/factory_{args.factory}_views.json' if args.views_only else f'docs/factory_{args.factory}_acceptance.json')
    def save(): dest.write_text(json.dumps(report,indent=2),encoding='utf-8')
    def check(name,passed,**values):
        report['checks'][name]={'passed':bool(passed),**values}; save(); print('V2_CHECK '+json.dumps({name:report['checks'][name]}),flush=True)
    try:
        from controllers.factory_g1_runtime import FactoryG1Runtime
        from navigation.local_planner import pose,wrap
        from pxr import Usd,UsdGeom,UsdPhysics,Gf
        runtime=FactoryG1Runtime(app,cfg,self_test=True)
        report.update(startup_seconds=time.perf_counter()-start,run_directory=str(runtime.directory),scene=runtime.scene_checks)
        def advance(seconds):
            rows=[]
            for _ in range(round(seconds*50)):
                app.update(); r=runtime.tick()
                if r: rows.append(r)
                if runtime.fault: raise RuntimeError(runtime.fault)
            return rows
        def release():
            for k in runtime.keys.mapping: runtime.inject(k,'RELEASE')
        def reset():
            release(); runtime.timeline.pause(); app.update(); runtime.safe_reset('factory_acceptance_case')
            runtime.timeline.play(); app.update(); runtime.keys.set_running(True); advance(.6)
        check('factory_reference',runtime.scene_checks['factory_reference_loaded'] and runtime.scene_checks['source_names_transforms_preserved'],selected_usd=cfg['factory_usd'])
        colliders=[p for p in runtime.stage.Traverse() if str(p.GetPath()).startswith('/World/FactoryScene') and p.HasAPI(UsdPhysics.CollisionAPI)]
        check('collider_count',len(colliders)==42 if args.factory=='v2' else len(colliders)>0,count=len(colliders))
        initial=pose(runtime.last_state)
        check('spawn_heading',math.dist(initial[:2],runtime.spawn[:2])<.025 and abs(wrap(initial[2]-math.radians(cfg['robot_spawn']['heading_deg'])))<.025,pose=list(initial),root_z=runtime.last_state['base_position_m'][2])
        rows=advance(3)
        check('standing',max(r['tilt_deg'] for r in rows)<15 and runtime.contact_counts.get('floor',0)>0,max_tilt_deg=max(r['tilt_deg'] for r in rows),floor_contacts=runtime.contact_counts.get('floor',0))
        nav=runtime.navigation
        check('navigation_new_spawn',nav.b is None and nav.mode=='MANUAL' and nav.grid.point_free(runtime.spawn[:2]),a=nav.a,b=nav.b)
        try: nav.start(); rejected=False
        except ValueError: rejected=True
        check('auto_unset_b_rejected',rejected)
        if args.views_only:
            advance(3)
            runtime.patrol_panel.top_view(); advance(.5); runtime.context['capture'](runtime.directory/'factory_top.png')
            from pxr import UsdGeom,Gf
            UsdGeom.Camera(runtime.stage.GetPrimAtPath('/World/Navigation/TopCamera')).CreateClippingRangeAttr(Gf.Vec2f(.1,100))
            advance(.5); runtime.context['capture'](runtime.directory/'factory_top_unclipped.png')
            runtime.set_camera_mode('overview'); advance(.5); runtime.context['capture'](runtime.directory/'factory_overview.png')
            report.update(status='passed',scope='Camera views rendered; visual inspection required'); save(); return
        if args.factory=='v2':
            check('islands_forbidden',not nav.grid.map.safe((-3.5,0)) and not nav.grid.map.safe((4,0)))
            for i,(label,position,expected) in enumerate([
                ('floor',[runtime.spawn[0]+1,runtime.spawn[1],1.4],.1),
                ('door',[-9.85,-13.75,1.2],.1),
                ('machine',[-8.95,8.83,3.5],2.43)]):
                cube=UsdGeom.Cube.Define(runtime.stage,'/World/CollisionProbe'); cube.CreateSizeAttr(.2); cube.AddTranslateOp().Set(Gf.Vec3d(*position))
                UsdPhysics.CollisionAPI.Apply(cube.GetPrim()); UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim()); UsdPhysics.MassAPI.Apply(cube.GetPrim()).CreateMassAttr(.05)
                advance(2.5); z=UsdGeom.XformCache().GetLocalToWorldTransform(cube.GetPrim()).ExtractTranslation()[2]
                check('drop_'+label,abs(z-expected)<.09,settled_z=z,expected_z=expected)
                runtime.stage.RemovePrim('/World/CollisionProbe')
        for key,seconds,kind,sign in [('W',3.,'linear',1),('UP',2.,'linear',1),('S',1.,'linear',-1),('DOWN',1.,'linear',-1),('A',2.5,'yaw',1),('LEFT',2.5,'yaw',1),('D',2.5,'yaw',-1),('RIGHT',2.5,'yaw',-1),('Q',1.,'lateral',1),('E',1.,'lateral',-1)]:
            reset(); before=pose(runtime.last_state); runtime.inject(key); rows=advance(seconds); after=pose(runtime.last_state)
            release(); advance(1)
            dx,dy=after[0]-before[0],after[1]-before[1]
            value=wrap(after[2]-before[2]) if kind=='yaw' else (math.cos(before[2])*dx+math.sin(before[2])*dy if kind=='linear' else -math.sin(before[2])*dx+math.cos(before[2])*dy)
            check('key_'+key,sign*value>(.12 if kind=='yaw' else .04),value=value,max_tilt_deg=max(r['tilt_deg'] for r in rows),world_delta=[dx,dy])
        reset(); runtime.inject('W'); advance(.5); stop_before=runtime.keys.stop_generation
        runtime.inject('SPACE'); advance(.04)
        check('space_binding',runtime.keys.command[0]>.05 and runtime.keys.stop_generation==stop_before and runtime.factory_cfg['camera']['mode']=='follow',mapping=runtime.keys.mapping['SPACE'])
        release(); advance(1)
        runtime.timeline.pause(); app.update(); runtime.inject('R'); runtime.tick()
        r=runtime.reset_events[-1]
        check('reset_new_spawn',math.dist(r['position_after_reset'][:2],runtime.spawn[:2])<.02 and r['history_terms_after_reset']==0 and r['policy_steps_after_reset']==0,reset=r)
        runtime.timeline.play(); app.update(); advance(1)
        runtime.set_camera_mode('overview'); runtime.context['capture'](runtime.directory/'factory_overview.png')
        runtime.patrol_panel.top_view(); advance(.4); runtime.context['capture'](runtime.directory/'factory_top.png')
        runtime.set_camera_mode('follow'); runtime.set_camera_mode('free')
        check('camera_modes',runtime.factory_cfg['camera']['mode']=='free')
        perf=runtime.gpu_samples
        report['performance']={'mean_fps':statistics.mean(r['loop_fps'] for r in perf) if perf else None,'mean_physics_steps_per_wall_second':statistics.mean(r['physics_steps_per_wall_second'] for r in perf) if perf else None,'peak_whole_gpu_mib':max((r['gpu_device_total_used_mib'] for r in perf),default=None),'cpu_ram':'sampled externally if available'}
        report['status']='passed' if all(r['passed'] for r in report['checks'].values()) else 'failed_checks'; save()
        print('FACTORY_V2_RESULT '+json.dumps({'status':report['status'],'failed':[k for k,v in report['checks'].items() if not v['passed']]}),flush=True)
    except BaseException as exc:
        report.update(status='failed',error=repr(exc)); save(); raise
    finally:
        if runtime: runtime.close()
        app.close()
    if report['status']!='passed': raise SystemExit(1)
if __name__=='__main__': main()
