"""GUI callbacks, route persistence, blocked branch and existing input contracts."""
from pathlib import Path
import sys,json,argparse
ROOT=Path(r'D:\unitree_g1'); sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--output-name',default='multi_route_ui_results.json'); args,_=parser.parse_known_args()
    if Path(args.output_name).name!=args.output_name: raise ValueError('Output must be a filename only')
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':False,'renderer':'RaytracedLighting','anti_aliasing':1,'multi_gpu':False,'extra_args':['--/app/vulkan=false','--/exts/isaacsim.physics.newton/auto_switch_on_startup=false','--/exts/isaacsim.core.simulation_manager/default_engine=physx']})
    runtime=None; report={'status':'running','checks':{}}
    try:
        from controllers.factory_selector import load_factory
        from controllers.factory_g1_runtime import FactoryG1Runtime
        from navigation.global_planner import NoValidPath
        from navigation.local_planner import pose,wrap
        import math
        from pxr import UsdGeom,UsdPhysics,Gf
        runtime=FactoryG1Runtime(app,load_factory('v2'),self_test=True); nav=runtime.navigation; panel=runtime.patrol_panel
        def advance(seconds):
            for _ in range(round(seconds*50)): app.update(); runtime.tick()
        advance(2)
        initial_speed=panel.speed.model.as_float
        report['checks']['verified_default_speed']=abs(initial_speed-nav.speed_profile['default_m_s'])<1e-5
        panel.speed.model.set_value(nav.speed_limit+.01)
        try: panel.apply_schedule(); rejected=False
        except ValueError: rejected=True
        report['checks']['reject_above_verified_speed']=rejected
        panel.speed.model.set_value(initial_speed); panel.apply_schedule()
        panel.route_choice.model.get_item_value_model().set_value(2); panel.apply_route(); panel.preview_route()
        report['checks']['alternate_selection_and_preview']=nav.route_mode=='alternate' and all(v['status']=='VALID' for v in nav.route_manager.status.values())
        path=runtime.directory/'route_settings_roundtrip.json'; nav.save(path)
        nav.select_route('route_2_brown'); nav.load(path); panel.sync()
        report['checks']['save_load_alternate']=nav.route_mode=='alternate' and panel.route_choice.model.get_item_value_model().as_int==2
        panel.start(); runtime.inject('W'); advance(.3); runtime.inject('W','RELEASE')
        report['checks']['wasd_not_added_to_auto']=nav.last_command==[0,0,0] and nav.scheduler.state=='WAITING_START'
        runtime.inject('X'); advance(.1); runtime.inject('X','RELEASE')
        report['checks']['x_pauses']=nav.scheduler.state=='PAUSED'
        generation=runtime.keys.stop_generation; runtime.inject('SPACE'); advance(.1); runtime.inject('SPACE','RELEASE')
        report['checks']['space_only_follows']=runtime.factory_cfg['camera']['mode']=='follow' and runtime.keys.stop_generation==generation and nav.scheduler.state=='PAUSED'
        nav.resume(); report['checks']['resume_explicit']=nav.scheduler.state=='WAITING_START'
        nav.stop(); nav.set_mode('MANUAL'); nav.select_route('route_1_blue')
        cube=UsdGeom.Cube.Define(runtime.stage,'/World/RouteTestBlocker'); cube.CreateSizeAttr(1)
        cube.AddTranslateOp().Set(Gf.Vec3d(0,0,.75)); cube.AddScaleOp().Set(Gf.Vec3f(2,1,1.5)); UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        advance(.1)
        try: nav.validate_route(); blocked=False
        except NoValidPath: blocked=True
        try: nav.start(); rejected=False
        except NoValidPath: rejected=True
        report['checks']['blocked_route1_no_switch']=blocked and rejected and nav.route_mode=='route_1_blue' and nav.scheduler.completed==0
        report['checks']['route2_still_valid']=nav.route_manager.validate(nav.grid,'route_2_brown')['route_id']=='route_2_brown'
        runtime.stage.RemovePrim('/World/RouteTestBlocker'); nav.refresh_obstacles(force=True); nav.validate_route()
        report['checks']['route1_valid_after_clear']=nav.route_manager.status['route_1_blue']['status']=='VALID'
        manual={}
        for key,axis,sign in [('W',0,1),('UP',0,1),('S',0,-1),('DOWN',0,-1),('A',2,1),('LEFT',2,1),('D',2,-1),('RIGHT',2,-1),('Q',1,1),('E',1,-1)]:
            before=pose(runtime.last_state); runtime.inject(key); advance(1.); after=pose(runtime.last_state)
            command=list(runtime.keys.command); runtime.inject(key,'RELEASE'); advance(.6)
            dx,dy=after[0]-before[0],after[1]-before[1]
            value=wrap(after[2]-before[2]) if axis==2 else (math.cos(before[2])*dx+math.sin(before[2])*dy if axis==0 else -math.sin(before[2])*dx+math.cos(before[2])*dy)
            manual[key]={'passed':sign*command[axis]>0 and sign*value>.02 and not runtime.fault,'measured_body_delta_or_yaw':value,'command':command}
        report['manual_keys']=manual; report['checks']['manual_regression']=all(v['passed'] for v in manual.values())
        panel.top_view(); advance(.5); runtime.context['capture'](runtime.directory/'multi_route_top.png')
        report.update(status='passed' if all(report['checks'].values()) else 'failed',run_directory=str(runtime.directory),scope='Actual GUI app and callbacks, Carb logical keys; not native OS mouse clicks. Obstacle test blocks start; not moving-human avoidance.')
        print('MULTI_ROUTE_UI '+json.dumps(report),flush=True)
    except BaseException as exc: report.update(status='failed',error=repr(exc)); raise
    finally:
        (ROOT/'docs/navigation'/args.output_name).write_text(json.dumps(report,indent=2))
        if runtime: runtime.close()
        app.close()
    if report['status']!='passed': raise SystemExit(1)
if __name__=='__main__': main()
