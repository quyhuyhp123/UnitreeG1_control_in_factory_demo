"""Actual G1 PhysX patrol tests plus measured braking calibration; no robot pose writes."""
from pathlib import Path
import sys,json,math,time,argparse,statistics
ROOT=Path(r'D:\unitree_g1'); sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--gui',action='store_true'); parser.add_argument('--calibrate-only',action='store_true')
    args,_=parser.parse_known_args()
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':not args.gui,'renderer':'RaytracedLighting','anti_aliasing':1,'multi_gpu':False,
        'extra_args':['--/app/vulkan=false','--/exts/isaacsim.physics.newton/auto_switch_on_startup=false','--/exts/isaacsim.core.simulation_manager/default_engine=physx']})
    runtime=None; report={'tests':{},'input_source':'Carb logical device and programmatic UI; no physical OS keyboard'}
    def save():
        name='braking_test_results.json' if args.calibrate_only else 'physics_test_results.json'
        (ROOT/'docs/navigation'/name).write_text(json.dumps(report,indent=2),encoding='utf-8')
    def add(key,passed,**details):
        report['tests'][key]={'passed':bool(passed),**details}; save(); print('PATROL_TEST '+json.dumps({key:report['tests'][key]}),flush=True)
    try:
        from controllers.factory_g1_runtime import FactoryG1Runtime
        from navigation.local_planner import pose
        cfg=json.loads((ROOT/'configs/factory_g1.json').read_text())
        if not args.gui: cfg['head_camera']['enabled']=False
        runtime=FactoryG1Runtime(app,cfg,self_test=True); nav=runtime.navigation
        from pxr import UsdGeom,Usd
        bounds=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render']).ComputeWorldBound(runtime.stage.GetPrimAtPath('/World/G1')).ComputeAlignedRange()
        report['robot_bounds_world']=[list(bounds.GetMin()),list(bounds.GetMax())]
        report['head_preview_enabled']=args.gui
        report['run_directory']=str(runtime.directory); report['patrol_directory']=str(nav.directory)
        def advance(seconds,callback=None):
            rows=[]
            for i in range(round(seconds*50)):
                if callback: callback(i/50)
                app.update(); row=runtime.tick()
                if row: rows.append(row)
                if runtime.fault: raise RuntimeError(runtime.fault)
            return rows
        def reset():
            nav.set_mode('MANUAL'); runtime.timeline.pause(); app.update(); runtime.safe_reset('patrol_test_between_cases')
            runtime.timeline.play(); app.update(); runtime.keys.set_running(True); advance(1)
        advance(2)
        add('map_spawn',nav.grid.point_free(pose(runtime.last_state)[:2]),floor=nav.detector.floor,
            colliders=len(nav.detector.obstacles),human_collider_parts=sum(o['kind']=='human' for o in nav.detector.obstacles),a=nav.a,b=nav.b)
        # Calibrate actual target-zero braking from straight walking, no arbitrary deceleration assertion.
        runtime.inject('W'); walking=advance(4); runtime.inject('W','RELEASE')
        start=pose(runtime.last_state); v0=math.hypot(*runtime.last_state['base_linear_velocity_world_m_s'][:2])
        runtime.inject('SPACE'); runtime.inject('SPACE','RELEASE'); stopped=advance(3)
        distances=[math.dist(start[:2],r['base_position_m'][:2]) for r in stopped]
        distance=max(distances); settled=all(math.hypot(*r['base_linear_velocity_world_m_s'][:2])<.1 for r in stopped[-25:])
        calibration={'validated':settled and v0>.15 and not runtime.fault,'initial_speed_m_s':v0,
            'max_base_displacement_after_zero_m':distance,'sample_seconds':3,'control_hz':50,
            'conservative_deceleration_m_s2':min(1.,v0*v0/(4*(distance+.05))),
            'source':'Measured real PhysX G1 base displacement after command zero; factor-two distance conservatism',
            'run_directory':str(runtime.directory),'date':'2026-11-08','scope':'Straight flat factory floor; not certified, not all-contact worst-case braking'}
        calibration['checkpoint_sha256']=runtime.loco_cfg['checkpoint_sha256']
        calibration['physics_dt']=1/runtime.loco_cfg['physics_hz']
        if not calibration['validated']: raise RuntimeError('Braking calibration failed')
        (ROOT/'configs/navigation_braking.json').write_text(json.dumps(calibration,indent=2))
        nav.braking=calibration; nav.safety.braking=calibration
        add('measured_braking',calibration['validated'],**calibration)
        if args.calibrate_only:
            report['status']='passed'; save(); return
        reset()
        # Both endpoints and connecting path are validated against actual USD before used in test.
        nav.a=list(pose(runtime.last_state)); nav.b=[0.,-2.5,math.pi/2]
        path=nav.validate_points()
        add('route_valid',all(nav.grid.segment_free(a,b) for a,b in zip(path,path[1:])),path=path)
        nav.scheduler.config.update(start_delay_seconds=.2,dwell_at_b_seconds=1.,repeat_interval_seconds=2.,repeat_enabled=True,max_cycles=2)
        nav.start(); trace=[]
        for i in range(160*50):
            app.update(); runtime.tick()
            if i%10==0: trace.append(nav.snapshot())
            if nav.scheduler.state in ('FAULT','STOPPED'): break
        add('two_cycles_a_b_a',nav.scheduler.completed==2 and nav.scheduler.state=='STOPPED' and not runtime.fault,
            completed=nav.scheduler.completed,state=nav.scheduler.state,reason=nav.reason,position=runtime.last_state['base_position_m'],
            transitions=nav.transitions,trajectory_samples=len(trace),fault=runtime.fault)
        (nav.directory/'patrol_trace.json').write_text(json.dumps(trace,indent=2))
        completions=[t['time'] for t in nav.transitions if t['state']=='CYCLE_COMPLETE']
        starts=[t['time'] for t in nav.transitions if t['state']=='PLANNING']
        add('repeat_interval_after_completion',bool(completions) and any(t>=completions[0]+2 for t in starts),completions=completions,starts=starts)
        add('lane_trajectory',all(nav.grid.map.safe(s['position'][:2]) for s in trace),samples=len(trace))
        # Explicit keyboard input during Auto cannot add to selected velocity. SPACE pauses.
        if nav.scheduler.state!='FAULT':
            reset(); nav.a=list(pose(runtime.last_state)); nav.b=[0,-2.5,None]
            nav.scheduler.config.update(start_delay_seconds=0,max_cycles=1); nav.start(); advance(1)
            runtime.inject('D'); advance(.2); runtime.inject('D','RELEASE')
            add('auto_ignores_wasd',nav.selector.mode=='AUTO PATROL' and abs(nav.last_command[2])<.2,auto_command=nav.last_command)
            runtime.inject('SPACE'); advance(.1); runtime.inject('SPACE','RELEASE')
            add('space_pauses_auto',nav.scheduler.state=='PAUSED' and nav.last_command==[0,0,0],state=nav.scheduler.state)
            runtime.timeline.pause(); app.update(); runtime.tick(); old_sim=nav.sim_time
            for _ in range(8): app.update(); runtime.tick()
            add('pause_clock',nav.sim_time==old_sim,clock_before=old_sim,clock_after=nav.sim_time)
            runtime.timeline.play(); app.update(); advance(.2)
            add('no_auto_resume',nav.scheduler.state=='PAUSED',state=nav.scheduler.state)
            nav.set_mode('MANUAL'); runtime.inject('W'); advance(.5); runtime.inject('W','RELEASE'); advance(1)
            add('manual_takeover',nav.mode=='MANUAL' and nav.scheduler.state=='STOPPED',mode=nav.mode)
        nav.set_mode('MANUAL'); nav.scheduler.transition('STOPPED',nav.now)
        test_config=nav.directory/'persistence_test.json'; nav.save(test_config)
        saved_a=list(nav.a); saved_b=list(nav.b); nav.a=None; nav.b=None; nav.load(test_config)
        add('persistence',nav.a==saved_a and nav.b==saved_b,path=str(test_config))
        report['status']='passed' if all(r['passed'] for r in report['tests'].values()) else 'partial_failed_tests'
        runtime.patrol_panel.draw()
        runtime.context['capture'](nav.directory/'patrol_view.png')
        save()
    except BaseException as error:
        report['status']='failed'; report['error']=str(error); save(); raise
    finally:
        if runtime: runtime.close()
        app.close()


if __name__=='__main__': main()
