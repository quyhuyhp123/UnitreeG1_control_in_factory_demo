"""Regression: return leg turns toward A before forward walking, using real PhysX."""
import sys,json,math,argparse
from pathlib import Path
ROOT=Path(r'D:\unitree_g1'); sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--cycles',type=int,default=1)
    parser.add_argument('--speed',type=float,default=.35)
    parser.add_argument('--interval',type=float,default=5.)
    args,_=parser.parse_known_args()
    if args.cycles<1: raise ValueError('cycles must be positive')
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'renderer':'RaytracedLighting','anti_aliasing':1,'multi_gpu':False,
        'extra_args':['--/app/vulkan=false','--/exts/isaacsim.physics.newton/auto_switch_on_startup=false','--/exts/isaacsim.core.simulation_manager/default_engine=physx']})
    runtime=None; report={'status':'running'}
    try:
        from controllers.factory_g1_runtime import FactoryG1Runtime
        from navigation.local_planner import pose,wrap
        cfg=json.loads((ROOT/'configs/factory_g1.json').read_text()); cfg['head_camera']['enabled']=False
        runtime=FactoryG1Runtime(app,cfg,self_test=True); nav=runtime.navigation
        for _ in range(100): app.update(); runtime.tick()
        nav.a=list(pose(runtime.last_state)); nav.b=[0.,-2.5,math.pi/2]
        nav.config['local']['max_speed_m_s']=args.speed
        from navigation.patrol_controller import validate_navigation
        validate_navigation(nav.config)
        nav.validate_points(); nav.scheduler.config.update(start_delay_seconds=.2,dwell_at_b_seconds=1.,repeat_interval_seconds=args.interval,repeat_enabled=True,max_cycles=args.cycles)
        nav.start(); rows=[]
        for i in range(args.cycles*120*50):
            app.update(); runtime.tick()
            if i%5==0: rows.append(nav.snapshot())
            if runtime.fault or nav.scheduler.state in ('FAULT','STOPPED'): break
        returning=[r for r in rows if r['state']=='MOVING_TO_A']
        rotating=[r for r in returning if abs(r['command'][2])>.15 and math.hypot(*r['command'][:2])<.001]
        walking=[r for r in returning if r['command'][0]>.05 and not r['arrival_position_hold']]
        errors=[abs(wrap(math.atan2(nav.a[1]-r['position'][1],nav.a[0]-r['position'][0])-r['heading_rad'])) for r in walking]
        completions=[t['time'] for t in nav.transitions if t['state']=='CYCLE_COMPLETE']
        starts=[t['time'] for t in nav.transitions if t['state']=='PLANNING']
        checks={'cycle_complete':nav.scheduler.completed==args.cycles and nav.scheduler.state=='STOPPED',
                'repeat_after_interval':args.cycles==1 or (bool(completions) and len(starts)==args.cycles and starts[1]>=completions[0]+args.interval),
                'return_has_in_place_turn':len(rotating)>10,
                'return_has_forward_walking':len(walking)>10,
                'no_reverse_travel_command':all(r['command'][0]>=-1e-9 for r in rows if not r['arrival_position_hold']),
                'turn_before_forward':bool(rotating and walking) and rotating[0]['sim_time']<walking[0]['sim_time'],
                'forward_faces_a':bool(errors) and max(errors)<.7,
                'no_runtime_fault':not runtime.fault}
        report.update(status='passed' if all(checks.values()) else 'failed',checks=checks,
            run_directory=str(runtime.directory),patrol_directory=str(nav.directory),
            completed=nav.scheduler.completed,state=nav.scheduler.state,reason=nav.reason,
            configured_speed_m_s=args.speed,max_forward_command_m_s=max((r['command'][0] for r in rows),default=0),
            configured_cycles=args.cycles,repeat_interval_seconds=args.interval,transitions=nav.transitions,
            return_rotation_samples=len(rotating),return_forward_samples=len(walking),
            max_return_heading_error_rad=max(errors,default=None),
            final_a_distance_m=math.dist(pose(runtime.last_state)[:2],nav.a[:2]),fault=runtime.fault)
        (nav.directory/'return_turn_trace.json').write_text(json.dumps(rows,indent=2))
        print('RETURN_TURN_RESULT '+json.dumps(report),flush=True)
    except BaseException as exc:
        report.update(status='failed',error=str(exc)); raise
    finally:
        name='repeat_speed_test_results.json' if args.cycles>1 else 'return_turn_test_results.json'
        (ROOT/'docs/navigation'/name).write_text(json.dumps(report,indent=2))
        if runtime: runtime.close()
        app.close()
    if report['status']!='passed': raise SystemExit(1)

if __name__=='__main__': main()
