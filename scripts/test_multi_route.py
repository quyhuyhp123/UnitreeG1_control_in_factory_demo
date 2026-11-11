"""Real G1/PhysX route sequence acceptance; no pose-based walking."""
from pathlib import Path
import sys,json,argparse,time,statistics
ROOT=Path(r'D:\unitree_g1'); sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
def main():
    p=argparse.ArgumentParser(); p.add_argument('--mode',choices=['route_1_blue','route_2_brown','alternate'],default='alternate'); p.add_argument('--cycles',type=int,default=2); p.add_argument('--max-sim-seconds',type=float,default=1600); args,_=p.parse_known_args()
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'renderer':'RaytracedLighting','anti_aliasing':1,'multi_gpu':False,'extra_args':['--/app/vulkan=false','--/exts/isaacsim.physics.newton/auto_switch_on_startup=false','--/exts/isaacsim.core.simulation_manager/default_engine=physx']})
    rt=None; report={'mode':args.mode,'status':'running','requested_cycles':args.cycles}
    dest=ROOT/'docs/navigation/multi_route_physics_results.json'
    try:
        from controllers.factory_selector import load_factory
        from controllers.factory_g1_runtime import FactoryG1Runtime
        cfg=load_factory('v2'); cfg['head_camera']['enabled']=False # Performance-only test setting; production preview unchanged.
        rt=FactoryG1Runtime(app,cfg,self_test=True); nav=rt.navigation
        report['run_directory']=str(rt.directory); report['patrol_directory']=str(nav.directory)
        for _ in range(100): app.update(); rt.tick()
        nav.select_route(args.mode); nav.validate_route(); nav.scheduler.config.update(start_delay_seconds=.2,repeat_enabled=args.cycles>1,max_cycles=args.cycles,repeat_interval_seconds=2.)
        nav.start(); trace=[]; last_message=-100
        for _ in range(int(args.max_sim_seconds*50)):
            app.update(); rt.tick()
            if rt.steps%10==0: trace.append(nav.snapshot())
            if nav.sim_time-last_message>20:
                last_message=nav.sim_time
                print('ROUTE_PROGRESS '+json.dumps({'sim':nav.sim_time,'state':nav.scheduler.state,'segment':nav.edge['id'] if nav.edge else None,'cycles':nav.scheduler.completed,'reason':nav.reason,'position':rt.last_state['base_position_m']}),flush=True)
                report.update(state=nav.scheduler.state,completed_cycles=nav.scheduler.completed,reason=nav.reason); dest.write_text(json.dumps(report,indent=2))
            if rt.fault or nav.scheduler.state in ('FAULT','STOPPED'): break
        report.update(status='passed' if nav.scheduler.completed==args.cycles and nav.scheduler.state=='STOPPED' and not rt.fault else 'failed',state=nav.scheduler.state,reason=nav.reason,fault=rt.fault,completed_cycles=nav.scheduler.completed,route_history=nav.route_complete_history,transitions=nav.transitions,simulation_seconds=nav.sim_time)
        report['metrics']={'mean_fps':statistics.mean(r['loop_fps'] for r in rt.gpu_samples) if rt.gpu_samples else None,'peak_whole_gpu_mib':max((r['gpu_device_total_used_mib'] for r in rt.gpu_samples),default=None),'planner_latency_ms_last':nav.planner_latency_ms}
        (nav.directory/'multi_route_trace.json').write_text(json.dumps(trace,indent=2))
        dest.write_text(json.dumps(report,indent=2)); print('MULTI_ROUTE_RESULT '+json.dumps({k:v for k,v in report.items() if k not in ['transitions']}),flush=True)
    except BaseException as ex:
        report.update(status='failed',error=repr(ex)); dest.write_text(json.dumps(report,indent=2)); raise
    finally:
        if rt: rt.close()
        app.close()
    if report['status']!='passed': raise SystemExit(1)
if __name__=='__main__': main()
