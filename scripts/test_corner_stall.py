"""Real PhysX reproduction of the observed ENTRY_INNER deadband stall."""
from pathlib import Path
import sys,json,math,argparse
ROOT=Path(r'D:\unitree_g1'); sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
parser=argparse.ArgumentParser(); parser.add_argument('--case',choices=['entry','bottom'],default='entry'); args,_=parser.parse_known_args()
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'renderer':'RaytracedLighting','multi_gpu':False,'extra_args':['--/app/vulkan=false','--/exts/isaacsim.physics.newton/auto_switch_on_startup=false','--/exts/isaacsim.core.simulation_manager/default_engine=physx']})
rt=None; result={'scope':'Independent near-corner initial pose from failed live run; not a complete route'}
try:
    from controllers.factory_selector import load_factory
    from controllers.factory_g1_runtime import FactoryG1Runtime
    cfg=load_factory('v2'); cfg['head_camera']['enabled']=False
    point,yaw,index=([-7.1524,-12.9859,0.],.247,0) if args.case=='entry' else ([-6.8919,-11.3909,0.],-3.0712,7)
    cfg['robot_spawn']['position_override_m']=point; cfg['robot_spawn']['heading_deg']=math.degrees(yaw)
    rt=FactoryG1Runtime(app,cfg,self_test=True); nav=rt.navigation
    for _ in range(100): app.update(); rt.tick()
    nav.active_route='route_1_blue'; nav.route_plan=nav.route_manager.validate(nav.grid,nav.active_route)
    nav.segment_index=index; nav.selector.select('AUTO PATROL'); nav.config['local']['max_speed_m_s']=.8; nav.begin_edge(rt.last_state)
    trace=[]
    for i in range(35*50):
        if i%5==0: app.update()
        rt.tick()
        if i%10==0: trace.append(nav.snapshot())
        if nav.segment_index>index or nav.scheduler.state=='FAULT' or rt.fault: break
    result.update(status='passed' if nav.segment_index==index+1 and not rt.fault else 'failed',case=args.case,segment_index=nav.segment_index,state=nav.scheduler.state,reason=nav.reason,
                  position=rt.last_state['base_position_m'],completed_cycles=nav.scheduler.completed,run_directory=str(rt.directory))
    (nav.directory/'corner_stall_trace.json').write_text(json.dumps(trace))
    print('CORNER_STALL '+json.dumps(result),flush=True)
finally:
    name='corner_stall_fix_results.json' if args.case=='entry' else 'bottom_corner_fix_results.json'
    (ROOT/'docs/navigation'/name).write_text(json.dumps(result,indent=2))
    if rt: rt.close()
    app.close()
