"""Independent initial-pose corner recovery test; NOT full-route completion."""
from pathlib import Path
import sys,json,math
ROOT=Path(r'D:\unitree_g1'); sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'renderer':'RaytracedLighting','anti_aliasing':1,'multi_gpu':False,'extra_args':['--/app/vulkan=false','--/exts/isaacsim.physics.newton/auto_switch_on_startup=false','--/exts/isaacsim.core.simulation_manager/default_engine=physx']})
runtime=None; report={'scope':'Independent initial pose at corner; no prior route traversal claimed'}
try:
    from controllers.factory_selector import load_factory
    from controllers.factory_g1_runtime import FactoryG1Runtime
    from navigation.local_planner import pose
    cfg=load_factory('v2'); cfg['head_camera']['enabled']=False
    cfg['robot_spawn']['position_override_m']=[-6.9639,-13.1367,0]; cfg['robot_spawn']['heading_deg']=-156.15
    runtime=FactoryG1Runtime(app,cfg,self_test=True); nav=runtime.navigation
    for _ in range(100): app.update(); runtime.tick()
    nav.active_route='route_1_blue'; nav.route_plan=nav.route_manager.validate(nav.grid,nav.active_route)
    nav.segment_index=8; nav.selector.select('AUTO PATROL'); nav.begin_edge(runtime.last_state); nav.local.position_reached=True
    initial=math.dist(pose(runtime.last_state)[:2],nav.edge['goal'])
    for _ in range(25*50):
        app.update(); runtime.tick()
        if nav.segment_index>8 or nav.scheduler.state=='FAULT': break
    report.update(status='passed' if nav.segment_index==9 and nav.scheduler.completed==0 else 'failed',initial_distance_m=initial,position=runtime.last_state['base_position_m'],segment_index=nav.segment_index,completed_cycles=nav.scheduler.completed,state=nav.scheduler.state,reason=nav.reason,run_directory=str(runtime.directory))
    print('CORNER_RESULT '+json.dumps(report),flush=True)
finally:
    (ROOT/'docs/navigation/multi_route_corner_results.json').write_text(json.dumps(report,indent=2))
    if runtime: runtime.close()
    app.close()
if report.get('status')!='passed': raise SystemExit(1)
