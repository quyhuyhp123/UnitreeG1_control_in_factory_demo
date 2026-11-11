"""Real runtime reset plus explicit software-fault injection (not a fall test)."""
from pathlib import Path
import sys,json,math
ROOT=Path(r'D:\unitree_g1'); sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'renderer':'RaytracedLighting','anti_aliasing':1,'multi_gpu':False,'extra_args':['--/app/vulkan=false','--/exts/isaacsim.physics.newton/auto_switch_on_startup=false','--/exts/isaacsim.core.simulation_manager/default_engine=physx']})
rt=None; report={'checks':{},'fault_scope':'Software runtime fault injection; no actual falling robot claimed'}
try:
    from controllers.factory_selector import load_factory
    from controllers.factory_g1_runtime import FactoryG1Runtime
    cfg=load_factory('v2'); cfg['head_camera']['enabled']=False
    rt=FactoryG1Runtime(app,cfg,self_test=True); nav=rt.navigation
    for _ in range(100): app.update(); rt.tick()
    nav.scheduler.config['start_delay_seconds']=.2
    nav.start()
    start=list(rt.last_state['base_position_m'])
    for _ in range(500):
        app.update(); rt.tick()
        if math.dist(start[:2],rt.last_state['base_position_m'][:2])>.3: break
    report['checks']['walking_before_blocker']=math.dist(start[:2],rt.last_state['base_position_m'][:2])>.3
    from pxr import UsdGeom,UsdPhysics,Gf
    cube=UsdGeom.Cube.Define(rt.stage,'/World/LiveRouteBlocker'); cube.CreateSizeAttr(1)
    target=nav.route_manager.data['nodes']['ENTRY_INNER']
    cube.AddTranslateOp().Set(Gf.Vec3d(target[0],target[1],.75))
    cube.AddScaleOp().Set(Gf.Vec3f(.3,2,1.5)); UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    for _ in range(500):
        app.update(); rt.tick()
        if nav.scheduler.state in ('WAITING_FOR_CLEAR_PATH','FAULT'): break
    report['checks']['live_blocker_stops_same_route']=nav.scheduler.state=='WAITING_FOR_CLEAR_PATH' and nav.last_command==[0,0,0] and nav.active_route=='route_1_blue' and nav.scheduler.completed==0
    report['blocked_state']=nav.scheduler.state; report['blocked_command']=list(nav.last_command); report['blocked_reason']=nav.reason
    report['blocked_position']=list(rt.last_state['base_position_m'])
    rt.stage.RemovePrim('/World/LiveRouteBlocker'); nav.refresh_obstacles(force=True)
    for _ in range(200):
        app.update(); rt.tick()
        if nav.scheduler.state in ('MOVING_SEGMENT','APPROACHING_TURN','TURNING') and any(abs(x)>.05 for x in nav.last_command): break
    report['checks']['cleared_blocker_resumes_same_route']=nav.scheduler.state in ('MOVING_SEGMENT','APPROACHING_TURN','TURNING') and nav.active_route=='route_1_blue' and not rt.fault
    rt.fault='INJECTED_TEST_RUNTIME_FAULT'; app.update(); rt.tick()
    report['checks']['fault_cancels_auto']=nav.scheduler.state=='FAULT' and nav.last_command==[0,0,0] and nav.scheduler.completed==0
    nav.stop(); report['checks']['stop_cannot_clear_fault']=nav.scheduler.state=='FAULT'
    rt.timeline.pause(); app.update(); rt.inject('R'); rt.tick()
    reset=rt.reset_events[-1]
    report['checks']['reset_at_entry']=math.dist(reset['position_after_reset'][:2],cfg['robot_spawn']['position_from_marker_m'][:2])<.02
    report['checks']['policy_history_cleared']=reset['history_terms_after_reset']==0 and reset['policy_steps_after_reset']==0
    report['checks']['reset_cancels_route']=nav.mode=='MANUAL' and nav.edge is None and nav.active_route is None and nav.scheduler.completed==0
    nav.acknowledge_fault(); report['checks']['ack_does_not_restart']=nav.scheduler.state=='STOPPED'
    report.update(status='passed' if all(report['checks'].values()) else 'failed',run_directory=str(rt.directory),reset=reset)
    print('ROUTE_RESET_RESULT '+json.dumps({k:v for k,v in report.items() if k!='reset'}),flush=True)
finally:
    (ROOT/'docs/navigation/multi_route_reset_results.json').write_text(json.dumps(report,indent=2))
    if rt: rt.close()
    app.close()
if report.get('status')!='passed': raise SystemExit(1)
