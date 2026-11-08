"""Real USD/PhysX test fixtures: crossing kinematic object/human and blocked lane.

Fixture transforms move obstacle actors only. G1 always uses locomotion joint targets.
"""
from pathlib import Path
import argparse,sys,json,math
ROOT=Path(r'D:\unitree_g1'); sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--gui',action='store_true'); args,_=parser.parse_known_args()
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':not args.gui,'renderer':'RaytracedLighting','anti_aliasing':1,'multi_gpu':False,'extra_args':[
        '--/app/vulkan=false','--/exts/isaacsim.physics.newton/auto_switch_on_startup=false',
        '--/exts/isaacsim.core.simulation_manager/default_engine=physx']})
    runtime=None; results={}
    def add(name,passed,**values):
        results[name]={'passed':bool(passed),**values}
        (ROOT/'docs/navigation/obstacle_test_results.json').write_text(json.dumps(results,indent=2))
        print('OBSTACLE_TEST '+json.dumps({name:results[name]}),flush=True)
    try:
        from controllers.factory_g1_runtime import FactoryG1Runtime
        from navigation.local_planner import pose
        from navigation.lane_map import box_distance
        from pxr import UsdGeom,UsdPhysics,Gf,Sdf
        cfg=json.loads((ROOT/'configs/factory_g1.json').read_text()); cfg['head_camera']['enabled']=bool(args.gui)
        runtime=FactoryG1Runtime(app,cfg,self_test=True); nav=runtime.navigation
        def advance(seconds,move=None):
            out=[]
            for i in range(round(seconds*50)):
                if move: move(i/50)
                app.update(); runtime.tick(); out.append(nav.snapshot())
                if runtime.fault: raise RuntimeError(runtime.fault)
            return out
        def reset():
            if nav.scheduler.state=='FAULT': nav.acknowledge_fault()
            nav.set_mode('MANUAL'); runtime.timeline.pause(); app.update(); runtime.safe_reset('obstacle_test_reset')
            runtime.timeline.play(); app.update(); runtime.keys.set_running(True); advance(1)
            nav.a=list(pose(runtime.last_state)); nav.b=[0.,-2.5,math.pi/2]
            nav.scheduler.config.update(start_delay_seconds=0,dwell_at_b_seconds=.5,repeat_enabled=False,max_cycles=1)
            nav.blocked_since=None
        def actor(name,position,size,kind='static',kinematic=False):
            path='/World/PatrolTest'+name
            cube=UsdGeom.Cube.Define(runtime.stage,path); cube.CreateSizeAttr(1.)
            translate=cube.AddTranslateOp(); translate.Set(Gf.Vec3d(*position)); cube.AddScaleOp().Set(Gf.Vec3d(*size))
            cube.CreateDisplayColorAttr([Gf.Vec3f(1,.2,.1) if kind=='human' else Gf.Vec3f(1,.5,0)])
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim()); cube.GetPrim().CreateAttribute('navigation:kind',Sdf.ValueTypeNames.String).Set(kind)
            if kinematic:
                body=UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim()); body.CreateKinematicEnabledAttr(True)
            return path,translate
        advance(1)
        for kind in ['dynamic','human']:
            reset(); nav.start(); advance(1)
            actual_speed=math.hypot(*runtime.last_state['base_linear_velocity_world_m_s'][:2])
            initial_separation=nav.grid.map.radius+.2+nav.safety.stop_distance(actual_speed)+.3
            y=runtime.last_state['base_position_m'][1]+initial_separation
            path,move=actor('Crossing'+kind,[-1.8,y,.8],[.4,.4,1.6],kind,True)
            states=[]; clearances=[]; positions=[]
            def crossing(t): move.Set(Gf.Vec3d(-1.8+.65*t,y,.8))
            for i in range(7*50):
                crossing(i/50); app.update(); runtime.tick()
                s=nav.snapshot(); states.append(s); positions.append(runtime.last_state['base_position_m'])
                obj=next((o for o in nav.detector.obstacles if o['id']==path),None)
                if obj: clearances.append(box_distance(s['position'][:2],obj['bounds'])-nav.grid.map.radius)
                if runtime.fault: break
            runtime.stage.RemovePrim(path); continuation=advance(3)
            waits=[s for s in states if s['state']=='WAITING_FOR_CLEAR_PATH']
            add(kind+'_crossing',bool(waits) and all(s['command']==[0,0,0] for s in waits) and min(clearances,default=-1)>0 and nav.scheduler.state!='FAULT',
                waits=len(waits),minimum_inflated_clearance_m=min(clearances,default=None),
                final_state=nav.scheduler.state,reason=nav.reason,source='Actual kinematic collision cube; human flag is scene metadata, not AI',
                resumed_after_clear=any(s['state'].startswith('MOVING') for s in continuation),run_directory=str(nav.directory))
        reset(); nav.start(); advance(1)
        y=runtime.last_state['base_position_m'][1]+1.0
        path,_=actor('FullLaneBlock',[0,y,.65],[2.2,.35,1.3])
        rows=advance(4)
        add('fully_blocked_lane',any(s['state']=='WAITING_FOR_CLEAR_PATH' for s in rows) and nav.last_command==[0,0,0],state=nav.scheduler.state,reason=nav.reason)
        runtime.stage.RemovePrim(path); resumed=advance(3)
        add('clear_path_replan',any(s['state'].startswith('MOVING') for s in resumed),states=sorted(set(s['state'] for s in resumed)))
        # Explicit stale-data fault injection, not a fake policy or a claimed sensor.
        nav.detector.timestamp=nav.sim_time-10; nav.last_scan=nav.sim_time
        advance(.02)
        add('stale_data_fault',nav.scheduler.state=='FAULT' and nav.last_command==[0,0,0],reason=nav.reason)
        advance(.8)
        add('fault_latched',nav.scheduler.state=='FAULT',state=nav.scheduler.state)
        nav.acknowledge_fault()
        add('fault_acknowledge',nav.mode=='MANUAL' and nav.scheduler.state=='STOPPED',mode=nav.mode)
        runtime.patrol_panel.top_view(); runtime.patrol_panel.draw()
        runtime.context['capture'](nav.directory/'navigation_top_view.png')
    finally:
        if runtime: runtime.close()
        app.close()


if __name__=='__main__': main()
