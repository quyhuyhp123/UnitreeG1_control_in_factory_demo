"""Map switching + route start smoke test. No AI avoidance or training."""
from pathlib import Path
import sys,json,time,math
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]

def main():
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':False,'width':960,'height':640,'renderer':'RaytracedLighting','multi_gpu':False,
        'extra_args':['--/app/vulkan=false','--/exts/isaacsim.physics.newton/auto_switch_on_startup=false',
                      '--/exts/isaacsim.core.simulation_manager/default_engine=physx']})
    runtime=None
    try:
        from controllers.factory_selector import load_factory
        from controllers.factory_g1_runtime import FactoryG1Runtime
        from pxr import UsdPhysics,UsdGeom
        from navigation.safety_supervisor import SafetySupervisor
        cfg=load_factory('v2');cfg['map_id']=1
        runtime=FactoryG1Runtime(app,cfg,self_test=True);nav=runtime.navigation
        rows=[]
        def advance(n):
            for _ in range(n):
                app.update();runtime.tick()
                if runtime.fault:raise RuntimeError(runtime.fault)
        def reset():
            nav.stop();runtime.timeline.pause();app.update()
            if not runtime.safe_reset('Map evaluation'):raise RuntimeError('Reset rejected')
            runtime.timeline.play();advance(20)
        advance(40)
        for map_id in (1,2,3):
            reset();runtime.set_obstacle_map(map_id)
            nav.refresh_obstacles(force=True)
            props=[o for o in nav.detector.obstacles if o.get('route_test_obstacle')]
            assert len(props)==(0 if map_id==1 else 4),(map_id,len(props))
            assert not any(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in runtime.stage.Traverse() if str(p.GetPath()).startswith('/World/MapObstacles'))
            for route in ['route_1_blue','route_2_brown']:
                nav.select_route(route);plan=nav.validate_route();nav.scheduler.config['start_delay_seconds']=0
                start_position=list(runtime.last_state['base_position_m'])
                app.update();nav.start();advance(140)
                rows.append({'map_id':map_id,'route':route,'state_after_start':nav.scheduler.state,
                    'command':nav.last_command,'test_obstacles':len(props),'route_length':plan['length_m'],'policy_fault':runtime.fault,
                    'displacement_m':math.dist(start_position[:2],runtime.last_state['base_position_m'][:2])})
                assert rows[-1]['displacement_m']>.03,'No physical progress after START'
                reset()
                assert len([o for o in nav.detector.obstacles if o.get('route_test_obstacle')])==len(props)
            runtime.factory_cfg['camera']['mode']='overview';runtime.update_camera()
            runtime.context['capture'](runtime.directory/f'map_{map_id}_overview.png')
        from pxr import Gf
        box=next(o for o in props if 'route_1_center_box/Collider' in o['id'])
        lo,hi=box['bounds'];drop=UsdGeom.Cube.Define(runtime.stage,'/World/MapColliderProbe');drop.CreateSizeAttr(.2)
        drop.AddTranslateOp().Set(Gf.Vec3d((lo[0]+hi[0])/2,(lo[1]+hi[1])/2,1.5))
        UsdPhysics.CollisionAPI.Apply(drop.GetPrim());UsdPhysics.RigidBodyAPI.Apply(drop.GetPrim());UsdPhysics.MassAPI.Apply(drop.GetPrim()).CreateMassAttr(1.)
        advance(80)
        probe_z=float(UsdGeom.XformCache().GetLocalToWorldTransform(drop.GetPrim()).ExtractTranslation()[2])
        assert .70<probe_z<.81,('Probe did not settle on box',probe_z)
        runtime.stage.RemovePrim('/World/MapColliderProbe')
        # A nearby synthetic robot feedback pose is used ONLY to test the
        # supervisor's response; the simulated robot is never teleported.
        obstacle=next(o for o in props if o['kind']!='human');lo,hi=obstacle['bounds']
        state=dict(runtime.last_state);state['base_position_m']=[(lo[0]+hi[0])/2,lo[1]-.8,.785]
        state['base_orientation_wxyz']=[math.sqrt(.5),0,0,math.sqrt(.5)]
        state['base_linear_velocity_world_m_s']=[0,.3,0];state['base_angular_velocity_world_rad_s']=[0,0,0]
        # Avoid unrelated factory walls in this focused supervisor logic check.
        class Map:
            radius=.67
            def safe(self,p):return True
        class Grid:
            map=Map()
            def segment_free(self,a,b):return True
        status,reason=nav.safety.check(state,[.3,0,0],Grid(),[obstacle],0,[0,.3])
        assert status=='WAITING_FOR_CLEAR_PATH',(status,reason)
        report={'run_directory':str(runtime.directory),'routes':rows,'safety_probe':{'status':status,'reason':reason,'scope':'synthetic feedback only'},
                'passed':True,'collider_drop_probe_z_m':probe_z,'full_route_completion_tested':False,'ai_avoidance':False}
        (runtime.directory/'factory_maps_results.json').write_text(json.dumps(report,indent=2))
        print('FACTORY_MAPS_RESULT '+json.dumps(report),flush=True)
    except BaseException:
        import traceback
        error=traceback.format_exc();print(error,flush=True)
        if runtime:(runtime.directory/'factory_maps_error.txt').write_text(error)
        raise
    finally:
        if runtime:runtime.close()
        app.close()

if __name__=='__main__':main()
