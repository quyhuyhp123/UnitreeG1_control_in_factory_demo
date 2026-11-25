"""Regression: pooled map visibility, reset state, PhysX collision on/off."""
from pathlib import Path
import sys,json,time
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
def main():
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'renderer':'RaytracedLighting','extra_args':['--/app/vulkan=false',
        '--/exts/isaacsim.physics.newton/auto_switch_on_startup=false','--/exts/isaacsim.core.simulation_manager/default_engine=physx']})
    runtime=None
    try:
        from pxr import UsdGeom,UsdPhysics,Gf
        from controllers.factory_g1_runtime import FactoryG1Runtime
        from controllers.factory_selector import load_factory
        from controllers.factory_maps import layout
        cfg=load_factory('v2');cfg['map_id']=1;runtime=FactoryG1Runtime(app,cfg,self_test=True)
        def advance(n):
            for _ in range(n):
                app.update();runtime.tick()
                if runtime.fault:raise RuntimeError(runtime.fault)
        advance(40);rows=[]
        for map_id in (2,3,1,2):
            runtime.patrol_panel.map_choice.model.get_item_value_model().set_value(map_id-1)
            before=runtime.navigation.grid;start=time.perf_counter();runtime.patrol_panel.apply_factory_map();transaction=time.perf_counter()-start
            advance(2)
            root=runtime.stage.GetPrimAtPath('/World/MapObstacles');self_visible=UsdGeom.Imageable(root).ComputeVisibility()
            assert (self_visible=='invisible')==(map_id==1)
            assert runtime.navigation.grid is before
            assert runtime.navigation.mode=='MANUAL'
            # Drop above the corresponding box. On Map1, hidden cached props
            # must NOT support the cube: it must fall to floor at z=0.1.
            spec=next(i for i in layout(map_id if map_id!=1 else 3) if i['name']=='route_1_center_box')
            cube=UsdGeom.Cube.Define(runtime.stage,'/World/PoolProbe');cube.CreateSizeAttr(.2)
            cube.AddTranslateOp().Set(Gf.Vec3d(spec['position_m'][0],spec['position_m'][1],1.5))
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim());UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim());UsdPhysics.MassAPI.Apply(cube.GetPrim()).CreateMassAttr(1.)
            advance(90);z=float(UsdGeom.XformCache().GetLocalToWorldTransform(cube.GetPrim()).ExtractTranslation()[2])
            expected=.1 if map_id==1 else .75
            assert abs(z-expected)<.04,(map_id,z,expected)
            runtime.stage.RemovePrim('/World/PoolProbe');advance(2)
            runtime.timeline.pause();app.update();assert runtime.safe_reset('pool regression')
            runtime.timeline.play();advance(20)
            actual=len([o for o in runtime.navigation.detector.obstacles if o.get('route_test_obstacle')])
            assert actual==(0 if map_id==1 else 4),(map_id,actual)
            if map_id!=1:
                detected={o['id'].split('/')[-2]:o for o in runtime.navigation.detector.obstacles if o.get('route_test_obstacle')}
                for item in layout(map_id):
                    lo,hi=detected[item['name']]['bounds']
                    assert all(abs((lo[i]+hi[i])/2-item['position_m'][i])<1e-4 for i in (0,1)),item
            rows.append({'map_id':map_id,'ui_apply_ms':transaction*1000,'probe_z':z,'grid_reused':True,'reset_props':actual})
        previous=runtime.navigation.grid
        wall=UsdGeom.Cube.Define(runtime.stage,'/World/GridInvalidationProbe');wall.CreateSizeAttr(.3)
        wall.AddTranslateOp().Set(Gf.Vec3d(3,-12,.5));UsdPhysics.CollisionAPI.Apply(wall.GetPrim())
        runtime.navigation.refresh_obstacles(force=True)
        assert runtime.navigation.grid is not previous,'Real planning obstacle failed to invalidate grid'
        runtime.stage.RemovePrim('/World/GridInvalidationProbe');runtime.navigation.refresh_obstacles(force=True)
        report={'rows':rows,'policy_fault':runtime.fault,'passed':True,'run_directory':str(runtime.directory)}
        (ROOT/'logs/map_pool_tests.json').write_text(json.dumps(report,indent=2));print('MAP_POOL '+json.dumps(report),flush=True)
    except BaseException:
        import traceback
        print(traceback.format_exc(),flush=True);raise
    finally:
        if runtime:runtime.close()
        app.close()
if __name__=='__main__':main()
