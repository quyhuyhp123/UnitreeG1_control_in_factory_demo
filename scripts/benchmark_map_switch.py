"""Measure map transaction and first update, no patrol auto-start."""
from pathlib import Path
import sys,time,json,argparse
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--label',default='optimized');args,_=parser.parse_known_args()
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':False,'renderer':'RaytracedLighting','multi_gpu':False,
        'extra_args':['--/app/vulkan=false','--/exts/isaacsim.physics.newton/auto_switch_on_startup=false','--/exts/isaacsim.core.simulation_manager/default_engine=physx']})
    runtime=None
    try:
        from controllers.factory_g1_runtime import FactoryG1Runtime
        from controllers.factory_selector import load_factory
        cfg=load_factory('v2');cfg['map_id']=1
        runtime=FactoryG1Runtime(app,cfg,self_test=True)
        for _ in range(40):app.update();runtime.tick()
        rows=[]
        for map_id in (2,3,1,2,3,1):
            grid=runtime.navigation.grid;start=time.perf_counter();runtime.set_obstacle_map(map_id);committed=time.perf_counter()
            app.update();runtime.tick();end=time.perf_counter()
            props=[o for o in runtime.navigation.detector.obstacles if o.get('route_test_obstacle')]
            assert len(props)==(0 if map_id==1 else 4)
            rows.append({'map_id':map_id,'switch_ms':(committed-start)*1000,'switch_plus_update_ms':(end-start)*1000,
                         'grid_reused':runtime.navigation.grid is grid,'props':len(props)})
        report={'label':args.label,'run_directory':str(runtime.directory),'rows':rows,'policy_fault':runtime.fault,'patrol_mode':runtime.navigation.mode}
        (ROOT/'logs'/f'map_switch_{args.label}.json').write_text(json.dumps(report,indent=2));print('MAP_SWITCH '+json.dumps(report),flush=True)
    except BaseException:
        import traceback
        print(traceback.format_exc(),flush=True);raise
    finally:
        if runtime:runtime.close()
        app.close()
if __name__=='__main__':main()
