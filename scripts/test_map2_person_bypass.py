"""Physical check: Map 2, dynamic avoidance, robot passes the chair and person in the shared left corridor.

Real G1 policy/PhysX with the RGB-D + YOLO + costmap pipeline. The robot is
never teleported. Pass = the robot reaches the far end of the left corridor
with no obstacle contact, no FAULT, and stays clear of every scenario prop.

    scripts/isaac_python.sh scripts/test_map2_person_bypass.py [--route route_1_blue] [--until-y 9.5]
"""
from pathlib import Path
import argparse,json,math,sys,time
ROOT=Path(__file__).resolve().parents[1]; sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]


def config(map_id):
    from controllers.factory_selector import load_factory
    from perception.rgbd_config import load_config as load_rgbd
    from perception.yolo_config import load_config as load_yolo,verify_model
    cfg=load_factory('v2'); cfg['dynamic_avoidance']=True; cfg['map_id']=map_id
    if sys.platform.startswith('linux'): cfg['rendering']['graphics_api']='vulkan'
    cfg['rgbd_camera']=load_rgbd(); cfg['rgbd_camera']['enabled']=True
    cfg['yolo']=load_yolo(); cfg['yolo']['enabled']=True; verify_model(cfg['yolo'])
    costmap=json.loads((ROOT/'configs/local_costmap.json').read_text()); costmap['enabled']=True
    costmap['size_m']=max(costmap['size_m'],8.,costmap.get('avoidance_size_m',8.)); costmap['debug_window']=False; cfg['local_costmap']=costmap
    obstacles=json.loads((ROOT/'configs/obstacle_localization.json').read_text()); obstacles['enabled']=True
    cfg['obstacle_localization']=obstacles
    return cfg


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--route',default='route_1_blue'); parser.add_argument('--map',type=int,default=2)
    parser.add_argument('--until-y',type=float,default=9.5); parser.add_argument('--max-sim-s',type=float,default=300.)
    args,_=parser.parse_known_args()
    cfg=config(args.map)
    from isaacsim import SimulationApp
    render=cfg['rendering']
    app=SimulationApp({'headless':True,'width':render['width'],'height':render['height'],'renderer':render['renderer'],'multi_gpu':False,
        'extra_args':['--/app/vulkan=true','--/exts/isaacsim.physics.newton/auto_switch_on_startup=false',
                      '--/exts/isaacsim.core.simulation_manager/default_engine=physx']})
    runtime=None; report={'passed':False}
    try:
        from controllers.factory_g1_runtime import FactoryG1Runtime
        from navigation.lane_map import box_distance
        runtime=FactoryG1Runtime(app,cfg,self_test=True); nav=runtime.navigation
        def advance(n=1):
            for _ in range(n):
                app.update(); runtime.tick()
                if runtime.fault: raise RuntimeError('Policy fault: '+str(runtime.fault))
        runtime.timeline.play(); advance(120)
        nav.refresh_obstacles(force=True)
        props=[o for o in nav.detector.obstacles if o.get('route_test_obstacle')]
        assert len(props)==4,len(props)
        left=[o for o in props if '/shared_left_' in o['id']]
        nav.select_route(args.route); nav.validate_route(); nav.scheduler.config['start_delay_seconds']=0
        nav.start(); begin=nav.sim_time; wall=time.perf_counter()
        contacts0=dict(runtime.contact_counts); min_clear={o['id'].split('/')[-2]:math.inf for o in left}
        track=[]; states=[]; bypass_events=[]
        while True:
            advance(5)
            p=runtime.last_state['base_position_m']
            for o in left: min_clear[o['id'].split('/')[-2]]=min(min_clear[o['id'].split('/')[-2]],box_distance(p[:2],o['bounds']))
            if not track or nav.sim_time-track[-1][0]>=.5:
                track.append([round(nav.sim_time-begin,2),round(p[0],3),round(p[1],3)])
            state=nav.scheduler.state
            if not states or states[-1][1]!=state or states[-1][2]!=nav.reason:
                states.append([round(nav.sim_time-begin,2),state,nav.reason]); print('STATE',states[-1],'pos',[round(v,2) for v in p[:2]],flush=True)
            bypass=(nav.edge or {}).get('bypass') or {}
            if bypass.get('active') and (not bypass_events or bypass_events[-1]['obstacles']!=bypass['obstacle']):
                bypass_events.append({'t':round(nav.sim_time-begin,2),'edge':nav.edge['id'],'obstacles':bypass['obstacle'],'offset_m':bypass['offset_m']})
                print('BYPASS',bypass_events[-1],flush=True)
            if state=='FAULT': break
            if p[1]>=args.until_y and p[0]<-4: break
            if nav.sim_time-begin>args.max_sim_s: break
        obstacle_contacts=runtime.contact_counts.get('obstacle',0)-contacts0.get('obstacle',0)
        p=runtime.last_state['base_position_m']
        reached=p[1]>=args.until_y and p[0]<-4
        report={'map':args.map,'route':args.route,'reached_end_of_left_corridor':reached,'final_state':nav.scheduler.state,'final_reason':nav.reason,
                'final_position':p[:2],'sim_seconds':nav.sim_time-begin,'wall_seconds':time.perf_counter()-wall,
                'min_centre_distance_to_prop_box_m':min_clear,'footprint_radius_m':nav.grid.map.radius,
                'obstacle_contacts':obstacle_contacts,'contact_examples':runtime.contact_examples[:5],
                'bypass_events':bypass_events,'state_changes':states,'track':track,'run_directory':str(runtime.directory)}
        report['passed']=bool(reached and obstacle_contacts==0 and nav.scheduler.state!='FAULT'
                              and all(v>.35 for v in min_clear.values()))
        (runtime.directory/'map2_person_bypass_results.json').write_text(json.dumps(report,indent=2))
        print('MAP2_BYPASS_RESULT '+json.dumps({k:v for k,v in report.items() if k not in ('track','state_changes')}),flush=True)
    finally:
        if runtime: runtime.close()
        app.close()
    if not report['passed']: raise SystemExit(1)


if __name__=='__main__': main()
