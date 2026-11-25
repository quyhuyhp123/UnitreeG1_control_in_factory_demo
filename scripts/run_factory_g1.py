"""One-command factory + G1 + verified ONNX + keyboard. Linux/Windows, Isaac Sim 6.1."""
from pathlib import Path
import argparse,json,sys,time,traceback,hashlib,math
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]

def preflight(cfg,flat):
    def file(value,label):
        path=ROOT/value
        if not path.is_file(): raise FileNotFoundError(f'Missing {label}: {path}')
        return path
    if not flat: file(cfg['factory_usd'],'USD')
    if cfg.get('factory_id')=='v2' and not flat:
        file(f"assets/factory/factory_layout_v2/maps/map_{int(cfg.get('map_id',1))}.usda",'factory map variant')
        source=file(cfg['source_factory_usd'],'V2 source USD')
        if hashlib.sha256(source.read_bytes()).hexdigest()!=cfg['source_factory_sha256']: raise ValueError('Factory V2 source changed; rebuild physics/navigation composition')
        nav=json.loads(file(cfg['navigation_map'],'V2 navigation map').read_text())
        if nav.get('factory_id')!='v2' or nav.get('source_factory_sha256')!=cfg['source_factory_sha256']: raise ValueError('V2 navigation map identity mismatch')
    loco=json.loads(file(cfg['locomotion_config'],'locomotion config').read_text(encoding='utf-8-sig'))
    checkpoint=file(cfg['checkpoint'],'checkpoint')
    if hashlib.sha256(checkpoint.read_bytes()).hexdigest()!=loco['checkpoint_sha256']: raise ValueError('Checkpoint hash mismatch: select a verified matching policy/config')
    file(loco['deploy_config'],'deploy config')
    robot=json.loads(file('configs/g1_robot.yaml','robot config').read_text(encoding='utf-8-sig'))
    file(robot['asset'],'G1 USD')
    if cfg['control_hz']!=loco['control_hz'] or abs(1/cfg['physics_dt']-loco['physics_hz'])>1e-6:
        raise ValueError('Timing differs from verified policy: use control_hz=50, physics_dt=0.005')
    if cfg['rendering']['renderer']!='RaytracedLighting': raise ValueError('Only RTX real-time renderer allowed; no path tracing')
    if cfg['rendering']['graphics_api'] not in ['d3d12','vulkan']: raise ValueError('Invalid graphics API')
    if cfg['camera']['mode'] not in ['free','follow','overview']: raise ValueError('Invalid camera mode')
    return loco

def classify(error):
    from controllers.controller_interface import ControllerUnavailable
    if isinstance(error,ControllerUnavailable): return 'CONTROLLER_UNAVAILABLE'
    s=str(error).lower()
    if 'out of memory' in s or 'cuda_error_out_of_memory' in s: return 'CUDA_OUT_OF_MEMORY'
    if isinstance(error,FileNotFoundError): return 'MISSING_ASSET_OR_CHECKPOINT'
    if 'joint' in s or 'dof' in s: return 'WRONG_JOINT_MAPPING'
    if 'observation' in s or 'contract' in s or 'shape' in s: return 'POLICY_DIMENSION_OR_CONTRACT_MISMATCH'
    if isinstance(error,(ImportError,AttributeError)): return 'ISAAC_API_INCOMPATIBILITY'
    return 'SIMULATION_ERROR'

def validation(runtime):
    import numpy as np
    from test_keyboard import yaw
    from controllers.g1_locomotion import inverse_rotate_wxyz
    report={'status':'running','mode':'flat' if runtime.flat else 'factory','input_source':'Carb logical keyboard, not physical OS keyboard',
            'run_directory':str(runtime.directory),'scene':runtime.scene_checks,'tests':{}}
    path=runtime.directory/'integration_results.json'
    def save():
        payload=json.dumps(report,indent=2)
        path.write_text(payload,encoding='utf-8')
        (ROOT/('logs/factory_g1_flat_results.json' if runtime.flat else 'logs/factory_g1_results.json')).write_text(payload,encoding='utf-8')
    def add(key,passed,**values):
        report['tests'][key]={'passed':bool(passed),**values}
        save(); print('FACTORY_TEST '+json.dumps({key:report['tests'][key]}),flush=True)
    def advance(seconds):
        rows=[]
        for _ in range(round(seconds*runtime.loco_cfg['control_hz'])):
            runtime.app.update(); row=runtime.tick()
            if row: rows.append(row)
            if runtime.fault: break
        return rows
    def release():
        for key in runtime.keys.mapping: runtime.inject(key,'RELEASE')
    def reset():
        release(); runtime.timeline.pause(); runtime.app.update()
        if not runtime.safe_reset(source='between_independent_integration_cases'): raise RuntimeError('Paused reset failed')
        runtime.timeline.play(); runtime.app.update(); runtime.keys.set_running(True)
        advance(.5)
    def motion(key,keys,axis,sign):
        reset(); before=runtime.policy.feedback.read_feedback()
        for k in keys: runtime.inject(k)
        seconds=1.4 if runtime.factory_cfg.get('factory_id')=='v2' and axis==0 and sign<0 else 3.5
        rows=advance(seconds)
        after=runtime.policy.feedback.read_feedback()
        release(); stop=advance(2)
        delta=np.array(after['base_position_m'])-np.array(before['base_position_m'])
        body_delta=inverse_rotate_wxyz(before['base_orientation_wxyz'],delta)
        angles=np.unwrap([yaw(r['base_orientation_wxyz']) for r in rows]+[yaw(after['base_orientation_wxyz'])])
        turn=float(angles[-1]-angles[0])
        value=body_delta[axis] if axis<2 else turn
        add(key,sign*value>(.2 if axis<2 else .12) and not runtime.fault,
            displacement_m=delta.tolist(),displacement_body_m=body_delta.tolist(),yaw_change_rad=turn,fault=runtime.fault,
            obstacle_contacts_total=runtime.contact_counts.get('obstacle',0),
            velocity_after_release_m_s=runtime.last_state['base_linear_velocity_world_m_s'],
            angular_velocity_after_release_rad_s=runtime.last_state['base_angular_velocity_world_rad_s'],
            final_held_command=rows[-1]['applied_command'] if rows else None)
        return stop
    add('A',runtime.flat or runtime.scene_checks.get('factory_reference_loaded',False),meaning='Factory reference opened' if not runtime.flat else 'Flat comparison stage opened')
    p=runtime.last_state['base_position_m']
    add('B',np.linalg.norm(np.array(p[:2])-runtime.spawn[:2])<.02 and .7<p[2]<.9,position_m=p,spawn_marker=runtime.scene_checks.get('spawn_marker'))
    rows=advance(4)
    add('C',len(rows)==200 and not runtime.fault and max(r['tilt_deg'] for r in rows)<20,
        samples=len(rows),max_tilt_deg=max([r['tilt_deg'] for r in rows],default=180),floor_contact_points=runtime.contact_counts.get('floor',0),fault=runtime.fault)
    motion('D',['W'],0,1); stop=motion('E',['S'],0,-1)
    final=runtime.last_state
    add('G',runtime.keys.command==[0,0,0] and not runtime.fault and np.linalg.norm(final['base_linear_velocity_world_m_s'])<.15,
        command=runtime.keys.command,measured_velocity_m_s=final['base_linear_velocity_world_m_s'])
    motion('F_left',['A'],2,1); motion('F_right',['D'],2,-1)
    # Walking turns are tested separately; never add hidden forward speed to A/D.
    motion('F_walking_left',['W','A'],2,1); motion('F_walking_right',['UP','RIGHT'],2,-1)
    if not runtime.flat:
        reset(); runtime.contact_counts.pop('obstacle',None); runtime.contact_examples.clear()
        runtime.inject('W'); trajectory=[]
        collision_seconds=80 if runtime.factory_cfg.get('factory_id')=='v2' else 40
        for _ in range(collision_seconds*runtime.loco_cfg['control_hz']):
            runtime.app.update(); row=runtime.tick()
            trajectory.append(runtime.last_state['base_position_m'])
            if runtime.fault: break
            if runtime.contact_counts.get('obstacle',0)>0:
                advance(.5); break
        release()
        contacts=runtime.contact_counts.get('obstacle',0)
        floor=runtime.navigation.detector.floor
        inside=all(all(floor[0][i]-.15<p[i]<floor[1][i]+.15 for i in [0,1]) and p[2]>.35 for p in trajectory)
        add('H',contacts>0 and inside,contact_points=contacts,contact_examples=runtime.contact_examples,
            trajectory_end_m=trajectory[-1],base_stayed_inside_factory=inside,fault=runtime.fault,
            scope='Actual G1/factory contact reports and base trajectory; not an exhaustive geometric penetration proof')
    else: report['tests']['H']={'passed':None,'reason':'No obstacles in flat comparison mode'}
    release(); runtime.timeline.pause(); runtime.app.update(); runtime.inject('R'); runtime.tick()
    rec=runtime.reset_events[-1]
    half=math.radians(runtime.factory_cfg['robot_spawn']['heading_deg'])/2
    expected_q=np.array([math.cos(half),0,0,math.sin(half)])
    heading_correct=abs(float(np.dot(expected_q,rec['orientation_after_reset_wxyz'])))>.999
    add('I',rec['source']=='R' and rec['history_terms_after_reset']==0 and rec['policy_steps_after_reset']==0 and heading_correct and
        np.linalg.norm(np.array(rec['position_after_reset'][:2])-runtime.spawn[:2])<.02,reset=rec,spawn_heading_restored=heading_correct)
    runtime.timeline.play(); runtime.app.update(); advance(1)
    runtime.update_camera()
    runtime.context['capture'](runtime.directory/'factory_g1_overview.png')
    (runtime.directory/'camera_snapshot.json').write_text(json.dumps(runtime.snapshot(),indent=2),encoding='utf-8')
    perf=[json.loads(line) for line in (runtime.directory/'performance.jsonl').read_text().splitlines()]
    add('J',bool(perf) and bool(runtime.gpu_samples),performance_samples=len(perf),
        average_loop_fps=float(np.mean([r['loop_fps'] for r in perf])) if perf else None,
        average_physics_steps_per_wall_second=float(np.mean([r['physics_steps_per_wall_second'] for r in perf])) if perf else None,
        peak_device_total_used_mib=max([r['gpu_device_total_used_mib'] for r in runtime.gpu_samples],default=None),
        gpu_total_mib=runtime.gpu_samples[-1]['gpu_total_mib'] if runtime.gpu_samples else None,
        vram_scope='Whole GPU including other applications, not process-only VRAM',
        fps_scope='App loop/render requests, accelerated self-test without pacing; not GPU present-time profiling')
    report['manual_native_keyboard_required']=True
    report['failed_tests']=[k for k,v in report['tests'].items() if v['passed'] is False]
    report['status']='passed_automated_checks' if not report['failed_tests'] else 'partial_with_failed_tests'
    save()
    print('FACTORY_RESULT '+json.dumps({'status':report['status'],'failed_tests':report['failed_tests'],'run_directory':str(runtime.directory)}),flush=True)
    return report

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--config',type=Path)
    parser.add_argument('--check',action='store_true',help='Verify selected factory assets/config without starting Isaac Sim')
    parser.add_argument('--factory',choices=['v1','v2'])
    parser.add_argument('--map',dest='map_id',type=int,choices=[1,2,3],default=1)
    parser.add_argument('--flat',action='store_true'); parser.add_argument('--self-test',action='store_true')
    parser.add_argument('--headless',action='store_true'); parser.add_argument('--duration',type=float,default=0)
    parser.add_argument('--overview',action='store_true')
    parser.add_argument('--controller',choices=['locomotion','sonic'],default='locomotion')
    parser.add_argument('--rgbd',action='store_true',help='Enable the forward RGB-D camera (perception only)')
    parser.add_argument('--rgbd-debug',action='store_true',help='Enable the RGB-D camera and its RGB/Depth debug window')
    parser.add_argument('--no-rgbd',action='store_true',help='Force the RGB-D camera off regardless of configs/rgbd_camera.json')
    parser.add_argument('--yolo',action='store_true',help='Enable YOLO detection on the RGB-D camera (detection output only)')
    parser.add_argument('--yolo-debug',action='store_true',help='Enable YOLO and show boxes/classes/confidence in the debug window')
    parser.add_argument('--no-yolo',action='store_true',help='Force YOLO off regardless of configs/yolo.json')
    parser.add_argument('--obstacles',action='store_true',help='Enable RGB-D obstacle localisation (camera + YOLO + depth; output only)')
    parser.add_argument('--obstacles-debug',action='store_true',help='Obstacle localisation plus the debug window with distances')
    parser.add_argument('--no-obstacles',action='store_true',help='Force obstacle localisation off regardless of its config')
    parser.add_argument('--costmap',action='store_true',help='Enable the local obstacle costmap (camera + YOLO + localisation)')
    parser.add_argument('--costmap-debug',action='store_true',help='Local costmap plus the debug window showing it')
    parser.add_argument('--no-costmap',action='store_true',help='Force the local costmap off regardless of its config')
    parser.add_argument('--dynamic-avoidance',action='store_true',help='Opt in to experimental Factory V2 costmap avoidance')
    args,_=parser.parse_known_args()
    (ROOT/'logs').mkdir(parents=True,exist_ok=True)
    app=runtime=None
    try:
        from controllers.controller_selector import validate_selection
        validate_selection(args.controller)
        from controllers.factory_selector import load_factory
        cfg=load_factory(args.factory,args.config)
        if args.dynamic_avoidance:
            if args.flat or args.controller!='locomotion' or not cfg.get('multi_route_enabled',False) or cfg.get('factory_id')!='v2':
                raise ValueError('Dynamic avoidance needs Factory V2 Multi-Route and locomotion RL')
            if args.no_costmap or args.no_obstacles or args.no_yolo or args.no_rgbd:
                raise ValueError('Dynamic avoidance requires RGB-D, YOLO, localization and costmap')
        cfg['dynamic_avoidance']=args.dynamic_avoidance
        if args.map_id!=1 and cfg.get('factory_id')!='v2':raise ValueError('Map variants require Factory V2')
        cfg['map_id']=args.map_id
        if args.overview: cfg['camera']['mode']='overview'
        if sys.platform.startswith('linux'): cfg['rendering']['graphics_api']='vulkan'
        preflight(cfg,args.flat)
        from perception.rgbd_config import load_config as load_rgbd_config
        cfg['rgbd_camera']=load_rgbd_config()
        if args.rgbd or args.rgbd_debug: cfg['rgbd_camera']['enabled']=True
        if args.rgbd_debug: cfg['rgbd_camera']['debug_window']=True
        if args.no_rgbd: cfg['rgbd_camera']['enabled']=False
        from perception.yolo_config import load_config as load_yolo_config,verify_model as verify_yolo_model
        cfg['yolo']=load_yolo_config()
        obstacles=json.loads((ROOT/'configs/obstacle_localization.json').read_text())
        costmap=json.loads((ROOT/'configs/local_costmap.json').read_text())
        # The YOLO debug launcher can enable avoidance from Navigation without
        # another restart. It still starts in the OFF state unless opted in.
        if args.costmap or args.costmap_debug or args.dynamic_avoidance or (args.yolo_debug and cfg.get('multi_route_enabled',False)):
            costmap['enabled']=True
        if cfg.get('multi_route_enabled',False) and (args.dynamic_avoidance or args.yolo_debug):
            costmap['size_m']=max(costmap['size_m'],8.0,costmap.get('avoidance_size_m',8.0))
        costmap['debug_window']=args.costmap_debug
        if args.no_costmap or args.no_obstacles or args.no_yolo or args.no_rgbd: costmap['enabled']=False
        cfg['local_costmap']=costmap
        if args.obstacles or args.obstacles_debug or costmap['enabled']: obstacles['enabled']=True
        if args.no_obstacles or args.no_yolo or args.no_rgbd: obstacles['enabled']=False
        cfg['obstacle_localization']=obstacles
        if args.yolo or args.yolo_debug or obstacles['enabled']: cfg['yolo']['enabled']=True
        if args.no_yolo or args.no_rgbd: cfg['yolo']['enabled']=False
        if cfg['yolo']['enabled']:
            verify_yolo_model(cfg['yolo']); cfg['rgbd_camera']['enabled']=True  # YOLO reads the camera's RGB frames
            if args.yolo_debug or args.obstacles_debug or args.costmap_debug: cfg['rgbd_camera']['debug_window']=True
        if args.check:
            print('PREFLIGHT_OK '+str(ROOT)+' factory='+cfg.get('factory_id','v1'),flush=True)
            return
        from isaacsim import SimulationApp
        render=cfg['rendering']
        app=SimulationApp({'headless':args.headless,'width':render['width'],'height':render['height'],
            'anti_aliasing':render.get('anti_aliasing',1),
            'renderer':render['renderer'],'multi_gpu':False,'extra_args':[
                '--/app/vulkan='+str(render['graphics_api']=='vulkan').lower(),
                '--/exts/isaacsim.physics.newton/auto_switch_on_startup=false',
                '--/exts/isaacsim.core.simulation_manager/default_engine=physx']})
        import carb
        settings=carb.settings.get_settings()
        if not args.self_test:
            # Verified settings in Isaac Sim's bundled .kit applications.
            # Let Kit pace app updates; patrol scheduling never sleeps the loop.
            settings.set('/app/runLoops/main/rateLimitEnabled',True)
            settings.set('/app/runLoops/main/rateLimitFrequency',render['limit_fps'])
        if render['low_quality']:
            for key in ['/rtx/reflections/enabled','/rtx/indirectDiffuse/enabled','/rtx/ambientOcclusion/enabled']:
                settings.set(key,False)
        from controllers.factory_g1_runtime import FactoryG1Runtime
        runtime=FactoryG1Runtime(app,cfg,flat=args.flat,self_test=args.self_test)
        print('FACTORY_READY '+str(runtime.directory),flush=True)
        if args.self_test:
            report=validation(runtime)
            if report['failed_tests']: raise RuntimeError('Integration checks failed: '+', '.join(report['failed_tests']))
        else:
            started=time.perf_counter()
            while app.is_running():
                app.update(); runtime.tick()
                if args.duration and time.perf_counter()-started>=args.duration: break
            if args.duration:
                runtime.context['capture'](runtime.directory/'factory_g1_gui.png')
                runtime.capture_head_preview(runtime.directory/'head_camera.png')
                if runtime.rgbd is not None and runtime.rgbd.frame is not None:
                    print('RGBD_SAVED '+str(runtime.rgbd.save(tag='duration_end')),flush=True)
                if runtime.yolo is not None and runtime.yolo.result is not None:
                    print('YOLO_SAVED '+str(runtime.yolo.save(tag='duration_end')),flush=True)
                    print('YOLO_STATS '+json.dumps({'provider':runtime.yolo.detector.provider,**runtime.yolo.worker.stats()}),flush=True)
                    if 'obstacles' in runtime.yolo.result:
                        print('OBSTACLES_SAVED '+str(runtime.yolo.save_obstacles(tag='duration_end')),flush=True)
                    if 'costmap' in runtime.yolo.result:
                        print('COSTMAP_SAVED '+str(runtime.yolo.save_costmap(tag='duration_end')),flush=True)
                        print('COSTMAP_STATS '+json.dumps(runtime.yolo.result['costmap']),flush=True)
                (runtime.directory/'gui_snapshot.json').write_text(json.dumps(runtime.snapshot(),indent=2),encoding='utf-8')
        if runtime.fault: raise RuntimeError('Policy fault: '+str(runtime.fault))
        print('FACTORY_FINISHED '+json.dumps({'run_directory':str(runtime.directory),'policy_fault':runtime.fault}),flush=True)
    except BaseException as error:
        record={'status':'failed','category':classify(error),'message':str(error),'traceback':traceback.format_exc()}
        (ROOT/'logs/factory_g1_error.json').write_text(json.dumps(record,indent=2),encoding='utf-8')
        print('FACTORY_ERROR '+json.dumps(record),flush=True)
        if record['category']=='CONTROLLER_UNAVAILABLE':
            print('CONTROLLER_UNAVAILABLE '+str(error),flush=True)
            raise SystemExit(2)
        raise
    finally:
        if runtime: runtime.close()
        if app: app.close()

if __name__=='__main__': main()
