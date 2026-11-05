"""One-command factory + G1 + verified ONNX + keyboard. Windows, Isaac Sim 6.1."""
from pathlib import Path
import argparse,json,sys,time,traceback,hashlib
ROOT=Path(r'D:\unitree_g1')
sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]

def preflight(cfg,flat):
    def file(value,label):
        path=ROOT/value
        if not path.is_file(): raise FileNotFoundError(f'Missing {label}: {path}')
        return path
    if not flat: file(cfg['factory_usd'],'USD')
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
        rows=advance(3.5)
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
        for _ in range(40*runtime.loco_cfg['control_hz']):
            runtime.app.update(); row=runtime.tick()
            trajectory.append(runtime.last_state['base_position_m'])
            if runtime.fault: break
            if runtime.contact_counts.get('obstacle',0)>0:
                advance(.5); break
        release()
        contacts=runtime.contact_counts.get('obstacle',0)
        inside=all(abs(p[0])<5.15 and abs(p[1])<5.15 and p[2]>.35 for p in trajectory)
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
    parser.add_argument('--config',type=Path,default=ROOT/'configs/factory_g1.json')
    parser.add_argument('--flat',action='store_true'); parser.add_argument('--self-test',action='store_true')
    parser.add_argument('--headless',action='store_true'); parser.add_argument('--duration',type=float,default=0)
    parser.add_argument('--overview',action='store_true')
    args,_=parser.parse_known_args()
    app=runtime=None
    try:
        cfg=json.loads(args.config.read_text(encoding='utf-8-sig'))
        if args.overview: cfg['camera']['mode']='overview'
        preflight(cfg,args.flat)
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
        if render['low_quality']:
            for key in ['/rtx/reflections/enabled','/rtx/indirectDiffuse/enabled','/rtx/ambientOcclusion/enabled']:
                settings.set(key,False)
        from controllers.factory_g1_runtime import FactoryG1Runtime
        runtime=FactoryG1Runtime(app,cfg,flat=args.flat,self_test=args.self_test)
        print('FACTORY_READY '+str(runtime.directory),flush=True)
        if args.self_test: validation(runtime)
        else:
            started=time.perf_counter(); next_tick=started
            while app.is_running():
                app.update(); runtime.tick()
                if args.duration and time.perf_counter()-started>=args.duration: break
                next_tick+=1/render['limit_fps']
                delay=next_tick-time.perf_counter()
                if delay>0: time.sleep(min(delay,.02))
                elif delay<-.1: next_tick=time.perf_counter()
            if args.duration:
                runtime.context['capture'](runtime.directory/'factory_g1_gui.png')
                runtime.capture_head_preview(runtime.directory/'head_camera.png')
                (runtime.directory/'gui_snapshot.json').write_text(json.dumps(runtime.snapshot(),indent=2),encoding='utf-8')
        print('FACTORY_FINISHED '+json.dumps({'run_directory':str(runtime.directory),'policy_fault':runtime.fault}),flush=True)
    except BaseException as error:
        record={'status':'failed','category':classify(error),'message':str(error),'traceback':traceback.format_exc()}
        (ROOT/'logs/factory_g1_error.json').write_text(json.dumps(record,indent=2),encoding='utf-8')
        print('FACTORY_ERROR '+json.dumps(record),flush=True)
        raise
    finally:
        if runtime: runtime.close()
        if app: app.close()

if __name__=='__main__': main()
