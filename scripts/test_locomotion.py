"""Single-G1 velocity-policy acceptance sequence, Isaac Sim 6.1 / PhysX.

All movement after reset comes from joint drives under gravity. No real robot I/O.
"""
from pathlib import Path
import argparse,json,math,re,sys,traceback
from datetime import datetime
PROJECT_ROOT=Path(r'D:\unitree_g1')
sys.path.insert(0,str(PROJECT_ROOT))
sys.path.insert(0,str(PROJECT_ROOT/'scripts'))

def yaw_of(q):
    w,x,y,z=q
    return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--gui',action='store_true')
    parser.add_argument('--no-capture',action='store_true')
    parser.add_argument('--stand-only',action='store_true')
    parser.add_argument('--forward-probe',action='store_true',help='Diagnostic: stand then 0.5 m/s for 4 seconds')
    parser.add_argument('--in-place-turns',action='store_true',help='Diagnostic: pure yaw; currently not accepted for this checkpoint')
    args,_=parser.parse_known_args()
    cfg=json.loads((PROJECT_ROOT/'configs/locomotion.yaml').read_text(encoding='utf-8'))
    deploy=json.loads((PROJECT_ROOT/cfg['deploy_config']).read_text(encoding='utf-8'))
    robot_cfg=json.loads((PROJECT_ROOT/'configs/g1_robot.yaml').read_text(encoding='utf-8'))
    robot_cfg['physics_hz']=cfg['physics_hz']
    robot_cfg['scene_output']='assets/robot/g1_locomotion_flat.usd'
    robot_cfg['self_collision']=cfg['self_collision']
    robot_cfg['floor_size_m']=cfg.get('floor_size_m',6)
    # Exact deploy PD gains are in SDK order, while default q/actions use policy
    # order. Retain physical torque/velocity limits from the robot config.
    original_groups=robot_cfg['actuator_groups']
    groups={}
    for sdk_index,name in enumerate(robot_cfg['sdk_joint_order']):
        source=next(g for g in original_groups.values() if any(re.fullmatch(p,name) for p in g['joint_names_expr']))
        groups[name]={**source,'joint_names_expr':[re.escape(name)],
                      'stiffness':deploy['stiffness'][sdk_index],'damping':deploy['damping'][sdk_index]}
    robot_cfg['actuator_groups']=groups
    robot_cfg['initial_joint_position_rules_rad']={re.escape(n):deploy['default_joint_pos'][i] for i,n in enumerate(cfg['policy_joint_names'])}
    directory=PROJECT_ROOT/'logs/locomotion_runs'/datetime.now().strftime('%Y%m%d_%H%M%S')
    directory.mkdir(parents=True,exist_ok=True)
    report={'status':'starting','run_directory':str(directory),'phases':[],'config':cfg}
    report_path=PROJECT_ROOT/'logs/locomotion_result.json'
    def write_report():
        payload=json.dumps(report,indent=2)
        report_path.write_text(payload,encoding='utf-8')
        (directory/'result.json').write_text(payload,encoding='utf-8')
    write_report()
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':not args.gui,'width':960,'height':720,'renderer':'RaytracedLighting','multi_gpu':False,
        'extra_args':['--/exts/isaacsim.physics.newton/auto_switch_on_startup=false','--/exts/isaacsim.core.simulation_manager/default_engine=physx']})
    try:
        import numpy as np
        import omni.kit.app,omni.timeline,omni.physics.core
        from pxr import UsdGeom,Gf
        from omni.physx import get_physx_interface
        from isaacsim.core.simulation_manager import SimulationManager
        from isaacsim.core.experimental.prims import Articulation
        from controllers.g1_locomotion import G1Locomotion,inverse_rotate_wxyz
        from controllers.g1_joint_targets import as_numpy
        from g1_scene import build_g1_scene
        omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate('isaacsim.pip.onnx',True)
        context=build_g1_scene(app,robot_cfg)
        if not args.no_capture: context['capture'](directory/'initial.png')
        articulation=Articulation(context['root_path'])
        SimulationManager.set_physics_dt(1/cfg['physics_hz'])
        timeline=omni.timeline.get_timeline_interface()
        timeline.play()
        app.update()  # initialize tensor API before the deterministic stepping loop
        timeline.set_auto_update(False)
        policy=G1Locomotion(articulation,cfg)
        if articulation.num_dofs!=29: raise RuntimeError('Unexpected DOF count')
        stiffness,damping=articulation.get_dof_gains()
        actual_kp=as_numpy(stiffness).reshape(-1)[policy.indices]
        actual_kd=as_numpy(damping).reshape(-1)[policy.indices]
        expected_kp=np.array([deploy['stiffness'][i] for i in deploy['joint_ids_map']])
        expected_kd=np.array([deploy['damping'][i] for i in deploy['joint_ids_map']])
        if not np.allclose(actual_kp,expected_kp,rtol=1e-5,atol=1e-4) or not np.allclose(actual_kd,expected_kd,rtol=1e-5,atol=1e-4):
            raise RuntimeError('Runtime PD gains do not match deploy SDK mapping')
        report['runtime_pd_gains_verified']=True
        report['runtime_stiffness_policy_order']=actual_kp.tolist()
        report['runtime_damping_policy_order']=actual_kd.tolist()
        report['joint_drives']=context['joints']
        policy.reset()
        physics_events={'count':0,'dt_sum':0.0,'dt_min':1.0,'dt_max':0.0}
        def physics_event(dt,context):
            physics_events['count']+=1
            physics_events['dt_sum']+=dt
            physics_events['dt_min']=min(physics_events['dt_min'],dt)
            physics_events['dt_max']=max(physics_events['dt_max'],dt)
        subscription=omni.physics.core.get_physics_simulation_interface().subscribe_physics_on_step_events(pre_step=False,order=0,on_update=physics_event)
        stage=context['stage']
        physx=get_physx_interface()
        telemetry_path=directory/'telemetry.jsonl'
        report['telemetry_file']=str(telemetry_path)
        report['policy_runtime_indices']=policy.indices
        report['initial_base_height_m']=context['height']
        report['supplemental_proxies']=context['added_proxies']
        report['physics_engine']=str(SimulationManager.get_active_physics_engine())
        phases=cfg['test_phases'][:1] if args.stand_only else cfg['test_phases']
        if args.forward_probe:
            phases=[cfg['test_phases'][0],{'name':'forward','duration_s':4.0,'command':[.5,0,0]}]
        if args.in_place_turns:
            phases=json.loads(json.dumps(phases))
            for phase in phases:
                if phase['name'] in ['turn_left','turn_right']: phase['command'][0]=0.0
        report['turn_test_mode']='in_place' if args.in_place_turns else cfg.get('turn_test_mode','in_place')
        total_control=0
        with telemetry_path.open('w',encoding='utf-8') as telemetry:
            for phase in phases:
                samples=[]
                count=round(phase['duration_s']*cfg['control_hz'])
                for index in range(count):
                    state=policy.step(phase['command'])
                    state.update({'phase':phase['name'],'simulation_time_s':total_control/cfg['control_hz']})
                    q=state['base_orientation_wxyz']
                    up=inverse_rotate_wxyz(q,[0,0,1])
                    tilt=math.degrees(math.acos(float(np.clip(up[2],-1,1))))
                    state['tilt_deg']=tilt
                    # Tensor link feedback avoids relying on viewport USD caches.
                    link_transforms=as_numpy(articulation._physics_articulation_view.get_link_transforms())[0]
                    state['foot_link_positions_m']={side:link_transforms[articulation.link_names.index(f'{side}_ankle_roll_link'),:3].tolist() for side in ['left','right']}
                    telemetry.write(json.dumps(state)+'\n')
                    samples.append(state)
                    total_control+=1
                    if tilt>65 or state['base_position_m'][2]<.4:
                        report['fallen_at']={'phase':phase['name'],'step':total_control,'tilt_deg':tilt,'position':state['base_position_m']}
                        raise RuntimeError('Robot fell; policy locomotion not accepted')
                    for _ in range(cfg['decimation']): SimulationManager.step()
                    physx.update_transformations(False,True,True,False)
                    app.update()  # timeline auto-update is disabled: render only
                end=policy.feedback.read_feedback()
                start=samples[0]
                displacement=np.array(end['base_position_m'])-np.array(start['base_position_m'])
                headings=np.unwrap([yaw_of(s['base_orientation_wxyz']) for s in samples]+[yaw_of(end['base_orientation_wxyz'])])
                tail=samples[len(samples)//2:]
                vx=float(np.mean([s['base_linear_velocity_body_m_s'][0] for s in tail]))
                speed=float(np.mean([np.linalg.norm(s['base_linear_velocity_body_m_s'][:2]) for s in tail]))
                yawrate=float(np.mean([inverse_rotate_wxyz(s['base_orientation_wxyz'],s['base_angular_velocity_world_rad_s'])[2] for s in tail]))
                joint_motion=float(np.max(np.std([s['joint_positions_rad'] for s in samples],axis=0)))
                foot_range={side:float(np.ptp([s['foot_link_positions_m'][side][2] for s in samples])) for side in ['left','right']}
                result={'name':phase['name'],'command':phase['command'],'duration_s':phase['duration_s'],
                        'displacement_world_m':displacement.tolist(),'yaw_change_rad':float(headings[-1]-headings[0]),
                        'mean_tail_vx_body_m_s':vx,'mean_tail_speed_body_m_s':speed,'mean_tail_yaw_rate_rad_s':yawrate,
                        'max_tilt_deg':max(s['tilt_deg'] for s in samples),'min_base_height_m':min(s['base_position_m'][2] for s in samples),
                        'max_joint_std_rad':joint_motion,'foot_height_range_m':foot_range,
                        'mean_inference_ms':float(np.mean([s['inference_ms'] for s in samples]))}
                stable=result['max_tilt_deg']<45 and result['min_base_height_m']>.5
                if phase['name']=='forward':
                    result['passed']=bool(stable and vx>.05 and np.linalg.norm(displacement[:2])>.25 and min(foot_range.values())>.015)
                elif phase['name']=='turn_left': result['passed']=bool(stable and result['yaw_change_rad']>.15 and yawrate>.03)
                elif phase['name']=='turn_right': result['passed']=bool(stable and result['yaw_change_rad']<-.15 and yawrate<-.03)
                else: result['passed']=bool(stable and speed<.15)
                report['phases'].append(result)
                write_report()
                print('LOCOMOTION_PHASE '+json.dumps(result),flush=True)
                if not args.no_capture:
                    base=end['base_position_m']
                    target=Gf.Vec3d(base[0],base[1],.7)
                    context['camera'].GetOrderedXformOps()[0].Set(Gf.Matrix4d().SetLookAt(target+Gf.Vec3d(2.3,-2.4,1),target,Gf.Vec3d(0,0,1)).GetInverse())
                    context['capture'](directory/(phase['name']+'.png'))
        expected_steps=total_control*cfg['decimation']
        report['physics_events']=physics_events
        report['control_steps']=total_control
        report['expected_physics_steps']=expected_steps
        if physics_events['count']!=expected_steps or abs(physics_events['dt_max']-1/cfg['physics_hz'])>1e-6:
            raise RuntimeError('Unexpected physics/control scheduling')
        report['status']='passed' if all(p['passed'] for p in report['phases']) else 'failed_acceptance_metrics'
        report['all_requested_phases_tested']=not (args.stand_only or args.forward_probe)
        write_report()
        if report['status']!='passed': raise RuntimeError('One or more velocity tests did not meet acceptance metrics')
        print('LOCOMOTION_RESULT '+json.dumps({'status':report['status'],'run_directory':str(directory),'physics_events':physics_events}),flush=True)
        timeline.stop()
    except BaseException:
        report['status']='failed'
        report['error']=traceback.format_exc()
        write_report()
        print(report['error'],flush=True)
        raise
    finally: app.close()

if __name__=='__main__': main()
