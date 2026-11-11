"""Measured flat-floor speed sweep; genuine policy/PhysX, resets only between trials."""
from pathlib import Path
import sys,json,math,statistics,argparse
ROOT=Path(r'D:\unitree_g1'); sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
parser=argparse.ArgumentParser(); parser.add_argument('--speeds',type=float,nargs='+',default=[.22,.35,.5,.65,.8,1.]); parser.add_argument('--output-name',default='speed_sweep_results.json'); args,_=parser.parse_known_args()
if Path(args.output_name).name!=args.output_name or any(not .12<=v<=1. for v in args.speeds): raise ValueError('Invalid speed/output')
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'renderer':'RaytracedLighting','anti_aliasing':1,'multi_gpu':False,'extra_args':['--/app/vulkan=false','--/exts/isaacsim.physics.newton/auto_switch_on_startup=false','--/exts/isaacsim.core.simulation_manager/default_engine=physx']})
rt=None; report={'scope':'Straight flat floor only; not factory route certification','trials':[]}
dest=ROOT/'docs/navigation'/args.output_name
try:
    from controllers.factory_selector import load_factory
    from controllers.factory_g1_runtime import FactoryG1Runtime
    cfg=load_factory('v2'); cfg['head_camera']['enabled']=False
    rt=FactoryG1Runtime(app,cfg,flat=True,self_test=True)
    command=[0.,0.,0.]; rt.select_motion_command=lambda dt:list(command)
    def step():
        app.update(); rt.tick()
        s=rt.last_state; q=s['base_orientation_wxyz']
        return {'position':list(s['base_position_m']),'velocity':list(s['base_linear_velocity_world_m_s']),
                'tilt_deg':math.degrees(math.acos(max(-1,min(1,1-2*(q[1]**2+q[2]**2))))),'command':list(command),'fault':rt.fault}
    for target in args.speeds:
        if report['trials']:
            rt.timeline.pause(); app.update(); rt.safe_reset('speed_sweep_between_trials'); rt.timeline.play(); app.update(); rt.keys.set_running(True)
        command[:]=[0.,0.,0.]
        for _ in range(100): step()
        rows=[]; hold=[]
        for _ in range(math.ceil((target/.4+3)*50)):
            command[0]=min(target,command[0]+.4/50); row=step(); rows.append(row)
            if command[0]>=target-1e-8: hold.append(row)
            if rt.fault: break
        release=list(rt.last_state['base_position_m']); command[:]=[0.,0.,0.]; stops=[]
        for _ in range(150):
            stops.append(step())
            if rt.fault: break
        mean=statistics.mean(r['velocity'][0] for r in hold[-100:]) if hold else 0
        passed=not rt.fault and min(r['position'][2] for r in rows+stops)>.6 and max(r['tilt_deg'] for r in rows+stops)<25 and abs(mean-target)<max(.12,.25*target) and abs(stops[-1]['position'][1])<.35
        trial={'command_m_s':target,'passed':passed,'mean_forward_m_s':mean,'max_tilt_deg':max(r['tilt_deg'] for r in rows+stops),'max_speed_m_s':max(math.hypot(*r['velocity'][:2]) for r in rows+stops),'max_stop_displacement_m':max(math.dist(release[:2],r['position'][:2]) for r in stops),'final_position':stops[-1]['position'],'fault':rt.fault}
        report['trials'].append(trial); report['run_directory']=str(rt.directory); dest.write_text(json.dumps(report,indent=2))
        (rt.directory/f'speed_{target:.2f}_trace.json').write_text(json.dumps(rows+stops))
        print('SPEED_TRIAL '+json.dumps(trial),flush=True)
        if not passed: break
    report['status']='completed'; dest.write_text(json.dumps(report,indent=2))
finally:
    if rt: rt.close()
    app.close()
