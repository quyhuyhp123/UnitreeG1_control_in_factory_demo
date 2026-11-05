"""Focused A/D and arrow yaw diagnosis using real ONNX and PhysX."""
from pathlib import Path
import json,sys,math
ROOT=Path(r'D:\unitree_g1')
sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]

def main():
    from run_factory_g1 import preflight
    cfg=json.loads((ROOT/'configs/factory_g1.json').read_text())
    preflight(cfg,False)
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'renderer':'RaytracedLighting','multi_gpu':False,
        'extra_args':['--/app/vulkan=false','--/exts/isaacsim.physics.newton/auto_switch_on_startup=false',
            '--/exts/isaacsim.core.simulation_manager/default_engine=physx']})
    runtime=None
    try:
        import numpy as np
        from controllers.factory_g1_runtime import FactoryG1Runtime
        from test_keyboard import yaw
        runtime=FactoryG1Runtime(app,cfg,self_test=True)
        report={'input_source':'Carb logical keyboard, not OS keyboard','run_directory':str(runtime.directory),'cases':[]}
        def advance(seconds):
            rows=[]
            for _ in range(round(seconds*50)):
                app.update(); row=runtime.tick()
                if row: rows.append(row)
                if runtime.fault: break
            return rows
        for name,keys,sign in [('A',['A'],1),('LEFT',['LEFT'],1),('D',['D'],-1),('RIGHT',['RIGHT'],-1),
                               ('W+A',['W','A'],1),('W+D',['W','D'],-1)]:
            for k in runtime.keys.mapping: runtime.inject(k,'RELEASE')
            runtime.timeline.pause(); app.update(); runtime.safe_reset(source='independent_turn_test')
            runtime.timeline.play(); app.update(); runtime.keys.set_running(True); advance(1)
            initial=runtime.last_state
            before_count=runtime.keys.event_count
            for key in keys: runtime.inject(key)
            rows=advance(6 if len(keys)==1 else 3.5)
            end=runtime.last_state
            angles=np.unwrap([yaw(initial['base_orientation_wxyz'])]+[yaw(r['base_orientation_wxyz']) for r in rows]+[yaw(end['base_orientation_wxyz'])])
            delta=float(angles[-1]-angles[0])
            case={'keys':name,'input_events_received':runtime.keys.event_count-before_count,
                'desired_command':runtime.keys.desired(),'applied_command':list(runtime.keys.command),
                'yaw_change_rad':delta,'yaw_change_deg':math.degrees(delta),
                'mean_measured_yaw_rate_last_second_rad_s':float(np.mean([r['base_angular_velocity_world_rad_s'][2] for r in rows[-50:]])),
                'translation_m':(np.array(end['base_position_m'])-np.array(initial['base_position_m'])).tolist(),
                'command_pass':sign*runtime.keys.command[2]>.15,'motion_pass':sign*delta>.12 and not runtime.fault,
                'fault':runtime.fault}
            report['cases'].append(case)
            print('TURN_TEST '+json.dumps(case),flush=True)
            for key in keys: runtime.inject(key,'RELEASE')
            advance(1)
        report['status']='all_passed' if all(c['motion_pass'] for c in report['cases']) else 'pure_yaw_not_achieved'
        payload=json.dumps(report,indent=2)
        (ROOT/'logs/turning_diagnostic.json').write_text(payload,encoding='utf-8')
        (runtime.directory/'turning_diagnostic.json').write_text(payload,encoding='utf-8')
    finally:
        if runtime: runtime.close()
        app.close()

if __name__=='__main__': main()
