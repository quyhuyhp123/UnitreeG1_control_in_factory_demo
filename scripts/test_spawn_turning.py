"""Validate entrance spawn and physical in-place turning of a selected policy."""
from pathlib import Path
import argparse,json,sys,math
ROOT=Path(r'D:\unitree_g1'); sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]

def main():
    p=argparse.ArgumentParser(); p.add_argument('--config',type=Path,default=ROOT/'configs/factory_g1.json'); args,_=p.parse_known_args()
    from run_factory_g1 import preflight
    cfg=json.loads(args.config.read_text()); preflight(cfg,False)
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'renderer':'RaytracedLighting','multi_gpu':False,'extra_args':[
        '--/app/vulkan=false','--/exts/isaacsim.physics.newton/auto_switch_on_startup=false','--/exts/isaacsim.core.simulation_manager/default_engine=physx']})
    runtime=None
    try:
        import numpy as np
        from controllers.factory_g1_runtime import FactoryG1Runtime
        from controllers.g1_locomotion import inverse_rotate_wxyz
        from test_keyboard import yaw
        runtime=FactoryG1Runtime(app,cfg,self_test=True)
        report={'config':str(args.config),'run_directory':str(runtime.directory),'cases':[],'input':'Carb logical keyboard + real ONNX/PhysX'}
        def save():
            payload=json.dumps(report,indent=2)
            (ROOT/'logs/spawn_turning_result.json').write_text(payload,encoding='utf-8')
            (runtime.directory/'spawn_turning_result.json').write_text(payload,encoding='utf-8')
        def advance(seconds):
            rows=[]
            for _ in range(round(seconds*50)):
                app.update(); row=runtime.tick()
                if row: rows.append(row)
                if runtime.fault: break
            return rows
        initial=runtime.last_state
        report['spawn']={'position_m':initial['base_position_m'],'yaw_deg':math.degrees(yaw(initial['base_orientation_wxyz'])),'blockers':runtime.scene_checks['spawn_blockers']}
        for name,keys,duration,sign in [('stand',[],3,0),('W',['W'],3.5,1),('S',['S'],3.5,-1),
                ('A',['A'],5,1),('D',['D'],5,-1),('LEFT',['LEFT'],5,1),('RIGHT',['RIGHT'],5,-1)]:
            for k in runtime.keys.mapping: runtime.inject(k,'RELEASE')
            runtime.timeline.pause(); app.update(); runtime.safe_reset(source='independent_spawn_turn_case')
            runtime.timeline.play(); app.update(); runtime.keys.set_running(True); advance(1)
            before=runtime.last_state
            for k in keys: runtime.inject(k)
            rows=advance(duration); end=runtime.last_state
            angles=np.unwrap([yaw(before['base_orientation_wxyz'])]+[yaw(r['base_orientation_wxyz']) for r in rows]+[yaw(end['base_orientation_wxyz'])])
            turn=float(angles[-1]-angles[0]); delta=np.array(end['base_position_m'])-np.array(before['base_position_m'])
            body=inverse_rotate_wxyz(before['base_orientation_wxyz'],delta)
            drift=max([np.linalg.norm(np.array(r['base_position_m'])[:2]-before['base_position_m'][:2]) for r in rows],default=0)
            stable=runtime.fault is None and end['base_position_m'][2]>.5
            passed=stable and (np.linalg.norm(delta[:2])<.15 if name=='stand' else (sign*body[0]>.2 if name in ['W','S'] else sign*turn>.3 and drift<.6))
            case={'name':name,'passed':bool(passed),'yaw_change_deg':math.degrees(turn),'world_delta_m':delta.tolist(),
                'body_delta_m':body.tolist(),'max_horizontal_drift_m':float(drift),'held_command':runtime.keys.command,'fault':runtime.fault}
            report['cases'].append(case); save(); print('SPAWN_TURN_TEST '+json.dumps(case),flush=True)
        report['status']='passed' if all(c['passed'] for c in report['cases']) else 'failed'
        save(); print('SPAWN_TURN_RESULT '+json.dumps({'status':report['status'],'run_directory':str(runtime.directory)}),flush=True)
    finally:
        if runtime: runtime.close()
        app.close()

if __name__=='__main__': main()
