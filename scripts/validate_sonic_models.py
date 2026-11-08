"""Inspect real model sessions; run idle planner using recorded real PhysX state.

Does not synthesize encoder inputs or attach SONIC to Articulation.
"""
from pathlib import Path
import argparse,json,sys,time
ROOT=Path(r'D:\unitree_g1')
sys.path.insert(0,str(ROOT))


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--telemetry',type=Path,required=True)
    args,_=parser.parse_known_args()
    environment=json.loads((ROOT/'configs/environment.json').read_text(encoding='utf-8-sig'))
    sys.path.insert(0,str(Path(environment['isaac_sim_root'])/'exts/isaacsim.pip.onnx/pip_prebundle'))
    import numpy as np
    import onnxruntime as ort
    from controllers.sonic_motion_planner import SonicMotionPlanner
    from controllers.sonic_observation_adapter import state_to_mujoco_qpos
    robot=json.loads((ROOT/'configs/g1_robot.yaml').read_text())
    config=json.loads((ROOT/'configs/sonic.yaml').read_text())
    output=ROOT/'docs/sonic_integration/model_validation.json'
    report={'ort_version':ort.__version__,'available_providers':ort.get_available_providers(),
            'provider_used':'CPUExecutionProvider','telemetry':str(args.telemetry),'models':{},
            'encoder_decoder_inference':'NOT TESTED: complete observation/reference parity not established',
            'physx_sonic_control':'BLOCKED'}
    for name in ['model_encoder.onnx','model_decoder.onnx','planner_sonic.onnx']:
        session=ort.InferenceSession(str(ROOT/config['directory']/name),providers=['CPUExecutionProvider'])
        def describe(items): return [{'name':x.name,'shape':x.shape,'dtype':x.type} for x in items]
        report['models'][name]={'inputs':describe(session.get_inputs()),'outputs':describe(session.get_outputs()),'load':'passed'}
        del session
    rows=[json.loads(line) for line in args.telemetry.read_text().splitlines()[:4]]
    if len(rows)!=4 or any(rows[i+1]['keyboard_step']!=rows[i]['keyboard_step']+1 for i in range(3)):
        raise ValueError('Require four consecutive actual simulator state samples')
    context=np.stack([state_to_mujoco_qpos(row,robot['sdk_joint_order']) for row in rows])[None]
    planner=SonicMotionPlanner(ROOT/config['directory']/'planner_sonic.onnx')
    samples=[]
    try:
        for _ in range(3):
            reference,n=planner.predict_idle(context)
            samples.append({'latency_ms':planner.last_latency_ms,'frames':n,
                            'root_z_range_m':[float(reference[...,2].min()),float(reference[...,2].max())]})
        report['planner_idle']={'status':'passed','runs':samples,'context':context.tolist(),
                                'scope':'Repeated offline idle inference, not closed-loop tracking or motion validation'}
    except Exception as error:
        report['planner_idle']={'status':'failed','error':str(error),'runs':samples}
        raise
    finally:
        output.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__': main()
