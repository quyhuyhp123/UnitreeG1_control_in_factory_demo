from pathlib import Path
import json,hashlib
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':640,'height':480,'renderer':'RaytracedLighting','multi_gpu':False})
try:
    import omni.kit.app
    omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate('isaacsim.pip.onnx',True)
    import onnxruntime as ort
    import numpy as np
    import yaml
    root=Path(r'D:\unitree_g1')
    folder=root/'checkpoints/unitree_g1_velocity_v0'
    deploy=yaml.safe_load((folder/'deploy.yaml').read_text(encoding='utf-8'))
    (folder/'deploy.json').write_text(json.dumps(deploy,indent=2),encoding='utf-8')
    opts=ort.SessionOptions()
    opts.intra_op_num_threads=2
    session=ort.InferenceSession(str(folder/'policy.onnx'),sess_options=opts,providers=['CPUExecutionProvider'])
    inputs=[{'name':v.name,'shape':v.shape,'type':v.type} for v in session.get_inputs()]
    outputs=[{'name':v.name,'shape':v.shape,'type':v.type} for v in session.get_outputs()]
    expected=sum(len(v['scale'])*v['history_length'] for v in deploy['observations'].values())
    assert len(inputs)==1 and inputs[0]['shape']==[1,expected] and expected==480
    assert outputs[0]['shape']==[1,29]
    observation=[]
    for name,term in deploy['observations'].items():
        row=np.zeros(len(term['scale']),dtype=np.float32)
        if name=='projected_gravity': row[2]=-1
        observation.extend(np.tile(row,term['history_length']).tolist())
    out=session.run(None,{inputs[0]['name']:np.array([observation],dtype=np.float32)})[0]
    assert np.isfinite(out).all()
    report={'status':'inference_verified_not_simulation_tested','onnxruntime':ort.__version__,'providers':session.get_providers(),
            'inputs':inputs,'outputs':outputs,'sha256':hashlib.sha256((folder/'policy.onnx').read_bytes()).hexdigest(),
            'bytes':(folder/'policy.onnx').stat().st_size,'metadata':session.get_modelmeta().custom_metadata_map,
            'nominal_observation_action':out.tolist(),'observation_dim':expected}
    try:
        import onnx
        model=onnx.load(str(folder/'policy.onnx'))
        onnx.checker.check_model(model)
        report['onnx_checker']='passed'
        report['graph_nodes']=[{'type':n.op_type,'name':n.name} for n in model.graph.node]
        report['initializers']=[{'name':i.name,'shape':list(i.dims)} for i in model.graph.initializer]
    except ImportError: report['onnx_checker']='onnx package unavailable; ORT loaded and inferred'
    (root/'logs/locomotion_policy_verification.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2),flush=True)
finally: app.close()
