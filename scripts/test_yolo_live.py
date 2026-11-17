"""Live YOLO check on the G1 camera in Factory V2. Headless, manual mode, no patrol.

Scenes: the unmodified factory from the entrance, realistic 3D person / chair /
bottle props placed in front of the robot (test-only, physics disabled, loaded
from the Isaac Sim asset server), and the factory's own stylised operator after
a turn. Detections come from the RGB image only; no USD semantics are read.

Run: ./scripts/isaac_python.sh scripts/test_yolo_live.py
"""
from pathlib import Path
import json,math,sys,time
ROOT=Path(__file__).resolve().parents[1]; sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':960,'height':640,'renderer':'RaytracedLighting','anti_aliasing':1,'multi_gpu':False,'extra_args':[
    '--/app/vulkan='+str(sys.platform.startswith('linux')).lower(),
    '--/exts/isaacsim.physics.newton/auto_switch_on_startup=false',
    '--/exts/isaacsim.core.simulation_manager/default_engine=physx']})
runtime=None
PROPS='/World/YoloTestProps'
# name: (asset, [forward, left] offset from the camera in metres, heading deg, extra tilt deg about X)
ASSETS={'person':('/Isaac/People/Characters/male_adult_construction_05_new/male_adult_construction_05_new.usd',[4.5,.2],270.,0.),  # facing the camera
        'chair':('/Isaac/Environments/Office/Props/SM_Chair.usd',[2.3,1.05],200.,0.),
        'bottle':('/Isaac/Props/YCB/Axis_Aligned/006_mustard_bottle.usd',[1.15,-.4],0.,-90.)}  # stood upright, cap up
try:
    import numpy as np
    import omni.usd
    from pxr import Gf,Usd,UsdGeom,UsdPhysics
    from controllers.factory_g1_runtime import FactoryG1Runtime
    from controllers.factory_selector import load_factory
    from perception.rgbd_config import load_config as load_rgbd
    from perception.yolo_detector import YoloOnnx,annotate,load_config as load_yolo
    from test_keyboard import yaw
    cfg=load_factory('v2'); cfg['rgbd_camera']=load_rgbd({'enabled':True}); cfg['yolo']=load_yolo({'enabled':True})
    runtime=FactoryG1Runtime(app,cfg,self_test=True)
    yolo=runtime.yolo; stage=runtime.stage
    report={'status':'running','model':cfg['yolo']['model'],'imgsz':cfg['yolo']['imgsz'],'provider':yolo.detector.provider,
            'cuda_device_id':cfg['yolo']['cuda_device_id'],'run_directory':str(runtime.directory),'image_directory':str(yolo.directory),'tests':{}}
    def add(name,passed,**values):
        report['tests'][name]={'passed':None if passed is None else bool(passed),**values}
        print('YOLO_LIVE '+json.dumps({name:report['tests'][name]},default=float)[:1800],flush=True)
    results=[]; tick_ms=[]; commands=[]
    def advance(seconds):
        for _ in range(round(seconds*runtime.loco_cfg['control_hz'])):
            started=time.perf_counter(); app.update(); runtime.tick(); tick_ms.append((time.perf_counter()-started)*1000)
            if runtime.fault: raise RuntimeError('Robot fault during YOLO test: '+runtime.fault)
            commands.append(list(runtime.navigation.last_command))
            if yolo.result is not None and (not results or results[-1]['result_index']!=yolo.result['result_index']): results.append(yolo.result)
    def observe(seconds):
        """Detection rate and mean confidence per class over a time window."""
        start=len(results); advance(seconds); window=results[start:]; summary={}
        for name in sorted({d['class_name'] for r in window for d in r['detections']}):
            hits=[max(d['confidence'] for d in r['detections'] if d['class_name']==name) for r in window if any(d['class_name']==name for d in r['detections'])]
            summary[name]={'detection_rate':len(hits)/len(window),'mean_confidence':float(np.mean(hits)),'max_confidence':float(max(hits))}
        return window,summary
    def release():
        for key in runtime.keys.mapping: runtime.inject(key,'RELEASE')
    def load_props(origin):
        """Reference test props, strip their physics, stand them on the floor (or a pedestal)."""
        from isaacsim.storage.native import get_assets_root_path
        root=get_assets_root_path(); UsdGeom.Xform.Define(stage,PROPS); info={}
        pedestal=UsdGeom.Cube.Define(stage,PROPS+'/Pedestal'); pedestal.CreateSizeAttr(1)
        pedestal.AddTranslateOp().Set(Gf.Vec3d(origin[0]+ASSETS['bottle'][1][0],origin[1]+ASSETS['bottle'][1][1],.4)); pedestal.AddScaleOp().Set(Gf.Vec3d(.3,.3,.8))
        pedestal.CreateDisplayColorAttr([Gf.Vec3f(.55,.55,.6)])
        for name,(asset,offset,heading,tilt) in ASSETS.items():
            url=root+asset; source=Usd.Stage.Open(url)
            scale=UsdGeom.GetStageMetersPerUnit(source); up=UsdGeom.GetStageUpAxis(source)
            # The asset keeps its own xform ops on a child prim; placement ops live on the parent.
            prim=UsdGeom.Xform.Define(stage,f'{PROPS}/{name}').GetPrim()
            UsdGeom.Xform.Define(stage,f'{PROPS}/{name}/asset').GetPrim().GetReferences().AddReference(url)
            xform=UsdGeom.Xformable(prim); translate=xform.AddTranslateOp(); xform.AddRotateZOp().Set(heading)
            if up=='Y' or tilt: xform.AddRotateXOp().Set((90. if up=='Y' else 0.)+tilt)
            xform.AddScaleOp().Set(Gf.Vec3d(scale,scale,scale))
            info[name]={'asset':asset,'meters_per_unit':scale,'up_axis':up,'translate':translate,'prim':prim,'offset':offset}
        for _ in range(600):
            app.update()
            if omni.usd.get_context().get_stage_loading_status()[2]==0: break
        for name,item in info.items():
            for p in Usd.PrimRange(item['prim'],Usd.TraverseInstanceProxies()):
                if p.IsInstanceProxy(): continue
                if p.HasAPI(UsdPhysics.RigidBodyAPI): UsdPhysics.RigidBodyAPI(p).CreateRigidBodyEnabledAttr(False)
                if p.HasAPI(UsdPhysics.CollisionAPI): UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr(False)
            bounds=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render'],False,True).ComputeWorldBound(item['prim']).ComputeAlignedRange()
            low,high=bounds.GetMin(),bounds.GetMax(); base=.8 if name=='bottle' else 0.
            target=[origin[0]+item['offset'][0],origin[1]+item['offset'][1]]
            item['translate'].Set(Gf.Vec3d(target[0]-(low[0]+high[0])/2,target[1]-(low[1]+high[1])/2,base-low[2]))
            item.update(size_m=[float(high[i]-low[i]) for i in range(3)],position_m=[*target,base],
                        distance_from_camera_m=float(math.hypot(*item['offset'])))
        return {name:{k:v for k,v in item.items() if k not in ('translate','prim','offset')} for name,item in info.items()}

    # ---- 1. unmodified factory from the entrance ---------------------------
    advance(3); window,summary=observe(3)
    yolo.save(window[-1],'factory_entrance')
    add('live_pipeline',len(window)>=10 and all(r['sim_time']<=runtime.steps/50 for r in window),results=len(window),
        provider=yolo.detector.provider,result_fields=sorted(k for k in window[-1] if k!='rgb'),
        example_detection=(window[-1]['detections'] or [None])[0],worker=yolo.worker.stats())
    add('factory_entrance_view',None,detections=summary,note='Informational: what the pretrained model reports on the stylised factory with no props')

    # ---- 2. realistic 3D props in front of the robot ------------------------
    origin=runtime.rgbd.world_pose()[:3,3]
    try: props=load_props(origin)
    except Exception as error: props=None; add('props_scene',None,skipped='Could not load Isaac assets: '+repr(error)[:300])
    if props is not None:
        advance(3); window,summary=observe(4); stem=yolo.save(window[-1],'props_person_chair_bottle')
        add('props_scene',all(summary.get(name,{}).get('detection_rate',0)>.8 for name in ('person','chair','bottle')),
            detections=summary,props=props,screenshot=str(stem)+'.png',imgsz=cfg['yolo']['imgsz'])
        # Same frame through the 320 export, to document the imgsz decision (main thread, test only).
        small=YoloOnnx(load_yolo({'imgsz':320,**cfg['yolo']['alternatives']['imgsz_320']}))
        detections,timing=small.detect(window[-1]['rgb'])
        from PIL import Image
        Image.fromarray(annotate(window[-1]['rgb'],detections,'YOLO11n 320 | same frame')).save(str(yolo.directory/'props_person_chair_bottle_320.png'))
        add('props_scene_imgsz_320',None,detections=[(d['class_name'],round(d['confidence'],3)) for d in detections],inference_ms=timing['inference_ms'])
        # Bottle close to the camera: the object must be large enough in a 320 input.
        low=UsdGeom.Xformable(stage.GetPrimAtPath(PROPS+'/bottle')); pedestal=UsdGeom.Xformable(stage.GetPrimAtPath(PROPS+'/Pedestal'))
        shift=Gf.Vec3d(-.45,.35,0)
        for item in (low,pedestal):
            op=item.GetOrderedXformOps()[0]; op.Set(op.Get()+shift)
        advance(1); window,summary=observe(3); yolo.save(window[-1],'bottle_closeup')
        add('bottle_closeup',None,detections=summary,bottle_detected='bottle' in summary,distance_from_camera_m=.7)
        stage.RemovePrim(PROPS); advance(3)

    # ---- 3. inference never blocks the physics loop -------------------------
    baseline=len(tick_ms); advance(4); normal=tick_ms[baseline:]
    original=yolo.detector.detect; dropped_before=yolo.worker.dropped
    def slow(rgb):
        time.sleep(.35); return original(rgb)   # a detector 200x slower than measured
    yolo.detector.detect=slow; mark=len(tick_ms); advance(4); stressed=tick_ms[mark:]; yolo.detector.detect=original
    add('non_blocking',np.median(stressed)<np.median(normal)+2. and np.percentile(stressed,95)<np.percentile(normal,95)+5. and yolo.worker.dropped>dropped_before,
        tick_ms_median_normal=float(np.median(normal)),tick_ms_p95_normal=float(np.percentile(normal,95)),
        tick_ms_median_with_350ms_detector=float(np.median(stressed)),tick_ms_p95_with_350ms_detector=float(np.percentile(stressed,95)),
        frames_dropped_while_slow=yolo.worker.dropped-dropped_before,
        meaning='With inference artificially slowed to 350 ms the simulation tick time is unchanged; stale frames are dropped, not queued')
    advance(1)

    # ---- 4. the factory's own stylised operator, after a turn ----------------
    still=[c for c in commands]
    runtime.inject('A')
    for _ in range(175):
        app.update(); runtime.tick()
        if runtime.fault: raise RuntimeError('Robot fault while turning: '+runtime.fault)
    release(); advance(2.5); window,summary=observe(3); yolo.save(window[-1],'factory_operator_after_turn')
    add('factory_operator_view',None,detections=summary,robot_yaw_rad=yaw(runtime.last_state['base_orientation_wxyz']),
        person_detected='person' in summary,note='Informational: low-poly blocky operator about 2.3 m ahead')

    # ---- 5. output only: no commands, no patrol; survives a robot reset -------
    nav=runtime.navigation
    add('no_commands_from_yolo',all(c==[0.,0.,0.] for c in still) and nav.mode=='MANUAL' and nav.scheduler.state in ('IDLE','STOPPED')
        and sum(len(r['detections']) for r in results)>0,
        ticks_with_detections_running=len(still),nonzero_commands=sum(c!=[0.,0.,0.] for c in still),navigation_mode=nav.mode,
        scheduler_state=nav.scheduler.state,total_detections=sum(len(r['detections']) for r in results),
        meaning='While YOLO was producing detections and no key was held, the velocity command stayed exactly zero')
    runtime.timeline.pause(); app.update(); count=len(results)
    reset_ok=runtime.safe_reset(source='yolo_live_test'); runtime.timeline.play(); app.update(); runtime.keys.set_running(True)
    advance(3)
    add('reset_keeps_running',reset_ok and len(results)>count+5,new_results_after_reset=len(results)-count)
    report['worker']=yolo.worker.stats(); report['results_logged']=yolo.logged
    report['failed_tests']=[k for k,v in report['tests'].items() if v['passed'] is False]
    report['status']='passed' if not report['failed_tests'] else 'failed'
    payload=json.dumps(report,indent=2,default=float)
    (yolo.directory/'yolo_live_results.json').write_text(payload)
    (ROOT/'docs/perception').mkdir(parents=True,exist_ok=True); (ROOT/'docs/perception/yolo_live_results.json').write_text(payload)
    print('YOLO_LIVE_RESULT '+json.dumps({'status':report['status'],'failed_tests':report['failed_tests'],'images':str(yolo.directory)}),flush=True)
finally:
    if runtime: runtime.close()
    app.close()
