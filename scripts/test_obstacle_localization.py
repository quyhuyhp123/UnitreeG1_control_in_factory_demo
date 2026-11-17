"""RGB-D obstacle localisation vs USD ground truth in Factory V2. Headless, manual mode.

Scenes
  A  person / chair / bottle-on-pedestal props (classes YOLO knows)
  B  the same props after the robot turns in place (transform check), and while turning
  C  plain shapes YOLO has no class for: box, cylinder, low pallet, thin pole
  D  the factory's own blocky operator after a large turn

USD world bounding boxes are read here for scoring only. The localiser itself
sees nothing but the RGB-D frame. No avoidance command is ever issued.

Run: ./scripts/isaac_python.sh scripts/test_obstacle_localization.py
"""
from pathlib import Path
import json,math,sys,traceback
ROOT=Path(__file__).resolve().parents[1]; sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':960,'height':640,'renderer':'RaytracedLighting','anti_aliasing':1,'multi_gpu':False,'extra_args':[
    '--/app/vulkan='+str(sys.platform.startswith('linux')).lower(),
    '--/exts/isaacsim.physics.newton/auto_switch_on_startup=false',
    '--/exts/isaacsim.core.simulation_manager/default_engine=physx']})
runtime=None
PROPS='/World/ObstacleTestProps'
# name: (asset, [forward, left] from the camera in metres, heading deg, tilt deg, base height)
ASSETS={'person':('/Isaac/People/Characters/male_adult_construction_05_new/male_adult_construction_05_new.usd',[4.5,.2],270.,0.,0.),
        'chair':('/Isaac/Environments/Office/Props/SM_Chair.usd',[2.3,1.05],180.,0.,0.),
        'bottle':('/Isaac/Props/YCB/Axis_Aligned/006_mustard_bottle.usd',[1.15,-.4],0.,-90.,.8)}
# name: (shape, size xyz or (radius, height), [forward, left], colour)
SHAPES={'box':('cube',(.5,.5,.5),[3.,1.4],(.55,.2,.15)),'cylinder':('cylinder',(.2,1.),[2.,-.5],(.2,.3,.6)),
        'low_pallet':('cube',(.8,.8,.15),[3.6,.1],(.5,.4,.25)),'thin_pole':('cylinder',(.03,1.2),[1.8,.55],(.8,.8,.1))}
try:
    import numpy as np
    import omni.usd
    from pxr import Gf,Usd,UsdGeom,UsdPhysics
    from controllers.factory_g1_runtime import FactoryG1Runtime
    from controllers.factory_selector import load_factory
    from perception.rgbd_config import load_config as load_rgbd
    from perception.yolo_config import load_config as load_yolo
    from perception.obstacle_localizer import load_config as load_obstacles
    from test_keyboard import yaw as quaternion_yaw
    cfg=load_factory('v2'); cfg['rgbd_camera']=load_rgbd({'enabled':True}); cfg['yolo']=load_yolo({'enabled':True})
    cfg['obstacle_localization']=load_obstacles({'enabled':True})
    runtime=FactoryG1Runtime(app,cfg,self_test=True)
    yolo=runtime.yolo; stage=runtime.stage
    report={'status':'running','run_directory':str(runtime.directory),'image_directory':str(yolo.obstacle_directory),
            'config':cfg['obstacle_localization'],'tests':{}}
    def add(name,passed,**values):
        report['tests'][name]={'passed':None if passed is None else bool(passed),**values}
        print('OBSTACLE_TEST '+json.dumps({name:report['tests'][name]},default=float)[:900],flush=True)
    results=[]; commands=[]
    def advance(seconds):
        for _ in range(round(seconds*runtime.loco_cfg['control_hz'])):
            app.update(); runtime.tick()
            if runtime.fault: raise RuntimeError('Robot fault during obstacle test: '+runtime.fault)
            commands.append((list(runtime.navigation.last_command),list(runtime.keys.command)))
            if yolo.result is not None and 'obstacles' in yolo.result and (not results or results[-1]['result_index']!=yolo.result['result_index']):
                results.append(yolo.result)
    def latest(seconds=1.5):
        advance(seconds); return results[-1]
    def release():
        for key in runtime.keys.mapping: runtime.inject(key,'RELEASE')

    # ---- ground truth helpers (evaluation only) ------------------------------
    def world_box(path):
        r=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render'],False,True).ComputeWorldBound(stage.GetPrimAtPath(path)).ComputeAlignedRange()
        return np.array(r.GetMin()),np.array(r.GetMax())
    def box_distance(point,low,high): return float(np.linalg.norm(np.maximum(np.maximum(low-point,0),point-high)))
    def truth(name,path,result,frame_camera,box=None):
        """Ground truth for one object, expressed for the robot pose of this very result."""
        low,high=box if box is not None else world_box(path); pose=result['robot_pose_world']; c,s=math.cos(pose['yaw_rad']),math.sin(pose['yaw_rad'])
        def to_robot(x,y): dx,dy=x-pose['x'],y-pose['y']; return [c*dx+s*dy,-s*dx+c*dy]
        corners=[to_robot(x,y) for x,y in [(low[0],low[1]),(high[0],low[1]),(high[0],high[1]),(low[0],high[1])]]
        centre=(low+high)/2
        return {'name':name,'world_box_low_m':low.tolist(),'world_box_high_m':high.tolist(),'centre_world_m':centre.tolist(),
            'centre_robot_m':to_robot(centre[0],centre[1]),'corners_robot_m':corners,
            'distance_robot_m':box_distance(np.array([pose['x'],pose['y']]),low[:2],high[:2]),
            'range_camera_m':box_distance(frame_camera,low,high),'size_m':(high-low).tolist()}
    def score(gt,obstacle):
        """Estimated obstacle vs ground truth. Position error is measured to the true box (0 if inside)."""
        if obstacle is None: return {'object':gt['name'],'found':False,'ground_truth_distance_robot_m':gt['distance_robot_m']}
        low,high=np.array(gt['world_box_low_m'][:2]),np.array(gt['world_box_high_m'][:2])
        position=np.array(obstacle['position_world_m'][:2])
        return {'object':gt['name'],'found':True,'class_name':obstacle['class_name'],'confidence':obstacle['confidence'],'source':obstacle['source'],
            'estimated_distance_robot_m':obstacle['distance_m'],'ground_truth_distance_robot_m':gt['distance_robot_m'],
            'distance_error_m':obstacle['distance_m']-gt['distance_robot_m'],
            'estimated_range_camera_m':obstacle['range_camera_m'],'ground_truth_range_camera_m':gt['range_camera_m'],
            'range_error_m':obstacle['range_camera_m']-gt['range_camera_m'],
            'position_robot_m':obstacle['position_robot_m'],'ground_truth_centre_robot_m':gt['centre_robot_m'],
            'position_world_m':obstacle['position_world_m'],'ground_truth_centre_world_m':gt['centre_world_m'],
            'position_error_to_true_box_m':box_distance(position,low,high),'bearing_deg':obstacle['bearing_deg'],
            'height_range_m':obstacle['height_range_m'],'ground_truth_height_m':gt['size_m'][2],'sim_time_s':obstacle['sim_time']}
    def match(gt,result,class_name=None):
        """Named objects by YOLO class; anything else by the obstacle nearest to the true box."""
        obstacles=result['obstacles']
        if class_name:
            named=[o for o in obstacles if o['class_name']==class_name]
            return max(named,key=lambda o:o['confidence']) if named else None
        low,high=np.array(gt['world_box_low_m'][:2]),np.array(gt['world_box_high_m'][:2])
        near=[(box_distance(np.array(o['position_world_m'][:2]),low,high),o) for o in obstacles]
        near=[item for item in near if item[0]<.3]
        return min(near,key=lambda item:item[0])[1] if near else None
    def camera_of(result): return np.array(result['_frame_camera'])

    # Keep the camera position that belongs to each result (the frame travels with the YOLO job).
    original=yolo._localize
    def localize(frame,detections):
        located=original(frame,detections); located['_frame_camera']=frame['T_world_optical'][:3,3].tolist(); return located
    yolo.worker.post=localize

    def load_props(origin):
        from isaacsim.storage.native import get_assets_root_path
        root=get_assets_root_path(); UsdGeom.Xform.Define(stage,PROPS); items={}
        pedestal=UsdGeom.Cube.Define(stage,PROPS+'/pedestal'); pedestal.CreateSizeAttr(1)
        pedestal.AddTranslateOp().Set(Gf.Vec3d(origin[0]+ASSETS['bottle'][1][0],origin[1]+ASSETS['bottle'][1][1],.4)); pedestal.AddScaleOp().Set(Gf.Vec3d(.3,.3,.8))
        pedestal.CreateDisplayColorAttr([Gf.Vec3f(.55,.55,.6)])
        for name,(asset,offset,heading,tilt,base) in ASSETS.items():
            url=root+asset; source=Usd.Stage.Open(url); scale=UsdGeom.GetStageMetersPerUnit(source); up=UsdGeom.GetStageUpAxis(source)
            prim=UsdGeom.Xform.Define(stage,f'{PROPS}/{name}').GetPrim()
            UsdGeom.Xform.Define(stage,f'{PROPS}/{name}/asset').GetPrim().GetReferences().AddReference(url)
            xform=UsdGeom.Xformable(prim); translate=xform.AddTranslateOp(); xform.AddRotateZOp().Set(heading)
            if up=='Y' or tilt: xform.AddRotateXOp().Set((90. if up=='Y' else 0.)+tilt)
            xform.AddScaleOp().Set(Gf.Vec3d(scale,scale,scale)); items[name]=(prim,translate,offset,base)
        for _ in range(600):
            app.update()
            if omni.usd.get_context().get_stage_loading_status()[2]==0: break
        for name,(prim,translate,offset,base) in items.items():
            for p in Usd.PrimRange(prim,Usd.TraverseInstanceProxies()):
                if p.IsInstanceProxy(): continue
                if p.HasAPI(UsdPhysics.RigidBodyAPI): UsdPhysics.RigidBodyAPI(p).CreateRigidBodyEnabledAttr(False)
                if p.HasAPI(UsdPhysics.CollisionAPI): UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr(False)
            low,high=world_box(str(prim.GetPath()))
            translate.Set(Gf.Vec3d(origin[0]+offset[0]-(low[0]+high[0])/2,origin[1]+offset[1]-(low[1]+high[1])/2,base-low[2]))
    def load_shapes(origin):
        UsdGeom.Xform.Define(stage,PROPS)
        for name,(shape,size,offset,colour) in SHAPES.items():
            if shape=='cube':
                prim=UsdGeom.Cube.Define(stage,f'{PROPS}/{name}'); prim.CreateSizeAttr(1)
                prim.AddTranslateOp().Set(Gf.Vec3d(origin[0]+offset[0],origin[1]+offset[1],size[2]/2)); prim.AddScaleOp().Set(Gf.Vec3d(*size))
            else:
                prim=UsdGeom.Cylinder.Define(stage,f'{PROPS}/{name}'); prim.CreateRadiusAttr(size[0]); prim.CreateHeightAttr(size[1]); prim.CreateAxisAttr('Z')
                prim.AddTranslateOp().Set(Gf.Vec3d(origin[0]+offset[0],origin[1]+offset[1],size[1]/2))
            prim.CreateDisplayColorAttr([Gf.Vec3f(*colour)])

    # ---- 0. empty entrance: floor and robot must not become obstacles ----------
    advance(3); result=latest(2); origin=runtime.rgbd.world_pose()[:3,3]
    close=[o for o in result['obstacles'] if o['distance_m']<1.]
    add('floor_and_self_rejected',not close and result['points']['floor']>10000,obstacles_within_1m=len(close),points=result['points'],
        obstacle_count=len(result['obstacles']),nearest_obstacle_m=min((o['distance_m'] for o in result['obstacles']),default=None),
        localization_ms=result['timing']['localization_ms'])
    add('synchronisation',result['sync']['yolo_frame_index']==result['sync']['depth_frame_index']==result['frame_index']
        and all(o['sim_time']==result['sim_time'] for o in result['obstacles']),sync=result['sync'],result_sim_time_s=result['sim_time'],
        meaning='RGB, depth, camera pose and YOLO boxes of one result all belong to the same camera frame')
    yolo.save_obstacles(result,'empty_entrance')

    # ---- A. props with classes YOLO knows --------------------------------------
    load_props(origin); advance(3); result=latest(2)
    paths={'person':PROPS+'/person','chair':PROPS+'/chair','bottle':PROPS+'/bottle','pedestal':PROPS+'/pedestal'}
    def evaluate(result,names):
        rows=[]; boxes=[]
        for name in names:
            gt=truth(name,paths[name],result,camera_of(result)); boxes.append(gt)
            rows.append(score(gt,match(gt,result,name if name in ASSETS else None)))
        return rows,boxes
    rows,boxes=evaluate(result,['person','chair','bottle','pedestal'])
    stem=yolo.save_obstacles(result,'A_named_props',boxes)
    found=[r for r in rows if r['found']]
    def within(r): return abs(r['distance_error_m'])<(.25 if r['object']=='person' else .12) and r['position_error_to_true_box_m']<.1
    add('named_props_standing',len(found)==4 and all(within(r) for r in found),
        objects=rows,screenshot=str(stem),note='person tolerance 0.25 m: its true box starts at the toe tips, which are inside the floor band',max_abs_distance_error_m=max((abs(r['distance_error_m']) for r in found),default=None))

    # ---- B. transform under rotation --------------------------------------------
    before={r['object']:r for r in rows}; yaw0=result['robot_pose_world']['yaw_rad']
    runtime.inject('A'); moving=[]; mark=len(results)
    for _ in range(45):
        app.update(); runtime.tick()
        if yolo.result is not None and 'obstacles' in yolo.result and results[-1]['result_index']!=yolo.result['result_index']: results.append(yolo.result)
    release()
    for item in results[mark:]:   # frames taken while the robot was rotating
        for name in ('chair','person'):
            gt=truth(name,paths[name],item,camera_of(item)); row=score(gt,match(gt,item,name))
            if row['found']: moving.append({'object':name,'yaw_rad':item['robot_pose_world']['yaw_rad'],'distance_error_m':row['distance_error_m'],
                                           'position_error_to_true_box_m':row['position_error_to_true_box_m']})
    advance(2.5); result=latest(1.5); turned=result['robot_pose_world']['yaw_rad']-yaw0
    rows,boxes=evaluate(result,['person','chair'])
    stem=yolo.save_obstacles(result,'B_after_turn',boxes); shifts=[]
    for r in rows:
        if r['found']:
            old=before[r['object']]
            r['world_position_shift_m']=float(math.dist(r['position_world_m'][:2],old['position_world_m'][:2]))
            r['robot_frame_bearing_change_deg']=r['bearing_deg']-old['bearing_deg']; shifts.append(r['world_position_shift_m'])
    found=[r for r in rows if r['found']]
    add('transform_after_rotation',abs(turned)>.25 and len(found)==2 and all(within(r) and r['world_position_shift_m']<.15 for r in found),robot_yaw_change_rad=turned,robot_yaw_change_deg=math.degrees(turned),objects=rows,
        meaning='Objects are fixed in the world: after the robot turns, robot-frame bearings change by about -yaw while world positions stay put')
    add('transform_while_rotating',len(moving)>=4 and max(abs(m['distance_error_m']) for m in moving)<.3 and max(m['position_error_to_true_box_m'] for m in moving)<.15,
        samples=len(moving),max_abs_distance_error_m=max((abs(m['distance_error_m']) for m in moving),default=None),
        max_position_error_m=max((m['position_error_to_true_box_m'] for m in moving),default=None),detail=moving[:12])

    # ---- C. shapes YOLO has no class for ------------------------------------------
    stage.RemovePrim(PROPS); advance(1)
    runtime.inject('D')
    for _ in range(45): app.update(); runtime.tick()
    release(); advance(2.5)
    origin=runtime.rgbd.world_pose(); forward=-origin[:3,2]; heading=math.atan2(forward[1],forward[0]); c,s=math.cos(heading),math.sin(heading)
    local=dict(SHAPES)
    for name,(shape,size,offset,colour) in local.items():   # place along the camera's current heading
        SHAPES[name]=(shape,size,[c*offset[0]-s*offset[1]+0.,s*offset[0]+c*offset[1]+0.],colour)
    load_shapes(origin[:3,3]); advance(2); result=latest(2)
    paths={name:f'{PROPS}/{name}' for name in SHAPES}; rows=[]; boxes=[]
    for name in SHAPES:
        gt=truth(name,paths[name],result,camera_of(result)); boxes.append(gt); row=score(gt,match(gt,result)); rows.append(row)
    stem=yolo.save_obstacles(result,'C_unknown_shapes',boxes)
    required=[r for r in rows if r['object'] in ('box','cylinder','low_pallet')]
    add('unknown_shapes_depth_only',all(r['found'] and abs(r['distance_error_m'])<.12 and r['position_error_to_true_box_m']<.1 for r in required),
        objects=rows,yolo_detections=[(d['class_name'],round(d['confidence'],2)) for d in result['detections']],screenshot=str(stem),
        note='box, cylinder and low pallet are required; the 6 cm pole is reported as a sensor-limit probe')
    stage.RemovePrim(PROPS); advance(1)

    # ---- D. the factory's own operator (YOLO does not recognise it) -----------------
    humans=[o for o in runtime.navigation.detector.obstacles if o['kind']=='human']
    def centre_of(o): return (np.array(o['bounds'][0])+np.array(o['bounds'][1]))/2
    here=np.array(runtime.last_state['base_position_m'][:2])
    operator=min(humans,key=lambda o:np.linalg.norm(centre_of(o)-here))
    target=math.atan2(*(centre_of(operator)-here)[::-1])
    def heading_error(): return (target-quaternion_yaw(runtime.last_state['base_orientation_wxyz'])+math.pi)%(2*math.pi)-math.pi
    key='A' if heading_error()>0 else 'D'; runtime.inject(key)   # turn in place until the operator is ahead
    for _ in range(600):
        app.update(); runtime.tick()
        if abs(heading_error())<.12: break
    release(); advance(2.5); result=latest(2)
    low=np.array([*operator['bounds'][0],operator['height_range_m'][0]]); high=np.array([*operator['bounds'][1],operator['height_range_m'][1]])
    gt=truth(operator['id'].split('/')[-1],None,result,camera_of(result),(low,high)); row=score(gt,match(gt,result))
    stem=yolo.save_obstacles(result,'D_factory_operator',[gt])
    add('factory_operator_depth_only',row['found'] and abs(row['distance_error_m'])<.15,object=row,heading_error_rad=heading_error(),
        yolo_detections=[(d['class_name'],round(d['confidence'],2)) for d in result['detections']],screenshot=str(stem),
        ground_truth='existing SIM GROUND TRUTH collider box of the nearest factory operator (evaluation only)')

    # ---- output only -------------------------------------------------------------------
    # The only motion in this test is the test's own key presses (turning in place). On every tick the
    # applied command must equal the keyboard controller's command: nothing else may contribute.
    nav=runtime.navigation; foreign=sum(applied!=keyboard for applied,keyboard in commands)
    add('no_avoidance_commands',foreign==0 and nav.mode=='MANUAL' and nav.scheduler.state in ('IDLE','STOPPED')
        and all(c[0][:2]==[0.,0.] for c in commands),
        ticks_checked=len(commands),ticks_where_command_differs_from_keyboard=foreign,
        ticks_with_linear_velocity=sum(c[0][:2]!=[0.,0.] for c in commands),navigation_mode=nav.mode,scheduler_state=nav.scheduler.state,
        navigation_obstacle_source=nav.detector.mode,
        meaning='Applied command == keyboard command on every tick, never any linear velocity; navigation still uses its own obstacle source')
    timings=[r['timing']['localization_ms'] for r in results]
    report['localization_ms']={'mean':float(np.mean(timings)),'p95':float(np.percentile(timings,95)),'max':float(max(timings)),'thread':'YOLO inference thread'}
    report['worker']=yolo.worker.stats(); report['results']=len(results)
    report['failed_tests']=[k for k,v in report['tests'].items() if v['passed'] is False]
    report['status']='passed' if not report['failed_tests'] else 'failed'
    payload=json.dumps(report,indent=2,default=float)
    (yolo.obstacle_directory/'obstacle_localization_results.json').write_text(payload)
    (ROOT/'docs/perception').mkdir(parents=True,exist_ok=True); (ROOT/'docs/perception/obstacle_localization_results.json').write_text(payload)
    print('OBSTACLE_RESULT '+json.dumps({'status':report['status'],'failed_tests':report['failed_tests'],'images':str(yolo.obstacle_directory),
                                         'localization_ms':report['localization_ms']}),flush=True)
except BaseException:
    traceback.print_exc(); sys.stdout.flush(); sys.stderr.flush()  # Kit may exit before Python reports it
    raise
finally:
    if runtime: runtime.close()
    app.close()
