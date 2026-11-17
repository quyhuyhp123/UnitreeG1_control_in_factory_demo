"""Local costmap check in Factory V2: standing, test obstacles, turning, stale data, slow walk.

Headless, manual mode. The robot only moves through the test's own key presses
(turn in place, walk forward slowly). USD bounding boxes are read for scoring
only. The costmap is never connected to navigation and no avoidance is issued.

Run: ./scripts/isaac_python.sh scripts/test_local_costmap.py
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
PROPS='/World/CostmapTestProps'
# name: (asset, [forward, left] from the robot in metres, heading deg)
ASSETS={'person':('/Isaac/People/Characters/male_adult_construction_05_new/male_adult_construction_05_new.usd',[2.8,-.1],270.),
        'chair':('/Isaac/Environments/Office/Props/SM_Chair.usd',[1.9,1.4],180.)}
try:
    import numpy as np
    import omni.usd
    from pxr import Gf,Usd,UsdGeom,UsdPhysics
    from controllers.factory_g1_runtime import FactoryG1Runtime
    from controllers.factory_selector import load_factory
    from perception.rgbd_config import load_config as load_rgbd
    from perception.yolo_config import load_config as load_yolo
    from perception.obstacle_localizer import load_config as load_obstacles
    from perception.local_costmap import load_config as load_costmap,UNKNOWN,FREE,OCCUPIED,INSCRIBED,LETHAL
    from test_keyboard import yaw as quaternion_yaw
    cfg=load_factory('v2'); cfg['rgbd_camera']=load_rgbd({'enabled':True}); cfg['yolo']=load_yolo({'enabled':True})
    cfg['obstacle_localization']=load_obstacles({'enabled':True}); cfg['local_costmap']=load_costmap({'enabled':True})
    runtime=FactoryG1Runtime(app,cfg,self_test=True)
    yolo=runtime.yolo; stage=runtime.stage; costmap=yolo.costmap; res=costmap.res
    report={'status':'running','run_directory':str(runtime.directory),'image_directory':str(yolo.costmap_directory),'config':cfg['local_costmap'],
        'grid':{'size_m':costmap.n*res,'cells':[costmap.n,costmap.n],'resolution_m':res,'frame':'world-aligned, robot-centred rolling window',
                'inscribed_radius_m':costmap.inscribed_radius,'inflation_radius_m':costmap.inflation_radius,'human_extra_m':costmap.human_extra},'tests':{}}
    def add(name,passed,**values):
        report['tests'][name]={'passed':None if passed is None else bool(passed),**values}
        print('COSTMAP_TEST '+json.dumps({name:report['tests'][name]},default=float)[:700],flush=True)
    results=[]; commands=[]; violations=[]; update_ms=[]
    def check(result):
        """Rules that must hold for every single map."""
        m=result['_costmap']; rx,ry=m['robot_xy']
        if (m['traversable']&m['static_lethal']).any(): violations.append('traversable cell inside a factory no-go area')
        if (m['traversable']&(m['state']!=FREE)).any(): violations.append('traversable cell that was not observed free')
        if (m['cost'][m['static_lethal']]!=LETHAL).any(): violations.append('factory no-go cell not lethal')
        if (m['cost'][m['state']==OCCUPIED]!=LETHAL).any(): violations.append('occupied cell not lethal')
        centre=[m['origin_world_m'][0]+costmap.n*res/2,m['origin_world_m'][1]+costmap.n*res/2]
        if abs(centre[0]-rx)>res+1e-6 or abs(centre[1]-ry)>res+1e-6: violations.append('window not centred on the robot')
        update_ms.append(m['update_ms'])
    def step():
        app.update(); runtime.tick()
        if runtime.fault: raise RuntimeError('Robot fault during costmap test: '+runtime.fault)
        commands.append((list(runtime.navigation.last_command),list(runtime.keys.command)))
        if yolo.result is not None and '_costmap' in yolo.result and (not results or results[-1]['result_index']!=yolo.result['result_index']):
            results.append(yolo.result); check(yolo.result)
    def advance(seconds):
        for _ in range(round(seconds*runtime.loco_cfg['control_hz'])): step()
        return results[-1]
    def release():
        for key in runtime.keys.mapping: runtime.inject(key,'RELEASE')
    def heading(): return quaternion_yaw(runtime.last_state['base_orientation_wxyz'])
    def turn_to(target,tolerance=.08):
        error=lambda:(target-heading()+math.pi)%(2*math.pi)-math.pi
        runtime.inject('A' if error()>0 else 'D')
        for _ in range(900):
            step()
            if abs(error())<tolerance: break
        release(); return advance(2.5)
    def at(m,x,y):
        index=costmap.cell(m,x,y)
        return None if index is None else {'state':int(m['state'][index]),'cost':int(m['cost'][index]),'traversable':bool(m['traversable'][index]),
                                           'static':bool(m['static_lethal'][index]),'class_id':int(m['class_id'][index])}
    def ahead(m,yaw,forward,left=0.):
        return m['robot_xy'][0]+math.cos(yaw)*forward-math.sin(yaw)*left,m['robot_xy'][1]+math.sin(yaw)*forward+math.cos(yaw)*left
    def world_box(path):
        r=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render'],False,True).ComputeWorldBound(stage.GetPrimAtPath(path)).ComputeAlignedRange()
        return np.array(r.GetMin()),np.array(r.GetMax())
    def cells_in(m,low,high,margin=.1):
        """Boolean mask of map cells whose centre lies in the (expanded) world rectangle."""
        centres=(np.arange(costmap.n)+.5)*res; x=m['origin_world_m'][0]+centres[:,None]; y=m['origin_world_m'][1]+centres[None,:]
        return (x>=low[0]-margin)&(x<=high[0]+margin)&(y>=low[1]-margin)&(y<=high[1]+margin)
    def occupied_world_cells(m,low,high):
        mask=cells_in(m,low,high)&(m['state']==OCCUPIED); ix,iy=np.nonzero(mask)
        return {(int(a)+m['origin_cell'][0],int(b)+m['origin_cell'][1]) for a,b in zip(ix,iy)}
    def box_distance(point,low,high): return float(np.linalg.norm(np.maximum(np.maximum(np.array(low[:2])-point,0),point-np.array(high[:2]))))
    def score(name,path,m,expected_class):
        low,high=world_box(path); cells=occupied_world_cells(m,low,high); robot=np.array(m['robot_xy'])
        truth=box_distance(robot,low,high)
        if not cells: return {'object':name,'found':False,'ground_truth_distance_m':truth}
        centres=np.array([[(a+.5)*res,(b+.5)*res] for a,b in cells]); nearest=float(np.min(np.linalg.norm(centres-robot,axis=1)))
        classes=m['class_id'][cells_in(m,low,high)&(m['state']==OCCUPIED)]; values,counts=np.unique(classes,return_counts=True)
        majority=int(values[counts.argmax()])
        return {'object':name,'found':True,'occupied_cells':len(cells),'nearest_occupied_cell_m':nearest,'ground_truth_distance_m':truth,
            'distance_error_m':nearest-truth,'majority_class_id':majority,'class_name':yolo.detector.names.get(majority,'unknown'),
            'expected_class':expected_class,'class_correct':yolo.detector.names.get(majority,'unknown')==expected_class,
            'ground_truth_box_m':[low[:2].tolist(),high[:2].tolist()]}
    def truth_boxes(paths): return [{'name':n,'low':world_box(p)[0][:2].tolist(),'high':world_box(p)[1][:2].tolist()} for n,p in paths.items()]
    def add_box(name,position,size,colour=(.55,.2,.15)):
        UsdGeom.Xform.Define(stage,PROPS)
        prim=UsdGeom.Cube.Define(stage,f'{PROPS}/{name}'); prim.CreateSizeAttr(1)
        prim.AddTranslateOp().Set(Gf.Vec3d(position[0],position[1],size[2]/2)); prim.AddScaleOp().Set(Gf.Vec3d(*size))
        prim.CreateDisplayColorAttr([Gf.Vec3f(*colour)]); return f'{PROPS}/{name}'
    def load_assets(robot_xy,yaw):
        from isaacsim.storage.native import get_assets_root_path
        root=get_assets_root_path(); UsdGeom.Xform.Define(stage,PROPS); items={}
        for name,(asset,offset,turn) in ASSETS.items():
            url=root+asset; source=Usd.Stage.Open(url); scale=UsdGeom.GetStageMetersPerUnit(source); up=UsdGeom.GetStageUpAxis(source)
            prim=UsdGeom.Xform.Define(stage,f'{PROPS}/{name}').GetPrim()
            UsdGeom.Xform.Define(stage,f'{PROPS}/{name}/asset').GetPrim().GetReferences().AddReference(url)
            xform=UsdGeom.Xformable(prim); translate=xform.AddTranslateOp(); xform.AddRotateZOp().Set(turn+math.degrees(yaw))
            if up=='Y': xform.AddRotateXOp().Set(90.)
            xform.AddScaleOp().Set(Gf.Vec3d(scale,scale,scale)); items[name]=(prim,translate,offset)
        for _ in range(600):
            app.update()
            if omni.usd.get_context().get_stage_loading_status()[2]==0: break
        for name,(prim,translate,offset) in items.items():
            for p in Usd.PrimRange(prim,Usd.TraverseInstanceProxies()):
                if p.IsInstanceProxy(): continue
                if p.HasAPI(UsdPhysics.RigidBodyAPI): UsdPhysics.RigidBodyAPI(p).CreateRigidBodyEnabledAttr(False)
                if p.HasAPI(UsdPhysics.CollisionAPI): UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr(False)
            low,high=world_box(str(prim.GetPath()))
            x=robot_xy[0]+math.cos(yaw)*offset[0]-math.sin(yaw)*offset[1]; y=robot_xy[1]+math.sin(yaw)*offset[0]+math.cos(yaw)*offset[1]
            translate.Set(Gf.Vec3d(x-(low[0]+high[0])/2,y-(low[1]+high[1])/2,-low[2]))

    # ---- 1. standing at the entrance, nothing added ------------------------------
    result=advance(5); m=result['_costmap']; yaw0=result['robot_pose_world']['yaw_rad']; stats=result['costmap']['cells']
    behind=at(m,*ahead(m,yaw0,-1.5)); feet=at(m,*ahead(m,yaw0,.8)); front=at(m,*ahead(m,yaw0,2.4)); here=at(m,*m['robot_xy'])
    outside=at(m,m['robot_xy'][0],-15.4)   # south of the building outline (allowed area ends at y = -15)
    add('standing_states',m['state'].shape==(60,60) and behind['state']==UNKNOWN and not behind['traversable'] and feet['state']==UNKNOWN
        and front['state']==FREE and front['traversable'] and here['state']==FREE and outside['static'] and outside['cost']==LETHAL and not violations,
        cells=stats,behind_robot_1_5m=behind,blind_zone_0_8m_ahead=feet,ahead_2_4m=front,under_robot=here,outside_building_south=outside,
        meaning='Unseen areas (behind, at the feet) are UNKNOWN and not traversable; seen floor is FREE; outside the walkable area is LETHAL')
    yolo.save_costmap(result,'1_standing_empty')

    # ---- 2a. one plain box on a fresh map: inflation rings and the shadow behind it -------
    robot=np.array(m['robot_xy']); box_xy=ahead(m,yaw0,2.0,.9); box=add_box('box',box_xy,(.5,.5,.5))
    advance(.6); costmap.reset(); result=advance(2.5); m=result['_costmap']   # fresh map, so nothing is remembered from before the box
    low,high=world_box(box); row=score('box',box,m,'unknown')
    # The robot faces +X: probe sideways from the box's right (low-Y) face, over floor the camera sees.
    probes={f'{d:.2f} m beside box':at(m,box_xy[0],low[1]-d) for d in (.20,.47,.90)}
    shadow=at(m,high[0]+.35,box_xy[1]); values=list(probes.values())
    stem=yolo.save_costmap(result,'2a_box_inflation',truth_boxes({'box':box}))
    add('inflation_and_shadow',row['found'] and abs(row['distance_error_m'])<.15 and values[0]['cost']>=INSCRIBED and 0<values[1]['cost']<INSCRIBED
        and values[2]['cost']==0 and values[2]['traversable'] and shadow['state']==UNKNOWN and not shadow['traversable'],
        box=row,inflation_probes=probes,shadow_behind_box=shadow,screenshot=str(stem),cells=result['costmap']['cells'],
        meaning='0.20 m: inside the 0.35 m inscribed radius; 0.47 m: inflation ring; 0.90 m: clear and traversable. Floor hidden behind the box is UNKNOWN')
    stage.RemovePrim(box); advance(1)

    # ---- 2b. person (YOLO), chair (YOLO) and a crate with no class ---------------------------
    load_assets(robot,yaw0)
    box_xy=ahead(m,yaw0,2.2,-.75); paths={'person':PROPS+'/person','chair':PROPS+'/chair','box':add_box('crate',box_xy,(.4,.4,.4))}
    result=advance(4); m=result['_costmap']
    rows=[score('person',paths['person'],m,'person'),score('chair',paths['chair'],m,'chair'),score('crate',paths['box'],m,'unknown')]
    low,high=world_box(paths['box'])
    person_low,person_high=world_box(paths['person'])
    # Probe 0.75 m in front of the nearest cell labelled 'person', on the row through the person's centre
    # (the crate stands inside the person's arm span, so the true box alone would pick up crate cells).
    centre_y=(person_low[1]+person_high[1])/2; row_index=costmap.cell(m,m['robot_xy'][0],centre_y)[1]
    columns=np.nonzero((m['state'][:,row_index]==OCCUPIED)&(m['class_id'][:,row_index]==0))[0]
    body_x=m['origin_world_m'][0]+(columns.min()+.5)*res if len(columns) else None
    person_probe=at(m,body_x-.75,centre_y) if body_x is not None else None
    stem=yolo.save_costmap(result,'2b_person_chair_crate',truth_boxes(paths))
    # The person's true box starts at the toe tips, which lie inside the floor band: allow 0.25 m there.
    add('test_obstacles',all(r['found'] and abs(r['distance_error_m'])<(.25 if r['object']=='person' else .15) and r['class_correct'] for r in rows)
        and person_probe is not None and 0<person_probe['cost']<INSCRIBED,
        objects=rows,person_clearance_probe_0_75m=person_probe,screenshot=str(stem),cells=result['costmap']['cells'],
        meaning='A cell 0.75 m in front of the person still carries cost (0.57 m inflation + 0.30 m human margin)')

    # ---- 3. an obstacle that disappears while in view is cleared ----------------------
    chair_low,chair_high=world_box(paths['chair']); before=len(occupied_world_cells(m,chair_low,chair_high))
    stage.RemovePrim(paths['chair']); result=advance(1.5); m=result['_costmap']
    after=len(occupied_world_cells(m,chair_low,chair_high)); centre=at(m,(chair_low[0]+chair_high[0])/2,(chair_low[1]+chair_high[1])/2)
    add('removed_obstacle_cleared',before>0 and after==0 and centre['state']==FREE,occupied_cells_before=before,occupied_cells_after=after,cell_at_old_chair=centre)

    # ---- 4. turning: the map stays put in the world, out-of-view obstacles are remembered -----
    box_before=occupied_world_cells(m,low,high); free_before=m['state']==FREE
    result=turn_to(yaw0+math.radians(18)); m=result['_costmap']; box_small_turn=occupied_world_cells(m,low,high)
    iou=len(box_before&box_small_turn)/max(1,len(box_before|box_small_turn))
    result=turn_to(yaw0+math.radians(75)); m=result['_costmap']; yaw1=result['robot_pose_world']['yaw_rad']; seen_until=result['sim_time']
    box_out_of_view=occupied_world_cells(m,low,high); newly_free=int(((m['state']==FREE)&~free_before).sum())
    kept=len(box_before&box_out_of_view)/max(1,len(box_before))
    stem=yolo.save_costmap(result,'4_after_turn_box_remembered',truth_boxes({'box':paths['box'],'person':paths['person']}))
    add('turning',iou>=.6 and kept>=.7 and newly_free>50 and not violations,
        robot_yaw_change_deg=math.degrees(yaw1-yaw0),box_cells_iou_after_18_deg=iou,box_cells_kept_when_out_of_view=kept,
        cells_newly_seen_free=newly_free,screenshot=str(stem),
        meaning='Occupied cells of the box are the same world cells before and after turning; once out of view they are kept from memory')

    # ---- 5. stale data is forgotten: becomes UNKNOWN, never FREE --------------------------------
    box_last_seen=max(r['sim_time'] for r in results if box_xy and any(o['distance_m']<3.5 and abs(o['position_world_m'][0]-box_xy[0])<.5
        and abs(o['position_world_m'][1]-box_xy[1])<.5 for o in r['obstacles']))
    stage.RemovePrim(PROPS)                      # nobody can see this happen: the robot is looking away
    result=advance(max(0.,box_last_seen+cfg['local_costmap']['occupied_memory_s']-result['sim_time'])-1.5); m=result['_costmap']
    still=len(occupied_world_cells(m,low,high))
    result=advance(3.5); m=result['_costmap']; cell=at(m,(low[0]+high[0])/2,(low[1]+high[1])/2)
    gone=len(occupied_world_cells(m,low,high)); person_gone=len(occupied_world_cells(m,person_low,person_high))
    yolo.save_costmap(result,'5_stale_forgotten')
    add('stale_data_timeout',still>0 and gone==0 and cell['state']==UNKNOWN and not cell['traversable'] and person_gone==0,
        box_last_seen_sim_time_s=box_last_seen,occupied_memory_s=cfg['local_costmap']['occupied_memory_s'],box_cells_just_before_timeout=still,
        box_cells_after_timeout=gone,cell_at_old_box=cell,age_checked_s=result['sim_time']-box_last_seen,
        meaning='Out of view, the box is remembered for the memory time, then its cells become UNKNOWN (not FREE)')

    # ---- 6. slow walk: the window follows the robot --------------------------------------------
    result=turn_to(yaw0,.05); m=result['_costmap']; start=np.array(m['robot_xy']); yaw=result['robot_pose_world']['yaw_rad']
    far_xy=ahead(m,yaw,4.2); far=add_box('far_box',far_xy,(.5,.5,.6),(.2,.3,.6)); advance(1)
    limit=float(runtime.keys.high[0]); runtime.keys.high[0]=.3   # test only: cap the keyboard's forward speed at 0.3 m/s
    first=len(results); runtime.inject('W'); speeds=[]
    for _ in range(330):
        step(); speeds.append(math.hypot(*runtime.last_state['base_linear_velocity_world_m_s'][:2]))
    release(); runtime.keys.high[0]=limit; result=advance(3); m=result['_costmap']; yaw=result['robot_pose_world']['yaw_rad']
    walked=float(np.linalg.norm(np.array(m['robot_xy'])-start)); far_low,far_high=world_box(far)
    row=score('far_box',far,m,'unknown')
    zone=[at(m,*ahead(m,yaw,d,side)) for d in (.6,.8,1.0,1.2) for side in (-.2,0.,.2)]
    remembered=sum(c['state']==FREE for c in zone)/len(zone)
    under=[at(r['_costmap'],*r['_costmap']['robot_xy']) for r in results[first:]]
    stem=yolo.save_costmap(result,'6_after_slow_walk',truth_boxes({'far_box':far}))
    add('slow_walk',walked>1. and row['found'] and abs(row['distance_error_m'])<.15 and remembered>=.75
        and all(c['state']==FREE and c['cost']<INSCRIBED for c in under) and not violations,
        distance_walked_m=walked,commanded_speed_limit_m_s=.3,peak_measured_speed_m_s=float(max(speeds)),mean_measured_speed_m_s=float(np.mean(speeds[100:])),
        far_box=row,blind_zone_cells_free_from_memory=remembered,maps_during_walk=len(under),window_origin_world_m=m['origin_world_m'],
        robot_xy=m['robot_xy'],screenshot=str(stem),
        meaning='The window stays centred while walking; floor seen earlier is still known when it enters the blind zone at the feet')
    stage.RemovePrim(PROPS)

    # ---- output only ------------------------------------------------------------------------------
    nav=runtime.navigation; foreign=sum(applied!=keyboard for applied,keyboard in commands)
    add('no_avoidance_commands',foreign==0 and nav.mode=='MANUAL' and nav.scheduler.state in ('IDLE','STOPPED') and nav.detector.mode=='SIM GROUND TRUTH',
        ticks_checked=len(commands),ticks_where_command_differs_from_keyboard=foreign,navigation_mode=nav.mode,scheduler_state=nav.scheduler.state,
        navigation_obstacle_source=nav.detector.mode,meaning='Every applied command is the test keyboard command; navigation keeps its own obstacle source')
    add('map_rules_every_frame',not violations,maps_checked=len(results),violations=sorted(set(violations)))
    report['update_ms']={'mean':float(np.mean(update_ms)),'p95':float(np.percentile(update_ms,95)),'max':float(max(update_ms)),'thread':'perception thread'}
    timing=[r['timing'] for r in results]
    report['pipeline_ms']={'yolo_mean':float(np.mean([t['inference_ms'] for t in timing])),'localization_and_costmap_mean':float(np.mean([t['localization_ms'] for t in timing])),
        'frame_to_map_mean':float(np.mean([t['submit_to_result_ms'] for t in timing])),'frame_to_map_p95':float(np.percentile([t['submit_to_result_ms'] for t in timing],95))}
    report['worker']=yolo.worker.stats()
    report['failed_tests']=[k for k,v in report['tests'].items() if v['passed'] is False]
    report['status']='passed' if not report['failed_tests'] else 'failed'
    payload=json.dumps(report,indent=2,default=float)
    (yolo.costmap_directory/'local_costmap_results.json').write_text(payload)
    (ROOT/'docs/perception').mkdir(parents=True,exist_ok=True); (ROOT/'docs/perception/local_costmap_results.json').write_text(payload)
    print('COSTMAP_RESULT '+json.dumps({'status':report['status'],'failed_tests':report['failed_tests'],'images':str(yolo.costmap_directory),
        'update_ms':report['update_ms'],'pipeline_ms':report['pipeline_ms']}),flush=True)
except BaseException:
    traceback.print_exc(); sys.stdout.flush(); sys.stderr.flush()  # Kit may exit before Python reports it
    raise
finally:
    if runtime: runtime.close()
    app.close()
