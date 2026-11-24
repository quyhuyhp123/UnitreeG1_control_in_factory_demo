"""50 unannotated G1-eye-height virtual sensor views; no training or patrol."""
from pathlib import Path
from datetime import datetime
import sys,json,math,time,hashlib
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]

def main():
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'renderer':'RaytracedLighting','anti_aliasing':1,'multi_gpu':False,
        'extra_args':['--/app/vulkan=false','--/exts/isaacsim.physics.newton/auto_switch_on_startup=false',
                      '--/exts/isaacsim.core.simulation_manager/default_engine=physx']})
    runtime=None
    output=ROOT/'Training_model/datasets'/('g1_factory_50_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
    (output/'images').mkdir(parents=True,exist_ok=False)
    try:
        import numpy as np,carb
        from PIL import Image
        from pxr import UsdGeom,Gf
        from controllers.factory_selector import load_factory
        from controllers.factory_g1_runtime import FactoryG1Runtime
        from controllers.factory_maps import layout
        from navigation.lane_map import LaneMap
        from perception.rgbd_config import load_config
        from perception.rgbd_camera import matrix
        carb.settings.get_settings().set('/app/asyncRendering',False)
        cfg=load_factory('v2');cfg['map_id']=2;cfg['rgbd_camera']=load_config({'enabled':True,'debug_window':False})
        runtime=FactoryG1Runtime(app,cfg,self_test=True);camera=runtime.rgbd
        product=camera.sensor.render_product.GetPrim()
        product.ApplyAPI('OmniRtxPostDebugSettingsAPI_1')
        attr=product.GetAttribute('omni:rtx:post:aa:op')
        if attr:attr.Set('none')
        for _ in range(50):app.update();runtime.tick()
        # Measure physical walking camera jitter before separate static capture.
        raw=[];stable=[];runtime.inject('W')
        for _ in range(150):
            app.update();runtime.tick()
            if runtime.fault:raise RuntimeError(runtime.fault)
            prim=runtime.stage.GetPrimAtPath(camera.path)
            parent=matrix(UsdGeom.XformCache().GetLocalToWorldTransform(prim.GetParent()))
            raw.append((parent@matrix(camera.mount)).tolist());stable.append(camera.world_pose().tolist())
        runtime.inject('W','RELEASE')
        a=np.array(raw)[30:];b=np.array(stable)[30:]
        metrics={'raw_height_delta_rms_m':float(np.sqrt(np.mean(np.diff(a[:,2,3])**2))),
            'stabilized_height_delta_rms_m':float(np.sqrt(np.mean(np.diff(b[:,2,3])**2))),
            'raw_horizon_roll_std_deg':float(np.std(np.degrees(np.arcsin(np.clip(a[:,2,0],-1,1))))),
            'stabilized_horizon_roll_std_deg':float(np.std(np.degrees(np.arcsin(np.clip(b[:,2,0],-1,1))))),
            'scope':'real policy-driven W command; torso rigid camera vs virtual gimbal pose, not optical-flow evaluation'}
        runtime.timeline.pause();app.update()
        if not runtime.safe_reset('Dataset preparation'):raise RuntimeError('Reset rejected')
        runtime.timeline.play();app.update()
        for _ in range(25):app.update();runtime.tick()
        runtime.timeline.pause();app.update()
        rng=np.random.default_rng(5024);records=[];hashes=set();used=[]
        for map_id in (2,3):
            runtime.set_obstacle_map(map_id)
            for _ in range(6):app.update()
            nav=runtime.navigation;nav.refresh_obstacles(force=True)
            obstacles=nav.detector.obstacles
            lane=LaneMap(nav.map_config,obstacles,nav.detector.floor)
            targets=[i for i in layout(map_id) if (map_id==2 and i['lane']!='route_2_right') or
                      (map_id==3 and (i['lane']=='route_2_right' or (i['lane']=='route_1_center' and i['kind']=='person')))]
            for item in targets:
                target=np.array(item['position_m'],float);target[2]={'person':.95,'chair':.55,'box':.35}[item['kind']]
                candidates=[]
                minimum=2.35 if item['kind']=='person' else 1.6
                for x in np.arange(target[0]-4.5,target[0]+4.51,.3):
                    for y in np.arange(target[1]-4.5,target[1]+4.51,.3):
                        d=math.hypot(x-target[0],y-target[1])
                        if not minimum<=d<=4.5 or not lane.safe((x,y)):continue
                        eye=np.array([x,y,1.23+rng.uniform(-.025,.025)])
                        ray=eye+np.linspace(0,.94,30)[:,None]*(target-eye)
                        visible=True
                        for o in obstacles:
                            if '/'+item['name']+'/' in o['id']:continue
                            lo,hi=np.array(o['bounds']);zlo,zhi=o['height_range_m']
                            if np.any(np.all((ray[:,:2]>=lo)&(ray[:,:2]<=hi),axis=1)&(ray[:,2]>=zlo)&(ray[:,2]<=zhi)):
                                visible=False;break
                        if not visible:continue
                        yaw=math.atan2(target[1]-y,target[0]-x)
                        feature=np.array([x*.3,y*.3,math.cos(yaw)*2,math.sin(yaw)*2])
                        candidates.append((eye,yaw,feature))
                rng.shuffle(candidates);chosen=[]
                while len(chosen)<5 and candidates:
                    idx=0 if not chosen else max(range(len(candidates)),key=lambda n:min(np.linalg.norm(candidates[n][2]-v[2]) for v in chosen))
                    eye,yaw,feature=candidates.pop(idx)
                    if any(np.linalg.norm(eye-e)<.2 and abs(math.atan2(math.sin(yaw-q),math.cos(yaw-q)))<.2 for e,q in used):continue
                    Tgf=Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(0,0,1)).GetInverse()
                    xf=UsdGeom.Xformable(runtime.stage.GetPrimAtPath(camera.path));ops=xf.GetOrderedXformOps()
                    if len(ops)!=1 or ops[0].GetOpType()!=UsdGeom.XformOp.TypeTransform:xf.ClearXformOpOrder();op=xf.AddTransformOp()
                    else:op=ops[0]
                    xf.SetResetXformStack(True);op.Set(Tgf)
                    # Static scene + same pose for several renders: no historic
                    # robot transform is substituted for this virtual camera.
                    for _ in range(6):app.update()
                    payload,_=camera.sensor.get_data('rgb',out=camera.rgb_cpu)
                    if payload is None:continue
                    image=np.array(payload.numpy(),dtype=np.uint8).reshape(camera.height,camera.width,-1)[:,:,:3].copy()
                    digest=hashlib.sha256(image.tobytes()).hexdigest()
                    # Gross readback corruption produces dense alternating
                    # channel/row stripes. Do not silently label such frames.
                    edge_noise=float(np.mean(np.abs(np.diff(image.astype(np.float32),axis=1))))
                    difference=np.abs(np.diff(image.astype(np.float32),axis=1)).mean(axis=2)
                    tile_noise=max(difference[y:y+24,x:x+24].mean() for y in range(0,336,24) for x in range(0,624,24))
                    if digest in hashes or image.std()<5 or edge_noise>12 or tile_noise>45:continue
                    number=len(records)+1;filename=f'{number:03d}_map{map_id}_{item["kind"]}_{len(chosen)+1}.png'
                    Image.fromarray(image).save(output/'images'/filename)
                    records.append({'image':'images/'+filename,'target_hint':item['kind'],'target_name':item['name'],'map_id':map_id,
                        'group':f'map{map_id}_{item["name"]}','camera_eye_world_m':eye.tolist(),
                        'T_world_camera_usd':matrix(Tgf).tolist(),'intrinsics':camera.intrinsics(),'sha256_rgb_pixels':digest,
                        'source':'virtual camera at G1 eye height; not actual robot trajectory','annotated':False})
                    hashes.add(digest);used.append((eye,yaw));chosen.append((eye,yaw,feature))
                    print('CAPTURE '+str(number)+' '+filename,flush=True)
                if len(chosen)!=5:raise RuntimeError(f'Only {len(chosen)} valid unique views for {item["name"]}')
        if len(records)!=50:raise RuntimeError(f'Expected50 images, got {len(records)}')
        report={'output_directory':str(output),'count':len(records),'resolution_wh':[640,360],'annotated':False,
            'training_started':False,'stabilization_walk_metrics':metrics,'images':records,
            'note':'Hints are scene targets, not AI labels. Label every visible object. Similar views must remain in one split; use independent data for final test.'}
        (output/'capture_manifest.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
        (output/'LABELME_README.txt').write_text('Open images/ in Labelme. Draw rectangle boxes for all visible person/chair/box instances. No annotations generated. Do not treat missing JSON as background. Images are 50 simulated virtual G1-eye-height views, not a real robot trajectory. Keep related views together when splitting train/val/test. No training was run.',encoding='utf-8')
        print('DATASET_READY '+str(output),flush=True)
    except BaseException:
        import traceback
        error=traceback.format_exc();(output/'capture_error.txt').write_text(error);print(error,flush=True);raise
    finally:
        if runtime:runtime.close()
        app.close()
if __name__=='__main__':main()

