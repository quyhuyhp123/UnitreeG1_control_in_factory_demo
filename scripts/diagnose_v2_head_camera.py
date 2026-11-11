from pathlib import Path
import sys,json
R=Path(r'D:\unitree_g1'); sys.path[:0]=[str(R),str(R/'scripts')]
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'renderer':'RaytracedLighting','anti_aliasing':1,'multi_gpu':False,'extra_args':['--/app/vulkan=false','--/exts/isaacsim.physics.newton/auto_switch_on_startup=false','--/exts/isaacsim.core.simulation_manager/default_engine=physx']})
runtime=None
try:
 from controllers.factory_g1_runtime import FactoryG1Runtime
 from controllers.factory_selector import load_factory
 from pxr import UsdGeom,UsdRender
 runtime=FactoryG1Runtime(app,load_factory('v2'),self_test=True)
 for _ in range(100): app.update(); runtime.tick()
 w=runtime.head_widget.viewport_api; world=UsdGeom.XformCache().GetLocalToWorldTransform(runtime.stage.GetPrimAtPath(runtime.head_camera_path))
 d={'directory':str(runtime.directory),'head_matrix':[list(r) for r in world],'head_path':str(w.camera_path),'render_product':w.render_product_path,'camera_relation':[str(p) for p in UsdRender.Product(runtime.stage.GetPrimAtPath(w.render_product_path)).GetCameraRel().GetTargets()]}
 runtime.capture_head_preview(runtime.directory/'head_original.png')
 w.camera_path='/World/TestCamera'; runtime._bind_render_camera(w)
 for _ in range(60): app.update()
 runtime.capture_head_preview(runtime.directory/'head_with_main_camera.png')
 runtime.set_camera_mode('free'); runtime._set_camera_transform(world)
 runtime.context['capture'](runtime.directory/'main_at_head_pose.png')
 (R/'logs/head_camera_diagnostic.json').write_text(json.dumps(d,indent=2)); print(json.dumps(d),flush=True)
finally:
 if runtime: runtime.close()
 app.close()
