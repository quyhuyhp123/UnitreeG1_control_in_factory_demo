from pathlib import Path
import asyncio,json
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':960,'height':640,'renderer':'RaytracedLighting','multi_gpu':False})
import omni.usd
from pxr import UsdLux
from omni.kit.viewport.utility import get_active_viewport,capture_viewport_to_file
root=Path(r'D:\unitree_g1')
omni.usd.get_context().open_stage(str(root/'assets/factory/factory_physics.usd'))
for _ in range(30): app.update()
stage=omni.usd.get_context().get_stage()
lights=[]
for prim in stage.Traverse():
    if prim.HasAPI(UsdLux.LightAPI):
        lights.append({'path':str(prim.GetPath()),'attributes':{a.GetName():str(a.Get()) for a in prim.GetAttributes() if a.GetName().startswith('inputs:')}})
(root/'logs/factory_lights.json').write_text(json.dumps(lights,indent=2),encoding='utf-8')
viewport=get_active_viewport()
viewport.camera_path='/World/OverviewCamera'
viewport.set_texture_resolution((960,640))
for _ in range(100): app.update()
capture=capture_viewport_to_file(viewport,str(root/'logs/factory_isaac_preview.png'))
task=asyncio.ensure_future(capture.wait_for_result())
for _ in range(200):
    app.update()
    if task.done(): break
print('CAPTURE',task.done(),task.result() if task.done() else 'timeout',flush=True)
app.close()
