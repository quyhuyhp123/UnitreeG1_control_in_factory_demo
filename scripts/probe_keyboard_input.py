from pathlib import Path
import sys,json
ROOT=Path(r'D:\unitree_g1')
sys.path.insert(0,str(ROOT))
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'renderer':'RaytracedLighting','width':640,'height':480,'multi_gpu':False})
import carb.input
from controllers.keyboard_controller import KeyboardController,CarbKeyboardInput
cfg=json.loads((ROOT/'configs/keyboard.yaml').read_text())
loc=json.loads((ROOT/'configs/locomotion.yaml').read_text())
dep=json.loads((ROOT/loc['deploy_config']).read_text())
c=KeyboardController(cfg,dep)
iface=carb.input.acquire_input_interface()
provider=iface.get_input_provider()
keyboard=provider.create_keyboard('G1 input diagnostic')
events=[]
binding=CarbKeyboardInput(c,keyboard=keyboard,focus_provider=lambda:True,event_logger=lambda e:events.append(e))
c.set_running(True)
later=[]
sub=iface.subscribe_to_input_events(lambda e:(later.append(e.event.input.name) or True),device=keyboard,order=-1)
for name in ['UP','X']:
    provider.buffer_keyboard_key_event(keyboard,carb.input.KeyboardEventType.KEY_PRESS,getattr(carb.input.KeyboardInput,name),0)
    provider.update_keyboard(keyboard)
    iface.distribute_buffered_events()
print('PROBE '+json.dumps({'ours':events,'later':later,'focus':c.focused,'running':c.running,'desired':c.desired()}),flush=True)
iface.unsubscribe_to_input_events(sub)
binding.close()
provider.destroy_keyboard(keyboard)
app.close()
