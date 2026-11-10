"""Input-state unit tests, separate from Carb/physics integration tests."""
from pathlib import Path
import json,sys
ROOT=Path(r'D:\unitree_g1')
sys.path.insert(0,str(ROOT))
from controllers.keyboard_controller import KeyboardController
cfg=json.loads((ROOT/'configs/keyboard.yaml').read_text())
loco=json.loads((ROOT/'configs/locomotion.yaml').read_text())
deploy=json.loads((ROOT/loco['deploy_config']).read_text())
results=[]
def new():
    controller=KeyboardController(cfg,deploy)
    controller.set_focus(True)
    controller.set_running(True)
    return controller
def hold(keys):
    c=new()
    for k in keys: c.handle_key(k,'PRESS')
    for _ in range(100): c.tick(.02)
    return c
for a,b in [('W','UP'),('S','DOWN'),('A','LEFT'),('D','RIGHT')]:
    assert hold([a]).command==hold([b]).command
    results.append(a+'='+b)
assert hold(['W','UP']).command==hold(['W']).command
for keys,axis in [(['W','S'],0),(['W','DOWN'],0),(['LEFT','RIGHT'],2),(['Q','E'],1)]:
    assert hold(keys).command[axis]==0
    results.append('opposed '+str(keys))
for keys,axes in [(['W','A'],[0,2]),(['UP','RIGHT'],[0,2]),(['W','Q'],[0,1])]:
    c=hold(keys)
    assert all(c.command[i]!=0 for i in axes)
    results.append('combo '+str(keys))
c=hold(['W'])
before=c.command[:]
c.handle_key('W','REPEAT')
assert c.tick(.02)==before
c.handle_key('W','RELEASE')
previous=c.command[:]
for _ in range(100):
    current=c.tick(.02)
    assert all(abs(current[i]-previous[i])<=cfg['deceleration_limit'][i]*.02+1e-12 for i in range(3))
    previous=current
assert c.command==[0,0,0]
results+=['repeat does not accumulate','release decelerates']
for trigger in ['focus','pause','stop']:
    c=hold(['W'])
    if trigger=='focus': c.set_focus(False); c.set_focus(True)
    elif trigger=='pause': c.set_running(False); c.handle_key('S','PRESS'); c.set_running(True)
    else: c.handle_key('X','PRESS'); c.handle_key('X','RELEASE')
    c.handle_key('W','REPEAT')
    assert c.tick(.02)==[0,0,0]
    c.handle_key('W','RELEASE'); c.handle_key('W','PRESS')
    assert c.tick(.02)[0]>0
    results.append(trigger+' clears stale movement and requires fresh press')
c=hold(['W']); c.handle_key('R','PRESS')
assert c.reset_requested and c.tick(.02)==[0,0,0]
results.append('R requests safe reset only')
try: KeyboardController({**cfg,'max_forward_velocity':2},deploy)
except ValueError: pass
else: raise AssertionError('Unsupported speed accepted')
c=hold(['W','S'])
c.reconcile_released_keys(lambda key:key in {'W','S'})
assert c.desired()==[0,0,0]
c.reconcile_released_keys(lambda key:key=='S')
assert c.desired()[0]<0 and c.pressed=={'S'}
results+=['physical opposite keys still cancel','lost W release reconciles and allows reverse']
c=hold(['W']); before=c.command[:]; stop_before=c.stop_generation
c.handle_key('SPACE','PRESS'); c.handle_key('SPACE','REPEAT')
assert c.tick(.02)==before and 'W' in c.pressed
assert c.follow_generation==1 and c.stop_generation==stop_before
c.handle_key('SPACE','RELEASE')
assert c.tick(.02)==before
results.append('SPACE only requests camera follow; held W and velocity preserved')
report={'status':'passed','kind':'unit_checks_not_physics','checks':results}
(ROOT/'logs/keyboard_contract_result.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report))
