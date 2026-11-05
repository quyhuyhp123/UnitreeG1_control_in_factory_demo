"""Deterministic observation math checks; no network, training or fake policy."""
from pathlib import Path
from types import SimpleNamespace
import json,sys
import numpy as np
ROOT=Path(r'D:\unitree_g1')
sys.path.insert(0,str(ROOT))
from controllers.g1_locomotion import G1Locomotion,inverse_rotate_wxyz

cfg=json.loads((ROOT/'configs/locomotion.yaml').read_text())
deploy=json.loads((ROOT/cfg['deploy_config']).read_text())
sdk=json.loads((ROOT/'configs/g1_robot.yaml').read_text())['sdk_joint_order']
names=cfg['policy_joint_names']
fixture=SimpleNamespace(deploy=deploy,policy_names=names,default=np.array(deploy['default_joint_pos']),previous_action=np.arange(29,dtype=np.float32)*.01,history={},command_low=np.array([-.5,-.3,-.2]),command_high=np.array([1,.3,.2]))
default=dict(zip(names,deploy['default_joint_pos']))
state={'joint_names':sdk,'joint_positions_rad':[default[n]+i*.01 for i,n in enumerate(sdk)],
       'joint_velocities_rad_s':[i*.1 for i in range(29)],'base_orientation_wxyz':[1,0,0,0],
       'base_angular_velocity_world_rad_s':[1,2,3]}
obs=G1Locomotion.make_observation(fixture,state,[.3,.1,.15]).reshape(-1)
np.testing.assert_allclose(obs[:15],np.tile([.2,.4,.6],5),atol=1e-6)
np.testing.assert_allclose(obs[15:30],np.tile([0,0,-1],5),atol=1e-6)
np.testing.assert_allclose(obs[30:45],np.tile([.3,.1,.15],5),atol=1e-6)
np.testing.assert_allclose(obs[45:190],np.tile(np.array(deploy['joint_ids_map'])*.01,5),atol=1e-6)
np.testing.assert_allclose(obs[190:335],np.tile(np.array(deploy['joint_ids_map'])*.1*.05,5),atol=1e-6)
np.testing.assert_allclose(obs[335:480],np.tile(fixture.previous_action,5),atol=1e-6)
state['base_angular_velocity_world_rad_s']=[4,5,6]
next_obs=G1Locomotion.make_observation(fixture,state,[0,0,0]).reshape(-1)
np.testing.assert_allclose(next_obs[:15],np.r_[np.tile([.2,.4,.6],4),[.8,1,1.2]],atol=1e-6)
np.testing.assert_allclose(inverse_rotate_wxyz([2**-.5,0,0,2**-.5],[1,0,0]),[0,-1,0],atol=1e-6)
fixture.command_low=fixture.command_low.astype(np.float32)
fixture.command_high=fixture.command_high.astype(np.float32)
G1Locomotion.make_observation(fixture,state,[1,.3,.2])
G1Locomotion.make_observation(fixture,state,[-.5,-.3,-.2])
try: G1Locomotion.make_observation(fixture,state,[1.1,0,0])
except ValueError: pass
else: raise AssertionError('Out-of-range velocity was accepted')
report={'status':'passed','checks':['term_order','SDK_to_policy_joint_mapping','per_term_scale','five_history_reset','oldest_to_newest_history_update','previous_raw_action','WXYZ_world_to_body_rotation','velocity_ranges'],'note':'These are numeric unit checks, not simulation evidence.'}
(ROOT/'logs/locomotion_contract_checks.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report))
