import unittest,json,sys
from pathlib import Path
ROOT=Path(r'D:\unitree_g1'); sys.path.append(str(ROOT/'scripts'))
from controllers.factory_selector import load_factory
_original_path=list(sys.path)
from run_factory_g1 import preflight
sys.path[:]=_original_path
from navigation.lane_map import LaneMap
class FactorySelectorTests(unittest.TestCase):
    def test_explicit_v1_v2(self):
        self.assertEqual(load_factory('v1')['factory_usd'],'assets/factory/factory_physics.usd')
        self.assertIn('factory_layout_v2_physics.usd',load_factory('v2')['factory_usd'])
        self.assertEqual(load_factory('v2')['robot_spawn']['heading_deg'],0)
    def test_v2_space_preserves_motion_x_stops(self):
        from controllers.keyboard_controller import KeyboardController
        cfg=json.loads((ROOT/'configs/keyboard.yaml').read_text())
        cfg.update(load_factory('v2')['keyboard_overrides'])
        loco=json.loads((ROOT/'configs/locomotion_mjlab.yaml').read_text())
        deploy=json.loads((ROOT/loco['deploy_config']).read_text())
        keys=KeyboardController(cfg,deploy); keys.set_focus(True); keys.set_running(True)
        keys.handle_key('W','PRESS')
        for _ in range(50): keys.tick(.02)
        before=list(keys.command); stop_before=keys.stop_generation
        keys.handle_key('SPACE','PRESS'); keys.handle_key('SPACE','RELEASE')
        self.assertEqual(keys.tick(.02),before)
        self.assertEqual(keys.follow_generation,1); self.assertEqual(keys.stop_generation,stop_before)
        self.assertIn('W',keys.pressed)
        keys.handle_key('X','PRESS')
        self.assertEqual(keys.tick(.02),[0,0,0]); self.assertEqual(keys.stop_generation,stop_before+1)
    def test_reject_mismatched_selection(self):
        with self.assertRaises(ValueError): load_factory('v1',ROOT/'configs/factory_v2.json')
    def test_preflight_both(self):
        for factory in ['v1','v2']: preflight(load_factory(factory),False)
    def test_stale_map_and_source_rejected(self):
        c=load_factory('v2'); c['navigation_map']='configs/navigation_map.json'
        with self.assertRaises(ValueError): preflight(c,False)
        c=load_factory('v2'); c['source_factory_sha256']='0'*64
        with self.assertRaises(ValueError): preflight(c,False)
    def test_holes_and_new_spawn(self):
        c=json.loads((ROOT/'configs/navigation_map_v2.json').read_text()); lane=LaneMap(c)
        self.assertTrue(lane.safe((-8.85,-13.75)))
        self.assertFalse(lane.safe((-3.5,0))); self.assertFalse(lane.safe((4,0)))
        self.assertEqual(len(c['forbidden_zones']),2)
    def test_v2_rejects_old_saved_patrol(self):
        from types import SimpleNamespace
        from navigation.patrol_controller import PatrolController
        p=ROOT/'logs/test_factory_identity_patrol.json'; p.write_text(json.dumps({'factory_id':'v1'}))
        nav=object.__new__(PatrolController); nav.selector=SimpleNamespace(mode='MANUAL'); nav.scheduler=SimpleNamespace(state='STOPPED'); nav.factory_id='v2'
        with self.assertRaisesRegex(ValueError,'another factory'): nav.load(p)
