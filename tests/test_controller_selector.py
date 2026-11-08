import unittest
from controllers.controller_selector import ControllerSelector,validate_selection
from controllers.controller_interface import ControllerUnavailable
from controllers.g1_sonic_controller import SonicBackend


class SelectorTests(unittest.TestCase):
    def test_default(self): self.assertEqual(validate_selection('locomotion'),'locomotion')
    def test_sonic_is_blocked_before_articulation(self):
        with self.assertRaises(ControllerUnavailable): validate_selection('sonic')
        with self.assertRaises(ControllerUnavailable): SonicBackend()
    def test_switch_never_transfers_ownership(self):
        selector=ControllerSelector()
        self.assertIn('Restart required',selector.request('sonic'))
        self.assertEqual(selector.current,'locomotion')
    def test_unknown_backend(self):
        with self.assertRaises(ValueError): validate_selection('other')
