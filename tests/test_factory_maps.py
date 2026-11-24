import unittest,math,json
from pathlib import Path
from controllers.factory_maps import layout,ROOT

class FactoryMapsTests(unittest.TestCase):
    def test_original_and_deterministic_random_variants(self):
        self.assertEqual(layout(1),[])
        self.assertEqual(layout(2),layout(2));self.assertNotEqual(layout(2),layout(3))
        for number in (2,3):
            items=layout(number)
            self.assertEqual(len(items),9)
            self.assertEqual({i['kind'] for i in items},{'chair','person','box'})
            self.assertEqual({i['lane'] for i in items},{'shared_left','route_1_center','route_2_right'})
            for i in items:
                self.assertGreater(math.dist(i['position_m'][:2],[-8.85,-13.75]),3)
            for a in items:
                for b in items:
                    if a is not b:self.assertGreater(math.dist(a['position_m'][:2],b['position_m'][:2]),2.)
    def test_invalid_selection_rejected(self):
        for n in (0,4):
            with self.assertRaises(ValueError):layout(n)
