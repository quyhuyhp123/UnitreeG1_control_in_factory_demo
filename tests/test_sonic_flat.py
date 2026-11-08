import unittest


class SonicFlatTests(unittest.TestCase):
    @unittest.skip('BLOCKED: full reference/observation and actuator parity not validated; no SONIC joint writes')
    def test_sonic_closed_loop_flat(self):
        raise AssertionError('Replace only with actual PhysX trajectory assertions after prerequisite validation')
