import unittest


class SonicFactoryTests(unittest.TestCase):
    @unittest.skip('BLOCKED: SONIC flat-ground validation must pass before factory control')
    def test_sonic_closed_loop_factory(self):
        raise AssertionError('Requires real keyboard, collision, gait and reset evidence')
