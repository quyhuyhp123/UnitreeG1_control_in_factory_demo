"""Pre-launch selection. Never silently substitutes a controller requested by user."""
from controllers.controller_interface import ControllerUnavailable

SONIC_BLOCKER = (
    'SONIC PhysX control is not validated: reference FK/body orientation, encoder '
    'observation parity and actuator/model parity are unresolved. '
    'No SONIC joint commands were enabled. Restart with -Controller locomotion. '
    'See docs/sonic_integration/compatibility_report.md.'
)


def validate_selection(name):
    if name not in ('locomotion', 'sonic'):
        raise ValueError('Controller must be locomotion or sonic')
    if name == 'sonic':
        raise ControllerUnavailable(SONIC_BLOCKER)
    return name


class ControllerSelector:
    """One active backend per session; a UI request cannot transfer ownership."""
    def __init__(self, current='locomotion'):
        self.current = validate_selection(current)

    def request(self, selected):
        if selected not in ('locomotion', 'sonic'):
            raise ValueError('Unknown controller')
        if selected == self.current:
            return 'Current backend already active; no changes.'
        return 'Restart required. ' + SONIC_BLOCKER
