"""Simulation controller contract; selecting a different backend requires restart."""
from typing import Protocol


class ControllerUnavailable(RuntimeError):
    pass


class ControllerBackend(Protocol):
    def reset(self): ...
    def step(self, command): ...


class LocomotionRLBackend:
    """Delegates to the existing policy without changing its observations/actions."""
    def __init__(self, policy):
        self.policy = policy

    def reset(self):
        return self.policy.reset()

    def step(self, command):
        return self.policy.step(command)

    def get_status(self):
        return {'backend': 'locomotion', 'policy_loaded': True}
