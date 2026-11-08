"""Deliberately unavailable until model/reference/control parity is validated.

Model session loading and offline planner inference are not a validated controller.
This gate prevents accidental joint writes using the existing locomotion PD profile.
"""
from controllers.controller_interface import ControllerUnavailable
from controllers.controller_selector import SONIC_BLOCKER


class SonicBackend:
    def __init__(self, *args, **kwargs):
        raise ControllerUnavailable(SONIC_BLOCKER)
