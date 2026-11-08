import unittest
from controllers.controller_interface import LocomotionRLBackend


class DelegationTests(unittest.TestCase):
    def test_existing_policy_contract_preserved(self):
        class RecordedPolicy:
            def reset(self): self.command=None
            def step(self,command): self.command=command; return command
        policy=RecordedPolicy(); backend=LocomotionRLBackend(policy)
        command=[-.35,0,.6]
        self.assertIs(backend.step(command),command)
        backend.reset(); self.assertIsNone(policy.command)
        # This tests delegation only. Actual PhysX regression is run_sim --self-test.
