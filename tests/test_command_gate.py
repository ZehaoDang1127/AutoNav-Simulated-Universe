"""Safety invariants for the steady-clock command gate."""
import importlib.util
import math
import unittest
from pathlib import Path


PATH = (Path(__file__).resolve().parents[1] / "integrations" /
        "ros_humble_fortress" / "command_gate.py")
spec = importlib.util.spec_from_file_location("command_gate", PATH)
module = importlib.util.module_from_spec(spec)
import sys
sys.modules[spec.name] = module
spec.loader.exec_module(module)
GateState = module.GateState


class CommandGateTests(unittest.TestCase):
    def test_disabled_and_stale_inputs_are_zero(self):
        gate = GateState()
        gate.set_command(0.7, 0.1, 0.0)
        self.assertEqual(gate.output(0.0), (0.0, 0.0))
        gate.set_enable(True, 1.0)
        gate.set_command(0.7, 0.1, 1.0)
        self.assertEqual(gate.output(1.1), (0.7, 0.1))
        self.assertEqual(gate.output(1.36), (0.0, 0.0))
        gate.set_enable(True, 1.37)
        self.assertEqual(gate.output(1.38), (0.0, 0.0))
        gate.set_command(0.4, 0.0, 1.38)
        gate.set_enable(True, 1.6)
        self.assertEqual(gate.output(1.79), (0.0, 0.0))

    def test_stop_and_referee_failure_latch_zero(self):
        gate = GateState()
        gate.set_enable(True, 1.0)
        gate.set_command(0.5, 0.0, 1.0)
        gate.set_enable(False, 1.1)
        self.assertEqual(gate.output(1.1), (0.0, 0.0))
        gate.set_enable(True, 1.2)
        gate.set_command(0.5, 0.0, 1.2)
        gate.set_referee_failure(True)
        gate.set_enable(True, 1.3)
        self.assertEqual(gate.output(1.3), (0.0, 0.0))

    def test_nonfinite_command_latches_failure(self):
        gate = GateState()
        gate.set_enable(True, 1.0)
        gate.set_command(math.nan, 0.0, 1.0)
        gate.set_command(0.5, 0.0, 1.1)
        self.assertTrue(gate.failed)
        self.assertEqual(gate.output(1.1), (0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
