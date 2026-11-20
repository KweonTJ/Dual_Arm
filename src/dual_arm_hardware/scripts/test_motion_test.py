"""Check the exact motion-test entry point before using it on real motors."""
from contextlib import redirect_stdout
from copy import deepcopy
import io
from pathlib import Path
import sys
import tempfile
import unittest

from ax12_guard import GOAL, TORQUE_ENABLE, PRESENT
from mock_bus import MockBus

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'src/dual_arm_hardware/examples'))
from motion_test import motion_test


class WristMotionTestTests(unittest.TestCase):
    def exercise(self, fail=False):
        capture = {}
        def bus_factory(motors):
            bus = MockBus(motors)
            # Include other joints outside the model range: they must stay OFF.
            actual = {1:112, 2:523, 3:470, 4:496, 5:504, 6:490, 7:500, 8:553, 9:522, 10:543}
            for motor_id, raw in actual.items():
                bus.set_register(motor_id, PRESENT, raw)
                bus.set_register(motor_id, GOAL, raw)
            capture.update(bus=bus, before=deepcopy(bus.memory))
            if fail:
                bus.ignore_sync_id = 2
            return bus
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()):
            report = motion_test(ROOT/'src/dual_arm_hardware/config/hardware.yaml',
                                 ROOT/'src/dual_arm_description/config',
                                 ROOT/'src/dual_arm_hardware/config/lowlevel.yaml',
                                 directory, duration=.3, bus_factory=bus_factory)
        return report, capture

    def test_relative_wrist_motion_preserves_other_motors_and_configuration(self):
        report, capture = self.exercise()
        self.assertTrue(report['passed'], report['error'])
        self.assertTrue(report['torque_off_confirmed'])
        self.assertEqual(report['initial_raw'], 523)
        self.assertEqual(report['target_raw'], 530)
        self.assertEqual(report['phases'][-1]['position_raw'], 523)
        bus = capture['bus']
        for motor_id, before in capture['before'].items():
            self.assertEqual(bus.memory[motor_id], before)
        writes = [e for e in bus.events if e[0]=='write']
        self.assertTrue(writes)
        self.assertTrue(all(e[1]==2 and e[2] in (GOAL,TORQUE_ENABLE) for e in writes))
        batches = [e[1] for e in bus.events if e[0]=='sync_write_positions']
        self.assertEqual(batches, [{2:530}, {2:523}])

    def test_failed_command_stops_selected_motor_and_records_failure(self):
        report, capture = self.exercise(fail=True)
        self.assertFalse(report['passed'])
        self.assertIn('readback differs', report['error'])
        self.assertTrue(report['torque_off_confirmed'])
        self.assertTrue(capture['bus'].closed)
        self.assertTrue(all(memory[TORQUE_ENABLE]==0 for memory in capture['bus'].memory.values()))


if __name__ == '__main__':
    unittest.main()
