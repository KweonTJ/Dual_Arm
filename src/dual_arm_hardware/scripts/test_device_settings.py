"""Exercise connection/control without writing protection settings; no real bus."""
from contextlib import redirect_stdout
from copy import deepcopy
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import yaml
from ax12_guard import (load_configuration, ProtectionError, CW, CCW, MAX_TORQUE,
                        TORQUE_LIMIT, SHUTDOWN, LOCK, GOAL, TORQUE_ENABLE)
from hardware_limits import main as limits_main
from lowlevel_control import DualArmLowLevel
from mock_bus import MockBus

ROOT = Path(__file__).resolve().parents[3]
DESCRIPTION = ROOT / 'src/dual_arm_description/config'


class ExistingDeviceSettingsTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'hardware.yaml'
        self.config = yaml.safe_load((ROOT / 'src/dual_arm_hardware/config/hardware.yaml').read_text())
        self.config.update(port='/dev/FAKE_NO_DEVICE', baudrate=1000000, configure_protection=False)
        self.config.pop('torque_limit_percent', None)
        self.path.write_text(yaml.safe_dump(self.config))
        _, self.motors = load_configuration(self.path, DESCRIPTION)
        self.bus = MockBus(self.motors)
        # Deliberately distinct caps, narrow device range, locked EEPROM and a
        # shutdown mask that differs from the host's optional protection preset.
        for motor in self.motors:
            for address, value in ((CW, 600), (CCW, 650), (MAX_TORQUE, 230),
                                   (TORQUE_LIMIT, 170), (SHUTDOWN, 1), (LOCK, 1)):
                self.bus.set_register(motor.id, address, value)

    def controller(self):
        with patch('lowlevel_control.SDKBus', return_value=self.bus):
            return DualArmLowLevel.from_config(self.path, DESCRIPTION)

    def test_output_percentage_is_not_required_or_substituted_with_full_output(self):
        config, motors = load_configuration(self.path, DESCRIPTION)
        self.assertFalse(config['configure_protection'])
        self.assertTrue(all(motor.cap is None for motor in motors))

    def test_protection_option_requires_a_boolean(self):
        for value in ('false', 0, None):
            with self.subTest(value=value):
                config = {**self.config, 'configure_protection': value}
                self.path.write_text(yaml.safe_dump(config))
                with self.assertRaisesRegex(ProtectionError, 'must be a boolean'):
                    load_configuration(self.path, DESCRIPTION)

    def test_apply_only_reads_and_keeps_existing_register_values(self):
        before = deepcopy(self.bus.memory)
        report_dir = Path(self.temp.name) / 'reports'
        arguments = ['hardware_limits.py', 'apply', '--config', str(self.path),
                     '--description-config', str(DESCRIPTION), '--report-dir', str(report_dir)]
        with patch('sys.argv', arguments), patch('hardware_limits.SDKBus', return_value=self.bus), \
                redirect_stdout(io.StringIO()):
            self.assertEqual(limits_main(), 0)
        report = json.loads(next(report_dir.glob('*.json')).read_text())
        self.assertEqual(report['status'], 'existing_device_settings_verified')
        self.assertIsNone(report['requested_output_percent'])
        self.assertTrue(report['verified_on_device'])
        self.assertEqual(self.bus.memory, before)
        self.assertFalse(any(event[0] == 'write' for event in self.bus.events))

    def test_start_enable_command_and_close_never_write_protection_registers(self):
        controller = self.controller()
        before = deepcopy(self.bus.memory)
        try:
            controller.start(background=False)
            self.assertFalse(controller.snapshot()['configure_protection'])
            self.assertEqual(self.bus.memory, before)
            self.assertFalse(any(event[0] == 'write' for event in self.bus.events))
            controller.enable()
            controller.set_positions({'right_wrist_roll_joint': .05})
            controller.poll()
            self.assertTrue(controller.snapshot()['sample_valid'])
        finally:
            controller.close()
        self.assertTrue(any(event[0] == 'sync_write_positions' for event in self.bus.events))
        writes = [event[2] for event in self.bus.events if event[0] == 'write']
        self.assertTrue(writes)
        self.assertLessEqual(set(writes), {GOAL, TORQUE_ENABLE})
        for motor in self.motors:
            for address, length in ((CW, 2), (CCW, 2), (MAX_TORQUE, 2),
                                    (TORQUE_LIMIT, 2), (SHUTDOWN, 1)):
                self.assertEqual(self.bus.memory[motor.id][address:address+length],
                                 before[motor.id][address:address+length])

    def test_commands_respect_limits_already_stored_on_motor(self):
        self.bus.set_register(2, CCW, 620)
        controller = self.controller()
        try:
            controller.start(background=False)
            controller.enable()
            self.bus.events.clear()
            with self.assertRaisesRegex(ProtectionError, 'existing tighter hardware limits'):
                controller.set_positions({'right_wrist_roll_joint': .1})
            self.assertFalse(any(event[0] == 'sync_write_positions' for event in self.bus.events))
        finally:
            controller.close()


if __name__ == '__main__':
    unittest.main()
