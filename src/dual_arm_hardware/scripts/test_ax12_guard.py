#!/usr/bin/env python3
"""Fault-injection tests with an in-memory bus; no physical serial device opened."""
from copy import deepcopy
import math
from pathlib import Path
import tempfile
import unittest

import yaml
from ax12_guard import (ProtectedArm, ProtectionError, load_configuration, MODEL, CW, CCW,
                        MAX_TORQUE, TORQUE_LIMIT, RETURN_LEVEL, SHUTDOWN, TORQUE_ENABLE,
                        GOAL, PRESENT, REGISTERED, LOCK)

PACKAGE = Path(__file__).resolve().parents[1]
DESCRIPTION_CONFIG = PACKAGE.parent/'dual_arm_description/config'


class FakeBus:
    def __init__(self, motors):
        self.registers = {m.id: {MODEL: 12, CW: 0, CCW: 1023, MAX_TORQUE: 1023,
            TORQUE_LIMIT: 1023, RETURN_LEVEL: 2, SHUTDOWN: 36, TORQUE_ENABLE: 0,
            GOAL: 0, PRESENT: m.zero, REGISTERED: 0, LOCK: 0} for m in motors}
        self.events = []
        self.ignore = None
        self.fail_write = None
        self.fail_read_id = None

    def read(self, motor_id, address):
        self.events.append(('read', motor_id, address))
        if motor_id == self.fail_read_id:
            raise ProtectionError('Injected status/communication error')
        return self.registers[motor_id][address]

    def write(self, motor_id, address, value):
        self.events.append(('write', motor_id, address, value))
        if (motor_id, address, value) == self.fail_write:
            raise ProtectionError('Injected write failure')
        if (motor_id, address) != self.ignore:
            self.registers[motor_id][address] = value

    def writes(self):
        return [e for e in self.events if e[0]=='write']


class HardwareProtectionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name)/'hardware.yaml'
        self.config = yaml.safe_load((PACKAGE/'config/hardware.yaml').read_text())
        self.config.update(port='/dev/FAKE_TEST_ONLY', baudrate=1000000, torque_limit_percent=25.0)
        for index, entry in enumerate(self.config['motors'].values(), 1):
            entry.update(id=index, direction=1 if index%2 else -1)
        self.write_config()
        _, self.motors = load_configuration(self.path, DESCRIPTION_CONFIG)
        self.bus = FakeBus(self.motors)
        self.arm = ProtectedArm(self.bus, self.motors)

    def write_config(self):
        self.path.write_text(yaml.safe_dump(self.config, sort_keys=False))

    def configured(self, enabled=False):
        self.arm.prepare(apply=True)
        if enabled:
            self.arm.enable()
        self.bus.events.clear()

    def test_unfilled_real_config_is_rejected(self):
        with self.assertRaisesRegex(ProtectionError, 'Missing hardware settings'):
            load_configuration(PACKAGE/'config/hardware.yaml', DESCRIPTION_CONFIG)

    def test_bad_id_direction_or_percentage_is_rejected(self):
        good = deepcopy(self.config)
        first = self.motors[0].joint
        for value in (254, 255, -1, True, 1.5):
            self.config = deepcopy(good)
            self.config['motors'][first]['id'] = value
            self.write_config()
            with self.assertRaises(ProtectionError): load_configuration(self.path, DESCRIPTION_CONFIG)
        for value in (None, 0, 2, True):
            self.config = deepcopy(good)
            self.config['motors'][first]['direction'] = value
            self.write_config()
            with self.assertRaises(ProtectionError): load_configuration(self.path, DESCRIPTION_CONFIG)
        for value in (None, math.nan, math.inf, 0, -1, 101, True):
            self.config = deepcopy(good)
            self.config['torque_limit_percent'] = value
            self.write_config()
            with self.assertRaises(ProtectionError): load_configuration(self.path, DESCRIPTION_CONFIG)

    def test_duplicate_ids_are_rejected(self):
        for entry in self.config['motors'].values(): entry['id'] = 1
        self.write_config()
        with self.assertRaises(ProtectionError): load_configuration(self.path, DESCRIPTION_CONFIG)

    def test_neutral_direction_and_inward_rounding(self):
        for m in self.motors:
            self.assertEqual(m.zero, 614)
            self.assertEqual(m.to_position(0), 614)
            self.assertEqual(m.cap, 255)  # floor(1023*0.25), not 0.30 Nm conversion.
            endpoints = sorted([m.to_radians(m.lower), m.to_radians(m.upper)])
            self.assertGreaterEqual(endpoints[0], m.q_lower-1e-12)
            self.assertLessEqual(endpoints[1], m.q_upper+1e-12)
            self.assertEqual(m.to_position(m.q_lower), m.lower if m.direction==1 else m.upper)
            self.assertEqual(m.to_position(m.q_upper), m.upper if m.direction==1 else m.lower)

    def test_read_only_preflight_never_writes(self):
        self.arm.prepare(apply=False)
        self.assertFalse(self.arm.ready)
        self.assertEqual(self.bus.writes(), [])

    def test_all_devices_checked_before_any_write(self):
        self.bus.registers[self.motors[-1].id][MODEL] = 29
        with self.assertRaises(ProtectionError): self.arm.prepare(apply=True)
        self.assertEqual(self.bus.writes(), [])

    def test_enabled_motor_is_not_silently_disabled(self):
        self.bus.registers[1][TORQUE_ENABLE] = 1
        with self.assertRaises(ProtectionError): self.arm.prepare(apply=True)
        self.assertEqual(self.bus.writes(), [])
        self.assertEqual(self.bus.registers[1][TORQUE_ENABLE], 1)

    def test_unsafe_preconditions_block_all_writes(self):
        for address, value in [(RETURN_LEVEL, 1), (REGISTERED, 1), (TORQUE_LIMIT, 0),
                               (MAX_TORQUE, 0), (LOCK, 1), (PRESENT, 0)]:
            bus = FakeBus(self.motors)
            bus.registers[12][address] = value
            with self.assertRaises(ProtectionError): ProtectedArm(bus, self.motors).prepare(apply=True)
            self.assertEqual(bus.writes(), [])
        self.bus.registers[1][CCW] = 0
        with self.assertRaises(ProtectionError): self.arm.prepare(apply=True)
        self.assertEqual(self.bus.writes(), [])

    def test_apply_readback_and_no_motion_or_speed_writes(self):
        self.arm.prepare(apply=True)
        self.assertTrue(self.arm.ready)
        for m in self.motors:
            self.assertEqual(self.bus.registers[m.id][MAX_TORQUE], 255)
            self.assertEqual(self.bus.registers[m.id][TORQUE_LIMIT], 255)
            self.assertEqual(self.bus.registers[m.id][CW], m.lower)
            self.assertEqual(self.bus.registers[m.id][CCW], m.upper)
        self.assertFalse(any(e[2] in (TORQUE_ENABLE, GOAL, 32) for e in self.bus.writes()))
        self.bus.events.clear()
        self.arm.prepare(apply=True)
        self.assertEqual(self.bus.writes(), [])  # No repeated EEPROM wear.

    def test_stricter_device_caps_and_other_shutdown_bits_preserved(self):
        self.bus.registers[1].update({MAX_TORQUE: 190, TORQUE_LIMIT: 170, CW: 600, CCW: 650, SHUTDOWN: 1})
        self.arm.prepare(apply=True)
        self.assertEqual(self.bus.registers[1][MAX_TORQUE], 170)
        self.assertEqual(self.bus.registers[1][TORQUE_LIMIT], 170)
        self.assertEqual(self.bus.registers[1][CW], 600)
        self.assertEqual(self.bus.registers[1][CCW], 650)
        self.assertEqual(self.bus.registers[1][SHUTDOWN], 0x25)

    def test_wrong_readback_blocks_enable_without_rollback(self):
        self.bus.ignore = (5, MAX_TORQUE)
        with self.assertRaises(ProtectionError): self.arm.prepare(apply=True)
        self.assertFalse(self.arm.ready)
        self.assertTrue(self.arm.fault)
        self.assertEqual(self.bus.registers[1][MAX_TORQUE], 255)
        with self.assertRaises(ProtectionError): self.arm.enable()
        self.assertFalse(any(e[2]==TORQUE_ENABLE for e in self.bus.writes()))

    def test_enable_requires_prepare(self):
        with self.assertRaises(ProtectionError): self.arm.enable()
        self.assertEqual(self.bus.writes(), [])

    def test_enable_holds_measured_positions_before_any_motor_is_on(self):
        for m in self.motors:
            self.bus.registers[m.id][PRESENT] = m.zero+(1 if m.upper>m.zero else -1)
        self.configured()
        self.arm.enable()
        writes = self.bus.writes()
        first_enable = next(i for i,e in enumerate(writes) if e[2]==TORQUE_ENABLE)
        self.assertEqual({e[1] for e in writes[:first_enable] if e[2]==GOAL}, set(range(1,13)))
        for m in self.motors:
            self.assertEqual(self.bus.registers[m.id][GOAL], self.bus.registers[m.id][PRESENT])
            self.assertEqual(self.bus.registers[m.id][TORQUE_ENABLE], 1)

    def test_partial_enable_failure_latches_fault_and_disables_every_motor(self):
        self.configured()
        self.bus.fail_write = (7, TORQUE_ENABLE, 1)
        with self.assertRaises(ProtectionError): self.arm.enable()
        self.assertTrue(self.arm.fault)
        self.assertFalse(self.arm.enabled)
        self.assertTrue(all(r[TORQUE_ENABLE]==0 for r in self.bus.registers.values()))
        with self.assertRaises(ProtectionError): self.arm.enable()

    def test_command_bounds_checked_for_whole_batch_before_writes(self):
        self.configured(enabled=True)
        names = [m.joint for m in self.motors]
        for value in (math.nan, math.inf, 100.0):
            with self.assertRaises(ProtectionError): self.arm.command(names, [0.0]*11+[value])
        with self.assertRaises(ProtectionError): self.arm.command(names[:-1], [0.0]*11)
        with self.assertRaises(ProtectionError): self.arm.command(names[:-1]+[names[0]], [0.0]*12)
        self.assertEqual(self.bus.writes(), [])

    def test_changed_cap_stops_without_sending_goals(self):
        self.configured(enabled=True)
        self.bus.registers[12][TORQUE_LIMIT] = 1023
        with self.assertRaises(ProtectionError): self.arm.command([m.joint for m in self.motors], [0.0]*12)
        self.assertFalse(any(e[2]==GOAL for e in self.bus.writes()))
        self.assertTrue(self.arm.fault)
        self.assertTrue(all(r[TORQUE_ENABLE]==0 for r in self.bus.registers.values()))

    def test_valid_full_command_and_feedback_follow_motor_direction(self):
        self.configured(enabled=True)
        targets = [(m.q_lower+m.q_upper)/2 for m in self.motors]
        self.arm.command([m.joint for m in self.motors], targets)
        for m, value in zip(self.motors, targets):
            self.assertEqual(self.bus.registers[m.id][GOAL], m.to_position(value))
            self.bus.registers[m.id][PRESENT] = m.to_position(value)
        for m, value in zip(self.motors, targets):
            self.assertLessEqual(abs(self.arm.positions()[m.joint]-value), math.radians(300/1023))
        self.assertFalse(any(e[2]==32 for e in self.bus.writes()))

    def test_existing_tighter_angle_limit_rejects_command(self):
        self.bus.registers[1].update({CW: 600, CCW: 650})
        self.configured(enabled=True)
        with self.assertRaises(ProtectionError):
            self.arm.command([m.joint for m in self.motors], [self.motors[0].q_upper]+[0.0]*11)
        self.assertEqual(self.bus.writes(), [])

    def test_device_restart_is_not_automatically_reenabled(self):
        self.configured(enabled=True)
        self.bus.registers[1][TORQUE_ENABLE] = 0
        with self.assertRaises(ProtectionError): self.arm.command([m.joint for m in self.motors], [0.0]*12)
        self.assertTrue(self.arm.fault)
        self.assertFalse(any(e[2]==TORQUE_ENABLE and e[3]==1 for e in self.bus.writes()))

    def test_stop_transmitted_even_when_reads_fail_and_failure_reported(self):
        self.configured(enabled=True)
        self.bus.fail_read_id = 1
        errors = self.arm.stop(fault=True)
        off_ids = {e[1] for e in self.bus.writes() if e[2:]==(TORQUE_ENABLE,0)}
        self.assertEqual(off_ids, set(range(1,13)))
        self.assertEqual(len(errors), 1)
        self.assertIn('ID 1', errors[0])


if __name__ == '__main__':
    unittest.main()
