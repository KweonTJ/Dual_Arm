#!/usr/bin/env python3
"""Telemetry/driver fault tests. Never open a physical device or DDS node."""
import math
from types import SimpleNamespace, MethodType
import unittest

from ax12_guard import Motor, ProtectionError, SDKBus, CW, CCW, TORQUE_LIMIT
from hardware_telemetry import (read_telemetry, diagnostic_records, signed_magnitude,
                                START, LENGTH)


class BlockBus:
    def __init__(self):
        self.data = [0] * LENGTH
        self.events = []
        for address, value in ((30, 650), (34, 255), (36, 614), (38, 300), (40, 1536)):
            self.data[address - START] = value & 255
            self.data[address + 1 - START] = value >> 8
        self.data[42 - START], self.data[43 - START] = 111, 32

    def read_block(self, motor_id, address, length):
        self.events.append((motor_id, address, length))
        return self.data

    def write(self, *args):
        raise AssertionError('Telemetry must not write to a motor')


def make_arm(direction=1):
    motor = Motor('left_wrist_roll_joint', 7, direction, 614, 500, 800, 255, -1.2, 1.2)
    return SimpleNamespace(bus=BlockBus(), motors=[motor], expected={7: {CW: 500, CCW: 800, TORQUE_LIMIT: 255}},
                           enabled=False, fault=False, stop_errors=[])


class TelemetryTests(unittest.TestCase):
    def test_direction_bit_and_two_zeros(self):
        self.assertEqual([signed_magnitude(x) for x in (0, 1024, 300, 1324, 2047)],
                         [0, 0, 300, -300, -1023])
        for raw in (-1, 2048):
            with self.assertRaises(ProtectionError):
                signed_magnitude(raw)

    def test_units_neutral_and_calibrated_velocity(self):
        for direction in (1, -1):
            arm = make_arm(direction)
            sample = read_telemetry(arm)[arm.motors[0].joint]
            self.assertEqual(sample['position_deg'], 0)
            self.assertAlmostEqual(sample['motor_position_deg'], 614 * 300 / 1023)
            self.assertAlmostEqual(sample['goal_deg'], direction * 36 * 300 / 1023)
            self.assertAlmostEqual(sample['velocity_deg_s'], direction * 199.8)
            self.assertAlmostEqual(sample['load_percent'], -512 * 100 / 1023)
            self.assertEqual(sample['voltage_v'], 11.1)
            self.assertEqual(sample['temperature_c'], 32)
            self.assertAlmostEqual(sample['torque_limit_percent'], 255 * 100 / 1023)
            self.assertEqual(arm.bus.events, [(7, 24, 23)])
            self.assertNotIn('effort', sample)

    def test_corrupt_changed_and_unsafe_readings_rejected(self):
        for address, value in ((24, 1), (34, 0), (37, 0), (39, 8), (41, 8), (44, 1), (46, 2), (31, 4)):
            arm = make_arm()
            arm.bus.data[address - START] = value
            with self.subTest(address=address), self.assertRaises(ProtectionError):
                read_telemetry(arm)
        for packet in ([0] * 22, [0] * 22 + [256]):
            arm = make_arm()
            arm.bus.data = packet
            with self.assertRaises(ProtectionError):
                read_telemetry(arm)

    def test_fault_records_never_contain_previous_measurements(self):
        arm = make_arm()
        samples = read_telemetry(arm)
        good = diagnostic_records(arm, samples)
        self.assertEqual(good[1]['values']['sample_valid'], 'true')
        arm.fault = True
        arm.stop_errors = ['ID 7: OFF unconfirmed']
        records = diagnostic_records(arm, samples, 'ID 7: status error 0x20')
        self.assertEqual(records[0]['level'], 2)
        self.assertIn('OFF unconfirmed', records[0]['values']['torque_off_unconfirmed'])
        self.assertNotIn('position_deg', records[1]['values'])
        self.assertEqual(records[1]['values']['sample_valid'], 'false')

    def test_sdk_block_checks_communication_status_and_length(self):
        bus = SDKBus.__new__(SDKBus)
        bus.port, bus.success, bus.events = object(), 0, []
        for data, result, error, fails in (([0] * 23, 0, 0, False),
                                         ([0] * 23, -1, 0, True),
                                         ([0] * 23, 0, 32, True), ([0] * 22, 0, 0, True)):
            bus.packet = SimpleNamespace(readTxRx=lambda *a: (data, result, error),
                                         getTxRxResult=lambda r: 'comm failed',
                                         getRxPacketError=lambda e: 'overload')
            if fails:
                with self.assertRaises(ProtectionError):
                    bus.read_block(7, 24, 23)
            else:
                self.assertEqual(len(bus.read_block(7, 24, 23)), 23)
        self.assertEqual(len(bus.events), 4)


class DriverBridgeTests(unittest.TestCase):
    def make_node(self):
        # ROS message construction/serialization without Node initialization or DDS.
        from hardware_node import HardwareNode
        from builtin_interfaces.msg import Time
        arm = make_arm()
        arm.verify = lambda enabled: None
        stopped = []

        def stop(fault=False):
            stopped.append(True)
            arm.fault, arm.enabled = fault, False
            return []

        arm.stop = stop
        messages, diagnostics = [], []
        node = SimpleNamespace(arm=arm, fault_message='', last_rejection='',
            publisher=SimpleNamespace(publish=messages.append),
            diagnostics=SimpleNamespace(publish=diagnostics.append),
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=42))),
            get_logger=lambda: SimpleNamespace(fatal=lambda text: None), report_stop=lambda errors: None)
        node.publish_diagnostics = MethodType(HardwareNode.publish_diagnostics, node)
        node.monitor = MethodType(HardwareNode.monitor, node)
        return node, messages, diagnostics, stopped

    def test_feedback_serializes_without_invented_effort(self):
        from rclpy.serialization import serialize_message, deserialize_message
        from diagnostic_msgs.msg import DiagnosticArray
        node, messages, diagnostics, stopped = self.make_node()
        node.monitor()
        self.assertEqual(list(messages[0].position), [0.0])
        self.assertEqual(list(messages[0].effort), [])
        self.assertAlmostEqual(messages[0].velocity[0], math.radians(199.8))
        decoded = deserialize_message(serialize_message(diagnostics[0]), DiagnosticArray)
        self.assertEqual(decoded.status[0].message, 'TORQUE_OFF')
        self.assertEqual(decoded.status[1].hardware_id, '7')
        self.assertFalse(stopped)

    def test_fault_stops_once_and_keeps_publishing_error_without_io(self):
        node, messages, diagnostics, stopped = self.make_node()
        node.arm.bus.data[34 - START] = 0
        node.monitor()
        reads = len(node.arm.bus.events)
        node.monitor()
        self.assertEqual(len(stopped), 1)
        self.assertEqual(len(node.arm.bus.events), reads)
        self.assertEqual(messages, [])
        self.assertEqual(len(diagnostics), 2)
        self.assertTrue(all(msg.status[0].level == bytes([2]) for msg in diagnostics))
        self.assertTrue(all('position_deg' not in [v.key for v in msg.status[1].values] for msg in diagnostics))


if __name__ == '__main__':
    unittest.main()
