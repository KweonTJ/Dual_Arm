#!/usr/bin/env python3
"""Regression checks for bounded synthetic states and the real Humble JSP parser."""
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
import xml.dom.minidom
import xml.etree.ElementTree as ET

from preview_limits import PreviewLimiter, read_limits

DESCRIPTION = (Path(__file__).resolve().parents[1]/'urdf/dual_arm.urdf').read_text()


class PreviewLimitsTests(unittest.TestCase):
    def setUp(self):
        self.limits = read_limits(DESCRIPTION)
        self.names = list(self.limits)
        self.guard = PreviewLimiter(self.limits)

    def test_requested_ranges_in_expanded_urdf(self):
        expected = {'shoulder_pitch': (-30, 60), 'shoulder_roll': (0, 45),
                    'elbow_pitch': (0, 90), 'wrist_roll': (-67.5, 67.5),
                    'inner_finger': (0, 30), 'outer_finger': (0, 30)}
        self.assertEqual(len(self.limits), 12)
        for side in ('left', 'right'):
            for role, values in expected.items():
                limit = self.limits[f'{side}_{role}_joint']
                for actual, degrees in zip((limit.lower, limit.upper), values):
                    self.assertAlmostEqual(actual, math.radians(degrees))

    def test_extreme_targets_and_reversal_are_instant_but_angle_bounded(self):
        now = 0.0
        for direction in (1, -1):
            self.guard.accept(self.names, [direction*1e6]*12, now)
            now += .001
            positions = self.guard.step(now)
            for name, limit in self.limits.items():
                self.assertEqual(positions[name], limit.upper if direction>0 else limit.lower)

    def test_bad_batches_are_rejected_atomically(self):
        self.guard.accept(self.names, [.1]*12, 0.0)
        original = dict(self.guard.targets)
        bad = [(self.names[:-1], [.2]*11), (self.names, [.2]*11),
               (self.names[:-1]+[self.names[0]], [.2]*12),
               (self.names[:-1]+['unknown'], [.2]*12)]
        bad += [(self.names, [.2]*11+[value]) for value in (math.nan, math.inf, -math.inf)]
        for names, positions in bad:
            with self.assertRaises(ValueError):
                self.guard.accept(names, positions, .1)
            self.assertEqual(self.guard.targets, original)
            self.assertEqual(self.guard.last_input, 0.0)

    def test_missing_and_stale_input_hold_current_pose(self):
        positions = self.guard.step(.1)
        self.assertTrue(all(value == 0 for value in positions.values()))
        self.guard.accept(self.names, [1.0]*12, .1)
        positions = self.guard.step(.2)
        self.guard.accept(self.names, [0.0]*12, .2)
        held = self.guard.step(.8)
        self.assertEqual(held, positions)
        self.assertEqual(self.guard.step(4.0), positions)

    def test_fresh_input_after_pause_applies_full_target(self):
        self.guard.accept(self.names, [1.0]*12, 100.0)
        positions = self.guard.step(100.0)
        for name, limit in self.limits.items():
            self.assertEqual(positions[name], min(1.0, limit.upper))

    def test_invalid_urdf_fails_closed(self):
        for attribute, value in [('lower','nan'), ('upper','-1'), ('velocity','0'),
                                 ('velocity','inf'), ('effort','0')]:
            root = ET.fromstring(DESCRIPTION)
            root.find("joint[@type='revolute']/limit").set(attribute, value)
            with self.assertRaises(ValueError):
                read_limits(ET.tostring(root, encoding='unicode'))

    def test_installed_joint_state_publisher_reads_the_same_ranges(self):
        # Run the installed ROS parser without starting a node or DDS transport.
        from joint_state_publisher.joint_state_publisher import JointStatePublisher
        probe = SimpleNamespace(use_small=True, use_mimic=True, dependent_joints={},
                                zeros={}, pub_def_positions=True, pub_def_vels=False,
                                pub_def_efforts=False, free_joints={}, joint_list=[])
        probe._init_joint = lambda lo, hi, zero: JointStatePublisher._init_joint(probe, lo, hi, zero)
        JointStatePublisher.init_urdf(probe, xml.dom.minidom.parseString(DESCRIPTION))
        self.assertEqual(set(probe.free_joints), set(self.limits))
        for name, limit in self.limits.items():
            joint = probe.free_joints[name]
            self.assertEqual((joint['min'], joint['max']), (limit.lower, limit.upper))
            self.assertEqual(joint['zero'], 0.0)


if __name__ == '__main__':
    unittest.main()
