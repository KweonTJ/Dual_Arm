#!/usr/bin/env python3
"""Exercise real MuJoCo dynamics and the desktop controls without hardware."""
import os
os.environ.setdefault('MUJOCO_GL', 'egl')
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import csv
from copy import deepcopy
from pathlib import Path
import tempfile
import unittest

import mujoco
import numpy as np
from simulation_model import build_scene, Simulation, controller_settings

ROOT = Path(__file__).resolve().parents[3]


class PhysicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.scene, cls.manifest = build_scene(ROOT / 'src/dual_arm_description/urdf/dual_arm.urdf',
            ROOT / 'src/dual_arm_description', ROOT / 'src/dual_arm_simulation/config/simulation.yaml', cls.temp.name)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def setUp(self):
        self.sim = Simulation(self.scene, self.manifest)

    def test_robot_mass_joints_and_wrist_offset_preserved(self):
        sim = self.sim
        self.assertEqual(len(sim.names), 12)
        self.assertEqual(sim.model.nu, 12)
        self.assertEqual(sim.model.nq, 19)  # 12 hinges + test object's free joint.
        self.assertAlmostEqual(sum(sim.model.body_mass) - .025, 1.0912)
        for side in ('left', 'right'):
            palm = sim.data.body(side + '_palm_link').xpos
            forearm = sim.data.body(side + '_forearm_link').xpos
            self.assertAlmostEqual(palm[0] - forearm[0], .013, places=6)
        self.assertEqual(sim.data.ncon, 0)

    def test_command_is_bounded_and_does_not_teleport(self):
        sim = self.sim
        before = sim.data.qpos.copy()
        sim.set_targets(np.full(12, 99))
        np.testing.assert_equal(sim.targets, sim.upper)
        np.testing.assert_equal(sim.data.qpos, before)
        for invalid in (np.zeros(11), np.full(12, np.nan), np.full(12, np.inf)):
            with self.assertRaises(ValueError):
                sim.set_targets(invalid)
        np.testing.assert_equal(sim.targets, sim.upper)

    def test_torque_cap_and_joint_stops_under_large_step(self):
        sim = self.sim
        sim.set_targets(sim.upper)
        saw_saturation = False
        max_violation = 0
        for _ in range(200):
            sim.step(10)
            sample = sim.snapshot()
            self.assertTrue(np.all(np.abs(sample['effort']) <= sim.caps + 1e-10))
            saw_saturation |= any(sample['saturated'])
            max_violation = max(max_violation, max(sim.lower - sample['position']), max(sample['position'] - sim.upper))
        self.assertTrue(saw_saturation)
        # Physical constraints have solver tolerance, unlike hard target clipping.
        self.assertLess(max_violation, np.radians(.5))
        self.assertFalse(sim.fault)
        self.assertGreater(np.max(np.abs(sample['position'])), .2)

    def test_gravity_and_floor_contact(self):
        sim = self.sim
        initial = sim.data.body('test_block').xpos[2]
        sim.step(1500)
        self.assertGreater(initial, .05)
        self.assertAlmostEqual(sim.data.body('test_block').xpos[2], .018, delta=.001)
        contacts = [(sim.model.geom(c.geom1).name, sim.model.geom(c.geom2).name) for c in sim.data.contact]
        self.assertTrue(any(set(pair) == {'floor', 'test_block_geom'} for pair in contacts))

    def test_motor_off_obeys_gravity_and_reenable_holds_position(self):
        sim = self.sim
        sim.data.qpos[sim.qindices[0]] = .5
        mujoco.mj_forward(sim.model, sim.data)
        sim.set_enabled(False)
        sim.step(1000)
        self.assertLess(sim.data.qpos[sim.qindices[0]], .4)
        np.testing.assert_equal(sim.snapshot()['effort'], np.zeros(12))
        sim.set_enabled(True)
        np.testing.assert_allclose(sim.targets, np.clip(sim.data.qpos[sim.qindices], sim.lower, sim.upper))

    def test_pause_and_reset(self):
        sim = self.sim
        sim.step(100)
        before = sim.data.qpos.copy()
        sim.paused = True
        sim.step(500)
        np.testing.assert_equal(before, sim.data.qpos)
        self.assertAlmostEqual(sim.data.time, .1)
        sim.reset()
        self.assertEqual(sim.data.time, 0)
        np.testing.assert_equal(sim.targets, np.zeros(12))

    def test_bad_physics_state_latches_fault(self):
        sim = self.sim
        # A solver warning must not be silently ignored and auto-resumed.
        sim.data.warning[mujoco.mjtWarning.mjWARN_BADQACC].number = 1
        with self.assertRaises(RuntimeError):
            sim.step()
        self.assertFalse(sim.enabled)
        self.assertTrue(sim.fault)
        with self.assertRaises(RuntimeError):
            sim.set_enabled(True)
        sim.reset()
        self.assertFalse(sim.fault)

    def test_controller_configuration_and_engine_caps_match(self):
        sim = self.sim
        for joint, cap in zip(self.manifest['joints'], sim.caps):
            self.assertEqual(joint['urdf_effort'], .3)
            np.testing.assert_allclose(sim.model.actuator(joint['name'] + '_servo').forcerange, [-cap, cap])
            np.testing.assert_allclose(sim.model.jnt_actfrcrange[sim.model.joint(joint['name']).id], [-cap, cap])
        for key, value in (('kp', 0), ('kd', -1), ('ki', float('nan')),
                           ('integral_limit_nm', 10), ('effort_limit_nm', True)):
            config = deepcopy(self.manifest['config'])
            config['controller']['profiles']['shoulder_pitch'][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                controller_settings(config, sim.names)

    def test_shoulders_hold_90_without_uncapped_compensation(self):
        sim = self.sim
        values = np.zeros(12)
        pitch = [i for i, name in enumerate(sim.names) if 'shoulder_pitch' in name]
        values[pitch] = np.pi / 2
        sim.set_targets(values)
        sim.step(6000)
        sample = sim.snapshot()
        self.assertLess(max(abs(np.degrees(sample['error']))), .5)
        for i in pitch:
            self.assertGreater(sample['effort'][i], .50)
            self.assertLess(sample['effort'][i], sim.caps[i])
        # No free external force or gravity disabling used to fake tracking.
        np.testing.assert_equal(sim.data.qfrc_applied, np.zeros(sim.model.nv))
        np.testing.assert_equal(sim.model.opt.gravity, [0, 0, -9.81])
        np.testing.assert_equal(sim.model.body_gravcomp, np.zeros(sim.model.nbody))

    def test_anti_windup_under_blocked_motion_and_recovery(self):
        sim = self.sim
        wrist = sim.names.index('left_wrist_roll_joint')
        values = np.zeros(12)
        values[wrist] = .5
        sim.set_targets(values)
        # Opposing load exceeds the motor cap; the joint rests against its stop.
        sim.data.qfrc_applied[sim.vindices[wrist]] = -.4
        sim.step(3000)
        self.assertTrue(sim.snapshot()['saturated'][wrist])
        self.assertLess(abs(sim.integral_torque[wrist]), 1e-8)
        sim.data.qfrc_applied[:] = 0
        sim.step(6000)
        self.assertLess(abs(np.degrees(sim.snapshot()['error'][wrist])), .5)
        sim.set_enabled(False)
        np.testing.assert_equal(sim.integral_torque, np.zeros(12))
        np.testing.assert_equal(sim.feedforward, np.zeros(12))

    def test_target_reversal_clears_integral_and_recovers(self):
        sim = self.sim
        sim.set_targets(sim.upper)
        sim.step(2000)
        sim.set_targets(sim.lower)
        np.testing.assert_equal(sim.integral_torque, np.zeros(12))
        sim.step(6000)
        self.assertLess(max(abs(np.degrees(sim.snapshot()['error']))), .5)

    def test_desktop_slider_camera_pause_and_csv(self):
        from PyQt5 import QtWidgets
        from simulation_app import SimulationWindow
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        window = SimulationWindow(self.sim, Path(self.temp.name) / 'records')
        try:
            self.assertEqual(len(window.spins), 12)
            window.shoulder_targets()
            self.assertEqual(window.spins[0].value(), 90)
            self.assertEqual(window.spins[6].value(), 90)
            window.spins[3].setValue(67.5)
            self.assertAlmostEqual(np.degrees(self.sim.targets[3]), 67.5)
            window.spins[3].setValue(100)
            self.assertAlmostEqual(np.degrees(self.sim.targets[3]), 67.5)
            window.bend_targets()
            self.sim.step(100)
            window.refresh()
            self.assertTrue(window.viewport.pixmap() is not None)
            window.pause_button.setChecked(True)
            self.assertTrue(self.sim.paused)
            window.torque_button.setChecked(False)
            np.testing.assert_equal(self.sim.snapshot()['effort'], np.zeros(12))
            window.viewport.reset_camera(True)
            self.assertEqual(window.viewport.camera.azimuth, 0)
            window.record_button.setChecked(True)
            window.refresh()
            window.record_button.setChecked(False)
            path = next((Path(self.temp.name) / 'records').glob('*.csv'))
            with path.open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 12)
            self.assertIn('actuator_torque_nm', rows[0])
            self.assertNotIn('temperature_c', rows[0])
        finally:
            window.close()
            app.processEvents()


if __name__ == '__main__':
    unittest.main()
