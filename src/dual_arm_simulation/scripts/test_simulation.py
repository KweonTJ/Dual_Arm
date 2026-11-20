#!/usr/bin/env python3
"""Exercise real MuJoCo dynamics and the desktop controls without hardware."""
import os
import sys
if sys.platform.startswith('linux'):
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
        self.assertEqual(len(sim.names), 10)
        self.assertEqual(sim.model.nu, 10)
        self.assertEqual(sim.model.nq, 25)  # 10 motor hinges + 8 passive joints + free object.
        self.assertEqual(sim.manifest['engine'], mujoco.__version__)
        self.assertEqual(sim.motor_ids, [10, 9, 8, 7, 6, 5, 4, 3, 2, 1])
        self.assertAlmostEqual(sum(sim.model.body_mass) - .025, 1.780994524904885,places=5)
        for side in ('left', 'right'):
            palm = sim.data.body(side + '_palm_link').xpos
            forearm = sim.data.body(side + '_forearm_link').xpos
            self.assertAlmostEqual(palm[0] - forearm[0], .013, places=6)
        self.assertEqual(sim.data.ncon, 0)

    def test_command_is_bounded_and_does_not_teleport(self):
        sim = self.sim
        before = sim.data.qpos.copy()
        sim.set_targets(np.full(10, 99))
        np.testing.assert_equal(sim.targets, sim.upper)
        np.testing.assert_equal(sim.data.qpos, before)
        for invalid in (np.zeros(9), np.full(10, np.nan), np.full(10, np.inf)):
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
        np.testing.assert_equal(sim.snapshot()['effort'], np.zeros(10))
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
        np.testing.assert_equal(sim.targets, np.zeros(10))

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

    def test_heavier_CAD_hands_expose_shoulder_torque_saturation(self):
        sim = self.sim
        values = np.zeros(10)
        pitch = [i for i, name in enumerate(sim.names) if 'shoulder_pitch' in name]
        values[pitch] = np.pi / 2
        sim.set_targets(values)
        sim.step(6000)
        sample = sim.snapshot()
        # Estimated steel CAD components increase load beyond existing caps.
        self.assertGreater(max(abs(np.degrees(sample['error']))), .5)
        for i in pitch:
            self.assertTrue(sample['saturated'][i])
            self.assertLessEqual(abs(sample['effort'][i]),sim.caps[i]+1e-10)
        # No free external force or gravity disabling used to fake tracking.
        np.testing.assert_equal(sim.data.qfrc_applied, np.zeros(sim.model.nv))
        np.testing.assert_equal(sim.model.opt.gravity, [0, 0, -9.81])
        np.testing.assert_equal(sim.model.body_gravcomp, np.zeros(sim.model.nbody))

    def test_anti_windup_under_blocked_motion_and_recovery(self):
        sim = self.sim
        wrist = sim.names.index('left_wrist_roll_joint')
        values = np.zeros(10)
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
        np.testing.assert_equal(sim.integral_torque, np.zeros(10))
        np.testing.assert_equal(sim.feedforward, np.zeros(10))

    def test_target_reversal_clears_integral_and_reports_limited_motion(self):
        sim = self.sim
        sim.set_targets(sim.upper)
        sim.step(2000)
        sim.set_targets(sim.lower)
        np.testing.assert_equal(sim.integral_torque, np.zeros(10))
        sim.step(6000)
        sample=sim.snapshot()
        for error,saturated in zip(np.degrees(sample['error']),sample['saturated']):
            if abs(error) >= .5:
                self.assertTrue(saturated) # New CAD load can exceed the existing elbow cap.
        self.assertTrue(np.all(np.abs(sample['effort']) <= sim.caps+1e-10))

    def test_slider_steps_and_reversals_settle_without_visible_rebound(self):
        sim = self.sim
        for index, name in enumerate(sim.names):
            sim.reset()
            for target_deg in (30, 5, 15, 0):
                with self.subTest(joint=name, target_deg=target_deg):
                    start_deg = np.degrees(sim.data.qpos[sim.qindices[index]])
                    target = np.zeros(len(sim.names))
                    target[index] = np.radians(target_deg)
                    sim.set_targets(target)
                    positions = []
                    for _ in range(600):
                        sim.step(5)
                        positions.append(np.degrees(sim.data.qpos[sim.qindices[index]]))
                    direction = np.sign(target_deg - start_deg)
                    overshoot = max(0., np.max(direction * (np.array(positions) - target_deg)))
                    # Below one AX-12A encoder increment (300/1023 degrees).
                    self.assertLess(overshoot, .25)
                    self.assertLess(np.ptp(positions[-100:]), .1)
                    # Large shoulder/elbow steps may stall at the torque cap.
                    if target_deg != 30:
                        self.assertAlmostEqual(positions[-1], target_deg, delta=.5)

    def test_continuous_slider_drag_and_release_do_not_rebound(self):
        sim = self.sim
        drag = np.concatenate((np.linspace(0, 20, 20), np.linspace(20, 5, 10),
                               np.linspace(5, 15, 10)))
        for index, name in enumerate(sim.names):
            with self.subTest(joint=name):
                sim.reset()
                target = np.zeros(len(sim.names))
                for degrees in drag:
                    target[index] = np.radians(degrees)
                    sim.set_targets(target)
                    sim.step(33)  # Match desktop refresh cadence while dragging.
                positions = []
                for _ in range(600):
                    sim.step(5)
                    positions.append(np.degrees(sim.data.qpos[sim.qindices[index]]))
                self.assertLess(max(positions) - 15, .25)
                self.assertLess(np.ptp(positions[-100:]), .1)
                self.assertAlmostEqual(positions[-1], 15, delta=.5)

    def test_single_motor_drives_both_CAD_jaws_through_closed_loops(self):
        sim=self.sim
        for fraction in (.5,1,0):
            values=np.zeros(len(sim.names))
            for i,name in enumerate(sim.names):
                if name.endswith('_gripper_joint'): values[i]=sim.upper[i]*fraction
            sim.set_targets(values);sim.step(5000)
            for side in ('left','right'):
                travel=[]
                for tag in ('a','b'):
                    joint=sim.model.joint(f'{side}_jaw_{tag}_joint')
                    travel.append(sim.data.qpos[joint.qposadr[0]])
                    residual=np.linalg.norm(sim.data.site(f'{side}_rod_{tag}_pin').xpos-
                                            sim.data.site(f'{side}_jaw_{tag}_pin').xpos)
                    self.assertLess(residual,1e-5)
                self.assertAlmostEqual(travel[0],travel[1],delta=1e-5)
                if fraction in (0,1):self.assertAlmostEqual(sum(travel),.070*fraction,delta=1e-4)
            self.assertTrue(np.all(np.abs(sim.snapshot()['effort'])<=sim.caps+1e-10))

    def test_desktop_slider_camera_pause_and_csv(self):
        from PyQt5 import QtWidgets
        from simulation_app import SimulationWindow
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        window = SimulationWindow(self.sim, Path(self.temp.name) / 'records')
        try:
            self.assertEqual(len(window.spins), 10)
            self.assertIn(f'MuJoCo {mujoco.__version__}', window.windowTitle())
            for name, motor_id in zip(self.sim.names, self.sim.motor_ids):
                label = window.findChild(QtWidgets.QLabel, name + '_motor_label')
                self.assertTrue(label.text().startswith(f'ID {motor_id:02d} ·'))
            window.shoulder_targets()
            self.assertEqual(window.spins[0].value(), 90)
            self.assertEqual(window.spins[5].value(), 90)
            window.spins[3].setValue(67.5)
            self.assertAlmostEqual(np.degrees(self.sim.targets[3]), 67.5)
            window.spins[3].setValue(100)
            self.assertAlmostEqual(np.degrees(self.sim.targets[3]), 67.5)
            window.bend_targets()
            self.sim.step(100)
            window.refresh()
            self.assertIn(f'MuJoCo {mujoco.__version__}', window.status.text())
            self.assertTrue(window.viewport.pixmap() is not None)
            window.pause_button.setChecked(True)
            self.assertTrue(self.sim.paused)
            window.torque_button.setChecked(False)
            np.testing.assert_equal(self.sim.snapshot()['effort'], np.zeros(10))
            window.viewport.reset_camera(True)
            self.assertEqual(window.viewport.camera.azimuth, 0)
            window.record_button.setChecked(True)
            window.refresh()
            window.record_button.setChecked(False)
            path = next((Path(self.temp.name) / 'records').glob('*.csv'))
            with path.open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 10)
            self.assertEqual({row['joint']: int(row['motor_id']) for row in rows},
                             dict(zip(self.sim.names, self.sim.motor_ids)))
            self.assertIn('actuator_torque_nm', rows[0])
            self.assertNotIn('temperature_c', rows[0])
        finally:
            window.close()
            app.processEvents()

    def test_measured_view_is_separate_and_alignment_is_explicit(self):
        sim=self.sim
        measured={n:0. for n in sim.names};measured['right_wrist_roll_joint']=.2
        original=sim.data.qpos.copy();targets=sim.targets.copy()
        display=sim.measured_data(measured)
        np.testing.assert_equal(sim.data.qpos,original)
        np.testing.assert_equal(sim.targets,targets)
        index=sim.names.index('right_wrist_roll_joint')
        self.assertAlmostEqual(display.qpos[sim.qindices[index]],.2)
        sim.align_to_measured(measured)
        self.assertAlmostEqual(sim.data.qpos[sim.qindices[index]],.2)
        self.assertAlmostEqual(sim.targets[index],.2)
        with self.assertRaises(ValueError):sim.measured_data({})
        with self.assertRaises(ValueError):sim.align_to_measured({**measured,'right_wrist_roll_joint':100})

    def test_twin_UI_seed_shared_target_pause_feedback_and_CSV(self):
        import time
        from PyQt5 import QtWidgets
        from simulation_app import SimulationWindow
        from twin_bridge import TwinBridge,controller_factory
        app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        bridge=TwinBridge(self.sim.names,self.sim.motor_ids,lambda:controller_factory('mock',
            ROOT/'src/dual_arm_description',None,ROOT/'src/dual_arm_hardware/config/lowlevel.yaml'),'mock')
        window=SimulationWindow(self.sim,Path(self.temp.name)/'twin_records',bridge)
        def wait_for(phase):
            deadline=time.monotonic()+3
            while time.monotonic()<deadline:
                app.processEvents();window.twin_tick()
                if bridge.snapshot()['phase']==phase:return
                time.sleep(.005)
            self.fail(str(bridge.snapshot()))
        try:
            window.twin_connect.click();wait_for('TORQUE_OFF')
            window.spins[3].setValue(30)
            window.twin_enable.click();wait_for('ACTIVE')
            self.assertAlmostEqual(self.sim.targets[3],0) # Discard the old GUI goal.
            window.spins[3].setValue(10)
            deadline=time.monotonic()+2
            while time.monotonic()<deadline:
                app.processEvents();window.twin_tick()
                if abs(float(window.measured_cells[3][0].text())-10)<.3:break
                time.sleep(.005)
            self.assertAlmostEqual(float(window.measured_cells[3][0].text()),10,delta=.3)
            window.measured_view.setChecked(True);window.refresh()
            self.assertIsNotNone(window.viewport.measured)
            window.pause_button.setChecked(True);wait_for('ACTIVE')
            self.assertFalse(window.spins[3].isEnabled());self.assertFalse(window.pose_buttons[0].isEnabled())
            window.record_button.setChecked(True);window.refresh();window.record_button.setChecked(False)
            path=next((Path(self.temp.name)/'twin_records').glob('*.csv'))
            with path.open() as stream:rows=list(csv.DictReader(stream))
            self.assertEqual({r['twin_mode'] for r in rows},{'mock'})
            self.assertTrue(all(r['measured_deg'] for r in rows))
            window.twin_stop.click();wait_for('FAULT');window.refresh()
            self.assertEqual(window.measured_cells[3][0].text(),'--')
            self.assertIn('실측 데이터 없음',window.viewport.text())
        finally:
            window.close();app.processEvents()


if __name__ == '__main__':
    unittest.main()
