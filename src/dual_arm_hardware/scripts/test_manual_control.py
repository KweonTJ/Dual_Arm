"""Manual absolute-angle controls with non-neutral real-world starting poses."""
import os
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
from copy import deepcopy
from pathlib import Path
import tempfile
import time
import unittest

import yaml
from ax12_guard import CW,CCW,PRESENT,GOAL,TORQUE_ENABLE,TICKS_PER_DEGREE,ProtectionError
from manual_control import create_controller,absolute_targets
from mock_bus import MockBus

ROOT = Path(__file__).resolve().parents[3]
CONFIG = ROOT/'src/dual_arm_hardware/config/hardware.yaml'
DESCRIPTION = ROOT/'src/dual_arm_description/config'
LOOP = ROOT/'src/dual_arm_hardware/config/lowlevel.yaml'
POSITIONS = {1:112,2:521,3:470,4:496,5:504,6:490,7:498,8:553,9:522,10:543}


class ManualControlTests(unittest.TestCase):
    def factory(self):
        def bus_factory(motors):
            bus = MockBus(motors)
            for motor_id,raw in POSITIONS.items():
                bus.set_register(motor_id,PRESENT,raw);bus.set_register(motor_id,GOAL,raw)
            self.bus = bus
            return bus
        return create_controller(CONFIG,DESCRIPTION,LOOP,bus_factory)

    def test_absolute_targets_hold_other_motors_and_preserve_configuration(self):
        controller = self.factory(); before = deepcopy(self.bus.memory)
        try:
            controller.start(background=False)
            self.assertFalse(any(e[0]=='write' for e in self.bus.events))
            controller.enable()
            targets = {name:s['motor_position_deg'] for name,s in controller.snapshot()['samples'].items()}
            targets['right_wrist_roll_joint'] = 530/TICKS_PER_DEGREE
            controller.set_positions(absolute_targets(targets,controller.snapshot()),require_all=True)
            packets = [e[1] for e in self.bus.events if e[0]=='sync_write_positions']
            self.assertEqual(packets,[{**POSITIONS,2:530}])
            self.assertTrue(controller.snapshot()['sample_valid'])
        finally:
            controller.close()
        self.assertTrue(all(e[2] in (24,30) for e in self.bus.events if e[0]=='write'))
        for motor_id,memory in before.items():
            self.assertEqual(self.bus.memory[motor_id][:24],memory[:24])
            self.assertEqual(self.bus.memory[motor_id][32:36],memory[32:36])

    def test_device_range_not_containing_model_zero_is_supported(self):
        def bus_factory(motors):
            bus = MockBus(motors)
            bus.set_register(1,CW,80);bus.set_register(1,CCW,200)
            bus.set_register(1,PRESENT,112);bus.set_register(1,GOAL,112)
            self.bus = bus;return bus
        controller = create_controller(CONFIG,DESCRIPTION,LOOP,bus_factory)
        try:
            controller.start(background=False);controller.enable()
            state = controller.snapshot()
            targets = {n:s['motor_position_deg'] for n,s in state['samples'].items()}
            targets['right_gripper_joint'] = 120/TICKS_PER_DEGREE
            controller.set_positions(absolute_targets(targets,state),require_all=True)
            self.assertEqual(self.bus.read(1,GOAL),120)
            targets['right_gripper_joint'] = 250/TICKS_PER_DEGREE
            with self.assertRaises(ValueError): absolute_targets(targets,state)
        finally:
            controller.close()

    def test_invalid_batch_does_not_reach_bus(self):
        controller = self.factory()
        try:
            controller.start(background=False);controller.enable()
            state = controller.snapshot()
            targets = {n:s['motor_position_deg'] for n,s in state['samples'].items()}
            before = len(self.bus.events)
            for value in (float('nan'),float('inf'),True,-1,301):
                with self.subTest(value=value),self.assertRaises(ValueError):
                    absolute_targets({**targets,'right_wrist_roll_joint':value},state)
            with self.assertRaises(ValueError):absolute_targets({},state)
            self.assertEqual(len(self.bus.events),before)
        finally:
            controller.close()

    def test_desktop_seeds_current_pose_before_user_control_and_closes_OFF(self):
        from PyQt5 import QtWidgets
        from manual_app import ManualWindow
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        config = yaml.safe_load(CONFIG.read_text())
        with tempfile.TemporaryDirectory() as directory:
            window = ManualWindow(config,self.factory,Path(directory),'mock')
            def until(phase):
                deadline = time.monotonic()+3
                while time.monotonic()<deadline:
                    app.processEvents();window.tick()
                    state = window.bridge.snapshot()
                    if state['phase']==phase:return state
                    if state['phase']=='FAULT':self.fail(state['error'])
                    time.sleep(.005)
                self.fail('Timed out waiting for '+phase)
            try:
                window.connect_button.click();until('TORQUE_OFF');window.tick()
                self.assertFalse(window.spins['right_wrist_roll_joint'].isEnabled())
                self.assertEqual(window.sliders['right_gripper_joint'].value(),112)
                self.assertFalse(any(e[0]=='write' for e in self.bus.events))
                window.enable_button.click();until('ACTIVE');window.tick()
                self.assertFalse(any(e[0]=='sync_write_positions' for e in self.bus.events))
                window.sliders['right_wrist_roll_joint'].setValue(530)
                deadline = time.monotonic()+2
                while time.monotonic()<deadline:
                    app.processEvents();window.tick()
                    if self.bus.read(2,GOAL)==530:break
                    time.sleep(.005)
                self.assertEqual(self.bus.read(2,GOAL),530)
                self.assertEqual(self.bus.read(1,GOAL),112)
                self.assertEqual(self.bus.read(10,GOAL),543)
            finally:
                window.close();app.processEvents()
            self.assertTrue(self.bus.closed)
            self.assertTrue(all(memory[TORQUE_ENABLE]==0 for memory in self.bus.memory.values()))


if __name__=='__main__':unittest.main()
