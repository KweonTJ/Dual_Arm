"""Failure-path tests for the actual low-level API, no serial device opened."""
from copy import deepcopy
import math
from pathlib import Path
import tempfile
import time
import unittest

import yaml
from ax12_guard import (load_configuration,ProtectionError,GOAL,TORQUE_ENABLE,
                        TORQUE_LIMIT,MAX_TORQUE,PRESENT,CW,CCW)
from lowlevel_control import DualArmLowLevel,loop_settings
from lowlevel_cli import execute
from mock_bus import MockBus

ROOT=Path(__file__).resolve().parents[3]
DESCRIPTION=ROOT/'src/dual_arm_description/config'


class LowLevelTests(unittest.TestCase):
    def setUp(self):
        config=yaml.safe_load((ROOT/'src/dual_arm_hardware/config/hardware.yaml').read_text())
        config.update(port='/dev/FAKE',baudrate=1000000,configure_protection=True,torque_limit_percent=25.)
        for i,entry in enumerate(config['motors'].values()):entry['direction']=1 if i%2 else -1
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'config.yaml';p.write_text(yaml.safe_dump(config,sort_keys=False))
            _,self.motors=load_configuration(p,DESCRIPTION)
        self.now=10.
        self.bus=MockBus(self.motors)
        self.controller=DualArmLowLevel(self.bus,self.motors,DESCRIPTION,clock=lambda:self.now)
        self.controller.start(background=False)
        self.addCleanup(self.controller.close)
        self.bus.events.clear()

    def enabled(self):
        self.controller.enable();self.bus.events.clear()

    def test_start_is_OFF_and_enable_holds_measured_position(self):
        c=self.controller
        self.assertEqual(c.snapshot()['state'],'TORQUE_OFF')
        self.assertTrue(c.snapshot()['sample_valid'])
        for m in self.motors:self.bus.set_register(m.id,PRESENT,m.zero+(1 if m.upper>m.zero else -1))
        c.enable()
        for m in self.motors:
            self.assertEqual(self.bus.read(m.id,GOAL),self.bus.read(m.id,PRESENT))
        goals=[e for e in self.bus.events if e[0]=='write' and e[2]==GOAL]
        first_on=next(i for i,e in enumerate(self.bus.events) if e[:1]==('write',) and e[2:]==(TORQUE_ENABLE,1))
        self.assertTrue(all(self.bus.events.index(e)<first_on for e in goals))

    def test_failed_preflight_does_not_disable_an_existing_controller(self):
        bus=MockBus(self.motors)
        bus.set_register(self.motors[-1].id,TORQUE_ENABLE,1)
        c=DualArmLowLevel(bus,self.motors,DESCRIPTION)
        with self.assertRaises(ProtectionError):c.start(background=False)
        self.assertTrue(bus.closed)
        self.assertFalse(any(e[0]=='write' for e in bus.events))
        self.assertEqual(bus.memory[self.motors[-1].id][TORQUE_ENABLE],1)

    def test_subset_is_batched_once_and_preserves_other_goals(self):
        self.enabled();c=self.controller
        c.set_positions({'right_wrist_roll_joint':.1})
        packets=[e for e in self.bus.events if e[0]=='sync_write_positions']
        self.assertEqual(len(packets),1);self.assertEqual(len(packets[0][1]),10)
        for m in self.motors:
            self.assertEqual(packets[0][1][m.id],m.to_position(.1 if m.joint=='right_wrist_roll_joint' else 0))
        self.assertTrue(c.snapshot()['sample_valid'])

    def test_bad_commands_are_atomic_and_do_not_refresh_lease(self):
        self.enabled();c=self.controller
        for bad in ({}, {'bogus':0},{'right_wrist_roll_joint':float('nan')},
                    {'right_wrist_roll_joint':True},{'right_wrist_roll_joint':1e9}):
            with self.assertRaises(ProtectionError):c.set_positions(bad)
        with self.assertRaises(ProtectionError):c.set_positions({'right_wrist_roll_joint':0},require_all=True)
        with self.assertRaises(ProtectionError):execute(c,'set right_wrist_roll_joint 0 right_wrist_roll_joint 0.1')
        self.assertFalse(any(e[0] in ('write','sync_write_positions') for e in self.bus.events))
        self.now+=1.01
        with self.assertRaises(ProtectionError):c.poll()
        self.assertIn('timeout',c.snapshot()['fault'])

    def test_watchdog_torque_OFF_is_latched_and_never_auto_reenabled(self):
        self.enabled();c=self.controller
        self.now+=1.01
        with self.assertRaises(ProtectionError):c.poll()
        self.assertTrue(c.arm.fault);self.assertFalse(c.arm.enabled)
        self.assertEqual(c.snapshot()['samples'],{})
        for m in self.motors:self.assertEqual(self.bus.read(m.id,TORQUE_ENABLE),0)
        for operation in (c.enable,c.keepalive,lambda:c.set_positions({'right_gripper_joint':0})):
            with self.assertRaises(ProtectionError):operation()

    def test_keepalive_does_not_bypass_an_expired_lease(self):
        self.enabled();self.now+=1.01
        with self.assertRaises(ProtectionError):self.controller.keepalive()
        self.assertTrue(self.controller.arm.fault)

    def test_live_keepalive_and_successful_commands_extend_lease(self):
        self.enabled();c=self.controller
        for _ in range(3):
            self.now+=.7;c.keepalive();c.poll()
        self.now+=.7;c.set_gripper('right',20)
        self.now+=.7;c.poll()
        self.assertFalse(c.arm.fault)

    def test_missing_broadcast_recipient_stops_all_and_reports_readback_failure(self):
        self.enabled();c=self.controller
        motor=c.by_name['right_wrist_roll_joint'];self.bus.ignore_sync_id=motor.id
        with self.assertRaisesRegex(ProtectionError,'readback differs'):
            c.set_positions({motor.joint:.2})
        self.assertTrue(c.arm.fault)
        for m in self.motors:self.assertEqual(self.bus.read(m.id,TORQUE_ENABLE),0)

    def test_changed_cap_blocks_goal_and_preserves_lower_cap(self):
        self.enabled();c=self.controller
        m=self.motors[-1];self.bus.set_register(m.id,TORQUE_LIMIT,100)
        with self.assertRaises(ProtectionError):c.set_positions({'right_wrist_roll_joint':.1})
        self.assertFalse(any(e[0]=='sync_write_positions' for e in self.bus.events))
        self.assertEqual(self.bus.read(m.id,TORQUE_LIMIT),100)

    def test_lost_motor_attempts_stop_for_every_ID_and_marks_unconfirmed(self):
        self.enabled();c=self.controller
        self.bus.fail_read_id=self.motors[-1].id
        with self.assertRaises(ProtectionError):c.poll()
        off=[e[1] for e in self.bus.events if e[0]=='write' and e[2:]==(TORQUE_ENABLE,0)]
        self.assertEqual(set(off),{m.id for m in self.motors})
        self.assertEqual(len(c.snapshot()['torque_off_unconfirmed']),1)
        self.assertFalse(c.snapshot()['sample_valid'])

    def test_external_goal_change_stops_and_emergency_stop_is_latched(self):
        self.enabled();c=self.controller
        self.bus.set_register(self.motors[0].id,GOAL,self.motors[0].zero+1)
        with self.assertRaisesRegex(ProtectionError,'outside this session'):c.poll()
        self.assertTrue(c.arm.fault)

    def test_gripper_mm_and_passive_TF_remain_centered(self):
        self.enabled();c=self.controller
        for gap in (0,35,70):
            snap=c.set_gripper('left',gap)
            q=snap['joint_positions_rad']
            self.assertAlmostEqual(q['left_jaw_a_joint'],q['left_jaw_b_joint'])
            self.assertAlmostEqual(2000*q['left_jaw_a_joint'],gap,delta=.3) # AX encoder resolution.
        for side,gap in [('other',1),('right',-1),('left',71),('left',float('inf'))]:
            with self.assertRaises(ProtectionError):c.set_gripper(side,gap)

    def test_stale_cache_is_never_published_as_fresh(self):
        self.now+=.51
        self.assertFalse(self.controller.snapshot()['sample_valid'])
        self.assertEqual(self.controller.snapshot()['joint_positions_rad'],{})

    def test_slow_bus_does_not_send_a_target_after_lease_expiry(self):
        self.enabled();c=self.controller
        verify=c.arm.verify
        def slow(enabled):verify(enabled);self.now+=1.01
        c.arm.verify=slow
        with self.assertRaises(ProtectionError):c.set_positions({'right_wrist_roll_joint':.1})
        self.assertFalse(any(e[0]=='sync_write_positions' for e in self.bus.events))

    def test_close_stops_and_closes_port(self):
        self.enabled();c=self.controller
        self.assertEqual(c.close(),[]);self.assertTrue(self.bus.closed)
        self.assertEqual(c.snapshot()['state'],'CLOSED')

    def test_hold_uses_measured_position_and_context_exit_reports_unconfirmed_OFF(self):
        self.enabled();c=self.controller
        c.set_positions({'right_wrist_roll_joint':.1});snap=c.hold()
        self.assertAlmostEqual(snap['samples']['right_wrist_roll_joint']['error_deg'],0)
        self.bus.fail_write_id=self.motors[-1].id
        with self.assertRaisesRegex(ProtectionError,'OFF unconfirmed'):c.__exit__(None,None,None)

    def test_background_watchdog_is_independent_of_client_polling(self):
        c=self.controller
        c.settings['control_hz']=50.;c.settings['command_timeout_s']=.06
        c.clock=time.monotonic;c._verify_at=0
        c.enable()
        import threading
        c._thread=threading.Thread(target=c._run,daemon=True);c._thread.start()
        c._thread.join(timeout=.5)
        self.assertFalse(c._thread.is_alive());self.assertTrue(c.arm.fault)
        self.assertFalse(c.arm.enabled)

    def test_loop_settings_reject_invalid_timing(self):
        for bad in ({'control_hz':0},{'other':1},{'control_hz':float('nan')},
                    {'command_timeout_s':.01},{'control_hz':101},{'protection_check_hz':30}):
            with self.assertRaises(ProtectionError):loop_settings(bad)


if __name__=='__main__':unittest.main()
