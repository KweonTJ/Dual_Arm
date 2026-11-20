"""Coupling tests with real low-level logic and an offline register emulator."""
from pathlib import Path
import time
import unittest

from twin_bridge import TwinBridge,controller_factory

ROOT=Path(__file__).resolve().parents[3]
DESCRIPTION=ROOT/'src/dual_arm_description'


class TwinBridgeTests(unittest.TestCase):
    def setUp(self):
        self.backend=controller_factory('mock',DESCRIPTION,None,ROOT/'src/dual_arm_hardware/config/lowlevel.yaml')
        self.names=self.backend.names;self.ids=[m.id for m in self.backend.arm.motors]
        self.factory_called=False
        def factory():self.factory_called=True;return self.backend
        self.bridge=TwinBridge(self.names,self.ids,factory,'mock')
        self.addCleanup(self.bridge.close)

    def until(self,phase,timeout=2):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            self.bridge.pulse();state=self.bridge.snapshot()
            if state['phase']==phase:return state
            if state['phase']=='FAULT':self.fail(state['error'])
            time.sleep(.005)
        self.fail('Timed out waiting for '+phase)

    def enable(self):
        self.bridge.request('connect');self.until('TORQUE_OFF')
        self.bridge.request('enable');state=self.until('AWAITING_SEED')
        self.bridge.acknowledge_seed(state['seed_token']);return self.until('ACTIVE')

    def test_opening_window_does_not_open_port_or_enable(self):
        self.assertEqual(self.bridge.snapshot()['phase'],'DISCONNECTED')
        self.assertFalse(self.factory_called)
        self.assertFalse(self.backend.arm.enabled)

    def test_stale_feedback_does_not_look_fresh(self):
        with self.bridge._lock:
            self.bridge._feedback=dict(sample_valid=True,received_at=time.monotonic()-1,
                                       samples={'fake':1},joint_positions_rad={'fake':1})
        feedback=self.bridge.snapshot()['feedback']
        self.assertFalse(feedback['sample_valid']);self.assertEqual(feedback['samples'],{})

    def test_enable_barrier_uses_measured_pose_and_discards_old_GUI_targets(self):
        from ax12_guard import PRESENT
        wrist=self.backend.by_name['right_wrist_roll_joint']
        self.backend.bus.set_register(wrist.id,PRESENT,wrist.zero+10)
        old={n:0. for n in self.names};old[wrist.joint]=.5
        self.assertFalse(self.bridge.submit_targets(old))
        self.bridge.request('connect');self.until('TORQUE_OFF')
        self.bridge.request('enable');state=self.until('AWAITING_SEED')
        self.assertGreater(state['seed_rad'][wrist.joint],0)
        self.assertEqual(state['feedback']['samples'][wrist.joint]['position_rad'],state['seed_rad'][wrist.joint])
        self.assertFalse(self.bridge.submit_targets(old))
        self.assertFalse(any(e[0]=='sync_write_positions' for e in self.backend.bus.events))
        with self.assertRaises(RuntimeError):self.bridge.acknowledge_seed(state['seed_token']-1)
        self.bridge.acknowledge_seed(state['seed_token']);self.until('ACTIVE')
        self.assertFalse(any(e[0]=='sync_write_positions' for e in self.backend.bus.events))

    def test_active_shared_goal_reaches_mock_and_feedback_is_measured(self):
        self.enable();target={n:0. for n in self.names};target['right_wrist_roll_joint']=.2
        self.bridge.submit_targets(target)
        deadline=time.monotonic()+1
        while time.monotonic()<deadline:
            self.bridge.pulse();state=self.bridge.snapshot()
            q=state['feedback'].get('samples',{}).get('right_wrist_roll_joint',{}).get('position_rad',0)
            if abs(q-.2)<.006:break
            time.sleep(.005)
        self.assertAlmostEqual(q,.2,delta=.006)
        self.assertTrue(state['feedback']['sample_valid'])

    def test_UI_freeze_stops_despite_background_bridge_thread(self):
        self.enable();self.bridge._ui_timeout=.06
        self.bridge._thread.join(timeout=.5)
        state=self.bridge.snapshot()
        self.assertEqual(state['phase'],'FAULT');self.assertIn('heartbeat',state['error'])
        self.assertFalse(self.backend.arm.enabled);self.assertEqual(state['torque_off_unconfirmed'],[])

    def test_pause_hold_reseeds_and_disables_forwarding_until_ack(self):
        self.enable();self.bridge.request('hold')
        self.assertFalse(self.bridge.submit_targets({n:0. for n in self.names}))
        state=self.until('AWAITING_SEED')
        self.assertTrue(self.backend.arm.enabled)
        self.bridge.acknowledge_seed(state['seed_token']);self.until('ACTIVE')
        self.bridge.request('disable');self.until('TORQUE_OFF')
        self.assertFalse(self.backend.arm.enabled)

    def test_emergency_stop_is_latched(self):
        self.enable();self.bridge.request('estop')
        self.bridge._thread.join(timeout=1)
        self.assertEqual(self.bridge.snapshot()['phase'],'FAULT')
        with self.assertRaises(RuntimeError):self.bridge.request('enable')

    def test_mapping_mismatch_fails_without_torque_writes(self):
        self.bridge.close()
        self.bridge=TwinBridge(self.names,list(reversed(self.ids)),lambda:self.backend,'mock')
        self.bridge.request('connect');self.bridge._thread.join(timeout=1)
        self.assertEqual(self.bridge.snapshot()['phase'],'FAULT')
        self.assertFalse(any(e[0]=='write' for e in self.backend.bus.events))


if __name__=='__main__':unittest.main()
