"""Asynchronous simulation target / hardware feedback link, no Qt or ROS required.

Serial I/O runs outside the UI. A fresh UI pulse is required to renew hardware
liveness. Enabling uses a seed/ack barrier so pre-existing GUI goals cannot move
the robot on enable. The backend retains authority over limits and fault stops.
"""
from copy import deepcopy
from datetime import datetime,timezone
import json
import math
from pathlib import Path
import sys
import threading
import time


def controller_factory(mode,description,hardware_config,loop_config):
    root=Path(description).resolve().parents[1]
    sys.path.insert(0,str(root/'src/dual_arm_hardware/scripts'))
    if mode=='mock':
        from lowlevel_cli import mock_controller
        import yaml
        return mock_controller(Path(description)/'config',yaml.safe_load(Path(loop_config).read_text()))
    if mode=='real':
        if not sys.platform.startswith('linux'):
            raise RuntimeError('Direct U2D2 control currently requires Linux')
        from lowlevel_control import DualArmLowLevel
        return DualArmLowLevel.from_config(hardware_config,Path(description)/'config',loop_config)
    raise ValueError('Twin mode must be mock or real')


class TwinBridge:
    def __init__(self,names,motor_ids,factory,mode,log_dir=None,clock=time.monotonic):
        self.names=tuple(names);self.motor_ids=tuple(motor_ids)
        if mode not in ('mock','real') or len(self.names)!=len(set(self.names)) or len(self.names)!=len(self.motor_ids):
            raise ValueError('Invalid twin mapping/mode')
        self.factory,self.mode,self.clock=factory,mode,clock
        self._lock=threading.Lock();self._wake=threading.Event();self._quit=threading.Event()
        self._phase='DISCONNECTED';self._error='';self._backend=None
        self._request=None;self._desired=None;self._sent=None;self._feedback={}
        self._ui_at=self.clock();self._token=0;self._ack=None;self._seed=None
        self._ui_timeout=.5;self._off_errors=[]
        self._log=None
        if log_dir is not None:
            path=Path(log_dir);path.mkdir(parents=True,exist_ok=True)
            self._log=(path/('twin_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')+'.jsonl')).open('x',encoding='utf-8')
        self._thread=threading.Thread(target=self._run,name='dual-arm-twin',daemon=True)
        self._thread.start()

    def pulse(self):
        with self._lock:self._ui_at=self.clock()

    def request(self,operation):
        with self._lock:
            allowed={'connect':('DISCONNECTED',),'enable':('TORQUE_OFF',),
                     'hold':('ACTIVE',),'disable':('ACTIVE','AWAITING_SEED','HOLDING'),
                     'estop':('TORQUE_OFF','ACTIVE','AWAITING_SEED','HOLDING','ENABLING')}
            if operation not in allowed or self._phase not in allowed[operation]:
                raise RuntimeError(f'Cannot {operation} while {self._phase}')
            self._phase={'connect':'CONNECTING','enable':'ENABLING','hold':'HOLDING',
                         'disable':'DISABLING','estop':'STOPPING'}[operation]
            self._request=operation;self._desired=None;self._sent=None;self._ui_at=self.clock()
        self._wake.set()

    def acknowledge_seed(self,token):
        with self._lock:
            if self._phase!='AWAITING_SEED' or token!=self._token:
                raise RuntimeError('Stale hardware pose acknowledgement')
            self._ack=token
        self._wake.set()

    def submit_targets(self,targets):
        if not isinstance(targets,dict) or set(targets)!=set(self.names) or any(
                type(v) not in (float,int) or not math.isfinite(v) for v in targets.values()):
            raise ValueError('Supply one finite radian target for every motor')
        with self._lock:
            if self._phase!='ACTIVE':return False
            self._desired=dict(targets)
        self._wake.set();return True

    def snapshot(self):
        with self._lock:
            result=dict(mode=self.mode,phase=self._phase,error=self._error,seed_token=self._token,
                        seed_rad=deepcopy(self._seed),feedback=deepcopy(self._feedback),
                        torque_off_unconfirmed=list(self._off_errors))
        # The cache belongs to this bridge, not a fresh bus scan at snapshot time.
        feedback=result['feedback']
        if feedback and self.clock()-feedback.get('received_at',0)>.5:
            feedback.update(sample_valid=False,samples={},joint_positions_rad={})
        return result

    def _event(self,event,data):
        if self._log:
            self._log.write(json.dumps(dict(time_utc=datetime.now(timezone.utc).isoformat(),mode=self.mode,
                                           event=event,data=data),allow_nan=False)+'\n');self._log.flush()

    def _seed_pose(self):
        sample=self._backend.snapshot()
        if not sample['sample_valid']:raise RuntimeError('No fresh pose for hardware synchronization')
        seed={n:sample['samples'][n]['position_rad'] for n in self.names}
        with self._lock:
            self._seed=seed;self._token+=1;self._ack=None;self._desired=None
            self._feedback={**sample,'received_at':self.clock()}
            self._phase='AWAITING_SEED'
        self._event('seed',seed)

    def _execute(self,operation):
        self._event('request',operation)
        if operation=='connect':
            backend=self.factory()
            mapping={m.joint:m.id for m in backend.arm.motors}
            if mapping!=dict(zip(self.names,self.motor_ids)):
                backend.close() # Not owned/prepared: close port without torque writes.
                raise RuntimeError('Simulation and hardware motor mappings differ')
            self._backend=backend
            backend.start()
            # A stopped UI must not finish connecting and later accept old input.
            if self.clock()-self._ui_at>self._ui_timeout:raise RuntimeError('UI stopped during connection')
            with self._lock:self._phase='TORQUE_OFF'
        elif operation=='enable':
            self._backend.enable();self._seed_pose()
        elif operation=='hold':
            self._backend.hold();self._seed_pose()
        elif operation=='disable':
            errors=self._backend.disable()
            if errors:raise RuntimeError('Torque OFF unconfirmed: '+'; '.join(errors))
            with self._lock:self._phase='TORQUE_OFF'
        elif operation=='estop':
            self._backend.emergency_stop('Twin emergency stop requested')
            raise RuntimeError('Twin emergency stop requested')

    def _run(self):
        try:
            while not self._quit.is_set():
                with self._lock:
                    request=self._request;self._request=None
                if request:self._execute(request)
                backend=self._backend
                if backend is not None:
                    state=backend.snapshot()
                    if state['fault']:raise RuntimeError(state['fault'])
                    if backend.arm.enabled:
                        with self._lock:
                            fresh=self.clock()-self._ui_at<=self._ui_timeout
                            if self._phase=='AWAITING_SEED' and self._ack==self._token:
                                self._phase='ACTIVE';self._sent=dict(self._seed)
                            active=self._phase=='ACTIVE';desired=deepcopy(self._desired)
                        if not fresh:raise RuntimeError('Simulation UI heartbeat lost')
                        backend.keepalive()
                        if active and desired is not None and desired!=self._sent:
                            backend.set_positions(desired,require_all=True)
                            self._sent=desired
                            self._event('target_rad',desired)
                    state=backend.snapshot();state['received_at']=self.clock()
                    with self._lock:self._feedback=state
                self._wake.wait(.05);self._wake.clear()
        except Exception as exc:
            reason=str(exc);errors=[]
            if self._backend is not None:
                errors=self._backend.emergency_stop(reason)
            with self._lock:
                self._phase='FAULT';self._error=reason;self._off_errors=list(errors)
                self._feedback={};self._desired=None
            self._event('fault',dict(error=reason,torque_off_unconfirmed=errors))
        finally:
            if self._backend is not None:
                errors=self._backend.close()
                with self._lock:self._off_errors=list(errors)
                self._event('closed',dict(torque_off_unconfirmed=errors))

    def close(self):
        self._quit.set();self._wake.set();self._thread.join()
        with self._lock:
            if self._phase!='FAULT':self._phase='CLOSED'
            errors=list(self._off_errors)
        if self._log:self._log.close();self._log=None
        return errors
