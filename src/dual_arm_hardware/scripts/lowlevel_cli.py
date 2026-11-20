#!/usr/bin/env python3
"""Standalone control shell. 'demo' uses only an in-memory register emulator."""
import argparse
from datetime import datetime,timezone
import json
import math
from pathlib import Path
import select
import shlex
import signal
import sys
import tempfile
import time

import yaml
from ax12_guard import (load_configuration,ProtectionError,SDKBus,MODEL,CW,CCW,
                        MAX_TORQUE,TORQUE_ENABLE,TORQUE_LIMIT,GOAL,PRESENT,RETURN_LEVEL)
from lowlevel_control import DualArmLowLevel,loop_settings
from mock_bus import MockBus

ROOT=Path(__file__).resolve().parents[3]
HELP='enable | hold | disable | status | set JOINT RAD [JOINT RAD ...] | grip left/right MM | estop | quit'


def mock_controller(description,settings):
    config=yaml.safe_load((ROOT/'src/dual_arm_hardware/config/hardware.yaml').read_text())
    config.update(port='/dev/MOCK_NO_DEVICE',baudrate=1000000,
                  configure_protection=True,torque_limit_percent=25.)
    for motor in config['motors'].values():motor['direction']=1
    with tempfile.TemporaryDirectory() as directory:
        path=Path(directory)/'mock.yaml';path.write_text(yaml.safe_dump(config))
        _,motors=load_configuration(path,description)
    return DualArmLowLevel(MockBus(motors),motors,description,settings)


def execute(controller,line):
    args=shlex.split(line)
    if not args:return None
    cmd,*values=args
    if cmd=='enable' and not values:return controller.enable()
    if cmd=='disable' and not values:return {'torque_off_unconfirmed':controller.disable()}
    if cmd=='status' and not values:return controller.snapshot()
    if cmd=='hold' and not values:return controller.hold()
    if cmd=='estop' and not values:return {'torque_off_unconfirmed':controller.emergency_stop()}
    if cmd=='grip' and len(values)==2:return controller.set_gripper(values[0],float(values[1]))
    if cmd=='set' and len(values)>=2 and len(values)%2==0:
        names=values[::2]
        if len(set(names))!=len(names):raise ProtectionError('Duplicate target joints')
        return controller.set_positions(dict(zip(names,map(float,values[1::2]))))
    if cmd=='quit' and not values:return 'quit'
    raise ProtectionError(HELP)


def probe(port,baudrate,ids):
    """Read only; direction and torque-cap calibration are not assumed."""
    if not port or not port.startswith('/dev/') or type(baudrate) is not int or baudrate<=0:
        raise ProtectionError('Probe requires --port /dev/... and a positive --baudrate')
    if not ids or len(set(ids))!=len(ids) or any(type(i) is not int or not 0<=i<=253 for i in ids):
        raise ProtectionError('Probe IDs must be unique in 0..253')
    bus=SDKBus(port,baudrate);results=[]
    try:
        for motor_id in ids:
            try:
                values=bus.read_registers(motor_id,(MODEL,CW,CCW,MAX_TORQUE,TORQUE_ENABLE,
                                                    TORQUE_LIMIT,GOAL,PRESENT,RETURN_LEVEL))
                results.append(dict(id=motor_id,model=values[MODEL],position_raw=values[PRESENT],
                    motor_position_deg=values[PRESENT]*300/1023,goal_raw=values[GOAL],
                    torque_enabled=values[TORQUE_ENABLE],angle_limits_raw=[values[CW],values[CCW]],
                    max_torque_raw=values[MAX_TORQUE],torque_limit_raw=values[TORQUE_LIMIT],
                    status_return_level=values[RETURN_LEVEL],error=''))
            except Exception as exc:results.append(dict(id=motor_id,error=str(exc)))
    finally:bus.close()
    return results


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('shell','demo','probe'))
    parser.add_argument('--mock',action='store_true',help='Use emulator instead of serial, including shell')
    parser.add_argument('--config',type=Path,default=ROOT/'src/dual_arm_hardware/config/hardware.yaml')
    parser.add_argument('--description-config',type=Path,default=ROOT/'src/dual_arm_description/config')
    parser.add_argument('--loop-config',type=Path,default=ROOT/'src/dual_arm_hardware/config/lowlevel.yaml')
    parser.add_argument('--log-dir',type=Path,default=ROOT/'log/lowlevel')
    parser.add_argument('--port',help='Probe only: actual /dev/... adapter')
    parser.add_argument('--baudrate',type=int,help='Probe only: explicit bus baudrate')
    parser.add_argument('--ids',type=int,nargs='+',default=list(range(1,11)),help='Probe only: IDs to read, default 1..10')
    args=parser.parse_args()
    settings=loop_settings(yaml.safe_load(args.loop_config.read_text()))
    mock=args.mock or args.action=='demo'
    if args.action=='probe' and args.mock:parser.error('probe reads a real port; use demo for offline testing')
    args.log_dir.mkdir(parents=True,exist_ok=True)
    path=args.log_dir/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')+'.jsonl')
    controller=None;code=0
    def interrupted(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupted)
    with path.open('x',encoding='utf-8') as log:
        def emit(event,data):
            record={'time_utc':datetime.now(timezone.utc).isoformat(),'mock':mock,'event':event,'data':data}
            line=json.dumps(record,ensure_ascii=False,allow_nan=False)
            log.write(line+'\n');log.flush();print(line,flush=True)
        try:
            if args.action=='probe':
                results=probe(args.port,args.baudrate,args.ids)
                emit('read_only_probe',results)
                return 2 if any(r['error'] for r in results) else 0
            controller=(mock_controller(args.description_config,settings) if mock else
                        DualArmLowLevel.from_config(args.config,args.description_config,args.loop_config))
            controller.start()
            emit('started',{'mode':'MOCK REGISTER EMULATOR' if mock else 'REAL AX-12A',
                            'commands':HELP,'motor_ids':{m.joint:m.id for m in controller.arm.motors}})
            if args.action=='demo':
                for command in ('enable','set right_wrist_roll_joint 0.1','grip right 20','grip left 20','status','disable'):
                    emit('command',{'text':command,'result':execute(controller,command)})
                emit('watchdog_demo',{'note':'Enable, then intentionally stop refreshing the command lease'})
                controller.enable()
                deadline=time.monotonic()+settings['command_timeout_s']+settings['feedback_timeout_s']+1
                while not controller.arm.fault and time.monotonic()<deadline:time.sleep(.02)
                snapshot=controller.snapshot();emit('watchdog_result',snapshot)
                if not snapshot['fault'] or snapshot['torque_off_unconfirmed']:
                    raise ProtectionError('Mock watchdog test did not stop cleanly')
            else:
                print(HELP,flush=True)
                while True:
                    if controller.arm.enabled:controller.keepalive()
                    ready,_,_=select.select([sys.stdin],[],[],min(.1,settings['command_timeout_s']/4))
                    if not ready:continue
                    line=sys.stdin.readline()
                    if not line:break
                    try:
                        result=execute(controller,line)
                        emit('command',{'text':line.strip(),'result':result})
                        if result=='quit':break
                    except (ValueError,ProtectionError) as exc:emit('rejected',str(exc))
        except KeyboardInterrupt:
            emit('interrupted','Stopping controller')
        except Exception as exc:
            code=2;emit('error',str(exc))
        finally:
            if controller is not None:
                errors=controller.close()
                if errors:code=2
                emit('closed',{'torque_off_unconfirmed':errors})
    print('Session log:',path)
    return code


if __name__=='__main__':raise SystemExit(main())
