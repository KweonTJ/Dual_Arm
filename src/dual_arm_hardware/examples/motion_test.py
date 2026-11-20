#!/usr/bin/env python3
"""One wrist: hold measured position, move a small relative step, return, OFF.

Uses the production low-level controller with a single motor. The other nine
motors are checked read-only and are never enabled. No protection/speed writes.
Defaults to a register emulator; --real explicitly selects the physical bus.
"""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/dual_arm_hardware/scripts'))
from ax12_guard import (load_configuration, ProtectedArm, SDKBus, ProtectionError,
                        TICKS_PER_DEGREE)
from lowlevel_control import DualArmLowLevel
from mock_bus import MockBus

WRISTS = ('right_wrist_roll_joint', 'left_wrist_roll_joint')


def motion_test(config_path, description, loop_config, output, joint=WRISTS[0],
                delta_deg=2., duration=1.5, real=False, bus_factory=None):
    import yaml
    from lowlevel_control import loop_settings
    if joint not in WRISTS or not math.isfinite(delta_deg) or not 0 < abs(delta_deg) <= 5:
        raise ValueError('Select a wrist and a nonzero relative test step within 5 degrees')
    if not math.isfinite(duration) or not .3 <= duration <= 5:
        raise ValueError('Use a measurement duration from 0.3 to 5 seconds per phase')
    config, motors = load_configuration(config_path, description)
    if config['configure_protection']:
        raise ValueError('This test requires configure_protection: false')
    settings = loop_settings(yaml.safe_load(Path(loop_config).read_text()))
    motor = next(m for m in motors if m.joint == joint)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    path = output / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ') + '_motion.json')
    report = dict(created_utc=datetime.now(timezone.utc).isoformat(), mock=not real,
                  joint=joint, motor_id=motor.id, requested_delta_deg=delta_deg,
                  configure_protection=False, passed=False, samples=[], phases=[],
                  torque_enabled_by_test=False, torque_off_confirmed=None, error='')
    # Establish the audit file before any device access.
    with path.open('x', encoding='utf-8') as stream:
        json.dump(report, stream, indent=2)
    bus = controller = None
    began = time.monotonic()
    try:
        if bus_factory:
            bus = bus_factory(motors)
        elif real:
            bus = SDKBus(config['port'], config['baudrate'], record=True)
        else:
            bus = MockBus(motors)
        # All devices must already be OFF. This preflight changes no settings.
        report['existing_registers'] = ProtectedArm(bus, motors, False).prepare(apply=False)
        controller = DualArmLowLevel(bus, [motor], description, settings,
                                     configure_protection=False)
        controller.start()
        sample = controller.snapshot()['samples'][joint]
        initial = sample['position_raw']
        baseline = motor.to_radians(initial)
        target = baseline + math.radians(delta_deg)
        motor.to_position(baseline)
        target_raw = motor.to_position(target)  # Validate before enabling torque.
        report.update(initial_raw=initial, target_raw=target_raw,
                      commanded_delta_deg=motor.direction*(target_raw-initial)/TICKS_PER_DEGREE)
        print(f'{joint} / ID {motor.id}: raw {initial} -> {target_raw} -> {initial}', flush=True)
        controller.enable()  # Only this wrist, goal seeded from measured position.
        report['torque_enabled_by_test'] = True
        for phase, goal, raw in (('offset', target, target_raw), ('return', baseline, initial)):
            controller.set_positions({joint: goal})
            deadline = time.monotonic() + duration
            phase_samples = []
            while time.monotonic() < deadline:
                controller.keepalive()
                snapshot = controller.snapshot()
                if not snapshot['sample_valid']:
                    raise ProtectionError(snapshot['fault'] or 'Feedback is not fresh')
                sample = snapshot['samples'][joint]
                item = dict(elapsed_s=time.monotonic()-began, phase=phase, **sample)
                report['samples'].append(item)
                phase_samples.append(item)
                time.sleep(.05)
            sample = phase_samples[-1]
            error_ticks = abs(sample['position_raw'] - raw)
            result = dict(phase=phase, goal_raw=raw, position_raw=sample['position_raw'],
                          error_deg=error_ticks/TICKS_PER_DEGREE,
                          measured_delta_deg=motor.direction*(sample['position_raw']-initial)/TICKS_PER_DEGREE,
                          voltage_v=sample['voltage_v'], temperature_c=sample['temperature_c'])
            report['phases'].append(result)
            print(json.dumps(result), flush=True)
            if error_ticks > 2:
                raise ProtectionError(f'{phase}: measured position differs by {error_ticks} ticks')
            if phase == 'offset' and (sample['position_raw']-initial)*(target_raw-initial) <= 0:
                raise ProtectionError('Encoder feedback did not move in the commanded direction')
        report['passed'] = True
    except (Exception, KeyboardInterrupt) as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
    finally:
        try:
            if controller is not None:
                errors = controller.close()  # OFF plus readback, then close the port.
                if report['torque_enabled_by_test']:
                    report['torque_off_confirmed'] = not errors
                report['torque_off_unconfirmed'] = errors
                if errors:
                    report['passed'] = False
            elif bus is not None:
                bus.close()
        except Exception as exc:
            report['passed'] = False
            report['torque_off_confirmed'] = False
            report['error'] += f' Cleanup: {type(exc).__name__}: {exc}'
        report['transactions'] = getattr(bus, 'events', [])
        report['elapsed_s'] = time.monotonic()-began
        path.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps({key: report[key] for key in ('mock','passed','torque_off_confirmed','error')}), flush=True)
    print(f'Report: {path}', flush=True)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--real', action='store_true')
    parser.add_argument('--joint', choices=WRISTS, default=WRISTS[0])
    parser.add_argument('--delta-deg', type=float, default=2.)
    parser.add_argument('--seconds-per-phase', type=float, default=1.5)
    parser.add_argument('--config', type=Path, default=ROOT/'src/dual_arm_hardware/config/hardware.yaml')
    parser.add_argument('--output', type=Path, default=ROOT/'log/motion_test')
    args = parser.parse_args()
    def interrupted(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    report = motion_test(args.config, ROOT/'src/dual_arm_description/config',
                         ROOT/'src/dual_arm_hardware/config/lowlevel.yaml', args.output,
                         args.joint, args.delta_deg, args.seconds_per_phase, args.real)
    return 0 if report['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
