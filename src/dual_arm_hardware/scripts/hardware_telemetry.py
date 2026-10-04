"""Read-only AX-12A telemetry and ROS-independent diagnostic records.

Units: https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/#present-speed-38
Present Load is an inferred output ratio, never torque in N.m or current in A.
"""
import math

from ax12_guard import (CW, CCW, TORQUE_ENABLE, GOAL, TORQUE_LIMIT, PRESENT,
                        REGISTERED, ProtectionError)

SPEED, LOAD, VOLTAGE, TEMPERATURE, MOVING = 38, 40, 42, 43, 46
START, LENGTH = TORQUE_ENABLE, MOVING - TORQUE_ENABLE + 1


def signed_magnitude(raw):
    if not 0 <= raw <= 2047:
        raise ProtectionError('Invalid AX-12A speed/load register')
    return -(raw & 1023) if raw & 1024 else raw


def read_telemetry(arm):
    """Return a complete fresh sample or raise; no partial/cached success."""
    samples = {}
    for motor in arm.motors:
        data = arm.bus.read_block(motor.id, START, LENGTH)
        if len(data) != LENGTH or any(type(b) is not int or not 0 <= b <= 255 for b in data):
            raise ProtectionError(f'ID {motor.id}: malformed telemetry packet')

        def byte(address):
            return data[address - START]

        def word(address):
            return byte(address) | byte(address + 1) << 8

        position, goal, cap = word(PRESENT), word(GOAL), word(TORQUE_LIMIT)
        bounds = arm.expected[motor.id]
        if not bounds[CW] <= position <= bounds[CCW]:
            raise ProtectionError(f'ID {motor.id}: measured position outside hardware bounds')
        if byte(TORQUE_ENABLE) != int(arm.enabled) or cap != bounds[TORQUE_LIMIT]:
            raise ProtectionError(f'ID {motor.id}: torque state/limit changed during telemetry read')
        if byte(REGISTERED) != 0 or byte(MOVING) not in (0, 1) or goal > 1023:
            raise ProtectionError(f'ID {motor.id}: invalid telemetry state')
        speed, load = word(SPEED), word(LOAD)
        q = motor.to_radians(position)
        goal_q = motor.to_radians(goal)
        # Position direction calibration also determines joint velocity sign.
        velocity = motor.direction * signed_magnitude(speed) * .111 * 2 * math.pi / 60
        samples[motor.joint] = {
            'motor_id': motor.id, 'position_raw': position,
            'position_rad': q, 'position_deg': math.degrees(q),
            'motor_position_deg': position * 300 / 1023,
            'goal_raw': goal, 'goal_deg': math.degrees(goal_q),
            'error_deg': math.degrees(goal_q - q),
            'speed_raw': speed, 'velocity_rad_s': velocity,
            'velocity_deg_s': math.degrees(velocity),
            'load_raw': load, 'load_percent': signed_magnitude(load) * 100 / 1023,
            'voltage_v': byte(VOLTAGE) / 10, 'temperature_c': byte(TEMPERATURE),
            'torque_enabled': byte(TORQUE_ENABLE), 'torque_limit_raw': cap,
            'torque_limit_percent': cap * 100 / 1023, 'moving': byte(MOVING),
        }
    return samples


def diagnostic_records(arm, samples=None, error='', last_rejection=''):
    """Stable key/value contract used by /dual_arm/diagnostics (DiagnosticArray)."""
    fault = arm.fault or bool(error)
    available = samples is not None and not fault
    state = 'FAULT' if fault else ('RUNNING' if arm.enabled else 'TORQUE_OFF')
    level = 2 if fault else (0 if available else 3)
    records = [{'name': 'dual_arm/system', 'hardware_id': 'AX-12A bus', 'level': level,
                'message': error or state, 'values': {
                    'state': state, 'sample_valid': str(available).lower(),
                    'motor_count': str(len(arm.motors)),
                    'torque_off_unconfirmed': '; '.join(arm.stop_errors),
                    'last_command_rejection': last_rejection}}]
    for motor in arm.motors:
        values = {'motor_id': str(motor.id), 'sample_valid': str(available).lower()}
        if available:
            values.update({key: str(value) for key, value in samples[motor.joint].items()})
        records.append({'name': 'dual_arm/' + motor.joint, 'hardware_id': str(motor.id),
                        'level': level, 'message': error or state, 'values': values})
    return records
