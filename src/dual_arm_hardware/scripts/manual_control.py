"""Absolute motor-angle control, independent of URDF neutral/direction calibration.

The position controller's internal zero is the raw position on connection.
Only this session uses those coordinates; no model calibration or device
configuration is modified. GUI targets are always absolute motor degrees.
"""
from dataclasses import replace
import math
from pathlib import Path

import yaml
from ax12_guard import (load_configuration, SDKBus, CW, CCW, PRESENT,
                        TICKS_PER_DEGREE, TICKS_PER_RADIAN, ProtectionError)
from lowlevel_control import DualArmLowLevel, loop_settings


class ManualMotorController(DualArmLowLevel):
    def snapshot(self):
        state = super().snapshot()
        state['control_coordinates'] = 'motor_offset_from_connection_radians'
        state['motor_reference_raw'] = {m.joint: m.zero for m in self.arm.motors}
        state['motor_bounds_raw'] = {m.joint: [m.lower, m.upper] for m in self.arm.motors}
        return state


def create_controller(hardware_config, description_config, loop_config, bus_factory=None):
    config, configured = load_configuration(hardware_config, description_config)
    if config['configure_protection']:
        raise ProtectionError('Manual control requires configure_protection: false')
    settings = loop_settings(yaml.safe_load(Path(loop_config).read_text()))
    bus = bus_factory(configured) if bus_factory else SDKBus(config['port'], config['baudrate'])
    try:
        motors = []
        for original in configured:
            registers = bus.read_registers(original.id, (CW, CCW, PRESENT))
            lo, hi, reference = (registers[a] for a in (CW, CCW, PRESENT))
            if not 0 <= lo <= reference <= hi <= 1023 or lo >= hi:
                raise ProtectionError(f'ID {original.id}: invalid existing motor range/position')
            motors.append(replace(original, direction=1, zero=reference, lower=lo, upper=hi, cap=None,
                                  q_lower=(lo-reference)/TICKS_PER_RADIAN,
                                  q_upper=(hi-reference)/TICKS_PER_RADIAN))
        return ManualMotorController(bus, motors, description_config, settings,
                                     configure_protection=False)
    except Exception:
        bus.close()
        raise


def absolute_targets(target_degrees, feedback):
    """Convert all GUI targets using this connection's explicit motor references."""
    references = feedback['motor_reference_raw']
    bounds = feedback['motor_bounds_raw']
    if set(target_degrees) != set(references):
        raise ValueError('Supply every motor target')
    result = {}
    for name, degrees in target_degrees.items():
        if type(degrees) not in (float, int) or not math.isfinite(degrees):
            raise ValueError('Targets must be finite motor angles')
        raw = round(degrees*TICKS_PER_DEGREE)
        lo, hi = bounds[name]
        if not lo <= raw <= hi or not 0 <= degrees <= 300:
            raise ValueError(f'{name}: target outside the motor range')
        result[name] = (raw-references[name])/TICKS_PER_RADIAN
    return result
