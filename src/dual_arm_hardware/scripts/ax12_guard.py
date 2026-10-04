"""AX-12A protection and verified startup, independent of ROS for fault tests.

Addresses/units: https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/
Only a direct Protocol 1.0 bus is supported. No automatic recovery or motion.
"""
from dataclasses import dataclass
import math
from pathlib import Path
import time

import yaml

MODEL, CW, CCW, MAX_TORQUE, RETURN_LEVEL, SHUTDOWN = 0, 6, 8, 14, 16, 18
TORQUE_ENABLE, GOAL, TORQUE_LIMIT, PRESENT, REGISTERED, LOCK = 24, 30, 34, 36, 44, 47
SIZES = {MODEL: 2, CW: 2, CCW: 2, MAX_TORQUE: 2, RETURN_LEVEL: 1,
         SHUTDOWN: 1, TORQUE_ENABLE: 1, GOAL: 2, TORQUE_LIMIT: 2,
         PRESENT: 2, REGISTERED: 1, LOCK: 1}
TICKS_PER_DEGREE = 1023/300
TICKS_PER_RADIAN = TICKS_PER_DEGREE*180/math.pi
REQUIRED_SHUTDOWN = 0x24  # Overheat + overload; retain any other enabled bits.


class ProtectionError(RuntimeError):
    pass


@dataclass(frozen=True)
class Motor:
    joint: str
    id: int
    direction: int
    zero: int
    lower: int
    upper: int
    cap: int
    q_lower: float
    q_upper: float

    def to_position(self, q):
        if not math.isfinite(q) or not self.q_lower <= q <= self.q_upper:
            raise ProtectionError(f'{self.joint}: command outside configured angle limits')
        raw = round(self.zero+self.direction*q*TICKS_PER_RADIAN)
        # Only absorb the sub-tick rounding at an inward-rounded endpoint.
        return min(self.upper, max(self.lower, raw))

    def to_radians(self, raw):
        return self.direction*(raw-self.zero)/TICKS_PER_RADIAN


def load_configuration(path, description_config):
    config = yaml.safe_load(Path(path).read_text())
    description_config = Path(description_config)
    limits = yaml.safe_load((description_config/'joint_limits.yaml').read_text())['joint_limits']
    calibration = yaml.safe_load((description_config/'motor_calibration.yaml').read_text())
    if not isinstance(config, dict):
        raise ProtectionError('Hardware configuration must be a mapping')
    missing = [key for key in ('port', 'baudrate', 'torque_limit_percent') if config.get(key) is None]
    entries = config.get('motors', {})
    if not isinstance(entries, dict) or set(entries) != set(limits) or set(entries) != set(calibration['joints']):
        raise ProtectionError('Motor mapping must name exactly the 12 configured joints')
    for name, entry in entries.items():
        if not isinstance(entry, dict) or entry.get('id') is None or entry.get('direction') is None:
            missing.append(name+' (id/direction)')
    if missing:
        raise ProtectionError('Missing hardware settings; no device opened: '+', '.join(missing))
    if not isinstance(config['port'], str) or not config['port'].startswith('/dev/'):
        raise ProtectionError('Set the actual /dev/... adapter port')
    if type(config['baudrate']) is not int or config['baudrate'] <= 0:
        raise ProtectionError('Baudrate must be a positive integer')
    percent = config['torque_limit_percent']
    if type(percent) not in (int, float) or not math.isfinite(percent) or not 0 < percent <= 100:
        raise ProtectionError('Torque output percentage must be finite and in (0, 100]')
    cap = math.floor(percent*1023/100)
    if cap < 1:
        raise ProtectionError('Torque output cap is below one register unit')
    zero_degrees = float(calibration['motor_zero_degrees'])
    if not math.isfinite(zero_degrees) or not 0 <= zero_degrees <= 300:
        raise ProtectionError('Motor neutral angle is outside AX-12A travel')
    zero = round(zero_degrees*TICKS_PER_DEGREE)
    motors, ids = [], set()
    for name, entry in entries.items():
        motor_id, sign = entry['id'], entry['direction']
        if type(motor_id) is not int or not 0 <= motor_id <= 253 or motor_id in ids:
            raise ProtectionError(f'{name}: ID must be unique and in 0..253 (no broadcast)')
        if type(sign) is not int or sign not in (-1, 1):
            raise ProtectionError(f'{name}: direction must be +1 or -1')
        lo, hi = (float(limits[name][key]) for key in ('min_position_deg', 'max_position_deg'))
        if not all(math.isfinite(v) for v in (lo, hi)) or not lo <= 0 <= hi or lo >= hi:
            raise ProtectionError(f'{name}: invalid joint angle limits')
        ends = sorted([zero+sign*lo*TICKS_PER_DEGREE, zero+sign*hi*TICKS_PER_DEGREE])
        lower, upper = math.ceil(ends[0]), math.floor(ends[1])
        if not 0 <= lower <= zero <= upper <= 1023 or lower >= upper:
            raise ProtectionError(f'{name}: calibrated range exceeds AX-12A travel')
        motors.append(Motor(name, motor_id, sign, zero, lower, upper, cap,
                            math.radians(lo), math.radians(hi)))
        ids.add(motor_id)
    return config, motors


class SDKBus:
    def __init__(self, port, baudrate, record=False):
        self.events = [] if record else None
        try:
            from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
        except ImportError as exc:
            raise ProtectionError('DYNAMIXEL SDK / pyserial required (ros-humble-dynamixel-sdk, python3-serial)') from exc
        self.port = PortHandler(port)
        self.packet = PacketHandler(1.0)
        self.success = COMM_SUCCESS
        if not self.port.setBaudRate(baudrate):
            raise ProtectionError(f'Cannot open {port} at {baudrate}')
        # Prevent another newly opened serial connection from sharing this bus.
        try:
            import fcntl
            import termios
            fcntl.ioctl(self.port.ser.fileno(), termios.TIOCEXCL)
        except Exception:
            self.close()
            raise

    def _check(self, result, error, motor_id):
        if result != self.success:
            raise ProtectionError(f'ID {motor_id}: {self.packet.getTxRxResult(result)}')
        if error:
            raise ProtectionError(f'ID {motor_id}: status error 0x{error:02x}: {self.packet.getRxPacketError(error)}')

    def read(self, motor_id, address):
        fn = getattr(self.packet, f'read{SIZES[address]}ByteTxRx')
        value, result, error = fn(self.port, motor_id, address)
        if self.events is not None:
            self.events.append({'operation': 'read', 'id': motor_id, 'address': address,
                                'value': value, 'comm_result': result, 'status_error': error})
        self._check(result, error, motor_id)
        return value

    def write(self, motor_id, address, value):
        fn = getattr(self.packet, f'write{SIZES[address]}ByteTxRx')
        result, error = fn(self.port, motor_id, address, value)
        if self.events is not None:
            self.events.append({'operation': 'write', 'id': motor_id, 'address': address,
                                'value': value, 'comm_result': result, 'status_error': error})
        self._check(result, error, motor_id)
        if address < TORQUE_ENABLE:
            time.sleep(.02)  # EEPROM settling; only changed values are written.

    def read_block(self, motor_id, address, length):
        """One contiguous read for telemetry; retain strict status-error handling."""
        data, result, error = self.packet.readTxRx(self.port, motor_id, address, length)
        if self.events is not None:
            self.events.append({'operation': 'read_block', 'id': motor_id,
                                'address': address, 'data': list(data),
                                'comm_result': result, 'status_error': error})
        self._check(result, error, motor_id)
        if len(data) != length:
            raise ProtectionError(f'ID {motor_id}: incomplete telemetry packet')
        return data

    def close(self):
        if self.port.is_open:
            self.port.closePort()


class ProtectedArm:
    def __init__(self, bus, motors):
        self.bus, self.motors = bus, motors
        self.expected = {}
        self.ready = False
        self.enabled = False
        self.fault = False
        self.stop_errors = []

    def _set(self, motor_id, address, value):
        if self.bus.read(motor_id, address) != value:
            self.bus.write(motor_id, address, value)
        actual = self.bus.read(motor_id, address)
        if actual != value:
            raise ProtectionError(f'ID {motor_id} register {address}: wrote {value}, read {actual}')

    def prepare(self, apply=False):
        """Read all motors first, then optionally apply; never enable or move them."""
        self.ready = False
        if self.fault or self.enabled:
            raise ProtectionError('Cannot configure an enabled or faulted session')
        plan = {}
        for m in self.motors:
            values = {a: self.bus.read(m.id, a) for a in
                      (MODEL, TORQUE_ENABLE, CW, CCW, MAX_TORQUE, TORQUE_LIMIT,
                       RETURN_LEVEL, SHUTDOWN, REGISTERED, LOCK, PRESENT)}
            if values[MODEL] != 12:
                raise ProtectionError(f'ID {m.id}: expected AX-12A model 12')
            if values[TORQUE_ENABLE] != 0:
                raise ProtectionError(f'ID {m.id}: torque is already ON; stop/support the robot first')
            if values[RETURN_LEVEL] != 2 or values[REGISTERED] != 0:
                raise ProtectionError(f'ID {m.id}: require Status Return Level 2 and no pending REG_WRITE')
            if values[CW] == values[CCW] == 0:
                raise ProtectionError(f'ID {m.id}: wheel mode is not accepted')
            if not 0 <= values[CW] < values[CCW] <= 1023:
                raise ProtectionError(f'ID {m.id}: invalid device angle limits')
            if not 0 < values[MAX_TORQUE] <= 1023 or not 0 < values[TORQUE_LIMIT] <= 1023:
                raise ProtectionError(f'ID {m.id}: zero/invalid torque cap; no automatic alarm recovery')
            lo, hi = max(m.lower, values[CW]), min(m.upper, values[CCW])
            cap = min(m.cap, values[MAX_TORQUE], values[TORQUE_LIMIT])
            if not lo <= m.zero <= hi or not lo <= values[PRESENT] <= hi or lo >= hi:
                raise ProtectionError(f'ID {m.id}: neutral or present position outside proposed angle limits')
            desired = {CW: lo, CCW: hi, MAX_TORQUE: cap, TORQUE_LIMIT: cap,
                       SHUTDOWN: values[SHUTDOWN] | REQUIRED_SHUTDOWN}
            if values[LOCK] and any(values[a] != desired[a] for a in (CW, CCW, MAX_TORQUE, SHUTDOWN)):
                raise ProtectionError(f'ID {m.id}: EEPROM is locked and required settings differ')
            plan[m.id] = desired
        self.expected = plan
        if not apply:
            return plan
        try:
            for m in self.motors:
                if self.bus.read(m.id, TORQUE_ENABLE) != 0:
                    raise ProtectionError(f'ID {m.id}: torque changed during setup')
                # Persist the smaller cap, then apply RAM cap; no torque enable writes.
                for a in (MAX_TORQUE, TORQUE_LIMIT, SHUTDOWN, CW, CCW):
                    self._set(m.id, a, plan[m.id][a])
            self.verify(enabled=False)
            self.ready = True
        except Exception:
            self.fault = True
            raise  # Partial writes are retained; never restore a larger cap.
        return plan

    def verify(self, enabled):
        for m in self.motors:
            for a, expected in self.expected[m.id].items():
                if self.bus.read(m.id, a) != expected:
                    raise ProtectionError(f'ID {m.id}: protection register {a} changed')
            if self.bus.read(m.id, TORQUE_ENABLE) != int(enabled):
                raise ProtectionError(f'ID {m.id}: unexpected torque state')
            if self.bus.read(m.id, REGISTERED) != 0:
                raise ProtectionError(f'ID {m.id}: unexpected pending REG_WRITE')

    def positions(self):
        result = {}
        for m in self.motors:
            raw = self.bus.read(m.id, PRESENT)
            bounds = self.expected[m.id]
            if not bounds[CW] <= raw <= bounds[CCW]:
                raise ProtectionError(f'ID {m.id}: measured position outside hardware bounds')
            result[m.joint] = m.to_radians(raw)
        return result

    def enable(self):
        if not self.ready or self.fault or self.enabled:
            raise ProtectionError('Enable requires a freshly verified, torque-OFF session')
        try:
            self.verify(enabled=False)
            # Hold the measured position, never jump to the URDF neutral pose.
            self.positions()
            for m in self.motors:
                current = self.bus.read(m.id, PRESENT)
                if not self.expected[m.id][CW] <= current <= self.expected[m.id][CCW]:
                    raise ProtectionError(f'ID {m.id}: position changed before enable')
                self._set(m.id, GOAL, current)
            self.verify(enabled=False)
            for m in self.motors:
                self._set(m.id, TORQUE_ENABLE, 1)
            self.enabled = True
            self.verify(enabled=True)
        except Exception:
            self.stop(fault=True)
            raise

    def command(self, names, positions):
        if not self.enabled or self.fault:
            raise ProtectionError('Arm is not enabled')
        if (len(names) != len(self.motors) or len(positions) != len(names)
                or len(set(names)) != len(names) or set(names) != {m.joint for m in self.motors}):
            raise ProtectionError('Supply exactly one target for every joint')
        targets = dict(zip(names, positions))
        raw = {}
        for m in self.motors:
            value = m.to_position(targets[m.joint])
            if not self.expected[m.id][CW] <= value <= self.expected[m.id][CCW]:
                raise ProtectionError(f'{m.joint}: target exceeds existing tighter hardware limits')
            raw[m.id] = value
        try:
            # Do not send motion after a reset, changed cap, or shutdown.
            self.verify(enabled=True)
            for m in self.motors:
                self._set(m.id, GOAL, raw[m.id])
        except Exception:
            self.stop(fault=True)
            raise

    def stop(self, fault=False):
        self.enabled = False
        self.fault = self.fault or fault
        failed = []
        for m in self.motors:
            try:
                # Still transmit OFF when reads report an overload/status error.
                self.bus.write(m.id, TORQUE_ENABLE, 0)
                if self.bus.read(m.id, TORQUE_ENABLE) != 0:
                    raise ProtectionError('Torque OFF readback differs')
            except Exception as exc:
                failed.append(f'ID {m.id}: {exc}')
        self.stop_errors = failed
        return failed  # Caller must report any unconfirmed stop.
