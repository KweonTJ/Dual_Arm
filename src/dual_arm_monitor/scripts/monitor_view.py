"""Offline-testable presentation; no serial, command publisher or service client."""
import csv
from datetime import datetime, timezone
import math
from pathlib import Path

JOINTS = tuple(f'{side}_{part}_joint' for side in ('left', 'right') for part in
               ('shoulder_pitch', 'shoulder_roll', 'elbow_pitch', 'wrist_roll',
                'inner_finger', 'outer_finger'))
FIELDS = ('motor_id', 'position_deg', 'goal_deg', 'error_deg', 'velocity_deg_s',
          'load_percent', 'voltage_v', 'temperature_c', 'torque_enabled',
          'torque_limit_percent', 'moving', 'position_raw', 'motor_position_deg',
          'goal_raw', 'speed_raw', 'load_raw', 'torque_limit_raw')


def records_from_message(message):
    return [{'name': s.name, 'level': s.level[0] if isinstance(s.level, bytes) else int(s.level),
             'message': s.message, 'values': {v.key: v.value for v in s.values}}
            for s in message.status if s.name.startswith('dual_arm/')]


def plain(value):
    # Error strings arriving on a topic must not issue terminal escape commands.
    return ''.join(c if c.isprintable() else ' ' for c in str(value))


def number(values, key, digits=1):
    try:
        value = float(values[key])
        return f'{value:.{digits}f}' if math.isfinite(value) else '--'
    except (KeyError, ValueError, TypeError):
        return '--'


class MonitorView:
    def __init__(self, stale_after=2.0):
        self.stale_after = stale_after
        self.received_at = None
        self.records = {}
        self.stamp = ''

    def update(self, records, now, stamp=''):
        # Replace the whole frame: a missing joint must not retain a normal row.
        self.records = {r['name']: r for r in records}
        self.received_at, self.stamp = now, stamp

    def render(self, now, demo=False):
        age = None if self.received_at is None else max(0.0, now - self.received_at)
        stale = age is None or age >= self.stale_after
        system = self.records.get('dual_arm/system', {})
        state = ('WAITING' if age is None else 'STALE') if stale else system.get('values', {}).get('state', 'UNKNOWN')
        if not stale and system.get('level', 3) >= 2:
            state = 'FAULT' if system['level'] == 2 else 'NO DATA'
        banner = 'DEMO / 예시 데이터 — 실제 로봇 값 아님' if demo else '실기 듀얼 팔 모니터 / 읽기 전용'
        age_text = '--' if age is None else f'{age:.1f}s'
        lines = [banner, f'상태: {plain(state)} | 마지막 수신: {age_text} 전 | ROS 시각: {plain(self.stamp) or "--"}',
                 'Joint                  ID   q(deg)    Goal     Err   deg/s   Load%     V   C  EN   Cap%  Status',
                 '-' * 101]
        for joint in JOINTS:
            record = self.records.get('dual_arm/' + joint, {})
            values = record.get('values', {})
            valid = values.get('sample_valid') == 'true' and record.get('level', 3) < 2
            row_state = 'OK' if valid else ('FAULT' if record.get('level') == 2 else 'NO DATA')
            if stale:
                row_state = 'STALE' if age is not None else 'WAIT'
            # Last valid values remain visible only with an explicit STALE label.
            measured = values if valid else {}
            name = joint.replace('left_', 'L.').replace('right_', 'R.').removesuffix('_joint')
            enable = {'0': 'OFF', '1': 'ON'}.get(measured.get('torque_enabled'), '--')
            line = (f'{name:<23} {number(values, "motor_id", 0):>3}'
                    f' {number(measured, "position_deg"):>8} {number(measured, "goal_deg"):>7}'
                    f' {number(measured, "error_deg"):>7} {number(measured, "velocity_deg_s"):>7}'
                    f' {number(measured, "load_percent"):>7} {number(measured, "voltage_v"):>5}'
                    f' {number(measured, "temperature_c", 0):>3} {enable:>3}'
                    f' {number(measured, "torque_limit_percent"):>6}  {row_state}')
            lines.append(line)
        lines += ['q: URDF 관절각(모터 영점 180° 기준), Err: Goal − q, Cap: 읽어 온 토크 제한 비율',
                  'Load: 추정 부하 비율(+CCW/−CW, 모터 기준). 실제 N·m / 전류 측정값이 아닙니다.']
        if stale:
            lines.append('수신 대기/중단: 실기 구동기 실행 상태와 ROS_DOMAIN_ID를 확인하세요. STALE 숫자는 과거 값입니다.')
        if system.get('message'):
            lines.append('구동기: ' + plain(system['message']))
        for key, title in (('torque_off_unconfirmed', '토크 OFF 확인 실패'),
                           ('last_command_rejection', '최근 명령 거부')):
            if system.get('values', {}).get(key):
                lines.append(title + ': ' + plain(system['values'][key]))
        return '\n'.join(lines)


class CsvRecorder:
    """One row per component per received frame, including faults and source time."""
    def __init__(self, path):
        path = Path(path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        self.file = path.open('x', newline='', encoding='utf-8-sig')
        self.writer = csv.DictWriter(self.file, fieldnames=(
            'received_utc', 'ros_stamp', 'component', 'level', 'message', 'sample_valid',
            'state', 'torque_off_unconfirmed', 'last_command_rejection', *FIELDS))
        self.writer.writeheader()
        self.file.flush()

    def write(self, records, stamp):
        received = datetime.now(timezone.utc).isoformat()
        for record in records:
            values = record['values']
            row = {key: values[key] for key in self.writer.fieldnames if key in values}
            if values.get('sample_valid') != 'true' or record['level'] >= 2:
                # Keep ID, erase measurements from invalid/error frames.
                for key in FIELDS[1:]:
                    row.pop(key, None)
            row.update(received_utc=received, ros_stamp=stamp, component=record['name'],
                       level=record['level'], message=record['message'])
            self.writer.writerow(row)
        self.file.flush()

    def close(self):
        self.file.close()


def demo_records():
    """Static example only, never published into ROS or sent to hardware."""
    records = [{'name': 'dual_arm/system', 'level': 0, 'message': 'DEMO: synthetic sample',
                'values': {'state': 'DEMO', 'sample_valid': 'true'}}]
    for index, joint in enumerate(JOINTS, 1):
        values = dict(motor_id=str(index), sample_valid='true', position_deg='0.0',
                      goal_deg='0.0', error_deg='0.0', velocity_deg_s='0.0',
                      load_percent='0.0', voltage_v='11.1', temperature_c='28',
                      torque_enabled='0', torque_limit_percent='25.0')
        records.append({'name': 'dual_arm/' + joint, 'level': 0, 'message': 'DEMO', 'values': values})
    return records
