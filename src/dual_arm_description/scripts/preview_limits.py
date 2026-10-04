"""Angle bounds for synthetic preview states, never hardware feedback."""
from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET


@dataclass(frozen=True)
class JointLimit:
    lower: float
    upper: float


def read_limits(description):
    limits = {}
    for joint in ET.fromstring(description).findall('joint'):
        if joint.get('type') == 'fixed':
            continue
        name = joint.get('name')
        if joint.get('type') != 'revolute' or not name or name in limits:
            raise ValueError(f'Expected a unique bounded revolute joint: {name}')
        tag = joint.find('limit')
        if tag is None:
            raise ValueError(f'Missing limits: {name}')
        lo, hi, speed, effort = (float(tag.get(key, 'nan'))
                                 for key in ('lower', 'upper', 'velocity', 'effort'))
        soft = joint.find('safety_controller')
        if soft is not None:
            soft_lo = float(soft.get('soft_lower_limit', lo))
            soft_hi = float(soft.get('soft_upper_limit', hi))
            if not all(math.isfinite(v) for v in (soft_lo, soft_hi)):
                raise ValueError(f'Non-finite soft limits: {name}')
            lo, hi = max(lo, soft_lo), min(hi, soft_hi)
        if not all(math.isfinite(v) for v in (lo, hi, speed, effort)):
            raise ValueError(f'Non-finite limits: {name}')
        if not (lo <= 0 <= hi and lo < hi and speed > 0 and effort > 0):
            raise ValueError(f'Invalid limits or neutral pose: {name}')
        # URDF velocity remains valid metadata; no preview speed cap is applied.
        limits[name] = JointLimit(lo, hi)
    if not limits:
        raise ValueError('No bounded joints')
    return limits


class PreviewLimiter:
    """Apply angle-clamped targets at the next update, hold on stale input.

    No speed interpolation, acceleration, collision or torque simulation.
    Time is local monotonic time, used only to reject stale inputs.
    """
    def __init__(self, limits, timeout=0.5):
        if not limits or not math.isfinite(timeout):
            raise ValueError('Invalid limiter configuration')
        if timeout <= 0:
            raise ValueError('Timeout must be positive')
        self.limits = limits
        self.positions = {name: 0.0 for name in limits}
        self.targets = dict(self.positions)
        self.last_input = None
        self.timeout = timeout

    def accept(self, names, positions, now):
        # Reject a malformed batch atomically; never partly apply it.
        if (len(names) != len(self.limits) or len(positions) != len(names)
                or len(set(names)) != len(names) or set(names) != set(self.limits)):
            raise ValueError('Expected one position for every preview joint')
        if not math.isfinite(now) or not all(math.isfinite(v) for v in positions):
            raise ValueError('Non-finite target')
        bounded = {}
        clamped = []
        for name, value in zip(names, positions):
            limit = self.limits[name]
            bounded[name] = min(limit.upper, max(limit.lower, value))
            if bounded[name] != value:
                clamped.append(name)
        self.targets = bounded
        self.last_input = now
        return clamped

    def step(self, now):
        if not math.isfinite(now):
            raise ValueError('Non-finite time')
        if self.last_input is None or not 0 <= now-self.last_input <= self.timeout:
            self.targets = dict(self.positions)
        self.positions = dict(self.targets)
        return dict(self.positions)
