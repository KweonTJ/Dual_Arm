"""Centered OpenArm linkage from supplied parallel_motion.py; metres/radians.

Derived from ORV centered gripper 2026-11-18, CERN-OHL-S-2.0.
The active motor angle is zero at closed jaws; passive joints are geometric.
"""
import math

RADIUS = .040
LINK_LENGTH = .035
OFFSET = .0186
CLOSED = .01836143


def absolute_angle(center):
    return math.asin((center*center+OFFSET**2+RADIUS**2-LINK_LENGTH**2)/
                     (2*RADIUS*math.hypot(center,OFFSET)))-math.atan2(OFFSET,center)


START = absolute_angle(CLOSED)
MAX_ANGLE = absolute_angle(CLOSED+.035)-START


def center(angle):
    return RADIUS*math.sin(angle)+math.sqrt(max(0, LINK_LENGTH**2-(RADIUS*math.cos(angle)-OFFSET)**2))


def linkage_angle(angle, sign):
    return math.atan2(sign*(center(angle)-RADIUS*math.sin(angle)),
                      sign*(OFFSET-RADIUS*math.cos(angle)))


def expand_positions(positions):
    result = dict(positions)
    for side in ('left','right'):
        key = side+'_gripper_joint'
        if key not in positions:
            continue
        q = min(MAX_ANGLE,max(0,positions[key])); angle = START+q
        travel = center(angle)-CLOSED
        for tag,sign in (('a',1),('b',-1)):
            result[f'{side}_jaw_{tag}_joint'] = travel
            result[f'{side}_rod_{tag}_joint'] = math.remainder(
                linkage_angle(angle,sign)-linkage_angle(START,sign)-q,math.tau)
    return result


def control_description(description):
    """JSP GUI exposes motors only; full URDF and published TF retain passive joints."""
    import xml.etree.ElementTree as ET
    root = ET.fromstring(description)
    motors = {t.find('joint').get('name') for t in root.findall('transmission')}
    for joint in root.findall('joint'):
        if joint.get('type') != 'fixed' and joint.get('name') not in motors:
            joint.set('type','fixed')
    return ET.tostring(root,encoding='unicode')
