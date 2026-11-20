"""OpenArm 1.0.1 follower on AX-12A. Angles in rad, mechanical constants in mm."""
import math
RADIUS=40.0
LINK_LENGTH=35.0
OFFSET=18.6
CLOSED=18.36143  # measured distance from the mounting-hole centroid to the inner pad
SWEEP=math.radians(60)
def absolute_angle(c):
    # (c-R*sin(t))^2+(OFFSET-R*cos(t))^2 = LINK_LENGTH^2
    return math.asin((c*c+OFFSET**2+RADIUS**2-LINK_LENGTH**2)/(2*RADIUS*math.hypot(c,OFFSET)))-math.atan2(OFFSET,c)
START=absolute_angle(CLOSED)
def center(theta):
    return RADIUS*math.sin(theta)+math.sqrt(LINK_LENGTH**2-(RADIUS*math.cos(theta)-OFFSET)**2)
def displacement(angle):
    return center(START+angle)-CLOSED
TRAVEL=displacement(SWEEP)
def motor_angle(travel_mm):
    if not math.isfinite(travel_mm) or not -1e-8<=travel_mm<=TRAVEL+1e-8:
        raise ValueError('finger coordinate outside CAD range')
    return absolute_angle(CLOSED+min(TRAVEL,max(0,travel_mm)))-START
def link_angle(t,sign):
    return math.atan2(sign*(center(t)-RADIUS*math.sin(t)),sign*(OFFSET-RADIUS*math.cos(t)))
def model_state(side,travel_m):
    a=motor_angle(travel_m*1000); t=START+a
    result={side+'_rotor_joint':a,side+'_finger_joint1':travel_m,side+'_finger_joint2':travel_m}
    for i,sign in enumerate(([1,-1] if side=='left' else [-1,1]),1):
        result[side+f'_rodlink{i}_joint']=math.remainder(link_angle(t,sign)-link_angle(START,sign)-a,math.tau)
    return result
