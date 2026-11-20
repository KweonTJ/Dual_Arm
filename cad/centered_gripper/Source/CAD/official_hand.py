"""Millimetres in the existing wrist frame: X forward, Y left, Z toward arm."""
import math
from pathlib import Path
import cadquery as cq
from parallel_motion import *
from mast_parts import box,cyl,prism
ROOT=Path(__file__).resolve().parents[1]
SRC=ROOT/'official'
OUTBOARD=30.0
PBC=[(8,0),(-8,0),(0,8),(0,-8)]
CAGE=[(x,y) for x in (-30,8) for y in (-20,20)]
def disk(diam,h,x,y,z):return cyl(diam,h,(x,y,z))
def yaw(s,angle):return s.rotate((0,0,0),(0,0,1),math.degrees(angle))
def move(s,x=0,y=0,z=0):return s.translate((x,y,z))
def load(name):return cq.importers.importStep(str(SRC/(name+'.STEP'))).val()
def normal(s):
    return s.rotate((0,0,0),(1,1,0),180)
def carriage_frame(s):
    # Native X -> -Z, Y -> -X, Z -> +Y
    return s.rotate((0,0,0),(0,1,0),90).rotate((0,0,0),(0,0,1),90)
def wrist_cradle(side):
    # The axis moves outboard on the gripper mounting shoe. The horn pad stays on the physical wrist axis.
    y0=-OUTBOARD*side
    lid=box(57,44,4,-12,0,-7,3).fuse(
        box(31,abs(y0)+2,4,0,y0/2,-7,2),disk(31,4,0,y0,-7),disk(22,3.1,0,y0,-3.1))
    for x in [-14.7,11.3]:
        triangle=[(-side*17,-22),(-side*20.4,-22),(-side*39,-6.9),(-side*17,-6.9)]
        lid=lid.fuse(prism(triangle,'YZ',x,(3.4,0,0)))
    lid=lid.cut(disk(7.5,9,0,y0,-8))
    for x,y in PBC:
        lid=lid.cut(disk(2.35,9,x,y0+y,-8),disk(4.8,2.4,x,y0+y,-7.02))
    parts=[box(57,44,3.1,-12,0,-10,3).cut(disk(25.4,5,0,y0,-11))]
    for sg in [-1,1]:
        sidewall=box(56,5.4,40.1,-12,sg*19.2,-50,1.4)
        parts.append(sidewall.cut(box(25,10,24,-12,sg*19.2,-42,3)))
    for x in [-39.9,13.9]:
        for y in [-12.8,12.8]:parts.append(box(2.6,7.8,40.1,x,y,-50,.8))
    s=lid.fuse(*parts)
    for x,y in CAGE:
        s=s.fuse(disk(7.6,9,x,y,-50)).cut(disk(4.0,5.6,x,y,-50.1))
    return s.clean()
def fixed_adapter():
    plate=box(56,47,7,-12,0,-57,3).fuse(disk(60,7,0,0,-57)).cut(disk(38,9,0,0,-58))
    for x,y in CAGE:
        plate=plate.cut(disk(3.5,9,x,y,-58),disk(6.4,3.3,x,y,-57.1))
    for k in range(6):
        a=k*math.pi/3
        plate=plate.cut(disk(4,5.3,25*math.sin(a),25*math.cos(a),-57.05))
    return plate.clean()
def horn_adapter():
    s=disk(35.4,7,0,0,-58).cut(disk(7.5,11,0,0,-61))
    for x,y in PBC:s=s.cut(disk(2.35,11,x,y,-61),disk(4.5,4.5,x,y,-58.01))
    for x in [-6.75,6.75]:
        for y in [-13.5*math.sqrt(3)/2,13.5*math.sqrt(3)/2]:
            s=s.cut(disk(4.,5.2,x,y,-58.01))
    return s.clean()
def guide():
    rail=box(9,155,5.5,0,0,-77)
    for y in range(-70,71,20):rail=rail.cut(disk(3.5,7,0,y,-78),disk(6,3.5,0,y,-77))
    return rail.clean()
def block():
    # Catalog envelope with rail channel and 15 x 10 fixing pattern.
    s=box(20,30,7.8,0,0,-81.5).cut(box(9.1,31,3.32,0,0,-77.01))
    for x in [-7.5,7.5]:
        for y in [-5,5]:s=s.cut(disk(2.5,3.2,x,y,-81.55))
    return s.clean()
def native_parts():
    return {k:load(k) for k in ('left_jaw','right_jaw','rail_connector','rail_CNC_attachment','swivel_link','swivel_rotor')}
def meshes():
    p=native_parts()
    arr={}
    def add(k,s,mat):arr[k]=(s,mat)
    for tag,sg in [('Left',1),('Right',-1)]:add('F03_WristCradle'+tag,wrist_cradle(sg),'petg')
    add('AX12A_envelope',box(50,32,40,-13,0,-50,1),'jaw')
    add('F01_FixedAdapter',fixed_adapter(),'petg')
    add('F02_HornAdapter',horn_adapter(),'petg')
    add('OA_RailConnector',move(normal(move(p['rail_connector'],-81.057975,-5,11.5)),z=-57),'metal')
    add('Catalog_RSE2B10_Rail',guide(),'metal')
    add('OA_SwivelRotor',move(normal(p['swivel_rotor']),z=-62),'metal')
    add('OA_SwivelLink',move(yaw(p['swivel_link'],-math.pi/2),z=-76.5),'metal')
    # Stock MR126ZZ rings. Each bearing is inserted from the opposite side of its link.
    add('Catalog_MR126_RotorEnd',disk(12,4,0,0,-75.5).cut(disk(6,6,0,0,-76)),'pin')
    add('Catalog_MR126_SliderEnd',disk(12,4,35,0,-76.5).cut(disk(6,6,35,0,-77)),'pin')
    plate=move(carriage_frame(move(p['rail_CNC_attachment'],0,13.100, -61.61143)),z=-86.5)
    for sign,tag,jaw,centroid in [(1,'Plus','left_jaw',-32.11143),(-1,'Minus','right_jaw',-61.71143)]:
        add('OA_Plate'+tag,plate if sign==1 else yaw(plate,math.pi),'metal')
        add('Catalog_Block'+tag,block(),'pin')
        jaw_s=move(carriage_frame(move(p[jaw],0,13.09997,-centroid)),z=-91.5)
        add('OA_Jaw'+tag,jaw_s,'jaw')
    return arr
def groups(side=1):
    return (['F03_WristCradle'+('Left' if side==1 else 'Right'),'AX12A_envelope','F01_FixedAdapter','OA_RailConnector','Catalog_RSE2B10_Rail'],
        ['F02_HornAdapter','OA_SwivelRotor'],['OA_SwivelLink','Catalog_MR126_RotorEnd','Catalog_MR126_SliderEnd'])
def scene(parts,degrees,side=1):
    t=START+math.radians(degrees);static,rot,rods=groups(side)
    scene=[(k,*parts[k]) for k in static]
    for k in rot:scene.append((k,yaw(parts[k][0],t),parts[k][1]))
    for sg,tag in [(1,'Plus'),(-1,'Minus')]:
        x,y=sg*RADIUS*math.cos(t),sg*RADIUS*math.sin(t)
        for k in rods:scene.append((k+tag,move(yaw(parts[k][0],link_angle(t,sg)),x,y),parts[k][1]))
        for k in ['OA_Plate','Catalog_Block','OA_Jaw']:
            n=k+tag;scene.append((n,move(parts[n][0],y=sg*center(t)),parts[n][1]))
    return scene
