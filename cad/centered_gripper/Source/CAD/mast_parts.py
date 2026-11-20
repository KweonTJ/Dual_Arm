"""ORV + BIOLOID, Rev C. Dimensions in mm. Requires CadQuery 2.7.
Run: python mobile_dual_arm.py --out build
This file models the printed parts, a camera envelope and simple stock datums.
The STEP folder contains positioned parts; STL files are oriented on the print bed.
"""
from pathlib import Path
import argparse, json, math
import cadquery as cq
from OCP.BRepCheck import BRepCheck_Analyzer

# +X front, +Y robot left, +Z upward. Floor at Z=0.
DECK = 88.0
SHOULDER = 390.0
BAR_DEPTH = 40.0
BAR_HEIGHT = 20.0
BAR_TO_AXIS = 48.0                  # pipe center to shoulder; see source URDF
CLAMP_SIDE_CLEARANCE = 0.30         # each side of the measured 40 mm face
BASE_HOLES = [(53.16,66.83),(53.16,-66.83),(-100.83,63.88),(-101.84,-63.19)]
BASE_HOLE_D = 3.60                  # printed plate only; original holes need checking
BASE_SLOT_TRAVEL = 4.00             # total movement along X, nominal center +/-2
ROD_X, ROD_Y = 30.0,20.0
ROD_BORE = 4.60
INSERT_HOLE = 4.00                  # adapt to actual M3 heat-set inserts and coupon
BASE_THICK, SADDLE_THICK = 10.0,12.0
BAR_BOT = SHOULDER-BAR_TO_AXIS-BAR_HEIGHT/2
M0 = DECK+BASE_THICK
S0 = BAR_BOT-SADDLE_THICK
MH = (S0-M0)/2
BAR_TOP = BAR_BOT+BAR_HEIGHT
CAMERA_PITCH = 20.0                 # fixed chest bracket, downward from horizontal
CAM_REAR = (50.0,0.0,BAR_TOP+2.0)  # center of the D455 rear mounting-hole pair
CAM_WIDTH,CAM_DEPTH,CAM_HEIGHT=124.0,26.0,29.0
CAM_PITCH_HOLES=95.0                # RealSense D455, rear M4 pair
CAM_CLEAR=4.5                       # backplate thickness
RODS=[(x,y) for x in [-ROD_X,ROD_X] for y in [-ROD_Y,ROD_Y]]
ROOT=Path(__file__).resolve().parent

def box(l,w,h,x=0,y=0,z=0,r=0):
    obj=cq.Workplane('XY').box(l,w,h,centered=(True,True,False))
    if r: obj=obj.edges('|Z').fillet(r)
    return obj.val().translate((x,y,z))
def cyl(d,h,p,direction=(0,0,1)):
    return cq.Solid.makeCylinder(d/2,h,cq.Vector(*p),cq.Vector(*direction))
def outline(points,z):
    return cq.Wire.makePolygon([cq.Vector(x,y,z) for x,y in points],close=True)
def loft(polys):
    return cq.Solid.makeLoft([outline(points,z) for z,points in polys],True)
def rectpoints(l,w,x=0,y=0,corner=3):
    a,b=l/2,w/2;k=min(corner,a/2,b/2)
    return [(x+dx,y+dy) for dx,dy in [(-a+k,-b),(a-k,-b),(a,-b+k),(a,b-k),(a-k,b),(-a+k,b),(-a,b-k),(-a,-b+k)]]
def std_solid(obj):
    obj=obj.clean()
    solids=obj.Solids()
    if len(solids)!=1 or not BRepCheck_Analyzer(solids[0].wrapped).IsValid():
        raise ValueError(f'Invalid or disconnected printable part ({len(solids)} solids)')
    return solids[0]
def prism(poly,plane,start,vec):
    if plane=='XZ': arr=[cq.Vector(x,start,z) for x,z in poly]
    else: arr=[cq.Vector(start,y,z) for y,z in poly]
    ring=cq.Wire.makePolygon(arr,close=True)
    return cq.Solid.extrudeLinear(ring,[],cq.Vector(*vec))
def male_at(z):
    return [cyl(10,3.05,(x,y,z-0.05)) for x,y in RODS]
def sockets_at(z):
    return [cyl(10.5,3.4,(x,y,z-0.05)) for x,y in RODS]
def rod_bores(lo,hi):
    return [cyl(ROD_BORE,hi-lo,(x,y,lo)) for x,y in RODS]
def base():
    main=box(180,150,BASE_THICK,-24,0,DECK,9).fuse(*male_at(M0))
    for x,y in BASE_HOLES:
        slot=cq.Workplane('XY').slot2D(BASE_HOLE_D+BASE_SLOT_TRAVEL,BASE_HOLE_D).extrude(BASE_THICK+2).val()
        main=main.cut(slot.translate((x,y,DECK-1)))
    windows=[box(45,38,14,-71,0,DECK-1,9),box(35,13,14,-62,43,DECK-1,6),
             box(35,13,14,-62,-43,DECK-1,6),box(32,21,14,7,0,DECK-1,6)]
    nuts=[cyl(12,5.35,(x,y,DECK-.05)) for x,y in RODS]
    return std_solid(main.cut(*windows,*nuts,*rod_bores(DECK-.1,M0+3.2)))
def mast():
    h=MH
    # Four printed sleeves carry compression. The M4 rods preload the interfaces.
    # The side flanges merge into the sleeves and provide a broad, flat print face.
    pieces=[]
    for x,y in RODS:
        pieces.append(cyl(13.2,h,(x,y,0)))
        pieces.append(box(8,4.6,h,x,(26.7 if y>0 else -26.7),0,1.2))
        for z in [0,h-9]:
            pieces.append(cyl(17.6,9,(x,y,z)))
    for z in [0,h-8]:
        rim=box(76,58,8,z=z,r=7)
        pieces.append(rim.cut(box(65.6,47.6,10,z=z-1,r=4)))
    # Opposite faces have opposite diagonals to brace both shear directions.
    # Strips are 4.4 mm thick; their width in the face plane exceeds 6 mm.
    for side in [-1,1]:
        x0=-side*ROD_X; x1=side*ROD_X
        y0=side*ROD_Y; y1=-side*ROD_Y
        yz_low=-29 if side<0 else 24.6
        xz_low=-36.3 if side<0 else 31.9
        pieces.append(prism([(x0-4,4),(x0+4,4),(x1+4,h-4),(x1-4,h-4)],
                            'XZ',yz_low,(0,4.4,0)))
        pieces.append(prism([(y0-4,4),(y0+4,4),(y1+4,h-4),(y1-4,h-4)],
                            'YZ',xz_low,(4.4,0,0)))
    # Preserve the cable-cover M3 insert faces and their full insertion depth.
    insert_points=[(18,7),(-18,h-8)]
    for x,z in insert_points:
        pieces.append(cyl(11,8.5,(x,-29,z),(0,1,0)))
    body=pieces[0].fuse(*pieces[1:],*male_at(h))
    holes=rod_bores(-.1,h+3.2)+sockets_at(0)
    for x,z in insert_points:
        holes.append(cyl(INSERT_HOLE,6.1,(x,-29.05,z),(0,1,0)))
    for z in [6,h-6]:
        holes.append(box(17,4,2.8,-35,7,z-1.4,1))
    return std_solid(body.cut(*holes))
def saddle():
    body=box(86,64,SADDLE_THICK,z=S0,r=5)
    edge=BAR_DEPTH/2+CLAMP_SIDE_CLEARANCE
    width=39-edge
    for s in [-1,1]:
        body=body.fuse(box(width,56,6.05,s*(edge+width/2),0,BAR_BOT-.05,2))
    body=body.cut(*sockets_at(S0),*rod_bores(S0-.1,BAR_BOT+6.2))
    for y in [-12,12]:
        body=body.cut(cyl(INSERT_HOLE,6,(43.05,y,S0+6),(-1,0,0)))
    return std_solid(body)
def clamp_cap():
    plate=box(82,58,8,z=BAR_TOP,r=6)
    return std_solid(plate.cut(*rod_bores(BAR_TOP-.1,BAR_TOP+8.1)))
def tilt(shape):
    return shape.rotate((0,0,0),(0,1,0),CAMERA_PITCH).translate(CAM_REAR)
def chest():
    # The apron and camera rail form one printable body. The upper camera rail
    # feeds its load to the saddle through the two existing front fasteners.
    def section(w,edge,nose):
        front=[(edge,-w/2),(nose-3,-w*.34),(nose,0),(nose-3,w*.34),(edge,w/2)]
        return front+[(x-3.3,y) for x,y in reversed(front)]
    apron=loft([(S0-53,section(49,48,55)),(S0-44,section(61,45,58)),
                (S0+7,section(90,43,60)),(S0+12,section(89,43,59))])
    web=loft([(S0+1,rectpoints(15,41,51,corner=3)),
              (S0+10,rectpoints(15,45,51,corner=3)),
              (CAM_REAR[2]-6,rectpoints(11,46,47,corner=3)),
              (CAM_REAR[2]+4,rectpoints(9,44,48,corner=3))])
    plate=tilt(box(CAM_CLEAR,114,28,-CAM_CLEAR/2,0,-14,2))
    web=web.cut(tilt(box(120,160,60,60,0,-20)))
    body=apron.fuse(web,plate)
    for y in [-12,12]:
        z=S0+6
        body=body.fuse(cyl(10,18,(43,y,z),(1,0,0)))
        body=body.cut(cyl(3.5,21,(42.8,y,z),(1,0,0)),cyl(6.4,6,(58.5,y,z),(1,0,0)))
    for y in [-CAM_PITCH_HOLES/2,CAM_PITCH_HOLES/2]:
        body=body.cut(tilt(cyl(4.5,12,(-8,y,0),(1,0,0))))
    # Recess for the USB-C plug on either lateral side below the optical bar.
    for y in [-37,37]:
        body=body.cut(tilt(box(30,21,22,12,y,-36.0,2)))
    return std_solid(body)
def camera_envelope():
    # Rounded rear/front profiles; dimensional envelope, not a vendor solid.
    b=cq.Workplane('YZ').rect(CAM_WIDTH,CAM_HEIGHT).extrude(CAM_DEPTH).edges('|X').fillet(11.5).val()
    return tilt(b)
def cable_fairing():
    # U-section protects the harness along the outside of the lower mast.
    def profile(x,w):
        pts=[(-.5*w,-30),(-.5*w,-35),(-.32*w,-40),(.30*w,-40),(.5*w,-35),(.5*w,-30),
             (.5*w-2.7,-30),(.5*w-2.7,-34),(.23*w,-37),(-.24*w,-37),(-.5*w+2.7,-34),(-.5*w+2.7,-30)]
        return [(x+xx,yy) for xx,yy in pts]
    shape=loft([(M0+2,profile(18,23)),(M0+13,profile(17,35)),(M0+MH-19,profile(-16,38)),
                (M0+MH-3,profile(-18,29)),(M0+MH+2,profile(-18,21))])
    for x,z in [(18,M0+7),(-18,M0+MH-8)]:
        shape=shape.fuse(cyl(10,11.1,(x,-29,z),(0,-1,0)))
        shape=shape.cut(cyl(3.5,14,(x,-27.8,z),(0,-1,0)),cyl(6.6,3,(x,-41,z),(0,1,0)))
    return std_solid(shape)
def cam_coupon():
    g=box(17,114,CAM_CLEAR,r=3)
    for y in [-CAM_PITCH_HOLES/2,CAM_PITCH_HOLES/2]:g=g.cut(cyl(4.5,7,(0,y,-1)))
    return std_solid(g)
def stl_pose(shape,axis,angle):
    p=shape.rotate((0,0,0),axis,angle) if angle else shape
    bb=p.BoundingBox()
    return p.translate((-(bb.xmin+bb.xmax)/2,-(bb.ymin+bb.ymax)/2,-bb.zmin))
def export_stl(shape,path):
    # OCC writes binary triangles in mm. Returned deflection is adequate for M3 holes.
    shape.exportStl(str(path),tolerance=.045,angularTolerance=.12)
def components():
    return {
        'P01_Deck':(base(),(0,0,1),0,'graphite'),
        'P02_Mast':(mast(),(1,0,0),90,'graphite'),
        'P03_Saddle':(saddle(),(0,0,1),0,'graphite'),
        'P04_ClampCap':(clamp_cap(),(0,0,1),0,'graphite'),
        'P05_D455_Chest':(chest(),(0,1,0),90,'white'),
        'P07_CableCover':(cable_fairing(),(1,0,0),90,'white'),
        'G01_D455_Gauge':(cam_coupon(),(0,0,1),0,'white')}
def print_instances(parts):
    out=[]
    for key,(shape,axis,angle,mat) in parts.items():
        if key.startswith('G'):continue
        if key=='P02_Mast':
            out.extend([(key+'_Lower',shape.translate((0,0,M0)),mat),(key+'_Upper',shape.translate((0,0,M0+MH)),mat)])
        else:out.append((key,shape,mat))
    return out
def finish(assembly,path):
    assembly.save(str(path),exportType='STEP',mode='default')
def run(out):
    for folder in ['stl','step','view']:
        (out/folder).mkdir(parents=True,exist_ok=True)
    parts=components()
    colors={'graphite':cq.Color(.22,.26,.29),'white':cq.Color(.90,.90,.87)}
    assemblies=cq.Assembly(name='ORV_DualArm_Printed')
    entries=[];ms={}
    for name,(shape,axis,ang,mat) in parts.items():
        sp=stl_pose(shape,axis,ang)
        export_stl(sp,out/'stl'/(name+'.stl'))
        bb=sp.BoundingBox()
        ms[name]={'bed_mm':[round(bb.xlen,2),round(bb.ylen,2),round(bb.zlen,2)],'cad_solid_cm3':round(shape.Volume()/1000,2)}
        if name.startswith('P'):
            # Individual part STEP located locally for convenient import.
            step_part=stl_pose(shape,(0,0,1),0)  # carry assembly axes into the local STEP
            cq.exporters.export(step_part,str(out/'step'/(name+'.step')))
    for name,shape,mat in print_instances(parts):
        assemblies.add(shape,name=name,color=colors[mat]);export_stl(shape,out/'view'/(name+'.stl'));entries.append([name,mat])
    cam=camera_envelope()
    export_stl(cam,out/'view'/'REF_Camera.stl')
    finish(assemblies,out/'step'/'01_Printed_Assembly.step')
    layout=cq.Assembly(name='ORV_Layout_mm')
    layout.add(assemblies,name='Printed_Components')
    gray=cq.Color(.39,.43,.46,.75);steel=cq.Color(.52,.55,.57);rubber=cq.Color(.12,.13,.14)
    # These reference solids indicate spatial envelopes; actual mechanism shown in the guide.
    layout.add(box(290,155,67,z=21,r=7),name='REF_Chassis_envelope',color=gray)
    for x in [-100,100]:
        for y in [-105,105]:
            layout.add(cyl(88,35,(x,y-17.5,44),(0,1,0)),name=f'REF_Wheel_{x}_{y}',color=rubber)
    layout.add(box(BAR_DEPTH,158,BAR_HEIGHT,z=BAR_BOT),name='REF_Aluminium_crossbar',color=steel)
    for i,s in enumerate([-1,1]):
        # Lower hands occupy a much wider neutral footprint than the pipe.
        layout.add(box(32,40,50,0,s*59,SHOULDER-38,r=2),name=f'REF_Inner_shoulder_{i}',color=rubber)
        layout.add(box(44,74,48,0,s*113,SHOULDER-32,r=4),name=f'REF_Outer_shoulder_{i}',color=gray)
        layout.add(box(43,42,152,0,s*121,SHOULDER-164,r=3),name=f'REF_Arm_envelope_{i}',color=gray)
        layout.add(box(52,78.5,65,10,s*121,SHOULDER-218,r=4),name=f'REF_Palm_envelope_{i}',color=gray)
        for side,length in [(-1,54),(1,83.3)]:
            yy=s*(121+side*20)
            layout.add(box(28,13,length,13,yy,SHOULDER-210-length,r=3),name=f'REF_Finger_{i}_{side}',color=gray)
    layout.add(cam,name='REF_RealSense_envelope',color=steel)
    for i,(x,y) in enumerate(RODS):
        layout.add(cyl(4,BAR_TOP+14.8-(DECK+.8),(x,y,DECK+.8)),name=f'STD_Tie_rod_M4_{i+1}',color=steel)
    finish(layout,out/'step'/'02_Robot_Layout.step')
    (out/'view'/'instances.json').write_text(json.dumps(entries))
    (out/'measurements.json').write_text(json.dumps(ms,indent=2))
    print(json.dumps(ms,indent=2))
if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,default=Path('build'))
    run(ap.parse_args().out)
