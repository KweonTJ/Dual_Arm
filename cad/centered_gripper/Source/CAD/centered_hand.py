from pathlib import Path
import sys,math,csv,itertools
import numpy as np, cadquery as cq
D=Path(__file__).resolve().parent; root=D.parents[1]
sys.path.insert(0,str(D))
import official_hand as h
from mast_parts import box,cyl,prism,export_stl,stl_pose
from functools import cache
src=root; out=root
def cut(s,pts,d,lo,hi):
 return s.cut(*[cyl(d,hi-lo,(x,y,lo)) for x,y in pts]).clean()
def load(id):
 cat='Reference' if id.startswith('REF_') else 'Parts'
 base=D/'Baseline' if id in ['OA_LeftJaw','OA_RightJaw'] else src
 return cq.importers.importStep(str(base/cat/(id+'.step'))).val()
casepts=list(itertools.product([-30.5,-22.5],[-13.5,13.5]))
new_cuts=[('Wrist_clear',h.PBC,2.35,-7,0),('Wrist_seat',h.PBC,4.8,-7,-4.62),('Plate_insert',h.CAGE,4,-50,-44.5),('Body_clear',casepts,2.4,-14,-3),('Body_seat',casepts,4.4,-10.3,-3)]
keypts=[(-8,-16),(8,16)]
def cross(dia,length,x,y,z,side):
 return cyl(dia,length,(x,y,z),(0,side,0))
def cap(blank=False):
 # The horn shoe can be fitted with the motor pod removed. Two keys locate the pod on installation.
 s=box(31,56.5,4,0,0,-7,2).fuse(cyl(22,3.1,(0,0,-3.1))).cut(cyl(7.5,9,(0,0,-8)))
 for sign in [-1,1]:
  s=s.fuse(box(25,6.,13.2,0,sign*25.25,-20,1))
  for x in (-7,7):s=s.cut(cross(3.4,7,x,sign*22,-14.5,sign))
 if not blank:
  s=cut(s,h.PBC,2.35,-7.05,.1);s=cut(s,h.PBC,4.8,-7.05,-4.62)
 return cut(s,keypts,4.4,-7.05,-5.05).translate((0,0,7))
def cradle(blank=False):
 s=box(57,44,3.0,-12,0,-10,3).cut(cyl(25.4,5,(0,0,-11)))
 parts=[]
 for sign in (-1,1):
  side=box(56,5.4,40.1,-12,sign*19.2,-50,1.4)
  parts.append(side.cut(box(25,10,24,-12,sign*19.2,-42,3)))
  parts.append(box(29,5.8,15,0,sign*19.1,-22,1))
 for x,y in itertools.product([-39.9,13.9],[-12.8,12.8]):
  parts.append(box(2.6,7.8,40.1,x,y,-50,.8))
 for x,y in h.CAGE:parts.append(cyl(7.6,9,(x,y,-50)))
 for x,y in casepts:parts.append(cyl(6.6,4.1,(x,y,-14)))
 for x,y in keypts:parts.append(cyl(4,1.7,(x,y,-7.05)))
 s=s.fuse(*parts).clean()
 if not blank:
  for tag,pts,d,lo,hi in new_cuts[2:]: s=cut(s,pts,d,lo,hi)
 for sign in [-1,1]:
  for x in (-7,7):s=s.cut(cross(4.,5.2,x,sign*22.05,-14.5,-sign))
 return s.clean().translate((0,0,50))
def rail():
 s=box(9,145,5.5)
 for y in range(-60,61,20):
  s=cut(s,[(0,y)],3.5,-.1,5.6)
  s=cut(s,[(0,y)],6,-.1,3.5)
 return s
def bracket():
 # Rail mounting taps at +/-40 and +/-60; end lips are outside the slide-block travel envelope.
 flange=cyl(57,3,(0,0,-60)).cut(cyl(37,5,(0,0,-61)))
 flange=cut(flange,[(25*math.sin(k*math.pi/3),25*math.cos(k*math.pi/3)) for k in range(6)],3.4,-61,-56)
 for sign in [-1,1]:
  poly=[(23,-57),(28.7,-57),(38.8,-68.5),(75,-68.5),(77.5,-71),(77.5,-81),(75,-81),(75,-71.5),(37.4,-71.5),(27.3,-60),(23,-60)]
  flange=flange.fuse(prism([(sign*y,z) for y,z in poly],'YZ',-5,(10,0,0)))
 flange=cut(flange,[(25*math.sin(k*math.pi/3),25*math.cos(k*math.pi/3)) for k in range(6)],6.4,-63.5,-59.98)
 return cut(flange,[(0,y) for y in [-60,-40,40,60]],2.5,-72,-68).translate((0,0,57))
def jaw(id):
 s=load(id)
 # Preserve the entire bolted root and fingertip; shorten the intervening web in Z.
 z1=-98+86.5;z2=-150+86.5
 k=(52-(162.50185-150))/52;d=(162.50185-150)
 def slab(lo,hi):return s.intersect(box(130,190,hi-lo,0,0,lo))
 top=slab(z1,1);mid=slab(z2,z1);tip=slab(-150,z2)
 mat=cq.Matrix([[1,0,0,0],[0,1,0,0],[0,0,k,z1*(1-k)],[0,0,0,1]])
 return top.fuse(mid.transformGeometry(mat),tip.translate((0,0,d))).clean()
@cache
def parts():
 transforms={'C02_WristShoe':cap,'C03_CenteredCradle':cradle,'C04_RailBracket155':bracket,'REF_Rail145':rail,'C05_JawA':lambda:jaw('OA_LeftJaw'),'C06_JawB':lambda:jaw('OA_RightJaw')}
 ids=['F01_FixedAdapter','F02_HornAdapter','REF_AX12A_Case','REF_B01_Horn','OA_Rotor','OA_Link','REF_Bearing','REF_Block','OA_CarriagePlate']
 return {**{i:f() for i,f in transforms.items()},**{i:load(i) for i in ids}}
def T(x=0,y=0,z=0):
 m=np.eye(4);m[:3,3]=[x,y,z];return m
def R(r):
 s,c=math.sin(r),math.cos(r);m=np.eye(4);m[:2,:2]=[[c,-s],[s,c]];return m
def loc(m):
 return cq.Location(cq.Vector(*m[:3,3]),cq.Vector(0,0,1),math.degrees(math.atan2(m[1,0],m[0,0])))
def state(gap):
 a=h.START+h.motor_angle(gap/2); c=h.center(a)
 items=[('Wrist_shoe','C02_WristShoe',T(z=-7)),('Wrist_cradle','C03_CenteredCradle',T(z=-50)),('Fixed_adapter','F01_FixedAdapter',T(z=-57)),('Rotating_hub','F02_HornAdapter',R(a)@T(z=-58)),('Rail_connector','C04_RailBracket155',T(z=-57)),('Rail','REF_Rail145',T(z=-77)),('Rotor','OA_Rotor',R(a)@T(z=-58)),('Gripper_AX12A_case','REF_AX12A_Case',T(z=-51)),('Gripper_B01','REF_B01_Horn',R(a)@T(z=-51)),('Existing_wrist_B01','REF_B01_Horn',T())]
 for sign,tag,id in [(1,'A','C05_JawA'),(-1,'B','C06_JawB')]:
  link=T(sign*40*math.cos(a),sign*40*math.sin(a))@R(h.link_angle(a,sign));slide=T(y=sign*c)
  items.extend([('Link_'+tag,'OA_Link',link@T(z=-76.5)),('Bearing_rotor_'+tag,'REF_Bearing',link@T(z=-75.5)),('Bearing_slide_'+tag,'REF_Bearing',link@T(35,0,-76.5)),('Plate_'+tag,'OA_CarriagePlate',slide@R(0 if sign==1 else math.pi)@T(z=-86.5)),('Block_'+tag,'REF_Block',slide@T(z=-81.5)),('Jaw_'+tag,id,slide@T(z=-86.5))])
 return items
def shapes(gap):
 p=parts();return {n:p[id].located(loc(m)) for n,id,m in state(gap)}
def overlap(s,t,tol=.0004):
 a,b=s.BoundingBox(),t.BoundingBox()
 if min(min(getattr(a,j+'max'),getattr(b,j+'max'))-max(getattr(a,j+'min'),getattr(b,j+'min')) for j in 'xyz')<tol:return 0.
 return s.intersect(t).Volume()
def intersect_pairs(gap):
 ss=shapes(gap)
 return [(n,k,round(v,3)) for (n,s),(k,t) in itertools.combinations(ss.items(),2) if (v:=overlap(s,t))>.005]
if __name__=='__main__':
 for n,s in parts().items():print(n,len(s.Solids()),s.isValid(),round(s.Volume(),2),flush=True)
 for gap in [0,35,70]:print('hand overlap',gap,intersect_pairs(gap),flush=True)
