#!/usr/bin/env python3
"""Generate original simplified meshes using dimensions from ROBOTIS drawings.

F2/F4/F5/F11 dimensions: official PDFs dated 2010, reviewed 2026-10-04.
Undimensioned outlines, ribs and wall thicknesses remain approximations.
No vendor CAD is redistributed. See config/parts.yaml and README.md.

Requires numpy and vtk (Ubuntu: python3-numpy python3-vtk9). All coordinates in m.
Servo local frame: shaft along X, width along Y, case extends down -Z from shaft.
"""
from pathlib import Path
import math
import struct
import numpy as np
import vtk
import yaml
from vtk.util.numpy_support import vtk_to_numpy

OUT = Path(__file__).resolve().parents[1] / 'meshes'
PARTS = yaml.safe_load((OUT.parent / 'config/parts.yaml').read_text())


def triangles(poly):
    filt = vtk.vtkTriangleFilter()
    filt.SetInputData(poly)
    filt.Update()
    p = filt.GetOutput()
    verts = vtk_to_numpy(p.GetPoints().GetData())
    cells = vtk_to_numpy(p.GetPolys().GetData()).reshape(-1, 4)[:, 1:]
    return verts[cells].astype(float)


def plate(outline, thickness, x=0, holes=()):
    """Extrude a YZ contour with real through holes along X."""
    points, lines = vtk.vtkPoints(), vtk.vtkCellArray()
    for loop in [outline, *holes]:
        ids = [points.InsertNextPoint(x - thickness / 2, u, v) for u, v in loop]
        lines.InsertNextCell(len(ids) + 1)
        for i in ids + ids[:1]:
            lines.InsertCellPoint(i)
    data = vtk.vtkPolyData()
    data.SetPoints(points)
    data.SetLines(lines)
    tri = vtk.vtkContourTriangulator()
    tri.SetInputData(data)
    tri.Update()
    ext = vtk.vtkLinearExtrusionFilter()
    ext.SetInputConnection(tri.GetOutputPort())
    ext.SetExtrusionTypeToVectorExtrusion()
    ext.SetVector(1, 0, 0)
    ext.SetScaleFactor(thickness)
    ext.CappingOn()
    ext.Update()
    return triangles(ext.GetOutput())


def circle(u, v, r, n=24):
    return [(u + r * math.cos(a), v + r * math.sin(a))
            for a in np.linspace(0, 2 * math.pi, n, endpoint=False)]


def rounded(w, bottom, top, bevel=.004):
    return [(-w/2+bevel, bottom), (w/2-bevel, bottom), (w/2, bottom+bevel),
            (w/2, top-bevel), (w/2-bevel, top), (-w/2+bevel, top),
            (-w/2, top-bevel), (-w/2, bottom+bevel)]


def box(size, center=(0, 0, 0)):
    x, y, z = size
    out = plate([(-y/2, -z/2), (y/2, -z/2), (y/2, z/2), (-y/2, z/2)], x)
    return out + np.array(center)


def cylinder(radius, length, center=(0, 0, 0), n=24):
    return plate(circle(0, 0, radius, n), length) + np.array(center)


def transformed(mesh, xyz=(0, 0, 0), rpy=(0, 0, 0)):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
    rz = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
    return mesh @ (rz @ ry @ rx).T + np.array(xyz)


def save(name, parts):
    mesh = np.concatenate(parts)
    normal = np.cross(mesh[:, 1] - mesh[:, 0], mesh[:, 2] - mesh[:, 0])
    lengths = np.linalg.norm(normal, axis=1)
    good = lengths > 1e-12
    mesh, normal = mesh[good], normal[good] / lengths[good, None]
    with (OUT / (name + '.stl')).open('wb') as f:
        f.write(b'Dual Arm simplified drawing-based mesh, metres'.ljust(80, b' '))
        f.write(struct.pack('<I', len(mesh)))
        for face, norm in zip(mesh, normal):
            f.write(struct.pack('<12fH', *norm, *face.reshape(-1), 0))
    print(name, len(mesh), 'triangles')


def bolts(x, positions, radius=.0022):
    return [cylinder(radius, .0014, (x, y, z), n=12) for y, z in positions]


def fork(part):
    """Shaft at origin; back mounting face below it. No anisotropic scaling."""
    p = PARTS[part]
    clear, wall = p['inner_width'], PARTS['approximation']['side_wall']
    back_wall = PARTS['approximation']['back_wall']
    drop, tip, width = p['axis_to_outer_back'], p['axis_to_tip'], p['back_width']
    side_drop = drop-.005
    outline = [(-width/2,-side_drop), (width/2,-side_drop), (width/2,-side_drop+.003),
               (.011,0), *[(.011*math.cos(a),tip*math.sin(a))
                            for a in np.linspace(0,math.pi,17)[1:]],
               (-width/2,-side_drop+.003)]
    horn_holes = [(0,.008),(0,-.008),(-.008,0),(.008,0)]
    # Recessed screw seats: retain the 2.1 mm through bore in the inner web.
    # Recess diameters/depths and corner rib outlines are visual approximations.
    holes = [circle(0,0,.004)] + [circle(y,z,.0027,20) for y,z in horn_holes]
    for z in p['side_hole_rows']:
        holes += [circle(y,z,.00085,12) for y in [-.008,.008]]
    result = [plate(outline,wall,x,holes) for x in [-(clear+wall)/2,(clear+wall)/2]]
    for sign in [-1,1]:
        for y,z in horn_holes:
            result.append(plate(circle(y,z,.0027,20),.0015,
                                sign*(clear/2+.00075),[circle(y,z,.00105,16)]))
    # Back mounting grid: 8 mm pitch, 2.1 mm holes, with a cable opening.
    grid = [circle(x,y,.00105,12) for x in [-.016,-.008,0,.008,.016]
            for y in [-.008,0,.008] if y or abs(x) > .008]
    grid.append([(-.005,-.003),(.005,-.003),(.005,.003),(-.005,.003)])
    cap_width = clear+2*wall-.010
    cap = plate(rounded(cap_width,-width/2,width/2,.001),back_wall,holes=grid)
    # plate's YZ contour becomes XY, and extrusion becomes Z.
    cap = cap[:,:,[1,2,0]] + [0,0,-drop+back_wall/2]
    result.append(cap)
    # Chamfered U corners join the back plate to the side walls, as in the PDF.
    half = clear/2+wall
    corner = [(half-.005,-drop),(half,-drop+.005),(half,-drop+.010),
              (clear/2,-drop+.010),(clear/2,-drop+.007),
              (half-.006,-drop+back_wall),(half-.006,-drop)]
    for sign in [-1,1]:
        result.append(transformed(plate([(-sign*x,z) for x,z in corner],width),rpy=(0,0,math.pi/2)))
    # Hexagonal nut seats on the inner face of the back plate.
    for x in [-.016,-.008,0,.008,.016]:
        for y in [-.008,.008]:
            seat=plate(circle(x,y,.0024,6),.0012,holes=[circle(x,y,.00105,12)])
            result.append(seat[:,:,[1,2,0]]+[0,0,-drop+back_wall+.0006])
    return result


def curved_toe(part, drop):
    """F11 envelope with holed mounting pad and rounded bent tip, in metres.

    Total length/width/depth follow the drawing; the smooth bend is inferred.
    """
    length, depth = part['length'], part['depth']
    mount_holes = [circle(y,z,.00105,16) for y in [-.008,.008] for z in [-.011,-.027]]
    mount_holes.append([(-.002,-.014),(.002,-.014),(.002,-.023),(-.002,-.023)])
    pad=plate(rounded(part['mount_width'],-.030,0,.002),.003,-.013,mount_holes)
    faces=[]
    distances=np.linspace(.030,length,25)
    sections=[]
    for distance in distances:
        t=(distance-.030)/(length-.030)
        x=-.013+.013*t*t*(3-2*t)
        half_width=part['tip_width']/2
        if distance > length-.010:
            half_width*=math.sqrt(max(0,1-((distance-length+.010)/.010)**2))
        sections.append(np.array([[x-.0015,-half_width,-distance],
                                  [x+.0015,-half_width,-distance],
                                  [x+.0015, half_width,-distance],
                                  [x-.0015, half_width,-distance]]))
    for a,b in zip(sections,sections[1:]):
        for j in range(4):
            k=(j+1)%4
            faces.extend([[a[j],b[j],b[k]],[a[j],b[k],a[k]]])
    for end in [sections[0],sections[-1]]:
        faces.extend([[end[0],end[1],end[2]],[end[0],end[2],end[3]]])
    flange=box((depth,part['mount_width'],.003),(0,0,-.0015))
    return [pad+[0,0,-drop],np.array(faces)+[0,0,-drop],flange+[0,0,-drop]]


def main():
    OUT.mkdir(exist_ok=True)
    case = [plate(rounded(.030, -.038, .012, .004), .035)]
    # Ribbed case edges and mounting flanges remain within the 40 x 32 x 50 envelope.
    for y in [-.014, .014]:
        case.append(box((.039, .003, .044), (0, y, -.013)))
        for z in np.arange(-.032, .009, .008):
            case.append(box((.040, .004, .0035), (0, y, z)))
    for x in [-.019, .019]:
        case.append(cylinder(.010, .002, (x, 0, 0)))
    case.append(box((.010, .014, .003), (-.006, 0, -.0365)))
    save('ax12a_case', case)
    metal = [cylinder(.0032, .002, (x, 0, 0), 24) for x in [-.021, .021]]
    for x in [-.0183, .0183]:
        metal += bolts(x, [(y, z) for y in [-.012, .012] for z in [-.032, .007]], .0012)
    save('ax12a_hardware', metal)
    save('horn_screws', [part for x in [-.025,.025] for part in
                        bolts(x, [(0,.008),(0,-.008),(-.008,0),(.008,0)], .0021)])
    save('ax12a_label', [box((.0004, .022, .009), (.018, 0, -.018))])
    text = vtk.vtkVectorText()
    text.SetText('AX-12A')
    text.Update()
    letters = triangles(text.GetOutput())
    lo, hi = letters.reshape(-1, 3).min(0), letters.reshape(-1, 3).max(0)
    letters -= lo
    letters *= .018 / (hi[0] - lo[0])
    # text XY -> model YZ, forward face at +X
    letters = np.stack([np.full(letters.shape[:2], .0183),
                        letters[:, :, 0] - .009, letters[:, :, 1] - .0205], axis=2)
    save('ax12a_lettering', [letters])
    save('shoulder_fork', fork('FP04-F2'))
    # Same F4 part is reused at the upper arm and on the short gripper finger.
    save('upper_yoke', fork('FP04-F4'))
    # Direct end mount: inward-facing long case meets the first shoulder output.
    # This short plate replaces the incorrect long shelf under two upright cases.
    save('shoulder_adapter', [box((.036,.002,.032),(0,.001,0))])
    # Wrist housing mount, no invented extra actuator.
    save('wrist_adapter', [box((.032,.040,.003),(0,0,-.0015)),
                           box((.032,.003,.040),(0,-.019,-.020)),
                           box((.032,.003,.040),(0,.019,-.020))])
    # FP04-F5: 78.5 mm outside / 72.5 mm inside span, 40 mm return.
    p = PARTS['FP04-F5']
    cap_half=p['outer_span']/2-.011
    # F5's back-plate horn pattern is offset 22.25 mm from one end.
    horn_y=-p['outer_span']/2+p['plate_horn_from_end']
    top_holes=[circle(0,horn_y,.004)]
    top_holes += [circle(x,y,.00105,16) for x,y in
                  [(0,horn_y-.008),(0,horn_y+.008),(-.008,horn_y),(.008,horn_y)]]
    top_holes += [circle(x,y,.00105,12) for x in [-.008,0,.008]
                  for y in [0,.008,.016,.024]]
    cap=plate(rounded(p['width'],-cap_half,cap_half,.001),.003,holes=top_holes)
    palm=[cap[:,:,[1,2,0]]+[0,0,-.0015]]
    corner=[(cap_half,0),(p['outer_span']/2,-.011),(p['outer_span']/2,-.015),
            (p['inner_span']/2,-.015),(p['inner_span']/2,-.012),(cap_half,-.003)]
    for sign in [-1,1]:
        palm.append(plate([(sign*y,z) for y,z in corner],p['width']))
    profile = rounded(p['width'],-p['return_depth'],-.011,.007)
    side = plate(profile,.003,holes=[circle(0,-p['axis_to_back'],.004)] +
                 [circle(x,z,.00105,12) for x,z in [(0,-.021),(0,-.037),(-.008,-.029),(.008,-.029)]])
    for y in [-(p['inner_span']+.003)/2,(p['inner_span']+.003)/2]:
        palm.append(transformed(side,xyz=(0,y,0),rpy=(0,0,math.pi/2)))
    save('palm_frame',palm)
    save('finger_short',fork('FP04-F4'))
    # FP04-F11 bent finger, mounted to the outer F2 back. Mount alignment is inferred.
    p = PARTS['FP04-F11']
    drop = PARTS['FP04-F2']['axis_to_outer_back']
    # Rotate only the lower F11 about its mounting normal (-90 degrees about Z).
    # Its flat wall moves from local -X to +Y. The arm Xacro mirrors this with
    # yaw=pi on the right outer finger, so both walls face away from the body.
    toe = [transformed(piece,rpy=(0,0,-math.pi/2)) for piece in curved_toe(p,drop)]
    save('finger_long',fork('FP04-F2') + toe)
    print('Generated original STL assets in', OUT)


if __name__ == '__main__':
    main()
