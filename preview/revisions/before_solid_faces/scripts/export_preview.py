#!/usr/bin/env python3
"""Render the actual URDF and embed its meshes/kinematics in an offline viewer."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import struct
import xml.etree.ElementTree as ET
import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]


def numbers(value, default='0 0 0'):
    return np.array([float(v) for v in (value or default).split()])


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    m = np.eye(4)
    m[:3, :3] = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]]) @ np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]]) @ np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    m[:3, 3] = xyz
    return m


def rotation(axis, angle):
    a = np.array(axis, dtype=float)
    a /= np.linalg.norm(a)
    x, y, z = a
    k = np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    m = np.eye(4)
    m[:3, :3] = np.eye(3) + np.sin(angle)*k + (1-np.cos(angle))*(k@k)
    return m


def origin(element):
    o = element.find('origin')
    return transform() if o is None else transform(numbers(o.get('xyz')), numbers(o.get('rpy')))


def stl(path):
    raw = path.read_bytes()
    count, = struct.unpack_from('<I', raw, 80)
    dtype = np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')])
    return np.frombuffer(raw, dtype=dtype, offset=84, count=count)['vertices'].astype(float)


def load_model(path):
    path = Path(path).resolve()
    mesh_directory = path.parents[1]/'meshes'
    root = ET.parse(path).getroot()
    materials = {m.get('name'): numbers(m.find('color').get('rgba'))[:3].tolist() for m in root.findall('material')}
    model = {'name': root.get('name'), 'links': {}, 'joints': [], 'meshes': {}}
    for link in root.findall('link'):
        visuals = []
        for v in link.findall('visual'):
            mesh = v.find('geometry/mesh')
            if mesh is None:
                raise ValueError('Preview currently expects the description package mesh visuals')
            filename = mesh.get('filename').split('/')[-1]
            if filename not in model['meshes']:
                model['meshes'][filename] = stl(mesh_directory/filename).round(7).tolist()
            m = origin(v)
            m[:3, :3] = m[:3, :3] @ np.diag(numbers(mesh.get('scale'), '1 1 1'))
            visuals.append({'mesh':filename, 'matrix':m.tolist(), 'color':materials[v.find('material').get('name')]})
        model['links'][link.get('name')] = visuals
    for joint in root.findall('joint'):
        j = {'name':joint.get('name'), 'type':joint.get('type'),
             'parent':joint.find('parent').get('link'), 'child':joint.find('child').get('link'),
             'matrix':origin(joint).tolist()}
        if j['type'] != 'fixed':
            j.update(axis=numbers(joint.find('axis').get('xyz')).tolist(),
                     lower=float(joint.find('limit').get('lower')), upper=float(joint.find('limit').get('upper')))
        model['joints'].append(j)
    return model


def fk(model, positions=None):
    positions = positions or {}
    frames = {'base_link': np.eye(4)}
    pending = list(model['joints'])
    while pending:
        ready = [j for j in pending if j['parent'] in frames]
        if not ready:
            raise ValueError('Disconnected or cyclic URDF')
        for j in ready:
            motion = rotation(j['axis'], positions.get(j['name'],0)) if j['type'] != 'fixed' else np.eye(4)
            frames[j['child']] = frames[j['parent']] @ np.array(j['matrix']) @ motion
            pending.remove(j)
    return frames


def world_triangles(model, positions=None):
    frames = fk(model, positions)
    faces, colors = [], []
    for name, visuals in model['links'].items():
        for visual in visuals:
            m = frames[name] @ np.array(visual['matrix'])
            tris = np.array(model['meshes'][visual['mesh']]) @ m[:3,:3].T + m[:3,3]
            faces.append(tris)
            colors.extend([visual['color']] * len(tris))
    return np.concatenate(faces), np.array(colors)


def render(model, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    faces, colors = world_triangles(model)
    vertices = faces.reshape(-1,3)
    center = (vertices.min(axis=0)+vertices.max(axis=0))/2
    span = np.maximum(np.ptp(vertices,axis=0),[.18,.20,.20])*1.14
    frames = fk(model)
    arm_length = np.linalg.norm(frames['left_upper_arm_link'][:3,3]-frames['left_forearm_link'][:3,3])*1000
    finger_lengths = []
    for role in ['inner','outer']:
        visuals = model['links'][f'left_{role}_finger_link']
        points = []
        for visual in visuals:
            m = np.array(visual['matrix'])
            points.append(np.array(model['meshes'][visual['mesh']]).reshape(-1,3) @ m[:3,:3].T + m[:3,3])
        finger_lengths.append(-np.concatenate(points)[:,2].min()*1000)
    normals = np.cross(faces[:,1]-faces[:,0], faces[:,2]-faces[:,0])
    normals /= np.maximum(np.linalg.norm(normals,axis=1)[:,None], 1e-12)
    light = np.array([.8,-.35,.75]); light /= np.linalg.norm(light)
    shade = .58 + .42*np.abs(normals@light)
    colors = np.clip(colors*shade[:,None]+.04, 0, 1)
    fig = plt.figure(figsize=(15,9), facecolor='#edf0f4')
    fig.text(.045,.94,'DUAL ARM  /  AX-12A',fontsize=25,weight='bold',color='#1f2c3b')
    fig.text(.045,.902,'FP04 drawing dimensions  |  F2 + F4 arm frames  |  F5 palm  |  F4 / F2 + F11 fingers',fontsize=12,color='#586879')
    for i, (azim,elev,title) in enumerate([(0,0,'FRONT'),(32,19,'PERSPECTIVE')]):
        ax = fig.add_axes([.025+i*.49,.10,.47,.75],projection='3d',facecolor='#edf0f4')
        ax.add_collection3d(Poly3DCollection(faces,facecolors=colors,edgecolors='none',zsort='average'))
        ax.set_proj_type('ortho'); ax.view_init(elev=elev,azim=azim)
        ax.dist = 8.4 if i == 0 else 9.2
        for axis,c,s in zip('xyz',center,span):
            getattr(ax,f'set_{axis}lim')(c-s/2,c+s/2)
        # Older matplotlib normalizes array input in-place; preserve bounds for view 2.
        ax.set_box_aspect(tuple(span)); ax.set_axis_off()
        ax.text2D(.07,.96,title,transform=ax.transAxes,fontsize=11,color='#607386',weight='bold')
    fig.text(.045,.076,f'Shoulder to elbow: {arm_length:g} mm  |  Inner finger: {finger_lengths[0]:g} mm  |  Outer finger: {finger_lengths[1]:g} mm',fontsize=12,color='#24354a')
    fig.text(.045,.048,'12 AX-12A  /  motors: black  /  frames: gray  /  assembly offsets and joint zeros remain provisional',fontsize=10,color='#687789')
    fig.text(.045,.021,'Updated: '+datetime.now().astimezone().isoformat(timespec='seconds'),fontsize=9,color='#687789')
    fig.savefig(output,dpi=150,facecolor=fig.get_facecolor())
    plt.close(fig)


def render_comparison(before, after, output):
    """Compare actual meshes with the same camera, scale and joint pose."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    def part(model, name):
        faces=np.array(model['meshes'][name])
        return faces, np.full((len(faces),3),.65)

    panels=[('F2 / SHOULDER BRACKET','shoulder_fork.stl'),
            ('F5 / PALM FRAME','palm_frame.stl'),
            ('F2 + F11 / OUTER FINGER','finger_long.stl')]
    fig=plt.figure(figsize=(15,10),facecolor='#edf0f4')
    fig.text(.04,.95,'FRAME GEOMETRY / BEFORE AND AFTER',fontsize=24,weight='bold',color='#1f2c3b')
    fig.text(.04,.912,'Same pose, camera and scale in each column. Gray meshes used by the URDF and RViz.',fontsize=12,color='#586879')
    for column,(title,name) in enumerate(panels):
        old,_=part(before,name); new,_=part(after,name)
        vertices=np.concatenate([old.reshape(-1,3),new.reshape(-1,3)])
        center=(vertices.min(0)+vertices.max(0))/2
        span=np.maximum(np.ptp(vertices,axis=0),.035)*1.18
        for row,model in enumerate([before,after]):
            ax=fig.add_axes([.025+column*.325,.49-row*.39,.30,.34],projection='3d',facecolor='#edf0f4')
            faces,colors=part(model,name)
            n=np.cross(faces[:,1]-faces[:,0],faces[:,2]-faces[:,0]); n/=np.maximum(np.linalg.norm(n,axis=1)[:,None],1e-12)
            colors=colors*(.50+.45*np.abs(n@np.array([.7,-.35,.62])))[:,None]+.04
            ax.add_collection3d(Poly3DCollection(faces,facecolors=colors,edgecolors='none'))
            ax.set_proj_type('ortho');ax.view_init(elev=23,azim=32);ax.dist=8.2
            for axis,c,s in zip('xyz',center,span):getattr(ax,f'set_{axis}lim')(c-s/2,c+s/2)
            ax.set_box_aspect(tuple(span));ax.set_axis_off()
            ax.text2D(.02,.94,'BEFORE' if row==0 else 'AFTER',transform=ax.transAxes,fontsize=11,weight='bold',color='#65788b' if row==0 else '#24644c')
            if row==0:ax.text2D(.02,1.10,title,transform=ax.transAxes,fontsize=11,color='#24354a',weight='bold')
    fig.text(.04,.06,'Changed: chamfered U corners / recessed screw seats / holed palm plate / rounded bent fingertip',fontsize=12,color='#24354a')
    fig.text(.04,.029,'Main drawing dimensions retained. Undimensioned recesses, ribs and bend curves are approximations.',fontsize=10,color='#687789')
    fig.savefig(output,dpi=150,facecolor=fig.get_facecolor());plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,default=PACKAGE.parents[1]/'preview')
    parser.add_argument('--compare-to',type=Path,help='Earlier URDF, with sibling meshes directory')
    args = parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    model = load_model(PACKAGE/'urdf/dual_arm.urdf')
    render(model,args.output/'dual_arm_preview.png')
    template = (PACKAGE/'scripts/viewer_template.html').read_text()
    (args.output/'dual_arm_viewer.html').write_text(template.replace('__MODEL_JSON__',json.dumps(model,separators=(',',':'))).replace('__UPDATED_AT__',datetime.now().astimezone().isoformat(timespec='seconds')))
    revision=hashlib.sha256(json.dumps(model,sort_keys=True,separators=(',',':')).encode()).hexdigest()[:10]
    versioned=args.output/f'dual_arm_preview_{revision}.png'
    shutil.copyfile(args.output/'dual_arm_preview.png',versioned)
    if args.compare_to:
        render_comparison(load_model(args.compare_to),model,args.output/'frame_geometry_comparison.png')
    manifest={'model_revision':revision,'preview':str(versioned.resolve()),
              'urdf_sha256':hashlib.sha256((PACKAGE/'urdf/dual_arm.urdf').read_bytes()).hexdigest(),
              'mesh_sha256':{name:hashlib.sha256((PACKAGE/'meshes'/name).read_bytes()).hexdigest() for name in model['meshes']}}
    (args.output/'model_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print('Rendered actual URDF:',args.output)
    print('Geometry revision:',revision)
    print('Versioned PNG:',versioned.resolve())


if __name__ == '__main__':
    main()
