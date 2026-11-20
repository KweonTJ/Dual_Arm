#!/usr/bin/env python3
"""Render the actual URDF and embed its meshes/kinematics in an offline viewer."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from gripper_kinematics import expand_positions, MAX_ANGLE

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
    motors = {t.find('joint').get('name') for t in root.findall('transmission')}
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
        j['actuated'] = j['name'] in motors if motors else j['type'] != 'fixed'
        if j['type'] != 'fixed':
            j.update(axis=numbers(joint.find('axis').get('xyz')).tolist(),
                     lower=float(joint.find('limit').get('lower')), upper=float(joint.find('limit').get('upper')),
                     velocity=float(joint.find('limit').get('velocity')))
        model['joints'].append(j)
    return model


def fk(model, positions=None):
    positions = dict(positions or {})
    if 'left_gripper_rotor_link' in model['links']:
        positions = expand_positions({'left_gripper_joint':0,'right_gripper_joint':0,**positions})
    frames = {'base_link': np.eye(4)}
    pending = list(model['joints'])
    while pending:
        ready = [j for j in pending if j['parent'] in frames]
        if not ready:
            raise ValueError('Disconnected or cyclic URDF')
        for j in ready:
            motion = np.eye(4)
            if j['type'] == 'prismatic':
                motion[:3,3] = np.array(j['axis'])*positions.get(j['name'],0)
            elif j['type'] != 'fixed':
                motion = rotation(j['axis'], positions.get(j['name'],0))
            frames[j['child']] = frames[j['parent']] @ np.array(j['matrix']) @ motion
            pending.remove(j)
    return frames


def attach_motor_ids(model, motor_ids):
    """Associate each bus ID with its joint and the actual servo casing for labels."""
    moving = [j for j in model['joints'] if j['actuated']]
    if (set(motor_ids) != {j['name'] for j in moving}
            or len(set(motor_ids.values())) != len(moving)
            or any(type(value) is not int or not 0 <= value <= 253 for value in motor_ids.values())):
        raise ValueError('Motor IDs must be unique and cover exactly the moving joints')
    for joint in moving:
        joint['motor_id'] = motor_ids[joint['name']]
    model['motor_labels'] = []
    for side in ('left', 'right'):
        # Servo casings can belong to the parent of the joint they drive.
        locations = [('shoulder_pitch', 'mount', 0), ('shoulder_roll', 'shoulder', 0),
                     ('elbow_pitch', 'forearm', 0), ('wrist_roll', 'forearm', 1),
                     ('gripper', 'gripper_motor', 0)]
        for role, body, index in locations:
            name, link = f'{side}_{role}_joint', f'{side}_{body}_link'
            cases = [v for v in model['links'][link] if v['mesh'] in ('ax12a_case.stl','ax12a_cg_case.stl')]
            center = np.array(cases[index]['matrix']) @ np.array([-.0135,0,-.030,1] if role=='gripper' else [0,0,-.013,1])
            model['motor_labels'].append({'joint': name, 'motor_id': motor_ids[name],
                                          'link': link, 'xyz': center[:3].tolist()})


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


def visible_faces(faces, colors, azim, elev):
    """Cull back faces so the preview exposes inverted/missing walls as RViz does."""
    azim, elev = np.deg2rad([azim, elev])
    eye = np.array([np.cos(elev)*np.cos(azim), np.cos(elev)*np.sin(azim), np.sin(elev)])
    normals = np.cross(faces[:, 1]-faces[:, 0], faces[:, 2]-faces[:, 0])
    visible = normals @ eye > 0
    return faces[visible], colors[visible]


def shade_torso_cavity(model, faces, colors):
    """Ambient shading of inner body webs makes front windows legible in PNGs."""
    center = fk(model)['torso_link'][:3,3]
    local = faces.mean(axis=1)-center
    extents = np.ptp(np.array(model['meshes']['premium_torso.stl']).reshape(-1,3),axis=0)/2
    inside = np.all(np.abs(local) <= extents+1e-7,axis=1)
    webs = np.isclose(np.abs(local[:,0]),extents[0]-.003,atol=1e-7)
    webs |= np.isclose(np.abs(local[:,2]),extents[2]-.003,atol=1e-7)
    colors[inside & webs] *= .62
    return colors


def render(model, output, revision, motor_zero):
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
    for role in ['a','b']:
        visuals = model['links'][f'left_jaw_{role}_link']
        points = []
        for visual in visuals:
            m = np.array(visual['matrix'])
            points.append(np.array(model['meshes'][visual['mesh']]).reshape(-1,3) @ m[:3,:3].T + m[:3,3])
        finger_lengths.append((-np.concatenate(points)[:,2].min()+.0865)*1000)
    normals = np.cross(faces[:,1]-faces[:,0], faces[:,2]-faces[:,0])
    normals /= np.maximum(np.linalg.norm(normals,axis=1)[:,None], 1e-12)
    light = np.array([.8,-.35,.75]); light /= np.linalg.norm(light)
    shade = .58 + .42*np.maximum(normals@light, 0)
    colors = np.clip(colors*shade[:,None]+.04, 0, 1)
    colors = shade_torso_cavity(model, faces, colors)
    fig = plt.figure(figsize=(15,9), facecolor='#edf0f4')
    fig.text(.045,.94,'DUAL ARM  /  AX-12A',fontsize=25,weight='bold',color='#1f2c3b')
    body_size = np.ptp(np.array(model['meshes']['premium_torso.stl']).reshape(-1,3), axis=0)*1000
    mount_spacing = abs(frames['left_mount_link'][1,3]-frames['right_mount_link'][1,3])*1000
    fig.text(.045,.902,f'F51 / F52 torso: {body_size[1]:g} W x {body_size[2]:g} H x {body_size[0]:g} D mm  |  Shoulder centres: {mount_spacing:g} mm  |  Stand excluded',fontsize=12,color='#586879')
    for i, (azim,elev,title) in enumerate([(0,0,'FRONT'),(32,19,'PERSPECTIVE')]):
        ax = fig.add_axes([.025+i*.49,.10,.47,.75],projection='3d',facecolor='#edf0f4')
        front_faces, front_colors = visible_faces(faces, colors, azim, elev)
        ax.add_collection3d(Poly3DCollection(front_faces,facecolors=front_colors,edgecolors='none',antialiased=False,zsort='average'))
        ax.set_proj_type('ortho'); ax.view_init(elev=elev,azim=azim)
        ax.dist = 8.4 if i == 0 else 9.2
        for axis,c,s in zip('xyz',center,span):
            getattr(ax,f'set_{axis}lim')(c-s/2,c+s/2)
        # Older matplotlib normalizes array input in-place; preserve bounds for view 2.
        ax.set_box_aspect(tuple(span)); ax.set_axis_off()
        ax.text2D(.07,.96,title,transform=ax.transAxes,fontsize=11,color='#607386',weight='bold')
        if i == 0:
            ax.computed_zorder = False
            for label in model['motor_labels']:
                position = frames[label['link']] @ np.array([*label['xyz'], 1])
                ax.text(*position[:3], str(label['motor_id']), ha='center', va='center',
                        fontsize=12, weight='bold', color='#ffe0a3', zorder=100,
                        bbox=dict(boxstyle='round,pad=.18', facecolor='#172a3a', edgecolor='#ffe0a3', linewidth=.6))
            for side in ('right', 'left'):
                position = frames[f'{side}_upper_arm_link'][:3, 3].copy()
                position[2] += .042
                ax.text(*position, 'ROBOT ' + side.upper(), ha='center', fontsize=9,
                        color='#52667d', weight='bold', zorder=100)
    fig.text(.045,.076,f'Shoulder to elbow: {arm_length:g} mm  |  Centered hand: 150 mm  |  Jaw opening: 0 - 70 mm',fontsize=12,color='#24354a')
    fig.text(.045,.048,f'10 AX-12A  /  motors: black  /  torso + grippers: gray  /  URDF neutral: 0 rad = motor zero: {motor_zero:g} deg',fontsize=10,color='#687789')
    fig.text(.045,.021,'Model: '+revision+'  |  Updated: '+datetime.now().astimezone().isoformat(timespec='seconds'),fontsize=9,color='#687789')
    fig.savefig(output,dpi=150,facecolor=fig.get_facecolor())
    plt.close(fig)


def render_comparison(before, after, output, surface_check=False):
    """Compare actual meshes with the same camera, scale and joint pose."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    def part(model, name):
        faces=np.array(model['meshes'][name])
        return faces, np.full((len(faces),3),.04 if name.startswith('ax12a_') else .65)

    panels=[('F2 / SHOULDER BRACKET','shoulder_fork.stl'),
            ('F5 / PALM FRAME','palm_frame.stl'),
            ('F2 + F11 / OUTER FINGER','finger_long.stl')]
    if surface_check:
        panels[0] = ('AX-12A / CASE', 'ax12a_case.stl')
        panels[1] = ('F2 / SHOULDER BRACKET', 'shoulder_fork.stl')
    azim, elev = (145, 23) if surface_check else (32, 23)
    fig=plt.figure(figsize=(15,10),facecolor='#edf0f4')
    fig.text(.04,.95,'SOLID SURFACES / BEFORE AND AFTER' if surface_check else 'FRAME GEOMETRY / BEFORE AND AFTER',fontsize=24,weight='bold',color='#1f2c3b')
    fig.text(.04,.912,'Same pose, camera and scale. Back-face culling enabled.' if surface_check else 'Same pose, camera and scale in each column. Gray meshes used by the URDF and RViz.',fontsize=12,color='#586879')
    for column,(title,name) in enumerate(panels):
        old,_=part(before,name); new,_=part(after,name)
        vertices=np.concatenate([old.reshape(-1,3),new.reshape(-1,3)])
        center=(vertices.min(0)+vertices.max(0))/2
        span=np.maximum(np.ptp(vertices,axis=0),.035)*1.18
        for row,model in enumerate([before,after]):
            ax=fig.add_axes([.025+column*.325,.49-row*.39,.30,.34],projection='3d',facecolor='#edf0f4')
            faces,colors=part(model,name)
            n=np.cross(faces[:,1]-faces[:,0],faces[:,2]-faces[:,0]); n/=np.maximum(np.linalg.norm(n,axis=1)[:,None],1e-12)
            colors=colors*(.50+.45*np.maximum(n@np.array([.7,-.35,.62]),0))[:,None]+.04
            faces, colors = visible_faces(faces, colors, azim, elev)
            ax.add_collection3d(Poly3DCollection(faces,facecolors=colors,edgecolors='none',antialiased=False))
            ax.set_proj_type('ortho');ax.view_init(elev=elev,azim=azim);ax.dist=8.2
            for axis,c,s in zip('xyz',center,span):getattr(ax,f'set_{axis}lim')(c-s/2,c+s/2)
            ax.set_box_aspect(tuple(span));ax.set_axis_off()
            ax.text2D(.02,.94,'BEFORE' if row==0 else 'AFTER',transform=ax.transAxes,fontsize=11,weight='bold',color='#65788b' if row==0 else '#24644c')
            if row==0:ax.text2D(.02,1.10,title,transform=ax.transAxes,fontsize=11,color='#24354a',weight='bold')
    fig.text(.04,.06,'Closed volumes / outward-facing triangles / opaque motors and frames' if surface_check else 'Changed: chamfered U corners / recessed screw seats / holed palm plate / rounded bent fingertip',fontsize=12,color='#24354a')
    fig.text(.04,.029,'Mounting bores and bracket openings retained. Rendered from the actual STL files.' if surface_check else 'Main drawing dimensions retained. Undimensioned recesses, ribs and bend curves are approximations.',fontsize=10,color='#687789')
    fig.savefig(output,dpi=150,facecolor=fig.get_facecolor());plt.close(fig)


def render_wrist_layout(before, after, output):
    """Compare the actual wrist housing and two-motor hand centrelines."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    names = ['left_forearm_link', 'left_forearm_frame_visual_link',
             'left_palm_link', 'left_palm_frame_visual_link']
    panels = []
    sources = [('BEFORE', before), ('AFTER', after)] if before else [('CURRENT', after)]
    for title, model in sources:
        frames = fk(model)
        faces, colors = [], []
        for name in names:
            for visual in model['links'][name]:
                matrix = frames[name] @ np.array(visual['matrix'])
                tris = np.array(model['meshes'][visual['mesh']]) @ matrix[:3, :3].T + matrix[:3, 3]
                faces.append(tris)
                colors.extend([visual['color']]*len(tris))
        wrist = [v for v in model['links']['left_forearm_link'] if v['mesh']=='ax12a_case.stl'][1]
        wrist_center = (frames['left_forearm_link'] @ np.array(wrist['matrix']) @ np.array([0, 0, -.013, 1]))[:3]
        hand = [v for v in model['links']['left_palm_link'] if v['mesh']=='ax12a_case.stl']
        hand_center = np.mean([(frames['left_palm_link'] @ np.array(v['matrix']) @
                              np.array([0, 0, -.013, 1]))[:3] for v in hand], axis=0)
        panels.append((title, np.concatenate(faces), np.array(colors), wrist_center, hand_center))
    vertices = np.concatenate([p[1].reshape(-1, 3) for p in panels])
    center = (vertices.min(0)+vertices.max(0))/2
    span = np.maximum(np.ptp(vertices, axis=0), .060)*1.16
    fig = plt.figure(figsize=(12, 7), facecolor='#edf0f4')
    fig.text(.05, .93, 'WRIST / HAND OFFSET - SIDE VIEW', fontsize=22, weight='bold', color='#1f2c3b')
    fig.text(.05, .875, 'Corrected direction: the two-motor hand is forward of the wrist housing.', fontsize=11, color='#586879')
    panel_width = .96/len(panels)
    for index, (title, faces, colors, wrist, hand) in enumerate(panels):
        ax = fig.add_axes([.02+index*panel_width, .14, panel_width-.02, .66], projection='3d', facecolor='#edf0f4')
        ax.computed_zorder = False
        normal = np.cross(faces[:, 1]-faces[:, 0], faces[:, 2]-faces[:, 0])
        normal /= np.maximum(np.linalg.norm(normal, axis=1)[:, None], 1e-12)
        colors = colors*(.58+.42*np.maximum(normal@np.array([.5, -.7, .5]), 0))[:, None]+.04
        faces, colors = visible_faces(faces, colors, -90, 0)
        ax.add_collection3d(Poly3DCollection(faces, facecolors=colors, edgecolors='none', antialiased=False, zorder=1))
        z_bounds = [hand[2]-.025, vertices[:, 2].max()+.002]
        near_y = vertices[:, 1].min()-.01
        for x, color in [(hand[0], '#2775b5'), (wrist[0], '#258466')]:
            ax.plot([x, x], [near_y, near_y], z_bounds, '--', color=color, linewidth=1.7, zorder=5)
        ax.set_proj_type('ortho'); ax.view_init(elev=0, azim=-90); ax.dist=8.8
        for axis, c, size in zip('xyz', center, span):
            getattr(ax, f'set_{axis}lim')(c-size/2, c+size/2)
        ax.set_box_aspect(tuple(span)); ax.set_axis_off()
        delta = (wrist[0]-hand[0])*1000
        ax.text2D(.08, .97, f'{title} / BODY OFFSET {delta:+g} mm', transform=ax.transAxes, fontsize=12, weight='bold', color='#24644c' if title!='BEFORE' else '#65788b')
        ax.text2D(.45, .005, 'FRONT (+X)  ->', transform=ax.transAxes, fontsize=11, color='#24644c')
    fig.text(.05, .075, 'Blue dashed: hand centre     Green dashed: wrist housing centre', fontsize=11, color='#586879')
    fig.text(.05, .037, 'Same mounting on both arms. Body offset follows the simplified case geometry; final spacing needs measurement.', fontsize=9, color='#586879')
    fig.savefig(output, dpi=150, facecolor=fig.get_facecolor()); plt.close(fig)


def render_torso(model, output, revision):
    """Close inspection of the current torso and both shoulder attachments."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    # Only body-fixed motors: the complete arms are shown in 01_양팔_전체.png.
    selected = {'torso_link', 'left_mount_link', 'right_mount_link'}
    subset = dict(model, links={name: visuals if name in selected else []
                               for name, visuals in model['links'].items()})
    faces, colors = world_triangles(subset)
    normals = np.cross(faces[:,1]-faces[:,0],faces[:,2]-faces[:,0])
    normals /= np.maximum(np.linalg.norm(normals,axis=1)[:,None],1e-12)
    light = np.array([.7,-.4,.8]); light /= np.linalg.norm(light)
    colors = np.clip(colors*(.6+.4*np.maximum(normals@light,0))[:,None]+.04,0,1)
    colors = shade_torso_cavity(model, faces, colors)
    center = fk(model)['torso_link'][:3,3]
    span = np.array([.115,.300,.115])
    fig = plt.figure(figsize=(14,9),facecolor='#edf0f4')
    fig.text(.04,.945,'PREMIUM TORSO  /  F51 + F52',fontsize=24,weight='bold',color='#1f2c3b')
    fig.text(.04,.91,'90 W x 66 H x 44 D mm  |  Gray body, side-mounted shoulders  |  Black stand excluded',fontsize=12,color='#586879')
    for index,(azim,elev,title) in enumerate([(0,0,'FRONT / F52'),(180,0,'REAR / F51'),(0,90,'TOP / F51'),(35,24,'ASSEMBLY')]):
        ax = fig.add_axes([.01+(index%2)*.5,.10+(1-index//2)*.39,.48,.37],projection='3d',facecolor='#edf0f4')
        visible, shaded = visible_faces(faces, colors, azim, elev)
        ax.add_collection3d(Poly3DCollection(visible,facecolors=shaded,edgecolors='none',antialiased=False))
        for axis,c,s in zip('xyz',center,span):
            getattr(ax,f'set_{axis}lim')(c-s/2,c+s/2)
        ax.set_box_aspect(tuple(span)); ax.set_proj_type('ortho'); ax.view_init(elev=elev,azim=azim)
        ax.dist=5.1; ax.set_axis_off()
        ax.text2D(.06,.95,title,transform=ax.transAxes,fontsize=11,weight='bold',color='#607386')
    fig.text(.04,.06,'Drawing-based envelope and hole grid; aperture profiles and mounting tabs simplified. Assembly mass is estimated.',fontsize=10,color='#586879')
    fig.text(.04,.03,'Model: '+revision+'  |  '+datetime.now().astimezone().isoformat(timespec='seconds'),fontsize=9,color='#687789')
    fig.savefig(output,dpi=150,facecolor=fig.get_facecolor()); plt.close(fig)


def render_centered_gripper(model, output, revision):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    names = {n for n in model['links'] if n.startswith('left_') and any(
        s in n for s in ('palm','gripper','jaw_','rod_'))}
    subset = dict(model,links={n:v if n in names else [] for n,v in model['links'].items()})
    fig=plt.figure(figsize=(14,9),facecolor='#edf0f4')
    fig.text(.04,.94,'CENTERED GRIPPER / STEP GEOMETRY',fontsize=24,weight='bold',color='#1f2c3b')
    fig.text(.04,.895,'Wrist axis = rotor axis = grasp centreline  |  150 mm long  |  155 mm rail bracket',fontsize=12,color='#586879')
    for index,(angle,title) in enumerate(((0,'CLOSED / 0 mm'),(MAX_ANGLE,'OPEN / 70 mm'))):
        faces,colors=world_triangles(subset,{'left_gripper_joint':angle})
        origin=fk(model)['left_palm_link'][:3,3]
        faces-=origin
        normal=np.cross(faces[:,1]-faces[:,0],faces[:,2]-faces[:,0])
        normal/=np.maximum(np.linalg.norm(normal,axis=1)[:,None],1e-15)
        colors=np.clip(colors*(.55+.45*np.maximum(normal@np.array([.6,-.4,.7]),0))[:,None]+.04,0,1)
        faces,colors=visible_faces(faces,colors,25,15)
        ax=fig.add_axes([.01+index*.5,.12,.48,.70],projection='3d',facecolor='#edf0f4')
        ax.add_collection3d(Poly3DCollection(faces,facecolors=colors,edgecolors='none',antialiased=False))
        ax.plot([0,0],[0,0],[.008,-.163],'--',color='#298277',linewidth=1.6)
        center=(0,0,-.075);span=(.14,.205,.19)
        for axis,c,s in zip('xyz',center,span):getattr(ax,f'set_{axis}lim')(c-s/2,c+s/2)
        ax.set_box_aspect(span);ax.set_proj_type('ortho');ax.view_init(elev=15,azim=25);ax.dist=7.5;ax.set_axis_off()
        ax.text2D(.08,.96,title,transform=ax.transAxes,fontsize=15,weight='bold',color='#24644c')
    fig.text(.04,.07,'One AX-12A per hand: right ID 1 / left ID 6. Jaw travel 35 mm each; motor travel 47.87 deg.',fontsize=12,color='#24354a')
    fig.text(.04,.04,'Motor housings black; structure gray. Source: supplied ORV centered gripper CAD (2026-11-18).',fontsize=10,color='#687789')
    fig.text(.04,.015,'Model: '+revision,fontsize=9,color='#687789')
    fig.savefig(output,dpi=150,facecolor=fig.get_facecolor());plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,default=PACKAGE.parents[1]/'preview')
    parser.add_argument('--compare-to',type=Path,help='Earlier URDF, with sibling meshes directory')
    parser.add_argument('--surface-compare-to',type=Path,help='URDF before solid-surface repair, with sibling meshes')
    parser.add_argument('--wrist-compare-to',type=Path,help='Previous wrist mounting URDF with sibling meshes')
    args = parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    model = load_model(PACKAGE/'urdf/dual_arm.urdf')
    calibration = yaml.safe_load((PACKAGE/'config/motor_calibration.yaml').read_text())
    attach_motor_ids(model, calibration['motor_ids'])
    motor_zero = calibration['motor_zero_degrees']
    revision=hashlib.sha256(json.dumps(model,sort_keys=True,separators=(',',':')).encode()).hexdigest()[:10]
    preview_path = args.output/'01_양팔_전체.png'
    render(model,preview_path,revision,motor_zero)
    render_torso(model,args.output/'07_몸체_상세.png',revision)
    render_centered_gripper(model,args.output/'05_중앙정렬_그리퍼.png',revision)
    template = (PACKAGE/'scripts/viewer_template.html').read_text()
    (args.output/'dual_arm_viewer.html').write_text(template.replace('__MODEL_JSON__',json.dumps(model,separators=(',',':'))).replace('__UPDATED_AT__',datetime.now().astimezone().isoformat(timespec='seconds')).replace('__MOTOR_ZERO_DEG__',f'{motor_zero:g}'))
    # 02/04 are retained historical arm-part comparisons. The current hand is
    # a different CAD assembly; 05 presents its actual closed/open geometry.
    manifest={'model_revision':revision,'preview':str(preview_path.resolve()),
              'motor_calibration':calibration,
              'urdf_sha256':hashlib.sha256((PACKAGE/'urdf/dual_arm.urdf').read_bytes()).hexdigest(),
              'mesh_sha256':{name:hashlib.sha256((PACKAGE/'meshes'/name).read_bytes()).hexdigest() for name in model['meshes']}}
    (args.output/'model_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print('Rendered actual URDF:',args.output)
    print('Geometry revision:',revision)
    print('Preview PNG:',preview_path.resolve())


if __name__ == '__main__':
    main()
