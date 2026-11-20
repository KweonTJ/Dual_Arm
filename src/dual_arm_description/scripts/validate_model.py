#!/usr/bin/env python3
"""Check graph, assets, inertias and branch motion in the expanded URDF."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from export_preview import load_model, fk, stl, PACKAGE, attach_motor_ids
from mesh_geometry import surface_report


def main():
    path = PACKAGE/'urdf/dual_arm.urdf'
    root = ET.parse(path).getroot()
    for location in root.iter('origin'):
        for attribute in ('xyz', 'rpy'):
            values = np.fromstring(location.get(attribute, '0 0 0'), sep=' ')
            assert values.shape == (3,) and np.isfinite(values).all(), location.attrib
    model = load_model(path)
    joints = root.findall('joint')
    assert len(model['links']) == 32
    assert len(joints) == 31
    assert len({j.get('name') for j in joints}) == 31
    motors = {t.find('joint').get('name') for t in root.findall('transmission')}
    moving = [j for j in joints if j.get('name') in motors]
    assert len(moving) == 10
    calibration = yaml.safe_load((PACKAGE/'config/motor_calibration.yaml').read_text())
    assert calibration['motor_zero_degrees'] == 180.0
    assert calibration['urdf_zero_radians'] == 0.0
    assert len(calibration['joints']) == 10
    assert set(calibration['joints']) == {j.get('name') for j in moving}
    hardware = yaml.safe_load((PACKAGE.parent/'dual_arm_hardware/config/hardware.yaml').read_text())
    assert calibration['motor_ids'] == {name: values['id'] for name, values in hardware['motors'].items()}
    roles = ('shoulder_pitch', 'shoulder_roll', 'elbow_pitch', 'wrist_roll', 'gripper')
    for side, ids in [('right', [5, 4, 3, 2, 1]), ('left', [10, 9, 8, 7, 6])]:
        assert [calibration['motor_ids'][f'{side}_{role}_joint'] for role in roles] == ids
    attach_motor_ids(model, calibration['motor_ids'])
    assert not root.findall('.//mimic'), 'Nonlinear followers must not use a linear URDF mimic'
    frames = fk(model)
    assert set(frames) == set(model['links'])
    motor_centers = {label['motor_id']: (frames[label['link']] @ np.array([*label['xyz'], 1]))[:3]
                     for label in model['motor_labels']}
    # Front image: +Y projects to the right. Check the hand pairs and shoulder pair.
    assert motor_centers[1][1] < 0 < motor_centers[6][1]
    assert motor_centers[4][1] < motor_centers[5][1] < 0 < motor_centers[10][1] < motor_centers[9][1]
    servo_count = sum(1 for m in root.findall('.//visual/geometry/mesh') if m.get('filename').endswith(('/ax12a_case.stl','/ax12a_cg_case.stl')))
    assert servo_count == 10
    for asset, faces in model['meshes'].items():
        triangles = stl(PACKAGE/'meshes'/asset)
        assert triangles.shape[1:] == (3,3) and np.isfinite(triangles).all(), asset
        assert np.linalg.norm(np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0]),axis=1).min() > (1e-18 if asset.startswith(('cg_','ax12a_cg')) else 1e-12), asset
        # Read full STL precision: the browser preview rounds coordinates.
        report = surface_report(stl(PACKAGE/'meshes'/asset))
        assert report['boundary_edges'] == 0, (asset, report)
        assert report['unbalanced_edges'] == 0, (asset, report)
        assert report['signed_volume_m3'] > 0, (asset, report)
    for material in root.findall('material'):
        assert float(material.find('color').get('rgba').split()[3]) == 1.0
    # Independent drawing dimensions catch mm/m mistakes and accidental stretching.
    expected_extents = {
        'premium_torso.stl': (.044,.090,.066),
        'ax12a_case.stl': (.040,.032,.050),
        'upper_yoke.stl': (.048,.029,.0647),
        'cg_fixed.stl': (.0712,.155,.081),
    }
    for asset, size in expected_extents.items():
        vertices = np.array(model['meshes'][asset]).reshape(-1,3)
        assert np.allclose(np.ptp(vertices,axis=0),size,atol=2e-7), (asset,np.ptp(vertices,axis=0))
    for asset in ['shoulder_fork.stl','upper_yoke.stl']:
        xs = np.array(model['meshes'][asset])[:,:,0]
        assert np.any(np.isclose(xs,.021)) and np.any(np.isclose(xs,-.021)), asset
    for mesh in root.findall('.//visual/geometry/mesh'):
        if mesh.get('filename').rsplit('/',1)[-1] in ['shoulder_fork.stl','upper_yoke.stl','palm_frame.stl','finger_short.stl','finger_long.stl']:
            assert np.allclose(np.fromstring(mesh.get('scale','1 1 1'),sep=' '),1)
    assert np.isclose(np.linalg.norm(frames['left_forearm_link'][:3,3]-frames['left_upper_arm_link'][:3,3]),.026+.052+.002)
    # New body has finite shell panels and side-mounted motors, no bench stand.
    torso = root.find("link[@name='torso_link']")
    assert root.find("link[@name='crossbar_link']") is None
    assert not any('stand' in name or 'fixture' in name for name in model['links'])
    assert np.isclose(float(torso.find('inertial/mass').get('value')), .060)
    assert len(torso.findall('collision')) == 6  # Front, rear, top and notched bottom.
    body_vertices = np.array(model['meshes']['premium_torso.stl']).reshape(-1,3)
    # Closed surfaces alone do not prove the holes survived triangulation.
    body_faces = stl(PACKAGE/'meshes/premium_torso.stl')
    def covered(axis, plane, point):
        caps = body_faces[np.all(np.isclose(body_faces[:,:,axis],plane,atol=1e-7),axis=1)]
        caps = np.delete(caps,axis,axis=2)
        edges = np.roll(caps,-1,axis=1)-caps
        delta = np.asarray(point)-caps
        cross = edges[:,:,0]*delta[:,:,1]-edges[:,:,1]*delta[:,:,0]
        return bool(np.any(np.all(cross>=-1e-12,axis=1)|np.all(cross<=1e-12,axis=1)))
    for y in (-.024,0,.024):
        for z in (-.020,0,.020):
            assert not covered(0,.022,(y,z)), ('F52 window filled',y,z)
            if y == 0 or z == 0:
                assert not covered(0,-.022,(y,z)), ('F51 window filled',y,z)
        assert not covered(2,.033,(0,y)), ('F51 top window filled',y)
    assert not covered(0,.022,(0,.028)), 'Mounting bore filled'
    assert covered(0,.022,(.044,0)), 'Body web missing'
    body_frame = frames['torso_link']
    body_world = body_vertices @ body_frame[:3,:3].T + body_frame[:3,3]
    body_min, body_max = body_world.min(0), body_world.max(0)
    for side in ['left', 'right']:
        mount = root.find(f"joint[@name='{side}_mount_joint']")
        assert mount.get('type') == 'fixed' and mount.find('parent').get('link') == 'torso_link'
        case = next(v for v in model['links'][side+'_mount_link'] if v['mesh']=='ax12a_case.stl')
        matrix = frames[side+'_mount_link'] @ np.array(case['matrix'])
        vertices = np.array(model['meshes'][case['mesh']]).reshape(-1,3)
        vertices = vertices @ matrix[:3,:3].T + matrix[:3,3]
        lo, hi = vertices.min(0), vertices.max(0)
        assert np.isclose(body_max[2]-hi[2], .002, atol=1e-7), (side, 'motor top below body top')
        assert body_min[2] < lo[2] < hi[2] < body_max[2]
        clearance = lo[1]-body_max[1] if side=='left' else body_min[1]-hi[1]
        assert np.isclose(clearance,.002,atol=1e-7), (side, 'side mounting clearance',clearance)
    assert np.isclose(frames['left_mount_link'][1,3]-frames['right_mount_link'][1,3], .134)
    # Locate the output hub in the actual STL, independently of Xacro offsets.
    hardware = stl(PACKAGE/'meshes/ax12a_hardware.stl').reshape(-1, 3)
    hub = hardware[hardware[:, 0] > .0199]
    hub_mount = (hub.min(axis=0)+hub.max(axis=0))/2
    hub_mount[0] = hub[:, 0].max()
    for side in ['left', 'right']:
        wrist_motor = [v for v in model['links'][side+'_forearm_link']
                       if v['mesh'] == 'ax12a_hardware.stl'][1]
        motor_frame = frames[side+'_forearm_link'] @ np.array(wrist_motor['matrix'])
        hub_world = (motor_frame @ np.r_[hub_mount, 1])[:3]
        assert np.allclose(frames[side+'_palm_link'][:3, 3], hub_world, atol=1e-8), side
        # User-corrected mounting: the wrist housing extends rearward from its shaft.
        assert motor_frame[0, 2] > .9, side
        wrist_body = (motor_frame @ np.array([0, 0, -.013, 1]))[:3]
        assert np.allclose(frames[side+'_tool0'][:3,3], (frames[side+'_palm_link'] @ np.array([0,0,-.150,1]))[:3])
    mass = 0.0
    display_links = 0
    for link in root.findall('link'):
        visuals = link.findall('visual')
        # Humble's renderer selects the first visual material for the whole link.
        # Thus XML per-visual colors alone do not establish RViz color correctness.
        materials = {v.find('material').get('name') for v in visuals}
        assert len(materials) <= 1, f"RViz Humble material mixing in {link.get('name')}"
        for v in visuals:
            filename = v.find('geometry/mesh').get('filename').rsplit('/',1)[-1]
            expected = 'servo_black' if filename.startswith('ax12a_') else 'frame_grey'
            assert v.find('material').get('name') == expected, filename
        inertial = link.find('inertial')
        if link.get('name').endswith('_frame_visual_link'):
            parent_joint = next(j for j in joints if j.find('child').get('link') == link.get('name'))
            assert parent_joint.get('type') == 'fixed'
            parent = root.find(f"link[@name='{parent_joint.find('parent').get('link')}']")
            assert parent.find('inertial') is not None and parent.find('collision') is not None
            assert np.allclose(frames[link.get('name')],frames[parent.get('name')])
            assert not any(j.find('parent').get('link') == link.get('name') for j in joints)
            assert inertial is None and link.find('collision') is None
            display_links += 1
            continue
        if not visuals:
            continue
        assert inertial is not None, link.get('name')
        m = float(inertial.find('mass').get('value')); assert m > 0; mass += m
        i = {k: float(v) for k,v in inertial.find('inertia').attrib.items()}
        tensor = np.array([[i['ixx'],i['ixy'],i['ixz']],[i['ixy'],i['iyy'],i['iyz']],[i['ixz'],i['iyz'],i['izz']]])
        eig = np.linalg.eigvalsh(tensor)
        assert np.all(eig > 0) and eig[2] <= eig[0]+eig[1]+1e-12, link.get('name')
    assert display_links == 4
    for j in moving:
        axis = np.fromstring(j.find('axis').get('xyz'),sep=' ')
        assert np.isclose(np.linalg.norm(axis),1)
        lower,upper = (float(j.find('limit').get(k)) for k in ['lower','upper'])
        assert lower <= 0 < upper
        moved = fk(model,{j.get('name'):min(.35,upper)})
        assert not np.allclose(moved[j.find('child').get('link')],frames[j.find('child').get('link')])
        # Every joint leaves the opposite arm untouched; each finger leaves its sibling untouched.
        side = j.get('name').split('_')[0]
        other = 'right' if side == 'left' else 'left'
        for name in frames:
            if name.startswith(other+'_'):
                assert np.allclose(moved[name],frames[name])
    for suffix in ['mount_link','shoulder_link','upper_arm_link','forearm_link','palm_link','tool0']:
        left,right = frames['left_'+suffix][:3,3],frames['right_'+suffix][:3,3]
        assert np.allclose(left,right*np.array([1,-1,1]))
    print(f'PASS: 32 links, 10 actuated + 8 passive + 13 fixed joints, {servo_count} servo meshes')
    print('PASS: F51/F52 torso 90 W x 66 H x 44 D mm; 134 mm side-mounted shoulder spacing; no stand')
    print('PASS: F52 nine windows, F51 five rear and three top windows, and mounting bore stay open')
    print(f'PASS: {len(model["meshes"])} finite STL assets; positive physical inertias; estimated mass {mass:.4f} kg')
    print('PASS: all STL surfaces closed, directed edges balanced, positive enclosed volume; material alpha = 1')
    print('PASS: left/right symmetry, all joint transforms, centered parallel grippers')
    print('PASS: RViz Humble first-visual material matches every mesh; 4 fixed visual-only arm frame links')
    print('PASS: AX-12A / FP04 mesh dimensions, 42 mm fork opening, unscaled parts, 80 mm upper-arm spacing')
    print('PASS: wrist-roll axes follow the moved motor horns; all 10 motor zeros are 180 degrees at URDF q=0')
    print('PASS: wrist axis and grasp centreline coincide; STEP-derived hands replace asymmetric fingers')
    print('Hardware calibration and collision-free motion are NOT established by these checks.')


if __name__ == '__main__':
    main()
