#!/usr/bin/env python3
"""Check graph, assets, inertias and branch motion in the expanded URDF."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from export_preview import load_model, fk, stl, PACKAGE
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
    assert len(model['links']) == 23
    assert len(joints) == 22
    assert len({j.get('name') for j in joints}) == 22
    moving = [j for j in joints if j.get('type') == 'revolute']
    assert len(moving) == 12
    calibration = yaml.safe_load((PACKAGE/'config/motor_calibration.yaml').read_text())
    assert calibration['motor_zero_degrees'] == 180.0
    assert calibration['urdf_zero_radians'] == 0.0
    assert len(calibration['joints']) == 12
    assert set(calibration['joints']) == {j.get('name') for j in moving}
    assert not root.findall('.//mimic'), 'Both physical finger motors must stay independent'
    frames = fk(model)
    assert set(frames) == set(model['links'])
    servo_count = sum(1 for m in root.findall('.//visual/geometry/mesh') if m.get('filename').endswith('/ax12a_case.stl'))
    assert servo_count == 12
    for asset, faces in model['meshes'].items():
        triangles = np.array(faces)
        assert triangles.shape[1:] == (3,3) and np.isfinite(triangles).all(), asset
        assert np.linalg.norm(np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0]),axis=1).min() > 1e-12, asset
        # Read full STL precision: the browser preview rounds coordinates.
        report = surface_report(stl(PACKAGE/'meshes'/asset))
        assert report['boundary_edges'] == 0, (asset, report)
        assert report['unbalanced_edges'] == 0, (asset, report)
        assert report['signed_volume_m3'] > 0, (asset, report)
    for material in root.findall('material'):
        assert float(material.find('color').get('rgba').split()[3]) == 1.0
    # Independent drawing dimensions catch mm/m mistakes and accidental stretching.
    expected_extents = {
        'ax12a_case.stl': (.040,.032,.050),
        'upper_yoke.stl': (.048,.029,.0647),
        'finger_short.stl': (.048,.029,.0647),
        'palm_frame.stl': (.028,.0785,.040),
    }
    for asset, size in expected_extents.items():
        vertices = np.array(model['meshes'][asset]).reshape(-1,3)
        assert np.allclose(np.ptp(vertices,axis=0),size,atol=2e-7), (asset,np.ptp(vertices,axis=0))
    for asset in ['shoulder_fork.stl','upper_yoke.stl','finger_short.stl']:
        xs = np.array(model['meshes'][asset])[:,:,0]
        assert np.any(np.isclose(xs,.021)) and np.any(np.isclose(xs,-.021)), asset
    for mesh in root.findall('.//visual/geometry/mesh'):
        if mesh.get('filename').rsplit('/',1)[-1] in ['shoulder_fork.stl','upper_yoke.stl','palm_frame.stl','finger_short.stl','finger_long.stl']:
            assert np.allclose(np.fromstring(mesh.get('scale','1 1 1'),sep=' '),1)
    assert np.isclose(np.linalg.norm(frames['left_forearm_link'][:3,3]-frames['left_upper_arm_link'][:3,3]),.026+.052+.002)
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
        hand_motors = [v for v in model['links'][side+'_palm_link'] if v['mesh']=='ax12a_case.stl']
        hand_body = np.mean([(frames[side+'_palm_link'] @ np.array(v['matrix']) @
                             np.array([0, 0, -.013, 1]))[:3] for v in hand_motors], axis=0)
        assert hand_body[0] > wrist_body[0], (side, wrist_body, hand_body)
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
        assert inertial is not None and link.find('collision') is not None, link.get('name')
        m = float(inertial.find('mass').get('value')); assert m > 0; mass += m
        i = {k: float(v) for k,v in inertial.find('inertia').attrib.items()}
        tensor = np.array([[i['ixx'],i['ixy'],i['ixz']],[i['ixy'],i['iyy'],i['iyz']],[i['ixz'],i['iyz'],i['izz']]])
        eig = np.linalg.eigvalsh(tensor)
        assert np.all(eig > 0) and eig[2] <= eig[0]+eig[1]+1e-12, link.get('name')
    assert display_links == 6
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
        if 'finger' in j.get('name'):
            sibling = f"{side}_{'outer' if 'inner' in j.get('name') else 'inner'}_finger_link"
            assert np.allclose(moved[sibling],frames[sibling])
            assert np.allclose(moved[side+'_palm_link'],frames[side+'_palm_link'])
    for suffix in ['mount_link','shoulder_link','upper_arm_link','forearm_link','palm_link','tool0']:
        left,right = frames['left_'+suffix][:3,3],frames['right_'+suffix][:3,3]
        assert np.allclose(left,right*np.array([1,-1,1]))
    print(f'PASS: 23 links, 12 revolute + 10 fixed joints, {servo_count} servo meshes')
    print(f'PASS: {len(model["meshes"])} finite STL assets; positive physical inertias; estimated mass {mass:.4f} kg')
    print('PASS: all STL surfaces closed, directed edges balanced, positive enclosed volume; material alpha = 1')
    print('PASS: left/right symmetry, all joint transforms, independent gripper branches')
    print('PASS: RViz Humble first-visual material matches every mesh; 6 fixed visual-only frame links')
    print('PASS: AX-12A / FP04 mesh dimensions, 42 mm fork opening, unscaled parts, 80 mm upper-arm spacing')
    print('PASS: wrist-roll axes follow the moved motor horns; all 12 motor zeros are 180 degrees at URDF q=0')
    print('PASS: two-motor hand centres are forward of the wrist housings, matching the corrected direction')
    print('Hardware calibration and collision-free motion are NOT established by these checks.')


if __name__ == '__main__':
    main()
