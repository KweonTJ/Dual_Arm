#!/usr/bin/env python3
"""Tessellate user-supplied STEP parts and preserve closed assembly transforms.

Requires CadQuery 2.7 only when rebuilding CAD meshes. Runtime needs no CAD SDK.
Original and modified CAD sources/notices are retained under cad/centered_gripper.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import shutil
import sys

import cadquery as cq
import numpy as np
import yaml

from generate_meshes import save
from gripper_kinematics import START, LINK_LENGTH

PACKAGE = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('source',type=Path)
    args=parser.parse_args()
    source=args.source.resolve(); retained=PACKAGE.parents[1]/'cad/centered_gripper'
    retained.mkdir(parents=True,exist_ok=True)
    for directory in ('Parts','Reference','Source','InventorData'):
        shutil.copytree(source/directory,retained/directory,dirs_exist_ok=True)
    for name in ('00_START_KO.txt','02_검토결과.txt'):
        shutil.copy2(source/name,retained/name)
    rows=list(csv.DictReader((source/'InventorData/Placements.csv').read_text(encoding='utf-8-sig').splitlines()))
    rows=[r for r in rows if r['Assembly']=='Hand_Closed']
    entries={}; solids={}
    for row in rows:
        name=row['Occurrence']; part=row['ID']
        if part not in solids:
            path=source/('Reference' if part.startswith('REF_') else 'Parts')/(part+'.step')
            solids[part]=cq.importers.importStep(str(path)).val()
        shape=solids[part]
        points,indices=shape.tessellate(.15,.25)
        triangles=np.asarray([p.toTuple() for p in points])[indices]*.001
        # Join seam coordinates at sub-micron precision for STL edge checks.
        triangles=np.round(triangles,9)
        matrix=np.eye(4)
        matrix[:3]=[[float(row[f'm{i}{j}']) for j in range(1,5)] for i in range(1,4)]
        matrix[:3,3]*=.001
        triangles=triangles@matrix[:3,:3].T+matrix[:3,3]
        # Uniform density estimates; AX case mass uses ROBOTIS specified 54.6g.
        density=1240 if part.startswith(('C02','C03','C05','C06','F01','F02')) else 7800
        mass=.0546 if part=='REF_AX12A_Case' else shape.Volume()*1e-9*density
        entries[name]={'mesh':triangles,'mass':mass,'part':part}
    groups={
        'cg_fixed': ['Wrist_shoe','Wrist_cradle','Fixed_adapter','Rail_connector','Rail'],
        'ax12a_cg_case': ['Gripper_AX12A_case'],
        'cg_rotor': ['Rotating_hub','Rotor'],
        'ax12a_cg_horn': ['Gripper_B01'],
        'cg_rod_a': ['Link_A','Bearing_rotor_A','Bearing_slide_A'],
        'cg_rod_b': ['Link_B','Bearing_rotor_B','Bearing_slide_B'],
        'cg_jaw_a': ['Plate_A','Block_A','Jaw_A'],
        'cg_jaw_b': ['Plate_B','Block_B','Jaw_B']}
    properties={}
    from gripper_kinematics import CLOSED, RADIUS, linkage_angle
    for name,names in groups.items():
        # Each group is exported in its URDF link frame at q=0.
        origin=np.eye(4)
        if name in ('cg_rotor','ax12a_cg_horn') or name.startswith('cg_rod'):
            a=START;origin[:2,:2]=[[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]]
            origin[2,3]=-.058
        if name.startswith('cg_rod'):
            sign=1 if name.endswith('a') else -1
            a=linkage_angle(START,sign)
            origin[:2,:2]=[[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]]
            origin[:3,3]=[sign*RADIUS*np.cos(START),sign*RADIUS*np.sin(START),-.0765]
        if name.startswith('cg_jaw'):
            sign=1 if name.endswith('a') else -1
            origin[:3,3]=[0,sign*CLOSED,-.0865]
        inv=np.linalg.inv(origin)
        pieces=[e['mesh']@inv[:3,:3].T+inv[:3,3] for e in (entries[n] for n in names)]
        save(name,pieces,min_area=1e-18)
        points=np.concatenate(pieces).reshape(-1,3)
        properties[name]={'mass':sum(entries[n]['mass'] for n in names),
            'size':np.ptp(points,axis=0).tolist(),'center':((points.min(0)+points.max(0))/2).tolist(),
            'origin_xyz':origin[:3,3].tolist(),
            'origin_yaw':float(np.arctan2(origin[1,0],origin[0,0])), 'parts':names}
    output=PACKAGE/'config/centered_gripper.yaml'
    output.write_text(yaml.safe_dump({'groups':properties},sort_keys=False),encoding='utf-8')
    hashes={p.relative_to(retained).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
            for p in retained.rglob('*') if p.is_file()}
    (retained/'source_manifest.json').write_text(json.dumps(hashes,indent=2)+'\n')
    print('Hand mass estimate:',sum(p['mass'] for p in properties.values()),'kg')


if __name__=='__main__':main()
