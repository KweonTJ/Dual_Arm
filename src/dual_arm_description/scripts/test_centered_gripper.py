"""Compare URDF transforms and pad spacing to the supplied CAD assembly."""
import csv
import unittest
import numpy as np
from export_preview import load_model,fk,PACKAGE,stl
from gripper_kinematics import MAX_ANGLE,expand_positions


class CenteredGripperTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model=load_model(PACKAGE/'urdf/dual_arm.urdf')
        cls.rows=list(csv.DictReader((PACKAGE.parents[1]/'cad/centered_gripper/InventorData/Placements.csv').read_text(encoding='utf-8-sig').splitlines()))

    def test_closed_and_open_match_CAD_assembly(self):
        for assembly,q in [('Hand_Closed',0),('Hand_Open70',MAX_ANGLE)]:
            frames=fk(self.model,{'left_gripper_joint':q,'right_gripper_joint':q})
            for side in ('left','right'):
                hand=np.linalg.inv(frames[side+'_palm_link'])
                for row in self.rows:
                    if row['Assembly']!=assembly:continue
                    name=row['Occurrence']
                    if name.startswith('Jaw_'):suffix='jaw_'+name[-1].lower()
                    elif name.startswith('Link_'):suffix='rod_'+name[-1].lower()
                    else:continue
                    expected=np.eye(4)
                    expected[:3]=[[float(row[f'm{i}{j}']) for j in range(1,5)] for i in range(1,4)]
                    expected[:3,3]*=.001
                    np.testing.assert_allclose(hand@frames[f'{side}_{suffix}_link'],expected,atol=1e-8)

    def test_fingertips_centered_and_parallel_throughout_stroke(self):
        for q in np.linspace(0,MAX_ANGLE,9):
            frames=fk(self.model,{'left_gripper_joint':q,'right_gripper_joint':q})
            expected=expand_positions({'left_gripper_joint':q})['left_jaw_a_joint']*2
            for side in ('left','right'):
                hand=np.linalg.inv(frames[side+'_palm_link']);pads=[]
                for tag in ('a','b'):
                    matrix=hand@frames[f'{side}_jaw_{tag}_link']
                    np.testing.assert_allclose(matrix[:3,:3],np.eye(3),atol=1e-12)
                    points=stl(PACKAGE/f'meshes/cg_jaw_{tag}.stl').reshape(-1,3)
                    points=points@matrix[:3,:3].T+matrix[:3,3]
                    tip=points[points[:,2]<-.148] # Include the flat inner pad above the rounded tip.
                    pads.append(tip[:,1].min() if tag=='a' else tip[:,1].max())
                    rod=hand@frames[f'{side}_rod_{tag}_link']
                    np.testing.assert_allclose((rod@np.array([.035,0,0,1]))[:3],
                        (matrix@np.array([.0186 if tag=='a' else -.0186,0,.010,1]))[:3],atol=1e-8)
                self.assertAlmostEqual(sum(pads)/2,0,delta=2e-5)
                self.assertAlmostEqual(pads[0]-pads[1],expected,delta=2e-5)


if __name__=='__main__':unittest.main()
