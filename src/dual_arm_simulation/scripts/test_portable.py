"""Test the ROS-free launch path after relocating the model and changing limits."""
import math
import os
from pathlib import Path
import shutil
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

if sys.platform.startswith('linux'):
    os.environ.setdefault('MUJOCO_GL', 'egl')
import yaml
from launch_desktop import ROOT, prepare_urdf
from simulation_model import Simulation, build_scene


class PortableTests(unittest.TestCase):
    def test_relocated_model_reloads_yaml_without_ros(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp) / '윈도우 실행 폴더'
            description = root / 'description'
            shutil.copytree(ROOT / 'src/dual_arm_description', description,
                            ignore=shutil.ignore_patterns('__pycache__'))
            limits_path = description / 'config/joint_limits.yaml'
            limits = yaml.safe_load(limits_path.read_text(encoding='utf-8'))
            limits['joint_limits']['right_shoulder_pitch_joint']['max_position_deg'] = 60
            limits_path.write_text(yaml.safe_dump(limits), encoding='utf-8')
            urdf = prepare_urdf(description, root / 'output')
            parsed = ET.parse(urdf)
            upper = parsed.find("joint[@name='right_shoulder_pitch_joint']/limit").get('upper')
            self.assertAlmostEqual(float(upper), math.pi / 3)
            scene, manifest = build_scene(urdf, description,
                ROOT / 'src/dual_arm_simulation/config/simulation.yaml', root / 'output')
            sim = Simulation(scene, manifest)
            self.assertEqual(sim.motor_ids, [10, 9, 8, 7, 6, 5, 4, 3, 2, 1])
            self.assertAlmostEqual(sim.upper[5], math.pi / 3)
            self.assertAlmostEqual(sum(sim.model.body_mass) - .025, 1.780994524904885,places=5)
            self.assertIsNotNone(parsed.find("link[@name='torso_link']"))
            self.assertIsNone(ET.parse(scene).find(".//geom[@name='fixture']"))
            self.assertIsNone(ET.parse(scene).find(".//geom[@name='fixture_base']"))
            sim.step(100)
            self.assertFalse(sim.fault)


if __name__ == '__main__':
    unittest.main()
