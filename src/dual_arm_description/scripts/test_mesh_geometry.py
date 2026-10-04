#!/usr/bin/env python3
"""Regression checks for missing exterior faces and accidentally filled bores."""
import unittest
import numpy as np
from generate_meshes import box, circle, plate
from mesh_geometry import orient_solid, signed_volume, surface_report


class SolidSurfaceTests(unittest.TestCase):
    def test_box_has_an_outward_face_in_all_six_directions(self):
        center = np.array([.020, -.010, -.050])
        mesh = orient_solid(box((.040, .032, .050), center))
        normal = np.cross(mesh[:, 1]-mesh[:, 0], mesh[:, 2]-mesh[:, 0])
        self.assertTrue(np.all(np.einsum('ij,ij->i', normal, mesh.mean(axis=1)-center) > 0))
        for axis in np.vstack([np.eye(3), -np.eye(3)]):
            visible = normal @ axis > 1e-12
            self.assertEqual(np.count_nonzero(visible), 2)
        # VTK stores these input coordinates as float32.
        np.testing.assert_allclose(signed_volume(mesh), .040*.032*.050, rtol=1e-6)

    def test_mirrored_mounting_plate_keeps_its_bore(self):
        outer = circle(0, 0, .010, 24)
        bore = circle(0, 0, .004, 24)
        for outline in [outer, outer[::-1]]:
            mesh = orient_solid(plate(outline, .003, .0225, [bore]))
            polygon_area = 24/2*np.sin(2*np.pi/24)*(.010**2-.004**2)
            np.testing.assert_allclose(signed_volume(mesh), polygon_area*.003, rtol=1e-6)
            # The bore wall points into the empty passage, towards the axis.
            radii = np.linalg.norm(mesh[:, :, 1:], axis=2)
            inner = mesh[np.all(np.isclose(radii, .004, atol=1e-8), axis=1)]
            self.assertEqual(len(inner), 48)
            normal = np.cross(inner[:, 1]-inner[:, 0], inner[:, 2]-inner[:, 0])
            self.assertTrue(np.all(np.einsum('ij,ij->i', normal[:, 1:], inner.mean(axis=1)[:, 1:]) < 0))

    def test_disconnected_solids_are_each_outward(self):
        a = box((.010, .010, .010))
        b = box((.004, .004, .004), (.050, 0, 0))[:, ::-1]
        mesh = orient_solid(np.concatenate([a, b]))
        np.testing.assert_allclose(signed_volume(mesh[:len(a)]), .010**3, rtol=1e-6)
        np.testing.assert_allclose(signed_volume(mesh[len(a):]), .004**3, rtol=1e-6)
        self.assertEqual(surface_report(mesh)['unbalanced_edges'], 0)

    def test_missing_face_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'Open/non-manifold'):
            orient_solid(box((.010, .010, .010))[1:])


if __name__ == '__main__':
    unittest.main()
