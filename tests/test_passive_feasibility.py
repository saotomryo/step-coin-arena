import unittest
import numpy as np
from xrobocon.passive_feasibility import Geometry, surface_contact, force_feasible

class PassiveFeasibilityTests(unittest.TestCase):
    def test_vertical_face_and_corner_normals(self):
        distance,point,normal=surface_contact(np.array([-.18,.18]),.25)
        self.assertAlmostEqual(distance,.18)
        np.testing.assert_allclose(point,[0,.18])
        np.testing.assert_allclose(normal,[-1,0])
        distance,point,normal=surface_contact(np.array([-.12,.41]),.25)
        self.assertAlmostEqual(distance,.2)
        np.testing.assert_allclose(normal,[-.6,.8])

    def test_flat_weight_balance_and_unstable_com(self):
        state=np.array([-.6,.28,0,0])
        success,forces=force_feasible(state,Geometry(friction=0),.25)
        self.assertTrue(success)
        self.assertAlmostEqual(sum(forces[1::2]),8.6*9.8)
        self.assertTrue(all(f>=-1e-8 for f in forces[1::2]))
        success,_=force_feasible(state,Geometry(com_x=2,friction=0),.25)
        self.assertFalse(success)
