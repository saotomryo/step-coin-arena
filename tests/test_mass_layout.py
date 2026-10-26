import unittest
from pathlib import Path
import numpy as np
import mujoco
from xrobocon.mass_layout import aggregate_boxes
ROOT=Path(__file__).resolve().parents[1]
class MassLayoutTests(unittest.TestCase):
    def test_parallel_axis(self):
        mass,com,tensor=aggregate_boxes([dict(mass=1,pos=[x,0,0],size=[1,1,1]) for x in (-1,1)])
        self.assertEqual(mass,2);np.testing.assert_allclose(com,0)
        np.testing.assert_allclose(tensor,np.diag([1/3,7/3,7/3]))
    def test_bad_budget(self):
        with self.assertRaises(ValueError):aggregate_boxes([dict(mass=-1,pos=[0,0,0],size=[1,1,1])])
    def test_variants_preserve_contacts_and_dynamics(self):
        baseline=mujoco.MjModel.from_xml_path(str(ROOT/'xrobocon/assets/robot_passive_front_bogie_r18_steering.xml'))
        simple=mujoco.MjModel.from_xml_path(str(ROOT/'xrobocon/assets/robot_component_r18.xml'))
        detail=mujoco.MjModel.from_xml_path(str(ROOT/'xrobocon/assets/visual/robot_component_r18_detailed.xml'))
        for m in (simple,detail):
            self.assertEqual(m.nbody,baseline.nbody);self.assertEqual(m.njnt,baseline.njnt)
            np.testing.assert_allclose(m.body_mass,baseline.body_mass)
            np.testing.assert_allclose(m.body_inertia[2:],baseline.body_inertia[2:])
            np.testing.assert_allclose(m.body_ipos[2:],baseline.body_ipos[2:])
            np.testing.assert_allclose(m.body_iquat[2:],baseline.body_iquat[2:])
            active=np.flatnonzero(m.geom_contype|m.geom_conaffinity)
            np.testing.assert_allclose(m.geom_size[active],baseline.geom_size)
            np.testing.assert_allclose(m.geom_pos[active],baseline.geom_pos)
        np.testing.assert_allclose(simple.body_ipos,detail.body_ipos)
        np.testing.assert_allclose(simple.body_inertia,detail.body_inertia)
        np.testing.assert_allclose(simple.body_iquat,detail.body_iquat)
        self.assertAlmostEqual(simple.body_ipos[1,2]-baseline.body_ipos[1,2],.039425)
        self.assertTrue((simple.body_inertia[1]>0).all())
    def test_catalog_compiles_independent_variants_with_ros2(self):
        from xrobocon.match_setup import MatchCatalog,build_match_config
        catalog=MatchCatalog.load(ROOT/'configs/match_catalog.json')
        choices=catalog.defaults();choices['self']['robot']='component_r18';choices['opponent']['robot']='component_r18_detailed'
        config=build_match_config(catalog,choices)
        self.assertTrue(config['simulation']['robot_models']['self']['robot_asset'].endswith('/robot_component_r18.xml'))
        self.assertTrue(config['simulation']['robot_models']['opponent']['robot_asset'].endswith('/robot_component_r18_detailed.xml'))
        original=__import__('json').loads((ROOT/'configs/competition_ros2.json').read_text())
        self.assertEqual(config['transport'],original['transport'])
        self.assertEqual(catalog.defaults()['self']['robot'],'front_bogie_r18')
