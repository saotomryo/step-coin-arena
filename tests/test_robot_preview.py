import unittest
from pathlib import Path
from xrobocon.robot_preview import load_preview_model
ROOT=Path(__file__).resolve().parents[1]
class PreviewTests(unittest.TestCase):
    def test_actual_mjcf_and_no_time_advance(self):
        preview=load_preview_model(ROOT/'xrobocon/assets/robot_passive_front_bogie_r18_steering.xml')
        self.assertEqual(preview.data.time,0)
        self.assertEqual(preview.wheel_count,6)
        self.assertAlmostEqual(preview.wheel_diameter,.36)
        self.assertGreater(preview.mass,8)
        self.assertTrue(all(0<v<2 for v in preview.dimensions))
    def test_invalid_asset_rejected(self):
        with self.assertRaises((ValueError,FileNotFoundError)):load_preview_model(ROOT/'configs/match_catalog.json')
    def test_detailed_model_preserves_physical_properties(self):
        import mujoco,numpy as np,json
        physical=ROOT/'xrobocon/assets/robot_passive_front_bogie_r18_steering.xml'
        visual=ROOT/'xrobocon/assets/visual/robot_front_bogie_r18_detailed.xml'
        original=mujoco.MjModel.from_xml_path(str(physical));detail=mujoco.MjModel.from_xml_path(str(visual))
        np.testing.assert_allclose(detail.body_mass,original.body_mass)
        np.testing.assert_allclose(detail.body_inertia,original.body_inertia)
        self.assertEqual(detail.njnt,original.njnt)
        extras=[i for i in range(detail.ngeom) if detail.geom_group[i]==2]
        self.assertGreater(len(extras),500)
        self.assertTrue(all(detail.geom_contype[i]==0 and detail.geom_conaffinity[i]==0 for i in extras))
        preview=load_preview_model(visual);self.assertEqual(preview.wheel_count,6);self.assertAlmostEqual(preview.wheel_diameter,.36)
        catalog=json.loads((ROOT/'configs/match_catalog.json').read_text())
        self.assertEqual((ROOT/'configs'/catalog['robots']['front_bogie_r18']['robot_asset']).resolve(),physical)
    def test_visual_generator_source_hash_is_current(self):
        import hashlib,json
        manifest=json.loads((ROOT/'xrobocon/assets/visual/robot_front_bogie_r18_detailed.manifest.json').read_text())
        self.assertEqual(hashlib.sha256((ROOT/manifest['source']).read_bytes()).hexdigest(),manifest['source_sha256'])
