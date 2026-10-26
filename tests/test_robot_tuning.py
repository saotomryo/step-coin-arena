import json,tempfile,unittest
from pathlib import Path
from xrobocon.robot_tuning import load_tuning,select_candidate
ROOT=Path(__file__).resolve().parents[1]
class TuningTests(unittest.TestCase):
    def test_profile_validation_and_copy(self):
        tuning=load_tuning(ROOT/'configs/robot_tuning/front_bogie_r20.json')
        self.assertEqual(tuning['drive']['kp'],.3)
        self.assertEqual(tuning['motion']['safe']['climb_speed'],.1)
    def test_unknown_and_invalid_gain_rejected(self):
        for drive in ({'unknown':1},{'ki':-1},{'kp':True},{'steering_torque_limit':float('nan')}):
            with tempfile.TemporaryDirectory() as d:
                p=Path(d)/'bad.json';p.write_text(json.dumps(dict(schema_version=1,drive=drive,motion={})))
                with self.assertRaises(ValueError):load_tuning(p)
    def test_failed_or_partial_suites_cannot_win(self):
        failed=dict(candidate='fast',complete=True,passed=False,simulation_seconds=1)
        partial=dict(candidate='partial',complete=False,passed=True,simulation_seconds=1)
        success=dict(candidate='safe',complete=True,passed=True,simulation_seconds=10)
        self.assertEqual(select_candidate([failed,partial,success]),'safe')
        self.assertIsNone(select_candidate([failed,partial]))
    def test_registered_new_robot_has_distinct_geometry(self):
        import mujoco
        old=mujoco.MjModel.from_xml_path(str(ROOT/'xrobocon/assets/robot_passive_front_bogie_r18_steering.xml'))
        new=mujoco.MjModel.from_xml_path(str(ROOT/'xrobocon/assets/robot_front_bogie_r20_wide.xml'))
        self.assertEqual(old.njnt,new.njnt);self.assertAlmostEqual(sum(old.body_mass),sum(new.body_mass))
        wheel=mujoco.mj_name2id(new,mujoco.mjtObj.mjOBJ_BODY,'left_wheel_front')
        self.assertAlmostEqual(new.geom_size[new.body_geomadr[wheel],0],.2)
        rocker=mujoco.mj_name2id(new,mujoco.mjtObj.mjOBJ_BODY,'left_rocker')
        self.assertAlmostEqual(new.body_pos[rocker,1],.135)
    def test_tuning_applies_to_each_robot_and_ros2_config(self):
        from xrobocon.match_setup import MatchCatalog,build_match_config
        catalog=MatchCatalog.load(ROOT/'configs/match_catalog.json');choices=catalog.defaults();choices['self']['robot']='front_bogie_r20_wide';choices['self']['control']='safe'
        cfg=build_match_config(catalog,choices)
        self.assertIn('r20_wide.xml',cfg['simulation']['robot_models']['self']['robot_asset'])
        self.assertIn('r18_steering.xml',cfg['simulation']['robot_models']['opponent']['robot_asset'])
        self.assertEqual(cfg['agents']['self']['motion_limits']['climb_speed'],.1)
        self.assertEqual(cfg['simulation']['robot_models']['self']['drive_parameters']['steering_kp'],12.)
        self.assertEqual(cfg['transport']['kind'],'ros2')
    def test_diagnostic_fixture_does_not_modify_competition(self):
        from xrobocon.match_setup import MatchCatalog,build_match_config
        from xrobocon.tuning_trials import diagnostic_step_config
        catalog=MatchCatalog.load(ROOT/'configs/match_catalog.json');cfg=build_match_config(catalog,catalog.defaults())
        fixture=diagnostic_step_config(cfg)
        self.assertEqual(fixture['competition']['participants'],['self']);self.assertNotIn('transport',fixture)
        self.assertEqual(cfg['transport']['kind'],'ros2');self.assertEqual(len(cfg['competition']['coins']),18)
        self.assertEqual([c['pos'][2] for c in fixture['competition']['coins']],[.10500000000000001,.355,.605])
    def test_robot_switch_uses_recommendation_but_allows_manual_selection(self):
        from xrobocon.match_setup import MatchCatalog,cycle_choice
        catalog=MatchCatalog.load(ROOT/'configs/match_catalog.json');choices=catalog.defaults()
        order=list(catalog.options('robots'));delta=order.index('front_bogie_r20_wide')-order.index(choices['self']['robot'])
        cycle_choice(catalog,choices,'self','robot',delta)
        self.assertEqual(choices['self']['robot'],'front_bogie_r20_wide')
        self.assertEqual(choices['self']['control'],'baseline')
        cycle_choice(catalog,choices,'self','control',1)
        self.assertEqual(choices['self']['control'],'safe')
        self.assertEqual(choices['opponent'],catalog.defaults()['opponent'])
    def test_changed_model_or_recipe_cannot_be_compared_or_reused(self):
        from xrobocon.robot_tuning import verify_comparison,verify_asset_binding,tuning_signature
        expected=dict(asset=dict(sha256='a'),tuning_signature='one')
        for plan in (dict(asset=dict(sha256='b'),tuning_signature='one'),dict(asset=dict(sha256='a'),tuning_signature='two')):
            with self.assertRaises(ValueError):verify_comparison(plan,expected)
        with self.assertRaises(ValueError):verify_asset_binding(dict(calibration=dict(asset=dict(sha256='wrong'))),ROOT/'xrobocon/assets/robot_front_bogie_r20_wide.xml')
        self.assertEqual(tuning_signature(dict(a=1,b=2)),tuning_signature(dict(b=2,a=1)))
