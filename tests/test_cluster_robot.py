import unittest
from dataclasses import asdict
from types import SimpleNamespace
from xrobocon.navigation import VelocityCommand,Pose2D
from xrobocon.cluster_agent import ClusterMobilityAgent
class StubAgent:
    def __init__(self):self.stage=0;self.done=False;self.phase='climb'
    def reset(self):self.stage=0
    def act(self,obs,dt):return VelocityCommand(.1,0,climb_stage=self.stage)
class ClusterAgentTests(unittest.TestCase):
    def observation(self,distance=.34,speed=0,frames=(0,0)):
        return SimpleNamespace(pose=Pose2D(-4.65-distance,0,0),telemetry=dict(speed=speed,frames=list(frames)),competition=SimpleNamespace(status='running'))
    def test_carrier_activates_only_near_step_after_stall(self):
        agent=ClusterMobilityAgent(StubAgent())
        far=agent.act(self.observation(distance=1),.1);self.assertEqual(far.frame_target,0)
        for _ in range(6):command=agent.act(self.observation(),.1)
        self.assertGreater(command.frame_target,0)
        self.assertLessEqual(command.linear,.05)
    def test_reset_and_missing_feedback(self):
        agent=ClusterMobilityAgent(StubAgent())
        for _ in range(8):agent.act(self.observation(),.1)
        agent.reset();self.assertEqual(agent.frame_target,0)
        with self.assertRaises(ValueError):agent.act(self.observation(frames=()),.1)
    def test_frame_command_json_validation(self):
        from xrobocon.ros2_wire import decode_command
        valid=asdict(VelocityCommand(.1,0,frame_target=2.))
        self.assertEqual(decode_command(valid).frame_target,2)
        for value in (True,float('nan'),'two'):
            with self.assertRaises(ValueError):decode_command(dict(valid,frame_target=value))
    def test_fixed_axle_robot_needs_no_steering_joints_to_align(self):
        from xrobocon.tier_agent import FixedTierAgent
        from math import pi
        coins=tuple(dict(id=i,pos=[r,0,h],tier=3-i,status='available') for i,(r,h) in enumerate([(4,.105),(2.7,.355),(1.4,.605)]))
        observation=SimpleNamespace(pose=Pose2D(4,0,pi),timestamp=0,competition=SimpleNamespace(status='running',coins=coins),telemetry=dict(speed=0,steering=[]))
        agent=FixedTierAgent([[0],[1],[2]]);agent.phase='straighten'
        for _ in range(100):agent.act(observation,.01)
        self.assertEqual(agent.phase,'climb')
    def test_structure_is_active_carrier_and_passive_casters(self):
        import mujoco
        from pathlib import Path
        root=Path(__file__).resolve().parents[1]
        model=mujoco.MjModel.from_xml_path(str(root/'xrobocon/assets/robot_tristar_carrier.xml'))
        self.assertGreaterEqual(mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,'left_tristar_joint'),0)
        self.assertGreaterEqual(mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,'tail_swivel_0'),0)
        self.assertEqual(mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,'left_bogie_joint'),-1)
        self.assertAlmostEqual(float(model.body_mass.sum()),8.)
    def test_catalog_compiles_mobility_independently_per_robot(self):
        from pathlib import Path
        from xrobocon.match_setup import MatchCatalog,build_match_config
        from xrobocon.agent_factory import make_agent
        root=Path(__file__).resolve().parents[1];catalog=MatchCatalog.load(root/'configs/match_catalog.json')
        choices=catalog.defaults();choices['self']['robot']='tristar_carrier';choices['self']['control']='baseline';config=build_match_config(catalog,choices)
        self.assertIsInstance(make_agent(config['agents']['self']),ClusterMobilityAgent)
        self.assertNotIn('locomotion',config['agents']['opponent'])
        self.assertEqual(config['simulation']['robot_models']['self']['drive_parameters']['frame_torque_limit'],15)
        self.assertEqual(config['simulation']['torque_limit'],6)
        self.assertEqual(config['transport']['kind'],'ros2')
    def test_stop_freezes_target_and_does_not_enter_passive_assist(self):
        agent=ClusterMobilityAgent(StubAgent());obs=self.observation()
        for _ in range(10):command=agent.act(obs,.1)
        target=agent.frame_target;obs.competition.status='paused'
        command=agent.act(obs,.1)
        self.assertEqual(command.frame_target,target);self.assertEqual(command.linear,0);self.assertIsNone(command.climb_stage)
    def test_collision_yield_freezes_carrier_and_completion_waits_for_it(self):
        base=StubAgent();agent=ClusterMobilityAgent(base)
        for _ in range(8):agent.act(self.observation(),.1)
        target=agent.frame_target;base.avoidance=SimpleNamespace(waiting=True)
        command=agent.act(self.observation(),.1)
        self.assertEqual(command.linear,0);self.assertEqual(agent.frame_target,target)
        base.done=True;self.assertFalse(agent.done)
    def test_stall_uses_actual_progress_not_vibrating_speed(self):
        agent=ClusterMobilityAgent(StubAgent())
        for _ in range(10):command=agent.act(self.observation(speed=.08),.1)
        self.assertGreater(command.frame_target,0)
    def test_moving_robot_does_not_trigger_carrier_on_low_speed_reading(self):
        agent=ClusterMobilityAgent(StubAgent())
        for i in range(15):command=agent.act(self.observation(distance=.34-.02*i,speed=0),.1)
        self.assertEqual(command.frame_target,0)
    def test_standard_joint_state_is_used_when_available(self):
        agent=ClusterMobilityAgent(StubAgent());obs=self.observation(frames=())
        obs.sensors={'joint_states':SimpleNamespace(data=dict(name=['left_tristar_joint','right_tristar_joint'],position=[0,0]))}
        self.assertEqual(agent.act(obs,.1).frame_target,0)
        obs.sensors['joint_states'].data['position']=[0]
        with self.assertRaises(ValueError):agent.act(obs,.1)
    def test_known_map_comes_from_selected_field(self):
        import json,tempfile
        from pathlib import Path
        from xrobocon.scenario import load_scenario
        from xrobocon.robot_tuning import resolve_locomotion
        root=Path(__file__).resolve().parents[1]
        field=load_scenario(root/'configs/scenarios/passive_front_bogie_r18_steering_tiers.json')
        for t in field['field']['tiers']:t['radius']+=.1
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'field.json';path.write_text(json.dumps(field))
            options=resolve_locomotion(dict(locomotion=dict(kind='tristar')),path)
        self.assertEqual(options['tiers'],[4.75,3.35,1.9500000000000002])
        self.assertEqual(options['heights'],[.1,.35,.6])
