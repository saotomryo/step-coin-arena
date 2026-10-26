import unittest
from dataclasses import asdict
from xrobocon.agent import Observation,PublicCompetition
from xrobocon.navigation import Pose2D,VelocityCommand
from xrobocon.sensors import SensorSample
from xrobocon.ros2_wire import encode_observation,decode_observation,decode_command

class WireTests(unittest.TestCase):
    def test_measurements_not_duplicated_in_state_and_round_trip(self):
        sample=SensorSample(.1,'blue/imu_link',4,'genesis_sensor',{'linear_acceleration':[0,0,9.8],'pos_offset':[0,0,0]})
        obs=Observation(Pose2D(1,2,3),.1,'truth_baseline',PublicCompetition('running',10,{},()),sensors={'imu':sample})
        wire=encode_observation(obs)
        self.assertNotIn('linear_acceleration',str(wire));self.assertNotIn('data',wire['sensors']['imu'])
        decoded=decode_observation(wire,{'imu':{'linear_acceleration':[0,0,9.8]}})
        self.assertEqual(decoded.sensors['imu'],sample);self.assertEqual(decoded.source,'ros2/truth_baseline')
    def test_command_rejects_nonfinite_and_invalid_flags(self):
        valid=asdict(VelocityCommand(.1,.2,1));self.assertEqual(decode_command(valid),VelocityCommand(.1,.2,1))
        for patch in ({'linear':float('nan')},{'angular':True},{'climb_stage':4},{'reset_drive':1},{'climb_stage':True}):
            with self.assertRaises(ValueError):decode_command(valid|patch)

class RunnerTransportTests(unittest.TestCase):
    def setup_runtime(self,fail=False):
        from xrobocon.runner import CompetitionRunner
        from xrobocon.competition import CompetitionManager
        class Simulator:
            def reset(self):self.stopped=False
            def read_pose(self,p):return Pose2D(0,0,0)
            def truth_positions(self):return {'self':[0,0,0]}
            def step(self,commands,dt):self.commands=commands
            def speeds(self):return {'self':0}
            def stop(self):self.stopped=True
        class Agent:
            def reset(self):pass
            def act(self,*args):raise AssertionError('direct access forbidden')
            def update(self,metadata):self.phase=metadata['phase']
        class Transport:
            last_exchange={'kind':'test'}
            def reset(self,names):pass
            def exchange(self,observations,dt):
                if fail:raise TimeoutError('DDS unavailable')
                return {'self':VelocityCommand(.1,0)},{'self':{'phase':'remote'}}
        sim=Simulator();runner=CompetitionRunner(sim,CompetitionManager([]),{'self':Agent()},transport=Transport());runner.start();return runner,sim
    def test_transport_supplies_commands_without_local_agent_call(self):
        runner,sim=self.setup_runtime();row=runner.tick()
        self.assertEqual(sim.commands['self'].linear,.1);self.assertEqual(row['agent_states']['self']['phase'],'remote')
    def test_timeout_stops_physics_without_direct_fallback(self):
        runner,sim=self.setup_runtime(True)
        with self.assertRaises(TimeoutError):runner.tick()
        self.assertTrue(sim.stopped)

class StorageBoundaryTests(unittest.TestCase):
    def test_unmounted_storage_never_launches_or_installs(self):
        from unittest.mock import patch
        from xrobocon.ros2_transport import Ros2Transport
        with patch.dict('os.environ',{'XROBOCON_RESOURCE_ROOT':'/Volumes/TestMissing/resources'}),patch('xrobocon.ros2_transport.os.path.ismount',return_value=False),patch('xrobocon.ros2_transport.subprocess.Popen') as popen:
            with self.assertRaises(RuntimeError):Ros2Transport('unused.json','unused-output')
            popen.assert_not_called()
