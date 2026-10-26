import unittest
from xrobocon.agent import FixedCoinAgent,Observation,PublicCompetition
from xrobocon.navigation import Pose2D

class AgentTests(unittest.TestCase):
    def test_wait_for_spawn_then_wait_for_authoritative_collection(self):
        a=FixedCoinAgent([0]);a.reset()
        def obs(status):return Observation(Pose2D(0,0,0),0,'truth_baseline',PublicCompetition('running',10,{'self':0},({ 'id':0,'pos':(1,0,0),'status':status,'owner':None},)))
        self.assertEqual(a.act(obs('pending'),.01).linear,0)
        self.assertGreater(a.act(obs('available'),.01).linear,0)
        self.assertEqual(a.act(obs('collected'),.01).linear,0)
        self.assertTrue(a.done)

    def test_runner_orders_physics_before_award_and_isolates_public_state(self):
        from xrobocon.runner import CompetitionRunner
        from xrobocon.competition import CompetitionManager
        from xrobocon.navigation import VelocityCommand
        calls=[]
        class Sim:
            def reset(self):calls.append('reset')
            def read_pose(self,p):return Pose2D(0,0,0)
            def step(self,commands,dt):calls.append('physics')
            def truth_positions(self):return {'self':[1.,0.,0.]}
            def speeds(self):return {'self':0.}
            def stop(self):calls.append('stop')
        class Agent:
            def reset(self):pass
            def act(self,obs,dt):
                calls.append('agent')
                self.assert_private='dwell_by' not in obs.competition.coins[0]
                obs.competition.scores['self']=999
                obs.competition.coins[0]['pos']=[99,99,99]
                return VelocityCommand(0,0)
        manager=CompetitionManager([dict(id=0,pos=[1,0,0],tier=3)],time_limit=1)
        agent=Agent();runner=CompetitionRunner(Sim(),manager,{'self':agent})
        runner.start();runner.tick()
        self.assertEqual(calls,['reset','agent','physics'])
        self.assertTrue(agent.assert_private);self.assertEqual(manager.scores['self'],1)
        self.assertEqual(manager.coins[0]['pos'],[1,0,0])
    def test_runner_rejects_mismatched_participants(self):
        from xrobocon.runner import CompetitionRunner
        from xrobocon.competition import CompetitionManager
        with self.assertRaises(ValueError):CompetitionRunner(None,CompetitionManager([]),{})

    def test_two_agents_share_one_physics_step_and_keep_scores_separate(self):
        from xrobocon.runner import CompetitionRunner
        from xrobocon.competition import CompetitionManager
        class Sim:
            def reset(self):self.steps=0
            def read_pose(self,p):return Pose2D(0 if p=='self' else 2,0,0)
            def step(self,commands,dt):
                self.commands=commands;self.steps+=1
            def truth_positions(self):return {'self':[0.,0.,0.],'opponent':[2.,0.,0.]}
            def speeds(self):return {'self':0.,'opponent':0.}
            def stop(self):pass
        m=CompetitionManager([dict(id=0,pos=[0,0,0],tier=3),dict(id=1,pos=[2,0,0],tier=3,points=2)],participants=['self','opponent'])
        sim=Sim();runner=CompetitionRunner(sim,m,{'self':FixedCoinAgent([0]),'opponent':FixedCoinAgent([1])})
        runner.start();row=runner.tick()
        self.assertEqual(sim.steps,1);self.assertEqual(set(sim.commands),{'self','opponent'})
        self.assertEqual(row['scores'],{'self':1,'opponent':2})
