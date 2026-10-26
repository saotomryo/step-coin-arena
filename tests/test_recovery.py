import unittest
from xrobocon.recovery import StuckMonitor
class RecoveryTests(unittest.TestCase):
    def test_twenty_seconds_and_movement_reset(self):
        m=StuckMonitor()
        for _ in range(19):self.assertIsNone(m.update('blue',[0,0,0],0,1,True))
        self.assertEqual(m.update('blue',[0,0,0],0,1,True),20)
        for i in range(30):self.assertIsNone(m.update('blue',[i*.1,0,0],0,1,True))
    def test_wait_exemption_and_successful_rotation(self):
        m=StuckMonitor(timeout=2)
        for i in range(20):self.assertIsNone(m.update('blue',[0,0,0],i*.2,.2,True,rotation_only=True))
        for _ in range(3):self.assertIsNone(m.update('blue',[0,0,0],0,1,False))
    def test_spinning_does_not_mask_blocked_translation(self):
        m=StuckMonitor(timeout=2)
        self.assertIsNone(m.update('blue',[0,0,0],0,1,True))
        self.assertEqual(m.update('blue',[0,0,0],2,1,True),2)
    def test_runner_only_resets_blocked_robot_and_preserves_scores(self):
        from xrobocon.runner import CompetitionRunner
        from xrobocon.competition import CompetitionManager
        from xrobocon.navigation import Pose2D,VelocityCommand
        class Sim:
            def reset(self):self.positions={'self':[0,0,0],'opponent':[2,0,0]};self.calls=[]
            def read_pose(self,p):return Pose2D(*self.positions[p][:2],0)
            def truth_positions(self):return {p:list(v) for p,v in self.positions.items()}
            def speeds(self):return {'self':0,'opponent':.1}
            def step(self,c,dt):self.positions['opponent'][0]+=.1
            def stop(self):pass
            def respawn(self,p):self.calls.append(p);self.positions[p]=[-1,0,0]
        class Agent:
            def reset(self):self.resets=getattr(self,'resets',0)+1
            def act(self,o,dt):return VelocityCommand(.1,0)
        manager=CompetitionManager([dict(id=0,pos=[0,0,0],tier=3)],participants=['self','opponent'],time_limit=10)
        sim=Sim();agents={p:Agent() for p in manager.participants};runner=CompetitionRunner(sim,manager,agents,1,{'enabled':True,'timeout':2})
        runner.start();runner.tick();row=runner.tick()
        self.assertEqual(sim.calls,['self']);self.assertAlmostEqual(sim.positions['opponent'][0],2.2)
        self.assertEqual(manager.scores['self'],1);self.assertEqual(manager.elapsed,2)
        self.assertEqual(manager.coins[0]['status'],'collected');self.assertEqual(agents['self'].resets,2);self.assertEqual(agents['opponent'].resets,1)
        self.assertEqual(row['recovery_counts'],{'self':1,'opponent':0});self.assertEqual(manager.recovery_blocked_seconds['self'],2)

    def test_intentional_waits_but_not_overturned_waits_are_exempt(self):
        from xrobocon.recovery import monitoring_state
        from xrobocon.navigation import VelocityCommand
        self.assertEqual(monitoring_state('align',VelocityCommand(0,.2),0),(False,True))
        self.assertEqual(monitoring_state('coins',VelocityCommand(0,0),0),(False,True))
        self.assertEqual(monitoring_state('straighten',VelocityCommand(0,0),180),(True,False))
        self.assertEqual(monitoring_state('coins',VelocityCommand(0,.2),0),(True,True))
        self.assertEqual(monitoring_state('climb',VelocityCommand(0,0),0),(True,True))
