import unittest
from xrobocon.session import CompetitionSession
from xrobocon.runner import CompetitionRunner
from xrobocon.competition import CompetitionManager
from xrobocon.agent import FixedCoinAgent
from xrobocon.navigation import Pose2D

class Sim:
    def reset(self):self.steps=0
    def read_pose(self,p):return Pose2D(0,0,0)
    def truth_positions(self):return {'self':[0,0,0]}
    def speeds(self):return {'self':0}
    def step(self,commands,dt):self.steps+=1
    def stop(self):pass

class SessionTests(unittest.TestCase):
    def session(self):
        m=CompetitionManager([dict(id=0,pos=[0,0,0],tier=3)],time_limit=.1)
        return CompetitionSession(CompetitionRunner(Sim(),m,{'self':FixedCoinAgent([0])},.01))
    def test_pause_freezes_physics_and_clock_resume_preserves_score(self):
        s=self.session();s.command('start');s.tick();s.command('pause');before=s.runner.manager.elapsed
        for _ in range(5):self.assertIsNone(s.tick())
        self.assertEqual(s.runner.simulator.steps,1);self.assertEqual(s.runner.manager.elapsed,before)
        s.command('resume');s.tick();self.assertEqual(s.runner.manager.scores['self'],1)
    def test_done_agents_do_not_finish_competition_reset_clears_state(self):
        s=self.session();s.command('start')
        for _ in range(5):s.tick()
        self.assertEqual(s.status,'running')
        s.command('reset');self.assertEqual(s.status,'ready');self.assertEqual(s.runner.manager.scores['self'],0)
        s.command('start')
        for _ in range(11):s.tick()
        self.assertEqual(s.status,'finished');self.assertEqual(s.runner.manager.elapsed,.1)
    def test_error_round_can_be_reset(self):
        s=self.session();s.command('start');s.fail();self.assertEqual(s.status,'error')
        self.assertEqual(s.runner.manager.events[-1]['reason'],'error')
        s.command('reset');self.assertEqual(s.status,'ready')

    def test_invalid_resume_and_stop(self):
        s=self.session()
        with self.assertRaises(ValueError):s.command('resume')
        s.command('start');s.command('stop');self.assertEqual(s.status,'finished')
        self.assertEqual(s.runner.manager.events[-1]['reason'],'operator_stop')

if __name__=='__main__':unittest.main()
