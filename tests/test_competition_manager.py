import unittest
from xrobocon.competition import CompetitionManager

class CompetitionTests(unittest.TestCase):
    def make(self,**coin):
        return CompetitionManager([dict(dict(id=0,pos=[0,0,.1],tier=3,points=2),**coin)],participants=['self','opponent'],time_limit=10)
    def test_scheduled_spawn_and_snapshot_isolation(self):
        m=self.make(spawn_at=2);m.start()
        self.assertEqual(m.snapshot()['coins'][0]['status'],'pending')
        m.advance(2,{})
        s=m.snapshot();self.assertEqual(s['coins'][0]['status'],'available')
        s['scores']['self']=99;self.assertEqual(m.snapshot()['scores']['self'],0)
    def test_single_owner_and_respawn(self):
        m=self.make(respawn_delay=2);m.start();m.advance(.1,{'self':[0,0,.1]})
        self.assertEqual(m.snapshot()['scores']['self'],2)
        m.advance(1,{'opponent':[0,0,.1]});self.assertEqual(m.snapshot()['scores']['opponent'],0)
        m.advance(1.1,{})
        self.assertEqual(m.snapshot()['coins'][0]['status'],'available')
        m.advance(.1,{'opponent':[0,0,.1]});self.assertEqual(m.snapshot()['scores']['opponent'],2)
    def test_contest_is_not_input_order_dependent(self):
        for order in [['self','opponent'],['opponent','self']]:
            m=self.make();m.start();m.advance(.1,{p:[0,0,.1] for p in order})
            self.assertEqual(m.snapshot()['scores'],{'self':0,'opponent':0})
            self.assertTrue(m.snapshot()['coins'][0]['contested'])
    def test_dwell_resets_on_missing_observation(self):
        m=self.make(dwell=2);m.start();m.advance(1,{'self':[0,0,.1]});m.advance(.1,{})
        m.advance(1,{'self':[0,0,.1]});self.assertEqual(m.snapshot()['scores']['self'],0)
        m.advance(1,{'self':[0,0,.1]});self.assertEqual(m.snapshot()['scores']['self'],2)
    def test_spawn_does_not_credit_time_before_appearance(self):
        m=self.make(spawn_at=2,dwell=1);m.start();m.advance(2.5,{'self':[0,0,.1]})
        self.assertEqual(m.snapshot()['scores']['self'],0)
        m.advance(.5,{'self':[0,0,.1]});self.assertEqual(m.snapshot()['scores']['self'],2)
    def test_end_reset_and_invalid_inputs(self):
        m=self.make();m.start();m.advance(10,{'self':[0,0,.1]})
        self.assertEqual(m.snapshot()['status'],'finished');self.assertEqual(m.snapshot()['scores']['self'],0)
        m.start();self.assertEqual(m.snapshot()['elapsed'],0)
        for dt in [0,-1,float('nan')]:
            with self.assertRaises(ValueError):m.advance(dt,{})
        with self.assertRaises(ValueError):m.advance(.1,{'unknown':[0,0,0]})
        with self.assertRaises(ValueError):m.advance(.1,{'self':[float('nan'),0,0]})
    def test_invalid_coin_definitions(self):
        for change in [dict(points=-1),dict(spawn_at=-1),dict(respawn_delay=0),dict(radius=0),dict(dwell=float('inf'))]:
            with self.assertRaises(ValueError):self.make(**change)
        with self.assertRaises(ValueError):CompetitionManager([dict(id=0,pos=[0,0,0],tier=1)]*2)
    def test_contest_clears_when_one_participant_leaves(self):
        m=self.make();m.start();m.advance(.1,{'self':[0,0,.1],'opponent':[0,0,.1]})
        m.advance(.1,{'self':[0,0,.1]})
        self.assertEqual(m.snapshot()['collection_counts']['self'],1)
        self.assertEqual(m.snapshot()['scores']['self'],2)
