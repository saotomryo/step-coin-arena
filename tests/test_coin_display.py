import unittest
from xrobocon.coin_display import CoinDisplay
from xrobocon.competition import CompetitionManager

class CoinDisplayTests(unittest.TestCase):
    def test_delayed_spawn_collection_respawn_and_reset(self):
        calls=[];display=CoinDisplay({0:(1,0,.005)},lambda key,pos:calls.append((key,pos)))
        manager=CompetitionManager([dict(id=0,pos=[1,0,.005],tier=3,spawn_at=.2,respawn_delay=.3,dwell=.1)],time_limit=2)
        manager.start();display.sync(manager.coins);self.assertFalse(display.visible[0])
        manager.advance(.2,{});display.sync(manager.coins);self.assertTrue(display.visible[0])
        manager.advance(.1,{'self':[1,0,.005]});display.sync(manager.coins);self.assertFalse(display.visible[0])
        manager.advance(.3,{});display.sync(manager.coins);self.assertTrue(display.visible[0])
        self.assertEqual(calls[-1],(0,(1,0,.005)))
        count=len(calls);display.sync(manager.coins);self.assertEqual(len(calls),count)
        manager.start();display.sync(manager.coins);self.assertFalse(display.visible[0])
    def test_mismatch_rejected(self):
        with self.assertRaises(ValueError):CoinDisplay({0:(0,0,0)},lambda *args:None).sync([])

    def test_physics_reset_requires_reprojection_even_if_visibility_unchanged(self):
        positions={0:(1,0,.005)}
        display=CoinDisplay(positions,lambda key,pos:positions.__setitem__(key,pos))
        coins=[dict(id=0,status='pending')]
        display.sync(coins);self.assertEqual(positions[0][2],-20)
        positions[0]=(1,0,.005) # external physics reset restores initial transforms
        display.invalidate()
        display.sync(coins)
        self.assertEqual(positions[0][2],-20)
