import unittest
from xrobocon.pid_drive import PID

class PIDTests(unittest.TestCase):
    def test_saturation_and_recovery(self):
        p = PID(1, 1, 0, 2)
        for _ in range(100): self.assertEqual(p.update(10, 0, 0.1), 2)
        self.assertEqual(p.integral, 0)
        self.assertEqual(p.update(0, 0, 0.1), 0)
        self.assertLess(p.update(-1, 0, 0.1), 0)
    def test_derivative_no_setpoint_kick(self):
        p = PID(0, 0, 1, 10)
        self.assertEqual(p.update(0, 0, 0.1), 0)
        self.assertEqual(p.update(10, 0, 0.1), 0)
        self.assertEqual(p.update(10, 0.1, 0.1), -1)
    def test_validation(self):
        with self.assertRaises(ValueError): PID(-1, 0, 0, 1)
        with self.assertRaises(ValueError): PID(1, 0, 0, 1).update(0, 0, 0)

    def test_feedforward_shares_limit_and_prevents_windup(self):
        p=PID(1,1,0,2)
        for _ in range(100):self.assertEqual(p.update(1,0,.1,feedforward=2),2)
        self.assertEqual(p.integral,0)
        self.assertEqual(p.update(0,0,.1),0)
        with self.assertRaises(ValueError):p.update(0,0,.1,feedforward=float('nan'))

    def test_feedback_limit_keeps_gravity_compensation(self):
        p=PID(1,1,0,6,feedback_limit=1)
        for _ in range(100):self.assertEqual(p.update(10,0,.1,feedforward=2),3)
        self.assertEqual(p.integral,0)
        with self.assertRaises(ValueError):PID(1,1,0,6,feedback_limit=7)
