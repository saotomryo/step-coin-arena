import unittest
from dataclasses import replace
from xrobocon.step_controller import StepObservation,GuardedStepController

class StepTests(unittest.TestCase):
    def test_contact_requires_nearby_persistent_stall(self):
        c=GuardedStepController();o=StepObservation(.15,0,0,0,0,0,False)
        for _ in range(49):c.update(o,.01)
        self.assertFalse(c.contact_inferred)
        c.update(o,.01)
        self.assertTrue(c.contact_inferred)
        self.assertEqual(c.phase,'climb')
        c=GuardedStepController()
        for _ in range(100):c.update(replace(o,edge_distance=.4),.01)
        self.assertFalse(c.contact_inferred)
    def test_tracking_and_posture_guard(self):
        c=GuardedStepController();c.phase='climb';c.frame_target=.5
        o=StepObservation(.1,0,0,0,0,0,False)
        c.update(o,.01);self.assertEqual(c.frame_target,.5)
        cmd=c.update(replace(o,pitch=.6),.01)
        self.assertEqual(c.phase,'abort_posture');self.assertEqual(cmd.linear,0)
    def test_align_and_recover(self):
        c=GuardedStepController();o=StepObservation(.2,.2,0,0,0,0,False)
        cmd=c.update(o,.01);self.assertEqual(c.phase,'align');self.assertEqual(cmd.linear,0)
        c.phase='climb';c.update(replace(o,on_top=True),.01)
        self.assertEqual(c.phase,'recover')

    def test_rotation_direction_and_wheel_pause(self):
        c=GuardedStepController(climb_speed=0, frame_direction=-1)
        c.phase='climb';o=StepObservation(.1,0,0,0,0,0,False)
        command=c.update(o,.01)
        self.assertLess(c.frame_target,0)
        self.assertEqual(command.linear,0)
        with self.assertRaises(ValueError): GuardedStepController(frame_direction=0)
