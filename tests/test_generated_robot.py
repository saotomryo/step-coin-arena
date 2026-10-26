"""Topology/program contract and persistence regressions for new robots."""
import copy
import tempfile
import unittest
from pathlib import Path
from xrobocon.workshop.generated import validate_design, apply_design, ControlProgram
from xrobocon.workshop.examples import lifting_robot
from xrobocon.workshop.design import WorkshopLibrary

class GeneratedRobotTests(unittest.TestCase):
    def test_new_topology_and_controller_are_validated_together(self):
        proposal = lifting_robot()
        snapshot = apply_design(None, proposal)
        self.assertEqual(snapshot['entry']['drive_kind'], 'program')
        self.assertEqual(len(snapshot['controller']['joints']), 5)
        program = ControlProgram(snapshot['controller']['code'], snapshot['controller']['joints'])
        state = dict(t=0.,dt=.01,linear=.12,angular=0.,lift=1.,frame_target=0.,climb_stage=0.,q={},qd={})
        targets = program.evaluate(state)
        self.assertGreater(targets['lift_joint'], 0)
        self.assertAlmostEqual(targets['left_front_joint'], 1.)

    def test_reject_host_access_and_unbounded_code(self):
        for code in ["import os\ndef control(s):\n return {}", "def control(s):\n while True: pass", "def control(s):\n return {'lift_joint': __import__('os').system('id')}", "def control(s):\n return {'lift_joint': 2**100000000}"]:
            with self.subTest(code=code), self.assertRaises(ValueError):
                ControlProgram(code, lifting_robot()['joints'])

    def test_reject_files_plugins_missing_joints_and_bad_limits(self):
        for mutate in [lambda p:p.update(mjcf=p['mjcf'].replace('<worldbody>', '<include file="secret.xml"/><worldbody>')),
                       lambda p:p['joints'][0].update(name='missing'),
                       lambda p:p['joints'][0].update(torque_limit=-1),
                       lambda p:p.update(controller_code=p['controller_code'].replace('lift_joint', 'unknown'))]:
            proposal = lifting_robot();mutate(proposal)
            with self.assertRaises(ValueError): validate_design(proposal)

    def test_save_load_hash_and_branch(self):
        with tempfile.TemporaryDirectory() as root:
            lib = WorkshopLibrary(root)
            one = apply_design(None, lifting_robot())
            path = lib.save(one, '持ち上げ機構を作成')
            loaded = lib.load(path)
            self.assertEqual(loaded['controller'], one['controller'])
            self.assertTrue((path/'controller.py').exists())
            two = apply_design(loaded, lifting_robot())
            next_path = lib.save(two, '修正')
            self.assertEqual(lib.load(next_path)['parent'], path.name)
            (path/'controller.py').write_text('tampered')
            with self.assertRaises(ValueError):lib.load(path)

    def test_stop_targets_do_not_walk_and_nonfinite_is_rejected(self):
        p = lifting_robot();program=ControlProgram(p['controller_code'],p['joints'])
        state=dict(t=10.,dt=.01,linear=0.,angular=0.,lift=0.,frame_target=0.,climb_stage=-1.,q={},qd={})
        self.assertTrue(all(v==0 for v in program.evaluate(state).values()))
        state['linear']=float('nan')
        with self.assertRaises(ValueError):program.evaluate(state)

class GeneratedWorkflowTests(unittest.TestCase):
    def test_register_compiles_controller_and_model_config(self):
        from xrobocon.match_setup import MatchCatalog,build_match_config
        from xrobocon.workshop.controller import WorkshopController
        root=Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as d:
            c=WorkshopController(MatchCatalog.load(root/'configs/match_catalog.json'),d)
            try:
                c.mode='new_robot';c.proposal=lifting_robot();c.prompt='新規機体'
                c.apply();c.save();catalog=MatchCatalog.load(c.register())
                choices=catalog.defaults();choices['self']['robot']='workshop_'+c.current['robot_id']
                config=build_match_config(catalog,choices)
                self.assertEqual(config['simulation']['robot_models']['self']['drive_kind'],'program')
                c.load_revision(c.directory)
                c.mode='program';c.proposal=lifting_robot()
                c.proposal['mjcf']=c.proposal['mjcf'].replace('mass="3"','mass="4"')
                with self.assertRaises(ValueError):c.apply()
            finally:c.close()

    def test_program_drive_clamps_limits_and_resets_phase(self):
        from xrobocon.program_drive import ProgramDrive
        from xrobocon.pid_drive import PID
        from xrobocon.navigation import VelocityCommand
        d=ProgramDrive.__new__(ProgramDrive)
        p=lifting_robot();d.specs=p['joints'];d.program=ControlProgram(p['controller_code'],p['joints'])
        d.indices=list(range(5));d.controllers=[PID(s['kp'],0,s['kd'],s['torque_limit']) for s in d.specs]
        d.ranges=[None]*4+[(0.,1.)];d.frame_target=0.;d.lift_override=None;d.reset()
        force=d.compute(VelocityCommand(100.,0.,climb_stage=0),.01,[0.]*5,[0.]*5)
        self.assertTrue(all(abs(v)<=s['torque_limit'] for v,s in zip(force,d.specs)))
        self.assertLessEqual(max(d.targets),12.)
        self.assertGreater(d.time,0);d.reset();self.assertEqual(d.time,0)

    def test_leg_gait_expression_and_zero_speed_stance(self):
        joints=[dict(name=f'{leg}_hip') for leg in ('fl','fr','rl','rr')]
        code='def control(s):\n    return {'+','.join(repr(j['name'])+": .3 * sin(6 * s['t'] + "+str(i%2*3.14159)+") if abs(s['linear']) > .001 else 0" for i,j in enumerate(joints))+'}\n'
        p=ControlProgram(code,joints)
        state=dict(t=.2,dt=.01,linear=.1,angular=0.,lift=0.,frame_target=0.,climb_stage=-1.,q={},qd={})
        self.assertTrue(any(abs(v)>.01 for v in p.evaluate(state).values()))
        state['linear']=0
        self.assertTrue(all(v==0 for v in p.evaluate(state).values()))

class SharedControlTests(unittest.TestCase):
    def test_shared_expressions_and_inverse_kinematics(self):
        code="def control(s):\n    x = .1 * s['linear']\n    z = .3\n    radius = sqrt(x*x + z*z)\n    knee = acos(clip((.08-radius*radius)/.08,-1,1))\n    return {'hip': atan2(x,z), 'knee': knee}\n"
        p=ControlProgram(code,[dict(name='hip'),dict(name='knee')])
        s=dict(t=0,dt=.01,linear=.1,angular=0,lift=0,frame_target=0,climb_stage=-1,q={},qd={})
        self.assertGreater(p.evaluate(s)['knee'],0)
        first=p.evaluate(s);s['linear']=0;p.evaluate(s);s['linear']=.1
        self.assertEqual(first,p.evaluate(s))
    def test_reject_unbounded_or_unsafe_locals(self):
        for body in ['x = x + 1','s = 1','sin = 1','x = 1\n    x = 2','x = unknown','x, y = 1, 2','x = s.__class__','x = [1]','x = 1\n    for i in range(10): pass']:
            with self.subTest(body=body),self.assertRaises(ValueError):
                ControlProgram('def control(s):\n    '+body+"\n    return {'hip': 0}",[dict(name='hip')])
    def test_resource_limits_and_numeric_domain_remain_bounded(self):
        locals_code='def control(s):\n'+''.join(f'    x{i} = 0\n' for i in range(65))+"    return {'hip': 0}"
        with self.assertRaises(ValueError):ControlProgram(locals_code,[dict(name='hip')])
        code="def control(s):\n    x = sqrt(s['linear'])\n    return {'hip': x}"
        p=ControlProgram(code,[dict(name='hip')])
        s=dict(t=0,dt=.01,linear=-1,angular=0,lift=0,frame_target=0,climb_stage=-1,q={},qd={})
        with self.assertRaises(ValueError):p.evaluate(s)
        # A wide expression, avoiding parser recursion as a confounder.
        expression='max('+','.join(['max('+','.join(['max('+','.join(['1']*4)+')']*4)+')']*4)+')'
        expression='max('+','.join([expression]*4)+')'
        code="def control(s):\n    return {"+','.join(repr('j'+str(i))+':'+expression for i in range(12))+'}'
        with self.assertRaisesRegex(ValueError,'AST is too large'):ControlProgram(code,[dict(name='j'+str(i)) for i in range(12)])
