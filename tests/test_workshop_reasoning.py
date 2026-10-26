"""Structural facts and staged reasoning are separate from physical validation."""
import json,tempfile,unittest,sys
from pathlib import Path
from xrobocon.workshop.reasoning import STAGES,mechanism_context,validate_analysis
from xrobocon.workshop.examples import lifting_robot
from xrobocon.workshop.inference import RobotImprovementGenerator,ParameterImprovementGenerator
from xrobocon.coin_selector import CodexCoinSelector

def analysis():
    return dict(stages=[dict(stage=s,status='needs_data',facts='入力事実',hypothesis='未確認',next_check='最小の確認') for s in STAGES],references=[],research_queries=[])
class ReasoningTests(unittest.TestCase):
    def test_mechanism_is_reference_pose_not_contact_measurement(self):
        snapshot=dict(xml=lifting_robot()['mjcf'],controller={})
        value=mechanism_context(snapshot)
        self.assertGreater(value['mass_kg'],0);self.assertTrue(value['joints'])
        self.assertFalse(value['drive_contract']['body_pose_and_contact_in_program'])
        self.assertIn('not observed contact',value['scope'])
    def test_stage_order_and_reference_validation(self):
        a=analysis();validate_analysis(a)
        a['stages'].reverse()
        with self.assertRaises(ValueError):validate_analysis(a)
        a=analysis();a['references']=[dict(url='file:///private/data',title='x',relevance='x',differences='x',status='consulted')]
        with self.assertRaises(ValueError):validate_analysis(a)
    def test_research_enabled_for_improvements_only(self):
        self.assertFalse(CodexCoinSelector.allow_web_research)
        for cls in (RobotImprovementGenerator,ParameterImprovementGenerator):
            self.assertTrue(cls.allow_web_research)
            self.assertIn('diagnostic_analysis',cls.schema['required'])
    def test_research_cli_uses_search_json_without_shell_and_counts_tool_items(self):
        class TestGenerator(CodexCoinSelector):
            allow_web_research=True
            instruction='Diagnostic test'
            def validate_response(self,result,payload):pass
        with tempfile.TemporaryDirectory() as d:
            exe=Path(d)/'fake-codex'
            exe.write_text('#!'+sys.executable+'\nimport sys,json\nfrom pathlib import Path\n'
                "assert '--search' in sys.argv and '--json' in sys.argv\n"
                "text=sys.stdin.read(); assert 'Use only built-in web search' in text\n"
                "assert 'Do not use tools, inspect files' not in text\n"
                "assert 'read-only' in sys.argv\n"
                "print(json.dumps({'type':'item.completed','item':{'type':'web_search','id':'w1'}}))\n"
                "Path(sys.argv[sys.argv.index('-o')+1]).write_text('{}')\n")
            exe.chmod(0o700);g=TestGenerator(str(exe),timeout=2)
            g.select({});self.assertEqual(g.last_research['web_search_items'],1)
    def test_analysis_survives_apply_save_and_display(self):
        from xrobocon.workshop.controller import WorkshopController
        from xrobocon.workshop.design import digest
        from xrobocon.match_setup import MatchCatalog
        from xrobocon.workshop.window import proposal_text
        with tempfile.TemporaryDirectory() as d:
            c=WorkshopController(MatchCatalog.load(Path(__file__).resolve().parents[1]/'configs/match_catalog.json'),d)
            try:
                c.mode='new_robot';c.proposal=lifting_robot();c.apply();c.save()
                c.proposal=dict(lifting_robot(),diagnostic_analysis=analysis(),constraint_review=dict(prompt_sha256=digest(b''),complies=True,summary='元要求は不明',checks=[]))
                c.require_constraint_review=True;c.mode='mechanism';c.apply();c.save()
                self.assertEqual(json.loads((c.directory/'diagnostic-analysis.json').read_text()),analysis())
                self.assertEqual(c.current['proposal']['diagnostic_analysis'],analysis())
                self.assertIn('段階的な原因分析',proposal_text(c))
            finally:c.close()

    def test_step_context_discloses_shared_runtime_and_reset_caveats(self):
        from xrobocon.workshop.reasoning import step_improvement_context
        snapshot=dict(controller={},diagnostic_context=dict(runs=[dict(run_id='new',suite='steps',matches_current=True,result=dict(coins=0)),dict(run_id='flat',suite='flat')]))
        c=step_improvement_context(snapshot)
        self.assertIn('Not fully independent',c['separation'])
        self.assertIn('tuning.drive',c['generated_limits'])
        self.assertFalse(c['fixture']['guarded_step_controller_used'])
        self.assertEqual(c['step_runs'][0]['run_id'],'new')
        self.assertEqual(len(c['step_runs']),1)
        self.assertIn('respawn',c['evidence_caveat'])
    def test_step_guidance_applies_to_both_improvement_modes(self):
        for cls in (RobotImprovementGenerator,ParameterImprovementGenerator):
            self.assertIn('first unresolved edge',cls.instruction)
            self.assertIn('NOT edge contact',cls.instruction)
            self.assertIn('unchanged model/code',cls.instruction)
            self.assertIn('flat regression',cls.instruction)
