"""Creation intent survives design/control iterations and reviewed candidates only apply."""
import copy,json,tempfile,time,unittest
from pathlib import Path
from unittest.mock import patch
from xrobocon.match_setup import MatchCatalog
from xrobocon.workshop.controller import WorkshopController
from xrobocon.workshop.examples import lifting_robot
from xrobocon.workshop.design import digest,write_json
ROOT=Path(__file__).resolve().parents[1]

class DesignImprovementTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.c=WorkshopController(MatchCatalog.load(ROOT/'configs/match_catalog.json'),self.tmp.name)
    def tearDown(self):self.c.close();self.tmp.cleanup()
    def create(self,prompt='四輪とリフトを維持。車輪半径0.12m。'):
        self.c.mode='new_robot';self.c.prompt=prompt;self.c.proposal=lifting_robot();self.c.apply();self.c.save()
    def review(self,prompt):
        return dict(prompt_sha256=digest((prompt or '').encode()),complies=True,summary='元の四輪とリフト、半径を維持',checks=[dict(requirement='車輪半径0.12m',verdict='preserved',requires_simulation=False,detail='車輪形状のsizeは0.12m')])
    def test_creation_prompt_is_not_overwritten_by_revision_prompt(self):
        self.create();original=self.c.current['creation_prompt']
        self.c.mode='program';self.c.prompt='PIDだけ調整';self.c.proposal=lifting_robot();self.c.apply();self.c.save()
        self.assertEqual(self.c.current['creation_prompt'],original)
        self.assertEqual(self.c.current['prompt'],'PIDだけ調整')
    def test_legacy_origin_is_recovered_but_edit_prompt_is_not_invented(self):
        self.create('元の作成条件');path=self.c.directory
        raw=json.loads((path/'manifest.json').read_text());raw.pop('creation_prompt');raw.pop('creation_prompt_source');write_json(path/'manifest.json',raw)
        self.assertEqual(self.c.library.load(path)['creation_prompt'],'元の作成条件')
        raw['mode']='robot_only';write_json(path/'manifest.json',raw)
        self.assertIsNone(self.c.library.load(path)['creation_prompt'])
    def test_design_change_keeps_identity_and_prompt_and_invalidates_result(self):
        self.create();identity=self.c.current['robot_id'];origin=self.c.current['creation_prompt']
        proposal=lifting_robot();proposal['mjcf']=proposal['mjcf'].replace('size=".24 .16 .055"','size=".24 .17 .055"')
        proposal['constraint_review']=self.review(origin)
        self.c.mode='mechanism';self.c.require_constraint_review=True;self.c.proposal=proposal
        self.c.current['diagnostic_result']={'passed':True};self.c.apply();self.c.save()
        self.assertEqual(self.c.current['robot_id'],identity);self.assertEqual(self.c.current['creation_prompt'],origin)
        self.assertNotIn('diagnostic_result',self.c.current)
        self.assertTrue(self.c.current['constraint_review']['complies'])
    def test_noncompliant_or_wrong_prompt_review_never_applies(self):
        self.create();before=copy.deepcopy(self.c.current)
        for key,value in [('complies',False),('prompt_sha256','wrong')]:
            proposal=lifting_robot();proposal['constraint_review']=self.review(before['creation_prompt']);proposal['constraint_review'][key]=value
            self.c.require_constraint_review=True;self.c.mode='mechanism';self.c.proposal=proposal
            with self.assertRaises(ValueError):self.c.apply()
            self.assertEqual(self.c.current,before)
    def test_loop_freezes_scope_and_creation_intent(self):
        self.create()
        with patch.object(self.c,'diagnostic'),patch.object(self.c,'improve_from_logs') as improve:
            self.c.start_automation(1,'flat',allow_robot_changes=True)
            self.c.automation.state['phase']='improve';self.c.automation.tick()
            self.assertTrue(improve.call_args.kwargs['allow_robot_changes'])
            self.assertEqual(self.c.automation.state['creation_prompt'],self.c.current['creation_prompt'])
            self.c.cancel()

    def test_unknown_prompt_can_be_improved_without_invented_constraints(self):
        from xrobocon.workshop.constraints import validate_review
        review=dict(prompt_sha256=digest(b''),complies=True,summary='元の作成プロンプトは不明',checks=[])
        self.assertIs(validate_review(review,None),review)
        review['checks']=self.review('x')['checks']
        with self.assertRaises(ValueError):validate_review(review,None)

    def test_retry_of_new_generation_keeps_the_original_not_repair_instruction(self):
        from xrobocon.workshop.inference import RejectedDesign
        class Generator:
            model='test';reasoning_effort='low'
            def propose(inner,snapshot,prompt,mode,cancelled,feedback=None):
                if feedback:return lifting_robot()
                raise RejectedDesign(lifting_robot(),'一時的な検証失敗')
        self.c.new_generator=Generator();self.c.propose('最初の作成条件','new_robot')
        for _ in range(100):
            self.c.poll()
            if not self.c.working:break
            time.sleep(.005)
        self.c.improve_from_logs('失敗を直す')
        for _ in range(100):
            self.c.poll()
            if not self.c.working:break
            time.sleep(.005)
        self.c.apply();self.assertEqual(self.c.current['creation_prompt'],'最初の作成条件')

    def test_generator_payload_and_reviewer_use_original_prompt(self):
        from xrobocon.workshop.inference import RobotImprovementGenerator
        self.create();snapshot=self.c.current;proposal=lifting_robot();proposal['constraint_review']=self.review(snapshot['creation_prompt'])
        generator=RobotImprovementGenerator()
        with patch('xrobocon.coin_selector.CodexCoinSelector.select',return_value=proposal) as select:
            generator.propose(snapshot,'改良する','mechanism')
        payload=select.call_args.args[0]
        self.assertEqual(payload['snapshot']['creation_prompt'],snapshot['creation_prompt'])
        self.assertEqual(payload['creation_prompt_sha256'],digest(snapshot['creation_prompt'].encode()))
        generator.validate_response(proposal,payload)
