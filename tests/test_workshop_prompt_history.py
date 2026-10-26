"""Trace actual prompt usage without adding comparison trials or AI calls."""
import json,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
from xrobocon.workshop.controller import WorkshopController
from xrobocon.workshop.automation import ImprovementLoop
from xrobocon.workshop.examples import lifting_robot
from xrobocon.match_setup import MatchCatalog
ROOT=Path(__file__).resolve().parents[1]

class PromptHistoryTests(unittest.TestCase):
    def test_fixed_prompt_is_used_on_every_attempt_and_linked_to_saved_revision(self):
        with tempfile.TemporaryDirectory() as d:
            c=WorkshopController(MatchCatalog.load(ROOT/'configs/match_catalog.json'),d)
            c.mode='new_robot';c.proposal=lifting_robot();c.apply();c.save()
            calls=[]
            def diagnostic(suite):
                c.last_result={'results':[dict(complete=True,passed=False)]};c.job={'root':Path(d)}
            def improve(prompt,**kwargs):
                calls.append((prompt,kwargs));c.prompt=prompt+'\n固定の制御指示';c.mode='program'
                c.proposal=lifting_robot()
            try:
                with patch.object(c,'diagnostic',side_effect=diagnostic),patch.object(c,'improve_from_logs',side_effect=improve):
                    loop=ImprovementLoop(c,2,'flat','元の改善目標',False,120)
                    for _ in range(20):loop.tick()
                self.assertEqual([x[0] for x in calls],['元の改善目標','元の改善目標'])
                self.assertNotIn('prompt_context',calls[1][1])
                self.assertEqual(loop.state['reason'],'limit_reached')
                saved=json.loads((c.directory/'manifest.json').read_text())
                usage=saved['improvement_prompt_usage']
                self.assertEqual(usage['version'],1);self.assertEqual(usage['effective_prompt'],saved['prompt'])
                self.assertIn('+00:00',usage['used_at'])
                history=json.loads((loop.directory/'loop.json').read_text())
                self.assertEqual(len(history['prompt_history']),1)
                versions=[e for e in history['history'] if e['kind']=='revision']
                self.assertEqual([e['prompt_version'] for e in versions],[1,1])
                self.assertEqual(versions[-1]['revision'],saved['id'])
            finally:c.close()

    def test_fixed_templates_separate_robot_work_from_coin_strategy(self):
        from xrobocon.coin_selector import CodexCoinSelector
        from xrobocon.workshop.inference import NewRobotGenerator,RobotImprovementGenerator,ParameterImprovementGenerator
        self.assertIn('opponent',CodexCoinSelector.context_instruction)
        for cls in (NewRobotGenerator,RobotImprovementGenerator,ParameterImprovementGenerator):
            self.assertEqual(cls.context_instruction,'')
        self.assertEqual(RobotImprovementGenerator.prompt_version,'2026-10-23-v5')
        self.assertIn('rollback',RobotImprovementGenerator.instruction)
