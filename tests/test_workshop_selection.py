"""Keep failed candidates as branches and restart from a tested accepted version."""
import tempfile,unittest
from pathlib import Path
from unittest.mock import patch
from xrobocon.workshop.selection import regression
from xrobocon.workshop.automation import ImprovementLoop
from xrobocon.workshop.controller import WorkshopController
from xrobocon.workshop.examples import lifting_robot
from xrobocon.match_setup import MatchCatalog
ROOT=Path(__file__).resolve().parents[1]
def report(straight,turn,passed=False):
    return dict(results=[dict(candidate='baseline',complete=True,passed=False,trials=[dict(case='straight',passed=passed,final_error=straight),dict(case='turn',passed=False,final_error=turn)])])
class SelectionTests(unittest.TestCase):
    def test_clear_regression_and_mixed_tradeoff(self):
        self.assertTrue(regression(report(.1,1,True),report(.3,.8),'flat')[0])
        self.assertTrue(regression(report(.5,1),report(.8,1.2),'flat')[0])
        self.assertFalse(regression(report(.5,1),report(.4,1.2),'flat')[0])
        self.assertFalse(regression(report(.5,1),report(.51,1.01),'flat')[0])
        self.assertFalse(regression(report(.5,1),report(.8,1.2),'joints')[0])
        self.assertFalse(regression(report(.5,1),dict(),'flat')[0])
        self.assertTrue(regression(report(.5,1),dict(error='process failed'),'flat')[0])
    def test_rollback_keeps_failed_branch_and_next_revision_parent(self):
        with tempfile.TemporaryDirectory() as d:
            c=WorkshopController(MatchCatalog.load(ROOT/'configs/match_catalog.json'),d)
            c.mode='new_robot';c.proposal=lifting_robot();c.apply();c.save();baseline=c.current['revision_id']
            results=iter([report(.1,1,True),report(.8,1.2),report(.1,.8,True)])
            starts=[]
            def diagnostic(suite):
                c.last_result=next(results);c.current['diagnostic_result']=c.last_result;c.job={'root':Path(d)}
            def improve(prompt,**kwargs):
                starts.append(c.current['revision_id']);c.mode='program';c.proposal=lifting_robot()
            try:
                with patch.object(c,'diagnostic',side_effect=diagnostic),patch.object(c,'improve_from_logs',side_effect=improve):
                    loop=ImprovementLoop(c,2,'flat',stop_on_pass=False)
                    for _ in range(20):loop.tick()
                self.assertEqual(starts,[baseline,baseline])
                self.assertEqual(c.current['parent'],baseline)
                selections=[e for e in loop.state['history'] if e['kind']=='selection']
                self.assertEqual([e['action'] for e in selections],['continue','rollback','continue'])
                failed=selections[1]['candidate_revision']
                self.assertTrue((Path(d)/'revisions'/failed/'manifest.json').is_file())
                self.assertEqual(selections[1]['accepted_revision'],baseline)
                self.assertNotEqual(c.current['revision_id'],failed)
            finally:c.close()
