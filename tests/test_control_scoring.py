"""Scores describe achieved performance, with missing data and lineage preserved."""
import json,tempfile,unittest
from pathlib import Path
from xrobocon.workshop.scoring import score_trial,score_report,compare_scores,path_progress

def trial(case='straight',passed=False,error=1.,time=25):
    return dict(case=case,passed=passed,status='reached_and_stopped' if passed else 'timeout',final_error=error,simulation_seconds=time,max_tilt_degrees=0,torque_saturated_fraction=0)
def report():
    return dict(plan=dict(protocol_version=1,suite='flat',fixture_signature='fixture-A'),results=[dict(candidate='baseline',complete=True,error=None,trials=[trial(),trial('turn'),trial('route')])])
class ScoreTests(unittest.TestCase):
    def test_idle_is_low_and_reaching_is_dominant(self):
        self.assertEqual(score_trial(trial())['score'],15.)
        self.assertGreater(score_trial(trial(passed=True,error=.1,time=19))['score'],80)
        self.assertLess(score_trial(trial(error=.1))['score'],50)
    def test_route_progress_uses_waypoints_not_final_endpoint(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'route.jsonl'
            p.write_text('\n'.join(json.dumps(dict(pose=dict(x=x,y=y))) for x,y in [(0,0),(.6,0),(.6,.6)]))
            self.assertAlmostEqual(path_progress('route',p),2/3)
            self.assertEqual(score_trial(trial('route'))['score'],None)
            self.assertIsNotNone(score_trial(trial('route'),p)['score'])
    def test_missing_smoke_and_incomplete_are_not_zero(self):
        self.assertIsNone(score_report(None)['score'])
        r=report();r['plan']['suite']='joints';self.assertIsNone(score_report(r)['score'])
        r=report();r['results'][0]['complete']=False;self.assertIsNone(score_report(r)['score'])
        t=trial();t['max_tilt_degrees']=float('nan');self.assertIsNone(score_trial(t)['score'])
    def test_unmeasured_steps_do_not_pollute_flat_score(self):
        r=report()
        for t in r['results'][0]['trials']:t['passed']=True
        value=score_report(r)
        self.assertIsNotNone(value['score']);self.assertIsNone(value['cases']['three_steps']['score'])
        r['plan']['suite']='all';self.assertIsNone(score_report(r)['score'])
    def test_comparison_requires_same_fixture_scope_and_policy(self):
        a=dict(score=70,comparison_key=['v1',1,'flat','baseline','same'])
        b=dict(score=60,comparison_key=['v1',1,'flat','baseline','same'])
        self.assertEqual(compare_scores(a,b)['delta'],10)
        b['comparison_key'][-1]='different';self.assertIsNone(compare_scores(a,b)['delta'])
        b['comparison_key']=a['comparison_key'][:-1]+[None];self.assertIsNone(compare_scores(a,b)['delta'])
    def test_snapshot_scores_are_bound_to_model_and_merge_suites(self):
        from xrobocon.workshop.scoring import score_snapshot
        from xrobocon.workshop.diagnostics import fingerprint
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);snapshot=dict(xml='<robot/>',controller={},tuning={},robot_id='test',diagnostic_runs=[])
            for index,suite in enumerate(('flat','steps')):
                r=report();r['plan']['suite']=suite
                r['results'][0]['trials']=[trial('three_steps',True,time=90)] if suite=='steps' else [trial(c,True,time=10) for c in ('straight','turn','route')]
                path=root/'jobs'/str(index)/'diagnostic.json';path.parent.mkdir(parents=True)
                path.write_text(json.dumps(dict(fingerprint=fingerprint(snapshot),robot_id='test',status='finished',run_id=str(index),result=r)))
                snapshot['diagnostic_runs'].append(str(path))
            scored=score_snapshot(snapshot,root)
            self.assertEqual(scored['scope'],'all');self.assertIsNotNone(scored['score'])
            snapshot['xml']='<changed/>'
            self.assertIsNone(score_snapshot(snapshot,root)['score'])
    def test_missing_pass_judgment_is_unrated(self):
        t=trial();del t['passed'];self.assertIsNone(score_trial(t)['score'])
