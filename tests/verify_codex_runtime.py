"""Bounded native Genesis + actual DDS + actual Codex inference integration."""
import argparse,json,os,secrets,signal,subprocess,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from xrobocon.match_setup import MatchCatalog,build_match_config
ROOT=Path(__file__).resolve().parents[1]

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--out',required=True);parser.add_argument('--strategy',choices=('codex_target','codex_plan'),default='codex_target');args=parser.parse_args()
    from xrobocon.workshop.design import ensure_storage
    ensure_storage(Path(args.out).parent)
    out=Path(args.out);out.mkdir(parents=True,exist_ok=False)
    catalog=MatchCatalog.load(ROOT/'configs/match_catalog.json');choices=catalog.defaults();choices['self']['strategy']=args.strategy
    cfg=build_match_config(catalog,choices);cfg['competition']['time_limit']=12.
    cfg['agents']['self']['game_rules']['time_limit']=12.
    cfg['transport']['domain_id']=100+secrets.randbelow(100)
    path=out/'config.json';path.write_text(json.dumps(cfg,indent=2))
    command=[sys.executable,str(ROOT/'scripts/run_competition.py'),'--config',str(path),'--out',str(out/'run'),'--panel','overlay','--autostart','--return-after-match']
    with (out/'process.log').open('w') as log:
        child=subprocess.Popen(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
        try:code=child.wait(timeout=240)
        finally:
            if child.poll() is None:child.send_signal(signal.SIGINT);child.wait(timeout=20)
    if code:raise RuntimeError(f'competition exited {code}; see process.log')
    rows=[json.loads(x) for x in (out/'run/trace.jsonl').read_text().splitlines()]
    assert rows and all(r['transport']['sources']=={'self':'ros2/truth_baseline','opponent':'ros2/truth_baseline'} for r in rows)
    plans=[r['route_plans']['self'] for r in rows if 'self' in r['route_plans']]
    decisions=[p['decision'] for p in plans if 'decision' in p and 'output' in p['decision']]
    assert decisions,'no actual Codex selection returned before the deadline'
    assert any(r['agent_states']['self']['decision_pending'] and r['commands']['self']['linear']==0 for r in rows)
    assert any(r['agent_states']['self']['target_id'] is not None and abs(r['commands']['self']['linear'])+abs(r['commands']['self']['angular'])>0 for r in rows),'no rule controller command after selection'
    assert not any(r['agent_states']['self']['decision_error'] for r in rows)
    state=json.loads((out/'run/session-state.json').read_text());assert state['control_status']=='finished'
    assert (out/'run/viewer-panel.png').exists()
    if args.strategy=='codex_plan':
        assert all('strategy' in d['output'] and 'coin_ids' in d['output'] and 'stop_conditions' in d['output'] for d in decisions)
        assert any(r['agent_states']['self']['remaining_coin_ids'] for r in rows)
    report=dict(passed=True,strategy=args.strategy,simulation_seconds=12,decisions=decisions,
        checks=['actual Codex structured inference','both agents use DDS','intentional stop while pending','rule controller resumes after selection','Genesis and score overlay','clean match finish'])
    (out/'verification.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k!='decisions'}),flush=True)
if __name__=='__main__':main()
