"""Local competition environment with operator panel and per-round records."""
import argparse,json,queue,time,sys,os
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from run_framework import build_runtime
from xrobocon.session import CompetitionSession
from xrobocon.control_server import ControlServer
from xrobocon.native_panel import NativePanel
from xrobocon.viewer_panel import ViewerPanel

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--panel',choices=('python','browser','overlay'),default='python')
    parser.add_argument('--config',default=str(Path(__file__).resolve().parents[1]/'configs/competition_ros2.json'));parser.add_argument('--out',help='record directory; default: timestamped directory on configured external SSD')
    parser.add_argument('--viewer',action=argparse.BooleanOptionalAction,default=True,help='Genesis 3D window (default: enabled; --no-viewer for headless checks)');parser.add_argument('--port',type=int,default=0)
    parser.add_argument('--autostart',action='store_true');parser.add_argument('--return-after-match',action='store_true')
    args=parser.parse_args()
    if args.out is None:
        from xrobocon.storage import resource_root
        out=resource_root()/'evaluation_results'/f'competition-{time.time_ns()}'
    else:out=Path(args.out)
    print(f'Competition records: {out.resolve()}',flush=True)
    if args.panel=='overlay' and not args.viewer:parser.error('--panel overlay requires --viewer')
    config,simulator,manager,agents,runner=build_runtime(args.config,out,args.viewer)
    rounds=out/'rounds';rounds.mkdir()
    def save_round(round_id,state,reason):
        (rounds/f'{round_id:04d}-state.json').write_text(json.dumps(state,indent=2))
        (rounds/f'{round_id:04d}-summary.json').write_text(json.dumps(dict(reason=reason,elapsed=state['elapsed'],scores=state['scores'],collection_counts=state['collection_counts'],robot_contact_steps=simulator.robot_contact_steps,recovery_counts=state['recovery_counts'],recovery_blocked_seconds=state['recovery_blocked_seconds']),indent=2))
    session=CompetitionSession(runner,save_round)
    if args.panel=='overlay':server=ViewerPanel(simulator.scene.viewer,str((out/'viewer-panel.png').resolve()))
    elif args.panel=='python':server=NativePanel(str((out/'python-panel.png').resolve()),navigation=config.get('desktop_navigation'))
    else:server=ControlServer(Path(__file__).resolve().parents[1]/'xrobocon/assets/competition_panel.html',args.port)
    closed=False;last_error=None
    def publish():
        state=session.snapshot()
        try:positions=simulator.truth_positions()
        except RuntimeError:positions=server.state.get('positions',{})
        state['agent_states']={p:dict(strategy_label=getattr(a,'strategy_label',None),remaining_coin_ids=getattr(a,'remaining_coin_ids',[]),replan_reason=getattr(a,'replan_reason',None),target_id=getattr(a,'target_id',None),decision_pending=getattr(a,'decision_pending',False),decision_error=getattr(a,'decision_error',None)) for p,a in agents.items()}
        state.update(positions=positions,field=simulator.field.tiers,robot_contact_steps=simulator.robot_contact_steps,error=last_error,
                     rules_note='暫定規則。出現周期・得点・制限時間は設定ファイルで変更します。')
        server.state=state
    try:
        simulator.reset();runner._sync_display();publish();server.start()
        if args.panel=='browser':
            (out/'control_server.json').write_text(json.dumps(dict(url=server.url),indent=2))
            print(f'Competition panel: {server.url}',flush=True)
        else:print(f'{args.panel} competition panel ready',flush=True)
        if args.autostart:session.command('start')
        with (out/'trace.jsonl').open('w') as trace,(out/'controls.jsonl').open('w') as controls:
            while not closed:
                began=time.perf_counter()
                while True:
                    try:action=server.commands.get_nowait()
                    except queue.Empty:break
                    last_error=None
                    try:
                        if action=='shutdown':
                            if session.status in ('running','paused'):session._finish('shutdown')
                            closed=True
                        else:session.command(action)
                        result='ok'
                    except ValueError as error:last_error=str(error);result='rejected'
                    controls.write(json.dumps(dict(round_id=session.round_id,elapsed=manager.elapsed,action=action,result=result,error=last_error))+'\n');controls.flush()
                if args.panel=='overlay' and server.error:raise RuntimeError(server.error)
                if args.panel=='overlay' and not simulator.scene.viewer.is_alive():closed=True
                if args.panel=='python' and not server.process.is_alive():
                    if session.status in ('running','paused'):session._finish('panel_closed')
                    closed=True
                if closed:break
                try:row=session.tick()
                except Exception as error:
                    last_error=str(error);session.fail();row=None
                    (out/'error.json').write_text(json.dumps(dict(type=type(error).__name__,message=last_error)))
                if row:trace.write(json.dumps(row)+'\n');trace.flush()
                publish()
                if args.return_after_match and session.status in ('finished','error'):closed=True
                if args.viewer and row is None and simulator.scene.viewer.is_alive():simulator.scene.viewer.update(force=True)
                time.sleep(max(0,(config['dt'] if session.status=='running' else .05)-(time.perf_counter()-began)))
    except KeyboardInterrupt:
        if session.status in ('running','paused'):session._finish('interrupted')
    except Exception as error:
        last_error=str(error)
        if session.status in ('running','paused'):session._finish('error')
        (out/'error.json').write_text(json.dumps(dict(type=type(error).__name__,message=last_error)));raise
    finally:
        runner.close()
        (out/'session-state.json').write_text(json.dumps(session.snapshot(),indent=2))
        server.close()

if __name__=='__main__':main()
