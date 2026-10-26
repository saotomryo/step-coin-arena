"""Real Genesis overlay/lifecycle integration check (no synthetic mouse input)."""
import json,sys,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from run_framework import build_runtime
from xrobocon.session import CompetitionSession
from xrobocon.viewer_panel import ViewerPanel

def main():
    out=Path(sys.argv[1]);config,sim,manager,agents,runner=build_runtime('configs/competition_smoke.json',out,True)
    session=CompetitionSession(runner);panel=ViewerPanel(sim.scene.viewer,str((out/'overlay.png').resolve()))
    def refresh():
        panel.state=session.snapshot();sim.scene.viewer.update(force=True)
        if panel.error:raise RuntimeError(panel.error)
    def command(action):
        panel.commands.put(action);session.command(panel.commands.get_nowait());refresh()
    try:
        sim.reset();runner._sync_display();panel.start();refresh()
        assert panel.drawn,'overlay did not draw'
        assert sim.scene._interactive_scene is None if hasattr(sim.scene,'_interactive_scene') else True
        command('start')
        for _ in range(20):session.tick()
        command('pause');elapsed=manager.elapsed;positions=sim.truth_positions()
        for _ in range(5):refresh();session.tick();time.sleep(.02)
        assert manager.elapsed==elapsed and sim.truth_positions()==positions
        command('resume');session.tick();assert manager.elapsed>elapsed
        command('stop');assert manager.status=='finished'
        command('reset');assert manager.elapsed==0 and all(v==0 for v in manager.scores.values())
        command('start')
        while session.status=='running':session.tick()
        refresh();assert manager.elapsed>=4 and session.status=='finished'
        command('reset');refresh()
        (out/'verification.json').write_text(json.dumps({'passed':True,'checks':['overlay rendered with real Genesis','command queue connected to lifecycle','pause freezes clock and positions while viewer refreshes','resume advances clock','manual stop','reset clears state','time limit finishes without ending environment']},indent=2))
        print('Overlay integration: 7 checks passed',flush=True)
    finally:
        panel.close();sim.stop()

if __name__=='__main__':main()
