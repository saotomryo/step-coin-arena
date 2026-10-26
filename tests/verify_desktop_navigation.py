"""Real local activation between Qt windows, preserving workshop text."""
import argparse,tempfile,time,json,sys
from pathlib import Path
from unittest.mock import Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from xrobocon.workshop.qt_runtime import prepare_plugins
prepare_plugins()
from PySide6.QtWidgets import QApplication
from PySide6.QtGui import QFont
from xrobocon.desktop_navigation import DesktopLinks
from xrobocon.match_setup import MatchCatalog
from xrobocon.match_setup_window import MatchSetupWindow
from xrobocon.workshop.controller import WorkshopController
from xrobocon.workshop.qt_window import WorkshopWindow

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--out',required=True);args=parser.parse_args()
    out=Path(args.out);out.mkdir(parents=True,exist_ok=False)
    app=QApplication([]);app.setFont(QFont('Hiragino Sans',12))
    with tempfile.TemporaryDirectory(prefix='xrobocon-navigation-') as d:
        catalog=MatchCatalog.load('configs/match_catalog.json')
        a=DesktopLinks('setup',d,catalog.path,out);b=DesktopLinks('workshop',d,catalog.path,out)
        launcher=Mock(catalog=catalog,output_root=out,running=False,preview_process=None);launcher.poll.return_value=None
        setup=MatchSetupWindow(catalog,launcher,d,a)
        controller=WorkshopController(catalog,d);workshop=WorkshopWindow(controller,links=b)
        activated=[]
        assert a.claim(setup,lambda:activated.append('setup'))
        assert b.claim(workshop,lambda:activated.append('workshop'))
        setup.show();workshop.show();workshop.prompt.setPlainText('切替しても編集中の日本語プロンプトを維持する')
        setup.workshop_button.click()
        until=time.monotonic()+3
        while 'workshop' not in activated and time.monotonic()<until:app.processEvents();time.sleep(.02)
        assert 'workshop' in activated,activated
        workshop.match_button.click()
        until=time.monotonic()+3
        while 'setup' not in activated and time.monotonic()<until:app.processEvents();time.sleep(.02)
        assert 'setup' in activated,activated
        assert workshop.prompt.toPlainText().startswith('切替しても')
        for _ in range(5):app.processEvents();time.sleep(.02)
        assert setup.grab().save(str(out/'setup.png'));assert workshop.grab().save(str(out/'workshop.png'))
        # Exercise the unopened-destination path, without touching any user window.
        b.server.close();workshop.close();app.processEvents()
        assert a.open('workshop')=='opened'
        child=a.children['workshop'];connected=False;until=time.monotonic()+25
        try:
            while time.monotonic()<until and child.poll() is None:
                app.processEvents()
                if a.activate('workshop'):connected=True;break
                time.sleep(.1)
            assert connected,'workshop child did not register an activation endpoint'
        finally:
            child.terminate()
            try:child.wait(timeout=5)
            except Exception:child.kill();child.wait()
        setup.close();app.processEvents()
        report=dict(passed=True,checks=['setup button activates existing workshop','workshop button activates existing setup','unsaved prompt preserved','unopened workshop launches with same library','child activation endpoint detected'])
        (out/'verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));print(json.dumps(report))
if __name__=='__main__':main()
