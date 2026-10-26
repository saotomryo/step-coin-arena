"""Launch the native prompt-based robot/controller workbench."""
import argparse,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from xrobocon.match_setup import MatchCatalog
from xrobocon.workshop.controller import WorkshopController
from xrobocon.workshop.design import ensure_storage

def default_library():
    from xrobocon.storage import resource_root
    return resource_root()/'robot_workshop'

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--library');parser.add_argument('--catalog');parser.add_argument('--ui-smoke',action='store_true');parser.add_argument('--screenshot');parser.add_argument('--metrics');parser.add_argument('--headless-smoke',action='store_true');args=parser.parse_args()
    generator=None
    if args.ui_smoke or args.headless_smoke:
        class SmokeGenerator:
            def propose(self,snapshot,prompt,mode,cancelled):
                from xrobocon.workshop.design import editable_elements
                geom=next(e for e in editable_elements(snapshot['xml']) if e['kind']=='geom' and e['name']=='base_link_geom_0')
                pos=[float(v) for v in geom['attributes'].get('pos','0 0 0').split()];pos[2]-=.01
                return dict(title='UI検証用・低重心R18',summary='車体を1cm下げ、制御設定は維持する案。' if mode=='robot_only' else '重心を1cm下げて基準速度を0.13m/sに変更する案。',model_changes=[dict(kind='geom',name=geom['name'],attribute='pos',value=' '.join(map(str,pos)))],drive_changes=[],motion_changes=[] if mode=='robot_only' else [dict(profile='baseline',key='max_speed',value=.13)])
        generator=SmokeGenerator()
    if args.headless_smoke:
        import os;os.environ['QT_QPA_PLATFORM']='offscreen'
    library=Path(args.library) if args.library else default_library()
    catalog=args.catalog or (library/'catalog.json' if (library/'catalog.json').is_file() else ROOT/'configs/match_catalog.json')
    controller=WorkshopController(MatchCatalog.load(catalog),library,generator)
    from xrobocon.workshop.window import show_workshop
    show_workshop(controller,args.screenshot,args.ui_smoke or args.headless_smoke,args.metrics)
if __name__=='__main__':main()
