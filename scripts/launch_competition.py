"""Select robots/controllers/strategies before constructing the Genesis scene."""
import argparse,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from xrobocon.match_setup import MatchCatalog
from xrobocon.match_launcher import MatchLauncher,external_match_root
from xrobocon.match_setup_window import show_setup

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--catalog');parser.add_argument('--library');parser.add_argument('--out-root');parser.add_argument('--screenshot');parser.add_argument('--ui-smoke',action='store_true');args=parser.parse_args()
    library=Path(args.library) if args.library else None
    catalog_path=args.catalog
    if catalog_path is None:
        from robot_workshop import default_library
        try:
            library=library or default_library()
            custom=library/'catalog.json'
            catalog_path=str(custom) if custom.is_file() else None
        except RuntimeError:pass
    catalog=MatchCatalog.load(catalog_path or ROOT/'configs/match_catalog.json');output=Path(args.out_root) if args.out_root else external_match_root()
    launcher=MatchLauncher(catalog,output);show_setup(catalog,launcher,args.screenshot,args.ui_smoke,library)
if __name__=='__main__':main()
