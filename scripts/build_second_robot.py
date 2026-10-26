"""Build the second physical robot from explicit dimensions, preserving R18."""
import json,hashlib
from math import isfinite
from pathlib import Path
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]
def main():
    path=ROOT/'configs/robot_designs/front_bogie_r20_wide.json';spec=json.loads(path.read_text())
    source=(path.parent/spec['source']).resolve();target=(path.parent/spec['target']).resolve()
    if spec.get('schema_version')!=1 or source==target:raise ValueError('invalid design schema or baseline overwrite')
    if any(not isinstance(v,(int,float)) or isinstance(v,bool) or not isfinite(v) or v<=0 for v in (spec['wheel_radius'],spec['rocker_lateral_offset'])):raise ValueError('positive finite dimensions required')
    if len(spec['base_geom_pos'])!=3 or not all(isinstance(v,(int,float)) and not isinstance(v,bool) and isfinite(v) for v in spec['base_geom_pos']):raise ValueError('finite XYZ body placement required')
    tree=ET.parse(source);root=tree.getroot();root.set('model','front_bogie_r20_wide_low')
    for side,sign in [('left',1),('right',-1)]:
        root.find(f".//body[@name='{side}_rocker']").set('pos',f"0 {sign*spec['rocker_lateral_offset']} 0")
        for position in ('front','middle','rear'):
            geom=root.find(f".//body[@name='{side}_wheel_{position}']/geom")
            size=geom.get('size').split();size[0]=str(spec['wheel_radius']);geom.set('size',' '.join(size))
    body=root.find(".//body[@name='base_link']/geom");body.set('pos',' '.join(map(str,spec['base_geom_pos'])));body.set('rgba','.2 .65 .35 1')
    ET.indent(tree,space='  ');tree.write(target,encoding='utf-8',xml_declaration=True)
    target.with_suffix('.design.json').write_text(json.dumps(dict(specification=str(path.relative_to(ROOT)),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),asset_sha256=hashlib.sha256(target.read_bytes()).hexdigest()),indent=2)+'\n')
    print(target)
if __name__=='__main__':main()
