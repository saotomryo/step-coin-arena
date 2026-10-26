"""Generate an active tri-star carrier robot; existing passive assets stay intact."""
import json,math,hashlib
from pathlib import Path
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]
def main():
    spec=json.loads((ROOT/'configs/robot_designs/tristar_carrier.json').read_text());r=spec['carrier_radius'];w=spec['wheel_radius']
    if spec['schema_version']!=1 or not 0<w<r or not all(math.isfinite(v) and v>0 for v in (r,w,spec['half_track'],spec['base_mass'],spec['wheel_mass'])):raise ValueError('invalid positive design dimensions/masses')
    root=ET.Element('mujoco',model='tristar_active_carrier');ET.SubElement(root,'compiler',angle='degree',inertiafromgeom='true')
    defaults=ET.SubElement(root,'default');ET.SubElement(defaults,'joint',damping='.1',armature='.01');ET.SubElement(defaults,'geom',friction='.8')
    base=ET.SubElement(ET.SubElement(root,'worldbody'),'body',name='base_link',pos='0 0 .4');ET.SubElement(base,'freejoint')
    ET.SubElement(base,'geom',type='box',size=' '.join(map(str,spec.get('base_half_size',[.23,.1,.035]))),pos=' '.join(map(str,spec['base_pos'])),mass=str(spec['base_mass']),rgba='.65 .25 .85 1')
    for side,sign in [('left',1),('right',-1)]:
        carrier=ET.SubElement(base,'body',name=side+'_tristar',pos=f"0 {sign*spec['half_track']} 0")
        ET.SubElement(carrier,'joint',name=side+'_tristar_joint',type='hinge',axis='0 1 0',damping='.2')
        ET.SubElement(carrier,'geom',type='cylinder',size='.035 .025',zaxis='0 1 0',mass='.3',rgba='.7 .3 .9 1')
        for i,(x,z) in enumerate([(math.sqrt(3)*r/2,-r/2),(-math.sqrt(3)*r/2,-r/2),(0,r)],1):
            ET.SubElement(carrier,'geom',type='capsule',fromto=f'0 0 0 {x} 0 {z}',size='.012',mass='.1',rgba='.65 .65 .7 1')
            wheel=ET.SubElement(carrier,'body',name=f'{side}_wheel_{i}',pos=f'{x} 0 {z}',zaxis='0 1 0')
            ET.SubElement(wheel,'joint',name=f'{side}_wheel_{i}_joint',type='hinge',axis='0 0 1',damping='.02')
            ET.SubElement(wheel,'geom',type='cylinder',size=f'{w} .025',mass=str(spec['wheel_mass']),rgba='.1 .1 .1 1')
    if spec.get('rear_casters'):
        for index,y in enumerate((-.10,.10)):
            ET.SubElement(base,'geom',name=f'tail_beam_{index}',type='capsule',fromto=f"-.08 {y} -.06 {spec.get('tail_pivot_x',-.36)} {y} -.08",size='.012',mass='0',contype='0',conaffinity='0',group='2',rgba='.55 .55 .6 1')
            fork=ET.SubElement(base,'body',name=f'tail_caster_{index}',pos=f"{spec.get('tail_pivot_x',-.36)} {y} -.08")
            ET.SubElement(fork,'joint',name=f'tail_swivel_{index}',axis='0 0 1',damping='.01',armature='.001')
            ET.SubElement(fork,'geom',type='capsule',fromto='0 0 0 -.035 0 -.07',size='.006',mass='.05',rgba='.6 .6 .65 1')
            wheel=ET.SubElement(fork,'body',name=f'tail_wheel_{index}',pos='-.035 0 -.07',zaxis='0 1 0')
            ET.SubElement(wheel,'joint',name=f'tail_roll_{index}',axis='0 0 1',damping='.005',armature='.001')
            ET.SubElement(wheel,'geom',type='cylinder',size='.10 .02',mass='.15',rgba='.15 .15 .15 1')
    tree=ET.ElementTree(root);ET.indent(tree,space='  ');target=ROOT/'xrobocon/assets/robot_tristar_carrier.xml';tree.write(target,encoding='utf-8',xml_declaration=True)
    target.with_suffix('.design.json').write_text(json.dumps(dict(specification='configs/robot_designs/tristar_carrier.json',asset_sha256=hashlib.sha256(target.read_bytes()).hexdigest()),indent=2)+'\n');print(target)
if __name__=='__main__':main()
