"""Generate explicit inertias from an auditable component budget; preserve baseline."""
import json,sys
from pathlib import Path
import xml.etree.ElementTree as ET
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import mujoco
from xrobocon.mass_layout import aggregate_boxes
ROOT=Path(__file__).resolve().parents[1]
def build():
    source=ROOT/'xrobocon/assets/robot_passive_front_bogie_r18_steering.xml'
    original=mujoco.MjModel.from_xml_path(str(source))
    layout=json.loads((ROOT/'configs/robot_component_layout.json').read_text())
    mass,com,inertia=aggregate_boxes(layout['components'])
    if abs(mass-4)>1e-10:raise ValueError('base mass budget must stay at 4kg')
    for detailed in (False,True):
        path=ROOT/'xrobocon/assets/visual/robot_front_bogie_r18_detailed.xml' if detailed else source
        tree=ET.parse(path);root=tree.getroot();root.set('model','component_mass_r18')
        root.find('compiler').set('inertiafromgeom','false')
        # Explicitly freeze all original tensors before adding a different base tensor.
        for body in root.findall('.//body'):
            bid=mujoco.mj_name2id(original,mujoco.mjtObj.mjOBJ_BODY,body.get('name'))
            old=body.find('inertial')
            if old is not None:body.remove(old)
            attrs=dict(mass=str(original.body_mass[bid]),pos=' '.join(map(str,original.body_ipos[bid])))
            if body.get('name')=='base_link':
                attrs.update(mass=str(mass),pos=' '.join(map(str,com)),fullinertia=' '.join(map(str,[inertia[0,0],inertia[1,1],inertia[2,2],inertia[0,1],inertia[0,2],inertia[1,2]])))
            else:attrs.update(quat=' '.join(map(str,original.body_iquat[bid])),diaginertia=' '.join(map(str,original.body_inertia[bid])))
            body.insert(0,ET.Element('inertial',attrs))
        target=ROOT/('xrobocon/assets/visual/robot_component_r18_detailed.xml' if detailed else 'xrobocon/assets/robot_component_r18.xml')
        ET.indent(tree,space='  ');tree.write(target,encoding='utf-8',xml_declaration=True)
    report=dict(base_mass=mass,base_com=com.tolist(),base_inertia=inertia.tolist(),total_mass=float(original.body_mass.sum()),baseline_base_com=original.body_ipos[1].tolist(),assumption=layout['description'])
    (ROOT/'xrobocon/assets/robot_component_r18.mass.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(report))
if __name__=='__main__':build()
