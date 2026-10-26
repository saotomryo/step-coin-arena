"""Typed robot edits and immutable robot/control revisions. Generated control bundles use a validated expression runtime."""
from copy import deepcopy
from pathlib import Path
import hashlib,json,math,os,tempfile,time,uuid
import xml.etree.ElementTree as ET
from xrobocon.robot_tuning import load_tuning,DRIVE_DEFAULTS
from xrobocon.motion_profiles import MotionProfile
from dataclasses import asdict

ATTRS={'body':{'pos'},'geom':{'pos','size','mass','friction','rgba','fromto'},'joint':{'range','damping','armature'}}

def digest(data):return hashlib.sha256(data).hexdigest()
def write_json(path,value):Path(path).write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
def ensure_storage(path):
    path=Path(path).resolve()
    if path.parts[1:2]==('Volumes',) and not os.path.ismount(Path('/Volumes')/path.parts[2]):
        raise RuntimeError('外部ディスクを接続してください。内蔵への代替保存は行いません。')
    return path

def canonical_xml(text):
    root=ET.fromstring(text)
    if root.tag!='mujoco' or root.find('include') is not None or root.find('asset') is not None:
        raise ValueError('workshop v1 requires self-contained primitive MJCF')
    used={e.get('name') for e in root.iter() if e.get('name')}
    for body in root.iter('body'):
        for n,geom in enumerate(body.findall('geom')):
            if not geom.get('name'):
                name=f"{body.get('name','body')}_geom_{n}"
                while name in used:name+='x'
                geom.set('name',name);used.add(name)
    ET.indent(root,space='  ')
    return ET.tostring(root,encoding='unicode')

def source_snapshot(catalog,robot_id):
    entry=deepcopy(catalog.options('robots')[robot_id]);asset=(catalog.path.parent/entry['robot_asset']).resolve()
    tuning=load_tuning(catalog.path.parent/entry['tuning_file']) if entry.get('tuning_file') else dict(schema_version=1,drive=deepcopy(DRIVE_DEFAULTS),motion={})
    tuning.pop('calibration',None)
    controller={}
    if entry.get('controller_file'):
        from xrobocon.workshop.generated import load_bundle
        controller['controller']=load_bundle(catalog.path.parent/entry['controller_file'],asset)
    return dict(**controller,xml=canonical_xml(asset.read_text()),tuning=tuning,entry=entry,parent=None,
                source_robot=robot_id,source_asset_sha256=digest(asset.read_bytes()),creation_prompt=None,creation_prompt_source='unavailable')

def editable_elements(xml):
    root=ET.fromstring(xml)
    return [dict(kind=e.tag,name=e.get('name'),attributes={k:v for k,v in e.attrib.items() if k in ATTRS[e.tag]},
                 geometry_type=e.get('type')) for e in root.iter() if e.tag in ATTRS and e.get('name')]

def validate_proposal(proposal,snapshot,mode):
    if mode not in ('robot','robot_only','control'):raise ValueError('unknown generation mode')
    if not isinstance(proposal,dict) or set(proposal)!={'title','summary','model_changes','drive_changes','motion_changes'}:raise ValueError('invalid proposal fields')
    for field,maximum in [('title',80),('summary',3000)]:
        if not isinstance(proposal[field],str) or not 1<=len(proposal[field].strip())<=maximum:raise ValueError('invalid '+field)
    for key in ('model_changes','drive_changes','motion_changes'):
        if not isinstance(proposal[key],list) or len(proposal[key])>100:raise ValueError('too many or invalid changes')
    if mode=='robot_only' and (proposal['drive_changes'] or proposal['motion_changes']):raise ValueError('robot-only proposals cannot change control settings')
    if mode=='control' and proposal['model_changes']:raise ValueError('control-only proposals cannot change robot geometry')
    root=ET.fromstring(snapshot['xml']);seen=set()
    for change in proposal['model_changes']:
        if not isinstance(change,dict) or set(change)!={'kind','name','attribute','value'}:raise ValueError('invalid model edit')
        kind,name,attr,value=(change[k] for k in ('kind','name','attribute','value'))
        if kind not in ATTRS or attr not in ATTRS[kind] or not isinstance(value,str):raise ValueError('unsupported model attribute')
        elements=[e for e in root.iter(kind) if e.get('name')==name]
        if len(elements)!=1:raise ValueError('model element not found or ambiguous: '+str(name))
        key=(kind,name,attr)
        if key in seen:raise ValueError('duplicate model edit')
        seen.add(key)
        try:values=[float(v) for v in value.split()]
        except ValueError:raise ValueError('model attributes must be numeric') from None
        lengths={'pos':(3,),'size':(1,2,3),'mass':(1,),'friction':(1,3),'rgba':(4,),'fromto':(6,),'range':(2,),'damping':(1,),'armature':(1,)}
        if len(values) not in lengths[attr] or not all(math.isfinite(v) and abs(v)<=100 for v in values):raise ValueError('invalid model numeric dimensions')
        if attr in ('size','mass') and min(values)<=0:raise ValueError('positive size and mass required')
        if attr in ('friction','damping','armature') and min(values)<0:raise ValueError('nonnegative material and joint properties required')
        if attr=='rgba' and not all(0<=v<=1 for v in values):raise ValueError('rgba must be 0..1')
        if attr=='range' and values[0]>=values[1]:raise ValueError('joint range must increase')
    seen=set()
    for change in proposal['drive_changes']:
        if not isinstance(change,dict) or set(change)!={'key','value'} or change['key'] not in DRIVE_DEFAULTS:raise ValueError('unknown drive edit')
        if change['key'] in seen:raise ValueError('duplicate drive edit')
        seen.add(change['key'])
        v=change['value']
        if type(v) not in (int,float) or not math.isfinite(v) or not 0<=v<=100:raise ValueError('invalid drive gain')
    seen=set();limits=set(asdict(MotionProfile()))
    for change in proposal['motion_changes']:
        if not isinstance(change,dict) or set(change)!={'profile','key','value'} or change['profile'] not in ('baseline','safe','fast') or change['key'] not in limits:raise ValueError('unknown motion edit')
        key=(change['profile'],change['key'])
        if key in seen:raise ValueError('duplicate motion edit')
        seen.add(key);v=change['value']
        if type(v) not in (int,float) or not math.isfinite(v) or not 0<v<=20:raise ValueError('invalid motion limit')
    if not any(proposal[k] for k in ('model_changes','drive_changes','motion_changes')):raise ValueError('この版で適用可能な変更がありません: '+proposal['summary'])
    return deepcopy(proposal)

def apply_proposal(snapshot,proposal,mode):
    proposal=validate_proposal(proposal,snapshot,mode)
    if 'controller' in snapshot and proposal['drive_changes']:raise ValueError('新規生成機体の関節ゲインは「制御コードを生成・修正」で変更してください')
    updated=deepcopy(snapshot);root=ET.fromstring(snapshot['xml'])
    for change in proposal['model_changes']:
        element=next(e for e in root.iter(change['kind']) if e.get('name')==change['name'])
        element.set(change['attribute'],change['value'])
    updated['xml']=canonical_xml(ET.tostring(root,encoding='unicode'))
    updated.pop('diagnostic_result',None)
    tuning=updated['tuning'];tuning.pop('calibration',None)
    for change in proposal['drive_changes']:tuning.setdefault('drive',{})[change['key']]=change['value']
    for change in proposal['motion_changes']:tuning.setdefault('motion',{}).setdefault(change['profile'],{})[change['key']]=change['value']
    # Shared validation and MuJoCo compilation run before any draft is saved.
    with tempfile.TemporaryDirectory(prefix='robot-validate-') as directory:
        path=Path(directory)/'tuning.json';write_json(path,tuning);updated['tuning']=load_tuning(path)
    import mujoco
    model=mujoco.MjModel.from_xml_string(updated['xml'])
    if model.nbody<2 or not math.isfinite(float(model.body_mass.sum())) or model.body_mass.sum()<=0:raise ValueError('robot must have physical mass')
    updated['summary']=dict(mass=float(model.body_mass.sum()),bodies=model.nbody,joints=model.njnt,geometries=model.ngeom)
    if 'controller' in updated:
        from xrobocon.workshop.generated import validate_design
        validate_design(dict(title=proposal['title'],summary=proposal['summary'],mjcf=updated['xml'],controller_code=updated['controller']['code'],joints=updated['controller']['joints']))
    updated['proposal']=proposal;updated['mode']=mode
    if not updated.get('robot_id'):
        from xrobocon.workshop.identity import new_robot
        updated=new_robot(updated,proposal['title'])
    return updated

class WorkshopLibrary:
    def __init__(self,root):self.root=ensure_storage(root)
    def save(self,snapshot,prompt):
        ensure_storage(self.root);identifier=f'v{time.time_ns()}-{uuid.uuid4().hex[:6]}'
        parent=self.root/'revisions';parent.mkdir(parents=True,exist_ok=True)
        temporary=Path(tempfile.mkdtemp(prefix='.saving-',dir=parent));target=parent/identifier
        try:
            (temporary/'model.xml').write_text(snapshot['xml']);write_json(temporary/'tuning.json',snapshot['tuning'])
            from xrobocon.workshop.generated import write_bundle
            write_bundle(temporary,snapshot)
            manifest={k:deepcopy(v) for k,v in snapshot.items() if k not in ('xml','tuning','revision_id')}
            if not manifest.get('robot_id'):
                from xrobocon.workshop.identity import new_robot
                manifest=new_robot(manifest,snapshot['proposal']['title'])
            manifest.update(id=identifier,parent=snapshot.get('revision_id') or snapshot.get('parent'),prompt=prompt,model_sha256=digest((temporary/'model.xml').read_bytes()),
                tuning_sha256=digest((temporary/'tuning.json').read_bytes()),title=snapshot['proposal']['title'])
            if 'controller' in snapshot:
                manifest['controller_sha256']=digest((temporary/'controller.json').read_bytes())
            if snapshot.get('proposal',{}).get('diagnostic_analysis'):
                write_json(temporary/'diagnostic-analysis.json',snapshot['proposal']['diagnostic_analysis'])
            write_json(temporary/'manifest.json',manifest);temporary.rename(target)
        except BaseException:
            import shutil;shutil.rmtree(temporary);raise
        return target
    def list(self):
        if not self.root.exists():return []
        return sorted((self.root/'revisions').glob('v*/manifest.json'),reverse=True)
    def load(self,directory):
        directory=Path(directory).resolve()
        if directory.parent!=(self.root/'revisions').resolve():raise ValueError('revision must belong to this library')
        manifest=json.loads((directory/'manifest.json').read_text())
        for file,key in [('model.xml','model_sha256'),('tuning.json','tuning_sha256')]:
            if digest((directory/file).read_bytes())!=manifest[key]:raise ValueError('saved revision integrity check failed')
        if manifest.get('controller_sha256'):
            if digest((directory/'controller.json').read_bytes())!=manifest['controller_sha256']:raise ValueError('controller manifest integrity check failed')
            from xrobocon.workshop.generated import load_bundle
            manifest['controller']=load_bundle(directory/'controller.json',directory/'model.xml')
        if not manifest.get('robot_id'):
            ancestor=manifest;visited=set()
            while ancestor.get('parent') and ancestor['parent'] not in visited:
                visited.add(ancestor['parent'])
                prior=(self.root/'revisions'/str(ancestor['parent'])/'manifest.json').resolve()
                if not prior.is_relative_to((self.root/'revisions').resolve()) or not prior.is_file():break
                ancestor=json.loads(prior.read_text())
            manifest['robot_id']='robot_legacy_'+ancestor['id'].replace('-','_')
            manifest['robot_name']=ancestor.get('robot_name',ancestor.get('title','保存機体'))
        from xrobocon.workshop.constraints import restore_creation_prompt
        restore_creation_prompt(manifest,self.root/'revisions')
        evaluations=sorted((directory/'evaluations').glob('*.json'))
        references=list(manifest.get('diagnostic_runs',[]))
        for path in evaluations:
            evaluation=json.loads(path.read_text())
            if evaluation.get('model_sha256')!=manifest['model_sha256']:continue
            manifest['diagnostic_result']=evaluation['result']
            if evaluation.get('run_manifest') and evaluation['run_manifest'] not in references:references.append(evaluation['run_manifest'])
        manifest['diagnostic_runs']=references
        return dict(manifest,xml=(directory/'model.xml').read_text(),tuning=load_tuning(directory/'tuning.json'),revision_id=manifest['id'])
