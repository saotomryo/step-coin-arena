"""New-topology bundles and a bounded, pure Python control expression language.

Generated code is inspectable Python, but evaluated as an AST, never exec/import.
No I/O, attribute access, loops, recursion, or dynamic subscripts.
Up to 64 single-assignment numeric locals are evaluated afresh on each call.
"""
import ast
import math
import operator
import re
from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET
from xrobocon.workshop.design import canonical_xml, digest

MAX_AST_NODES=6000
MAX_LOCALS=64
FUNCTIONS = {'sqrt':math.sqrt,'acos':math.acos,'asin':math.asin,'atan2':math.atan2,'sin':math.sin,'cos':math.cos,'abs':abs,'min':min,'max':max,
             'clip':lambda value,low,high:max(low,min(high,value))}
BINARY = {ast.Add:operator.add,ast.Sub:operator.sub,ast.Mult:operator.mul,
          ast.Div:operator.truediv,ast.Mod:operator.mod}
COMPARE = {ast.Lt:operator.lt,ast.LtE:operator.le,ast.Gt:operator.gt,
           ast.GtE:operator.ge,ast.Eq:operator.eq,ast.NotEq:operator.ne}
SCALARS = {'t','dt','linear','angular','lift','frame_target','climb_stage'}

class ControlProgram:
    def __init__(self, code, joints):
        if not isinstance(code,str) or not 1 <= len(code) <= 24000:
            raise ValueError('controller code must be 1..24000 characters')
        self.names = [j['name'] for j in joints]
        try:
            tree=ast.parse(code)
        except (SyntaxError,RecursionError) as exc:
            raise ValueError('invalid controller syntax') from exc
        nodes=list(ast.walk(tree))
        if len(nodes)>MAX_AST_NODES:raise ValueError(f'controller AST is too large: {len(nodes)} nodes > {MAX_AST_NODES}')
        if len(tree.body)!=1 or not isinstance(tree.body[0],ast.FunctionDef):raise ValueError('only def control(s) is allowed')
        func=tree.body[0]
        if func.name!='control' or func.decorator_list or func.returns or func.type_params or len(func.args.args)!=1 or func.args.args[0].arg!='s' or func.args.args[0].annotation or func.args.posonlyargs or func.args.kwonlyargs or func.args.defaults or func.args.vararg or func.args.kwarg:
            raise ValueError('expected def control(s) without decorators or defaults')
        if not func.body or not isinstance(func.body[-1],ast.Return) or not isinstance(func.body[-1].value,ast.Dict):
            raise ValueError('control must end with return {joint: target, ...}')
        if len(func.body)-1>MAX_LOCALS:raise ValueError('too many controller locals')
        self.locals={};self.assignments=[]
        for statement in func.body[:-1]:
            if not isinstance(statement,ast.Assign) or len(statement.targets)!=1 or not isinstance(statement.targets[0],ast.Name):raise ValueError('only numeric local assignments before return are allowed')
            name=statement.targets[0].id
            if not re.fullmatch('[a-zA-Z][a-zA-Z0-9_]{0,63}',name) or name=='s' or name in FUNCTIONS or name in self.locals:raise ValueError('invalid or repeated local name')
            self._check(statement.value,0)
            self.locals[name]=True;self.assignments.append((name,statement.value))
        self.output=func.body[-1].value
        keys=[k.value if isinstance(k,ast.Constant) else None for k in self.output.keys]
        if len(keys)!=len(set(keys)) or set(keys)!=set(self.names):raise ValueError('controller outputs must exactly match driven joints')
        for node in self.output.values:self._check(node,0)

    def _check(self,node,depth):
        if depth>32:raise ValueError('controller expression is too deep')
        children=[]
        if isinstance(node,ast.Name) and node.id in self.locals:return
        if isinstance(node,ast.Constant):
            if type(node.value) not in (int,float) or not math.isfinite(node.value) or abs(node.value)>10000:raise ValueError('invalid numeric constant')
        elif isinstance(node,ast.Subscript):
            if not isinstance(node.slice,ast.Constant) or not isinstance(node.slice.value,str):raise ValueError('literal state key required')
            key=node.slice.value
            if isinstance(node.value,ast.Name) and node.value.id=='s' and key in SCALARS:return
            if isinstance(node.value,ast.Subscript) and isinstance(node.value.value,ast.Name) and node.value.value.id=='s' and isinstance(node.value.slice,ast.Constant) and node.value.slice.value in ('q','qd') and key in self.names:return
            raise ValueError('unsupported state access')
        elif isinstance(node,ast.BinOp) and type(node.op) in BINARY:children=[node.left,node.right]
        elif isinstance(node,ast.UnaryOp) and isinstance(node.op,(ast.UAdd,ast.USub)):children=[node.operand]
        elif isinstance(node,ast.Call) and isinstance(node.func,ast.Name) and node.func.id in FUNCTIONS and not node.keywords and 1<=len(node.args)<=4:children=node.args
        elif isinstance(node,ast.IfExp):children=[node.test,node.body,node.orelse]
        elif isinstance(node,ast.Compare) and len(node.ops)==1 and type(node.ops[0]) in COMPARE:children=[node.left,node.comparators[0]]
        else:raise ValueError('unsupported controller expression: '+type(node).__name__)
        for child in children:self._check(child,depth+1)

    def evaluate(self,state):
        if any(type(state.get(k)) not in (int,float) or not math.isfinite(state[k]) for k in SCALARS):raise ValueError('finite controller state required')
        local_values={}
        def read(node):
            if isinstance(node,ast.Name):return local_values[node.id]
            if isinstance(node,ast.Constant):return node.value
            if isinstance(node,ast.Subscript):
                if isinstance(node.value,ast.Name):return state[node.slice.value]
                return state[node.value.slice.value][node.slice.value]
            if isinstance(node,ast.BinOp):value=BINARY[type(node.op)](read(node.left),read(node.right))
            elif isinstance(node,ast.UnaryOp):value=read(node.operand)*(-1 if isinstance(node.op,ast.USub) else 1)
            elif isinstance(node,ast.Call):value=FUNCTIONS[node.func.id](*(read(a) for a in node.args))
            elif isinstance(node,ast.Compare):return COMPARE[type(node.ops[0])](read(node.left),read(node.comparators[0]))
            elif isinstance(node,ast.IfExp):value=read(node.body if read(node.test) else node.orelse)
            else:raise ValueError('invalid controller AST')
            if not math.isfinite(value) or abs(value)>1e12:raise ValueError('controller arithmetic overflow')
            return value
        try:
            for name,expression in self.assignments:local_values[name]=read(expression)
            result={k.value:float(read(v)) for k,v in zip(self.output.keys,self.output.values)}
        except (ArithmeticError,KeyError,TypeError) as exc:raise ValueError('controller evaluation failed: '+str(exc)) from exc
        if not all(math.isfinite(v) and abs(v)<=1e6 for v in result.values()):raise ValueError('invalid controller output')
        return result


def validate_design(proposal):
    fields={'title','summary','mjcf','controller_code','joints'}
    if not isinstance(proposal,dict) or set(proposal)!=fields:raise ValueError('invalid new-robot proposal fields')
    for field,maximum in [('title',80),('summary',3000),('mjcf',80000)]:
        if not isinstance(proposal[field],str) or not 1<=len(proposal[field].strip())<=maximum:raise ValueError('invalid '+field)
    text=proposal['mjcf']
    if '<!' in text:raise ValueError('XML declarations/entities are not supported')
    try:root=ET.fromstring(text)
    except ET.ParseError as exc:raise ValueError('invalid MJCF XML') from exc
    allowed={'mujoco','compiler','option','worldbody','body','freejoint','joint','geom','inertial','default','motor','actuator','contact','exclude'}
    if root.tag!='mujoco' or any(e.tag not in allowed or any(k in e.attrib for k in ('file','mesh','hfield','plugin','texture','material')) for e in root.iter()):raise ValueError('only self-contained primitive MJCF without files/plugins is supported')
    elements=list(root.iter())
    if len(elements)>350 or len(list(root.iter('joint')))>32 or len(list(root.iter('geom')))>100:raise ValueError('robot complexity limit exceeded')
    world=root.find('worldbody')
    if world is None or len(world)!=1 or world[0].tag!='body' or world[0].get('name')!='base_link':raise ValueError('one floating base_link body is required')
    if len(list(root.iter('freejoint')))!=1 or world[0].find('freejoint') is None:raise ValueError('base_link needs exactly one freejoint')
    if any(g.get('type','sphere') not in ('box','sphere','cylinder','capsule','ellipsoid') for g in root.iter('geom')):raise ValueError('unsupported collision geometry')
    # Compiler cannot run includes/plugins; remaining numbers are checked by MuJoCo.
    import mujoco
    import numpy as np
    model=mujoco.MjModel.from_xml_string(canonical_xml(text))
    if not 0 < float(model.body_mass.sum()) <= 100 or model.nbody<2:raise ValueError('invalid robot mass')
    if model.njnt>33 or model.ngeom>100:raise ValueError('compiled robot complexity exceeded')
    data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    if not np.isfinite(data.qpos).all() or np.max(np.abs(data.geom_xpos))>2 or np.max(model.geom_size)>1:raise ValueError('robot dimensions exceed 2m workspace')
    joints=proposal['joints']
    if not isinstance(joints,list) or not 1<=len(joints)<=32:raise ValueError('1..32 driven joints required')
    seen=set()
    for spec in joints:
        if not isinstance(spec,dict) or set(spec)!={'name','mode','kp','kd','torque_limit','target_limit'}:raise ValueError('invalid joint drive fields')
        name=spec['name']
        if not isinstance(name,str) or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]{0,79}',name) or name in seen:raise ValueError('invalid or duplicate joint name')
        seen.add(name)
        index=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,name)
        if index<0 or int(model.jnt_type[index]) not in (int(mujoco.mjtJoint.mjJNT_HINGE),int(mujoco.mjtJoint.mjJNT_SLIDE)):raise ValueError('driven joint must be a named hinge/slide: '+name)
        if spec['mode'] not in ('position','velocity'):raise ValueError('joint mode must be position or velocity')
        for key,limit in [('kp',200),('kd',50),('torque_limit',30),('target_limit',100)]:
            value=spec[key]
            if type(value) not in (float,int) or not math.isfinite(value) or not 0<value<=limit:raise ValueError('invalid joint '+key)
    program=ControlProgram(proposal['controller_code'],joints)
    zero={name:0. for name in seen}
    for t in (0.,.5,1.,2.):
        for linear,angular,lift in [(0.,0.,0.),(.12,0.,0.),(0.,.3,0.),(.05,0.,1.)]:
            program.evaluate(dict(t=t,dt=.01,linear=linear,angular=angular,lift=lift,frame_target=0.,climb_stage=0. if lift else -1.,q=zero,qd=zero))
    return model


def apply_design(snapshot,proposal):
    model=validate_design(proposal)
    result=deepcopy(snapshot) if snapshot else {}
    result.pop('diagnostic_result',None)
    tuning=deepcopy(result.get('tuning',dict(schema_version=1,drive={},motion={})))
    tuning.pop('calibration',None);tuning.pop('locomotion',None)
    result.update(xml=canonical_xml(proposal['mjcf']),tuning=tuning,
        controller=dict(schema_version=1,code=proposal['controller_code'],joints=deepcopy(proposal['joints'])),
        entry=dict(label=proposal['title'],description=proposal['summary'],robot_type='standard',drive_kind='program',
                   controls=['baseline','safe','fast'],strategies=['adaptive','fixed','codex_target','codex_plan']),
        source_robot=result.get('source_robot','generated'),source_asset_sha256=result.get('source_asset_sha256'),
        parent=result.get('parent'),proposal=deepcopy(proposal),mode='new_robot',
        summary=dict(mass=float(model.body_mass.sum()),bodies=model.nbody,joints=model.njnt,geometries=model.ngeom))
    if not result.get('robot_id'):
        from xrobocon.workshop.identity import new_robot
        result=new_robot(result,proposal['title'])
    return result


def load_bundle(path,asset=None):
    import json
    path=Path(path)
    bundle=json.loads(path.read_text())
    if bundle.get('schema_version')!=1:raise ValueError('unsupported controller bundle')
    code=(path.parent/'controller.py').read_text()
    if digest(code.encode())!=bundle['code_sha256']:raise ValueError('controller integrity check failed')
    if asset and digest(Path(asset).read_bytes())!=bundle['model_sha256']:raise ValueError('controller belongs to a different robot model')
    proposal=dict(title='検証',summary='保存済み制御の再検証',mjcf=Path(asset or path.parent/'model.xml').read_text(),controller_code=code,joints=bundle['joints'])
    validate_design(proposal)
    return dict(schema_version=1,code=code,joints=bundle['joints'])


def write_bundle(directory,snapshot):
    from xrobocon.workshop.design import write_json
    if 'controller' not in snapshot:return
    directory=Path(directory);controller=snapshot['controller']
    (directory/'controller.py').write_text(controller['code'])
    write_json(directory/'controller.json',dict(schema_version=1,joints=controller['joints'],
        code_sha256=digest(controller['code'].encode()),model_sha256=digest(snapshot['xml'].encode())))
