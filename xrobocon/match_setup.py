"""Match choices and config compilation, independent of UI and simulation."""
from copy import deepcopy
from pathlib import Path
from dataclasses import dataclass
import json
from xrobocon.agent_factory import make_agent
from xrobocon.robot_configs import get_robot_config

@dataclass(frozen=True)
class MatchCatalog:
    path: Path
    data: dict
    @classmethod
    def load(cls,path):
        path=Path(path).resolve();data=json.loads(path.read_text())
        if data.get('schema_version')!=1:raise ValueError('unsupported match catalog')
        for category in ('robots','controls','strategies'):
            if not data.get(category):raise ValueError('nonempty match choices required')
        catalog=cls(path,data);build_match_config(catalog,catalog.defaults());return catalog
    def defaults(self):return deepcopy(self.data['defaults'])
    def options(self,category):return self.data[category]

def build_match_config(catalog,choices):
    base=catalog.path.parent/catalog.data['base_config'];config=json.loads(base.read_text())
    names=config['competition']['participants']
    if set(choices)!=set(names):raise ValueError('choices must match competition participants')
    config['simulation']['scenario']=str((base.parent/config['simulation']['scenario']).resolve())
    models={};agents={}
    for name,choice in choices.items():
        if set(choice)!={'robot','control','strategy'}:raise ValueError('robot/control/strategy required')
        for field,category in (('robot','robots'),('control','controls'),('strategy','strategies')):
            if choice[field] not in catalog.data[category]:raise ValueError(f'unknown {field}: {choice[field]}')
        robot=catalog.data['robots'][choice['robot']]
        if choice['control'] not in robot['controls'] or choice['strategy'] not in robot['strategies']:raise ValueError('incompatible robot/control/strategy')
        get_robot_config(robot['robot_type'])
        asset=(catalog.path.parent/robot['robot_asset']).resolve()
        if not asset.is_file():raise ValueError('robot asset missing')
        models[name]=dict(robot_type=robot['robot_type'],robot_asset=str(asset))
        if robot.get('drive_kind')=='program':
            from xrobocon.workshop.generated import load_bundle
            controller_file=(catalog.path.parent/robot['controller_file']).resolve()
            load_bundle(controller_file,asset)
            models[name].update(drive_kind='program',controller_file=str(controller_file))
        strategy=catalog.data['strategies'][choice['strategy']]
        agents[name]=deepcopy(strategy['agents'][name] if 'agents' in strategy else strategy['agent'])
        agents[name]['control_profile']=choice['control']
        if robot.get('tuning_file'):
            from xrobocon.robot_tuning import load_tuning,verify_asset_binding,resolve_locomotion
            tuning=load_tuning(catalog.path.parent/robot['tuning_file'])
            verify_asset_binding(tuning,asset)
            models[name]['drive_parameters']=deepcopy(tuning['drive'])
            limits=deepcopy(agents[name].get('motion_limits',{}))
            limits.update(tuning['motion'].get(choice['control'],{}))
            agents[name]['motion_limits']=limits
            if tuning.get('locomotion'):agents[name]['locomotion']=resolve_locomotion(tuning,config['simulation']['scenario'])
        if agents[name]['kind'] in ('codex_tier','strategic_tier'):
            agents[name]['game_rules']=deepcopy(config['competition'])
            agents[name]['game_rules']['participant']=name
        make_agent(agents[name])
    config['simulation']['robot_models']=models;config['agents']=agents
    config['match_selection']=deepcopy(choices)
    return config

def cycle_choice(catalog,choices,name,field,delta):
    """Cycle UI choices; a new robot starts with its validated recommendation."""
    categories={'robot':'robots','control':'controls','strategy':'strategies'}
    category=categories[field];allowed=list(catalog.options(category))
    if field!='robot':allowed=[key for key in allowed if key in catalog.options('robots')[choices[name]['robot']][category]]
    current=choices[name][field];choices[name][field]=allowed[(allowed.index(current)+delta)%len(allowed)]
    if field=='robot':
        robot=catalog.options('robots')[choices[name]['robot']]
        recommended=robot.get('recommended_control')
        if recommended in robot['controls']:choices[name]['control']=recommended
        for key,options in (('control','controls'),('strategy','strategies')):
            if choices[name][key] not in robot[options]:choices[name][key]=robot[options][0]
