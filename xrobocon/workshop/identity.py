"""Stable robot identity is separate from immutable revision identity."""
from copy import deepcopy
from uuid import uuid4

def new_robot(snapshot, name):
    if not isinstance(name,str) or not 1<=len(name.strip())<=80:
        raise ValueError('ロボット名は1〜80文字で入力してください')
    result=deepcopy(snapshot)
    origin=dict(robot_id=result.get('robot_id'),revision_id=result.get('revision_id'),source_robot=result.get('source_robot'))
    result.update(robot_id='robot_'+uuid4().hex,robot_name=name.strip(),parent=None)
    result.pop('revision_id',None)
    if any(origin.values()):result['derived_from']=origin
    result.pop('diagnostic_result',None)
    result.pop('diagnostic_runs',None)
    return result

def catalog_id(snapshot, directory):
    # Legacy catalogs remain loadable; newly saved robots use a stable ID.
    if snapshot.get('robot_id'):return 'workshop_'+snapshot['robot_id']
    return 'workshop_'+directory.name.replace('-','_')
