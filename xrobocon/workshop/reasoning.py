"""Bounded mechanism facts and staged diagnostic output for robot improvement."""

STAGES=('observation','mechanism','command_mapping','gait_and_support','feedback_and_limits','next_action')

def mechanism_context(snapshot):
    """Kinematics at the model reference pose, never a live contact measurement."""
    import mujoco
    from xrobocon.workshop.design import canonical_xml
    model=mujoco.MjModel.from_xml_string(canonical_xml(snapshot['xml']))
    data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    base=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'base_link')
    joints=[]
    for i in range(model.njnt):
        if model.jnt_type[i] == mujoco.mjtJoint.mjJNT_FREE:continue
        joints.append(dict(name=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_JOINT,i),
            type='hinge' if model.jnt_type[i]==mujoco.mjtJoint.mjJNT_HINGE else 'slide',
            axis_world_at_reference=data.xaxis[i].tolist(),anchor_world_at_reference=data.xanchor[i].tolist(),
            range=model.jnt_range[i].tolist() if model.jnt_limited[i] else None))
    geometry=[dict(name=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,i),
        body=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_BODY,int(model.geom_bodyid[i])),
        centre_world_at_reference=data.geom_xpos[i].tolist()) for i in range(model.ngeom)]
    program='controller' in snapshot
    return dict(scope='Reference-pose kinematics only, not observed contact, dynamic balance, or stability proof.',
        mass_kg=float(model.body_mass.sum()),com_world_at_reference=data.subtree_com[base].tolist() if base>=0 else None,
        joints=joints,geometry_centres_not_contact_points=geometry,
        drive_contract=dict(kind='generated_joint_program' if program else 'existing_drive_adapter',
            inputs=['t','dt','linear','angular','lift','frame_target','climb_stage','q','qd'] if program else [],
            body_pose_and_contact_in_program=False if program else None,
            persistent_program_state=False if program else None,
            numeric_local_assignments_max=64 if program else None,
            ast_nodes_max=6000 if program else None,
            inverse_kinematics_functions=['sqrt','acos','asin','atan2'] if program else [],
            units='linear=m/s, angular=rad/s; position targets=rad or m, velocity targets=rad/s or m/s'))


def validate_analysis(value):
    """Validate traceability, not the truth of a model's causal hypothesis."""
    from urllib.parse import urlsplit
    if not isinstance(value,dict):raise ValueError('diagnostic_analysis must be an object')
    stages=value.get('stages',[])
    if not isinstance(stages,list) or any(not isinstance(s,dict) for s in stages):raise ValueError('invalid diagnostic stages')
    if [s.get('stage') for s in stages] != list(STAGES):raise ValueError('diagnostic stages must be ordered')
    for s in stages:
        if s.get('status') not in ('assessed','needs_data','not_applicable'):raise ValueError('invalid diagnostic status')
        for key in ('facts','hypothesis','next_check'):
            if not isinstance(s.get(key),str) or len(s[key])>2000:raise ValueError('invalid diagnostic detail')
    refs=value.get('references',[])
    if not isinstance(refs,list) or len(refs)>4:raise ValueError('at most four references')
    for ref in refs:
        if not isinstance(ref,dict):raise ValueError('invalid reference')
        url=urlsplit(ref.get('url',''))
        if url.scheme!='https' or not url.netloc or url.username or url.password:raise ValueError('reference must be public HTTPS URL')
        if ref.get('status') not in ('consulted','unverified'):raise ValueError('invalid reference status')
        for key in ('title','relevance','differences'):
            if not isinstance(ref.get(key),str) or len(ref[key])>1500:raise ValueError('invalid reference detail')
    queries=value.get('research_queries',[])
    if not isinstance(queries,list) or len(queries)>3 or any(not isinstance(q,str) or not 1<=len(q)<=300 for q in queries):raise ValueError('invalid research queries')


def step_improvement_context(snapshot):
    """Execution facts for step-specific repair; no invented sensor observations."""
    program='controller' in snapshot
    runs=(snapshot.get('diagnostic_context') or {}).get('runs',[])
    relevant=[r for r in runs if r.get('suite') in ('steps','all')]
    return dict(
        separation='Separate flat/steps fixtures and upper-agent phases, but shared model and drive. Not fully independent controllers.',
        active_drive='ProgramDrive.control(s)' if program else 'existing wheel/steering/frame drive',
        mode_switch=dict(lift='1 when climb_stage is provided, otherwise 0',climb_stage='0/1/2 while climbing, -1 outside climb',
            warning='lift=1 includes approach to a step; it is not measured foot contact or edge proximity.'),
        generated_limits=('Same function for flat and climbing: branch explicitly on lift/climb_stage. No body pose, edge distance or contact input; t is drive time and may reset. '
            'ProgramDrive uses controller.joints kp/kd/torque_limit/target_limit and controller code; tuning.drive wheel/steering/frame gains do not tune generated joints.') if program else None,
        fixture=dict(start_x=-6.1,step_target_x=[-4.,-2.7,-1.4],target_heights=[.1,.35,.6],time_limit_seconds=120,
            goals='three sequential target coins, not direct proof of each foot landing',
            guarded_step_controller_used=False),
        evidence_caveat='Runner returns commands and can respawn after physics. Post-tick telemetry may combine a reset state with targets from before reset. Do not infer runtime corruption from one such sample.',
        step_runs=[dict(run_id=r.get('run_id'),matches_current=r.get('matches_current'),result=r.get('result')) for r in relevant[:3]],
        first_goal='Classify failure before approach / at first edge / front support transfer / rear clearance / upper-level stabilization; do not optimize all three steps at once.')
