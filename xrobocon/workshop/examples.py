"""Deterministic starting example for topology/control generation and diagnostics."""
def lifting_robot():
    wheels=[];joints=[];expressions=[]
    for side,y in [('left',.23),('right',-.23)]:
        for end,x in [('front',.18),('rear',-.18)]:
            name=f'{side}_{end}_joint'
            wheels.append(f'''<body name="{side}_{end}_wheel" pos="{x} {y} -.06">
              <joint name="{name}" type="hinge" axis="0 1 0" damping=".02"/>
              <geom type="cylinder" size=".12 .035" euler="1.57079632679 0 0" mass=".3" friction="1 .005 .001" rgba=".12 .13 .15 1"/>
            </body>''')
            joints.append(dict(name=name,mode='velocity',kp=.5,kd=.001,torque_limit=2.,target_limit=12.))
            expressions.append(f"'{name}': (s['linear'] - s['angular'] * {y}) / .12")
    joints.append(dict(name='lift_joint',mode='position',kp=12.,kd=1.,torque_limit=4.,target_limit=1.))
    expressions.append("'lift_joint': .6 * s['lift']")
    return dict(title='新規4輪・能動リフト試作機',summary='四輪差動駆動と前方の能動リフト。登坂指令中にリフトを展開する試作。段差性能は診断で確認してください。',
        mjcf='''<mujoco model="wheel_lift"><compiler angle="radian"/><worldbody><body name="base_link"><freejoint/>
        <geom type="box" size=".24 .16 .055" mass="3" rgba=".85 .5 .12 1"/>
        '''+''.join(wheels)+'''<body name="lift_arm" pos=".24 0 -.02"><joint name="lift_joint" type="hinge" axis="0 1 0" range="-.1 1" damping=".3"/>
        <geom type="capsule" fromto="0 0 0 .25 0 -.03" size=".025" mass=".25" rgba=".15 .55 .8 1"/>
        </body></body></worldbody></mujoco>''',
        controller_code='def control(s):\n    return {\n        '+',\n        '.join(expressions)+'\n    }\n',joints=joints)
