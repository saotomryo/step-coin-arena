"""Narrow workaround for Genesis comparing principal inertia to link-frame inertia.

Only suppress a warning when both numbers match the authored model after the
inertial-frame rotation. Genuine mismatches and all other warnings pass through.
"""
import logging,re

class InertiaFrameFilter(logging.Filter):
    def __init__(self,expected):
        super().__init__();self.expected=expected;self.corrected=0
    def filter(self,record):
        import numpy as np
        match=re.fullmatch(r"Link '([^']+)' has dubious inertia \[([^]]+)\] compared to the estimate from geometry \[([^]]+)\]\.",record.getMessage())
        if not match or match[1] not in self.expected:return True
        try:
            principal=[float(x.split('=')[1]) for x in match[2].split(',')]
            geometry=[float(x.split('=')[1]) for x in match[3].split(',')]
            expected_principal,rotated=self.expected[match[1]]
            equivalent=np.allclose(principal,expected_principal,rtol=.001,atol=1e-10) and np.allclose(geometry,rotated,rtol=.001,atol=1e-10)
        except (ValueError,IndexError):return True
        if equivalent:self.corrected+=1;return False
        return True

def model_inertia_frames(path):
    import mujoco,numpy as np
    model=mujoco.MjModel.from_xml_path(path);result={}
    for name in ('left_rocker','left_bogie','right_rocker','right_bogie'):
        i=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,name)
        if i<0:continue
        rotation=np.empty(9);mujoco.mju_quat2Mat(rotation,model.body_iquat[i]);rotation=rotation.reshape(3,3)
        principal=model.body_inertia[i]
        result[name]=(principal.copy(),np.diag(rotation@np.diag(principal)@rotation.T))
    return result
