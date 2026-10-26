import os

# 1. MPS Fallback Setting
# This must be set before torch is imported (which happens in stable_baselines3)
if 'PYTORCH_ENABLE_MPS_FALLBACK' not in os.environ:
    os.environ['PYTORCH_ENABLE_MPS_FALLBACK'] = '1'

import genesis as gs
from stable_baselines3 import PPO

def setup_genesis(backend=None, **kwargs):
    """Initialize once; select cpu/gpu explicitly or via XROBOCON_BACKEND.

    Backend cannot be switched inside an initialized process. Unexpected
    initialization failures propagate rather than leaving an unusable scene.
    """
    if backend is None:
        backend = os.environ.get('XROBOCON_BACKEND', 'gpu')
    if isinstance(backend, str):
        if backend not in ('cpu', 'gpu'):
            raise ValueError('backend must be cpu or gpu')
        backend = getattr(gs, backend)
    if getattr(gs, '_initialized', False):
        actual = getattr(gs, 'backend', None)
        if backend == gs.cpu and actual != gs.cpu:
            raise RuntimeError('Genesis already initialized on another backend; start a new process')
        if backend == gs.gpu and actual == gs.cpu:
            raise RuntimeError('Genesis already initialized on CPU; start a new process')
        return
    gs.init(backend=backend, **kwargs)

def load_trained_model(model_path, env):
    """
    Load a trained PPO model safely.
    """
    # SB3 automatically adds .zip, so we should check for that too
    if not os.path.exists(model_path):
        if os.path.exists(model_path + ".zip"):
            model_path += ".zip"
        else:
            raise FileNotFoundError(f"Model file not found: {model_path} (or {model_path}.zip)")
    
    print(f"Loading model from: {model_path}")
    model = PPO.load(model_path, env=env)
    return model
