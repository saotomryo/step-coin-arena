import genesis as gs
import numpy as np
import os
import sys

# Ensure root directory is in sys.path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from xrobocon.env import XRoboconEnv
from xrobocon.step_env import XRoboconStepEnv
import xrobocon.common as common
from stable_baselines3 import PPO

class SimulationRunner:
    def __init__(self, render=True):
        self.env = None
        self.model = None
        self.render = render
        self.running = False
        self.current_obs = None
        self.robot_type = 'tristar'
        self.env_type = 'flat'

    def setup_environment(self, env_type='flat', robot_type='tristar'):
        """Initialize the simulation environment."""
        self.env_type = env_type
        self.robot_type = robot_type
        
        if self.env:
            self.env.close()
        
        # Initialize Genesis if not already done (handled by env or common)
        # common.setup_genesis() # This might be needed if not called elsewhere
        
        if env_type == 'step':
            self.env = XRoboconStepEnv(render_mode="human" if self.render else None, robot_type=robot_type)
        else:
            self.env = XRoboconEnv(render_mode="human" if self.render else None, robot_type=robot_type)
            
        self.current_obs, _ = self.env.reset()
        print(f"Environment setup: {env_type} with {robot_type}")

    def load_model(self, model_path):
        """Load a trained RL model."""
        if not self.env:
            raise ValueError("Environment not set up. Call setup_environment first.")
        
        if not os.path.exists(model_path):
             if os.path.exists(model_path + ".zip"):
                 model_path += ".zip"
             else:
                 raise FileNotFoundError(f"Model file not found: {model_path}")

        # Use CPU for inference to avoid MPS issues on Mac
        self.model = PPO.load(model_path, env=self.env, device='cpu')
        print(f"Model loaded from {model_path}")

    def step(self):
        """Advance the simulation by one step."""
        if not self.env:
            return None
        
        action = None
        if self.model:
            action, _ = self.model.predict(self.current_obs, deterministic=True)
        else:
            # Default action (idle) - 4 dimensions for tristar
            action = np.zeros(4, dtype=np.float32)
            
        self.current_obs, reward, terminated, truncated, info = self.env.step(action)
        
        if terminated or truncated:
            self.current_obs, _ = self.env.reset()
            
        return self.current_obs, reward, terminated, truncated, info

    def reset(self):
        """Reset the environment."""
        if self.env:
            self.current_obs, _ = self.env.reset()

    def close(self):
        """Close the environment."""
        if self.env:
            self.env.close()
            self.env = None
