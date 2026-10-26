import os
import sys
import threading
import time

# Ensure root directory is in sys.path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from xrobocon.env import XRoboconEnv
from xrobocon.step_env import XRoboconStepEnv
import xrobocon.common as common

class GUIProgressCallback(BaseCallback):
    """Callback to report progress to GUI."""
    def __init__(self, update_callback=None, verbose=0):
        super().__init__(verbose)
        self.update_callback = update_callback
        self.episode_rewards = []
        self.episode_lengths = []
        self.current_episode_reward = 0
        self.current_episode_length = 0
        
    def _on_step(self) -> bool:
        self.current_episode_reward += self.locals['rewards'][0]
        self.current_episode_length += 1
        
        if self.locals['dones'][0]:
            self.episode_rewards.append(self.current_episode_reward)
            self.episode_lengths.append(self.current_episode_length)
            
            # Report progress
            if self.update_callback:
                avg_reward = sum(self.episode_rewards[-10:]) / min(len(self.episode_rewards), 10)
                self.update_callback({
                    'steps': self.num_timesteps,
                    'episodes': len(self.episode_rewards),
                    'last_reward': self.current_episode_reward,
                    'avg_reward': avg_reward
                })
            
            self.current_episode_reward = 0
            self.current_episode_length = 0
        
        return True

class TrainingRunner:
    def __init__(self):
        self.model = None
        self.env = None
        self.is_training = False
        self.thread = None

    def start_training(self, config, progress_callback=None, finished_callback=None):
        """
        Start training in a separate thread.
        config: dict with keys 'steps', 'base_model', 'env_type', 'robot_type', 'save_name'
        """
        if self.is_training:
            print("Training already in progress.")
            return

        self.is_training = True
        self.thread = threading.Thread(
            target=self._train_thread,
            args=(config, progress_callback, finished_callback)
        )
        self.thread.daemon = True
        self.thread.start()

    def _train_thread(self, config, progress_callback, finished_callback):
        try:
            steps = config.get('steps', 10000)
            base_model = config.get('base_model', None)
            env_type = config.get('env_type', 'flat')
            robot_type = config.get('robot_type', 'tristar')
            save_name = config.get('save_name', 'trained_model')
            
            # Setup Environment
            if env_type == 'step':
                self.env = XRoboconStepEnv(render_mode=None, robot_type=robot_type)
            else:
                self.env = XRoboconEnv(render_mode=None, robot_type=robot_type)
            
            # Setup Model
            if base_model and os.path.exists(base_model):
                print(f"Loading base model: {base_model}")
                self.model = common.load_trained_model(base_model, self.env)
                self.model.learning_rate = 0.0001 # Fine-tuning LR
                reset_timesteps = False
            else:
                print("Creating new model")
                self.model = PPO("MlpPolicy", self.env, verbose=1)
                reset_timesteps = True
            
            # Train
            callback = GUIProgressCallback(update_callback=progress_callback)
            self.model.learn(
                total_timesteps=steps,
                callback=callback,
                reset_num_timesteps=reset_timesteps
            )
            
            # Save
            self.model.save(save_name)
            print(f"Model saved to {save_name}.zip")
            
            if finished_callback:
                finished_callback(True, f"Training finished. Saved to {save_name}.zip")
                
        except Exception as e:
            print(f"Training error: {e}")
            if finished_callback:
                finished_callback(False, str(e))
        finally:
            self.is_training = False
            if self.env:
                self.env.close()

    def stop_training(self):
        # PPO doesn't have an easy stop method from outside without a custom callback check
        # For now, we just let it finish or rely on process termination if it was a process.
        # Since it's a thread, we can't force kill it easily safely.
        # We could add a 'stop' flag to the callback.
        pass
