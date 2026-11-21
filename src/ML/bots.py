from stable_baselines3 import PPO

from .env import Enviroment


class Ml:
    def __init__(self, env=None):
        self.env = env if env is not None else Enviroment(gui=False)
        self.model = PPO("MlpPolicy", self.env, device="cpu", verbose=1)

    def train(self, total_timesteps=100_000):
        return self.model.learn(total_timesteps=total_timesteps)


class DeterministicBot():
    def __init__(self):
        super().__init__()
