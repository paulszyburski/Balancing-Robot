import time
from math import degrees
from pathlib import Path

import pybullet as p
from stable_baselines3 import PPO

from src.ML.env import Enviroment


def main():
    checkpoint = Path(__file__).resolve().parents[3] / "balancing_bot.zip"
    model = PPO.load(checkpoint, device="cpu")
    env = Enviroment(gui=True)

    try:
        p.resetDebugVisualizerCamera(
            cameraDistance=0.65,
            cameraYaw=45,
            cameraPitch=-25,
            cameraTargetPosition=[0, 0, 0.08],
            physicsClientId=env.client,
        )
        while p.isConnected(env.client):
            observation, _ = env.reset()
            print(f"Starting tilt: {degrees(observation[0]):+.1f}°")
            time.sleep(0.5)
            while p.isConnected(env.client):
                action, _ = model.predict(observation, deterministic=True)
                observation, _, terminated, timed_out, info = env.step(action)
                time.sleep(4 * env.dt)
                if terminated or timed_out:
                    seconds = env.steps * 4 * env.dt
                    if info["fell"]:
                        result = "Fell"
                    elif info["drifted"]:
                        result = "Drove outside the balance area"
                    else:
                        result = "Reached the time limit"
                    print(f"{result} after {seconds:.2f} simulated seconds.")
                    time.sleep(1)
                    break
    except KeyboardInterrupt:
        pass
    finally:
        env.close()


if __name__ == "__main__":
    main()
