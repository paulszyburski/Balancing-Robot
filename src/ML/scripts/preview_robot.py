import time
from pathlib import Path

import pybullet as p
import pybullet_data


def main():
    model = Path(__file__).resolve().parents[3] / "models" / "balancing_robot.urdf"
    client = p.connect(p.GUI)

    try:
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client)
        p.loadURDF("plane.urdf", physicsClientId=client)
        p.loadURDF(
            str(model),
            basePosition=[0, 0, 0.051],
            useFixedBase=True,
            physicsClientId=client,
        )
        p.resetDebugVisualizerCamera(
            cameraDistance=0.42,
            cameraYaw=45,
            cameraPitch=-25,
            cameraTargetPosition=[0, 0, 0.085],
            physicsClientId=client,
        )

        while p.isConnected(client):
            time.sleep(1 / 60)
    except KeyboardInterrupt:
        pass
    finally:
        if p.isConnected(client):
            p.disconnect(client)


if __name__ == "__main__":
    main()
