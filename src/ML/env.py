from pathlib import Path

import gymnasium as gym
import numpy as np
import pybullet as p
import pybullet_data

from .controller import DriveController


class Enviroment(gym.Env):
    def __init__(self, gui=True):
        self.client = p.connect(p.GUI if gui else p.DIRECT)
        self.dt = 1 / 240
        self.steps = 0

        self.action_space = gym.spaces.Box(-1.0, 1.0, shape=(1,), dtype=np.float32)
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, shape=(3,), dtype=np.float32)

        p.setTimeStep(self.dt, physicsClientId=self.client)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.client)
        p.setGravity(0, 0, -9.81, physicsClientId=self.client)

        self.floor = p.loadURDF("plane.urdf", physicsClientId=self.client)
        model = Path(__file__).resolve().parents[2] / "models" / "balancing_robot.urdf"
        self.robot = p.loadURDF(
            str(model),
            basePosition=[0, 0, 0.031],
            flags=p.URDF_USE_INERTIA_FROM_FILE,
            physicsClientId=self.client,
        )

        self.controller = DriveController(self.robot, self.client)
        dynamics = p.getDynamicsInfo(self.robot, -1, physicsClientId=self.client)
        self.center_of_mass_offset = np.array(dynamics[3])
        self.initial_state = p.saveState(physicsClientId=self.client)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)

        client = self.client
        p.restoreState(self.initial_state, physicsClientId=client)
        position, _ = p.getBasePositionAndOrientation(self.robot, physicsClientId=client)

        tilt_degrees = self.np_random.uniform(2.0, 15.0)
        tilt_degrees *= self.np_random.choice([-1, 1])
        orientation = p.getQuaternionFromEuler([0, np.deg2rad(tilt_degrees), 0])

        axle_position = np.array(position) - self.center_of_mass_offset
        tilted_offset = p.rotateVector(orientation, self.center_of_mass_offset)
        tilted_position = axle_position + tilted_offset
        p.resetBasePositionAndOrientation(self.robot, tilted_position, orientation, physicsClientId=client)

        chassis_min, _ = p.getAABB(self.robot, -1, physicsClientId=client)
        floor_clearance = 0.001
        if chassis_min[2] < floor_clearance:
            tilted_position[2] += floor_clearance - chassis_min[2]
            p.resetBasePositionAndOrientation(
                self.robot, tilted_position, orientation, physicsClientId=client
            )

        self.steps = 0
        return self._observation(orientation), {}

    def step(self, action):
        speed = 10.0 * float(action[0])
        self.controller.drive(speed)

        chassis_hit_floor = False
        for _ in range(4):
            p.stepSimulation(physicsClientId=self.client)
            if p.getContactPoints(self.robot, self.floor, linkIndexA=-1, physicsClientId=self.client):
                chassis_hit_floor = True
                break

        self.steps += 1
        position, orientation = p.getBasePositionAndOrientation(self.robot, physicsClientId=self.client)
        observation = self._observation(orientation)
        pitch, pitch_rate, _ = observation
        axle_offset = p.rotateVector(orientation, self.center_of_mass_offset)
        axle_x = position[0] - axle_offset[0]
        linear_velocity, _ = p.getBaseVelocity(self.robot, physicsClientId=self.client)
        forward_speed = linear_velocity[0]

        fell = chassis_hit_floor or abs(pitch) > np.deg2rad(35.0)
        drifted = abs(axle_x) > 0.25
        terminated = bool(fell or drifted)
        timed_out = self.steps >= 600 and not terminated
        balance_reward = (
            1.0
            - abs(pitch)
            - 0.03 * abs(pitch_rate)
            - 2.0 * abs(axle_x)
            - 0.25 * abs(forward_speed)
        )
        reward = -1.0 if terminated else balance_reward
        info = {"fell": bool(fell), "drifted": bool(drifted)}

        return observation, float(reward), terminated, timed_out, info

    def _observation(self, orientation):
        client = self.client
        _, angular = p.getBaseVelocity(self.robot, physicsClientId=client)
        left_joint = self.controller.wheel_joints[0]
        _, left_wheel_speed, _, _ = p.getJointState(self.robot, left_joint, physicsClientId=client)

        pitch = p.getEulerFromQuaternion(orientation)[1]
        # Project the world angular velocity onto the body's Y axis, like the IMU.
        rotation = np.array(p.getMatrixFromQuaternion(orientation)).reshape(3, 3)
        pitch_rate = rotation[:, 1] @ angular
        return np.array([pitch, pitch_rate, left_wheel_speed], dtype=np.float32)

    def close(self):
        if p.isConnected(self.client):
            p.disconnect(self.client)
