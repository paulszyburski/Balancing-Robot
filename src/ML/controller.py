import pybullet as p


class DriveController:

    def __init__(self, robot, client, max_torque=0.2):
        self.robot = robot
        self.client = client
        self.max_torque = max_torque
        joints = {
            p.getJointInfo(robot, index, physicsClientId=client)[1].decode(): index
            for index in range(p.getNumJoints(robot, physicsClientId=client))
        }
        self.wheel_joints = [joints["left_wheel_joint"], joints["right_wheel_joint"]]

    def drive(self, speed):
        for joint in self.wheel_joints:
            p.setJointMotorControl2(
                bodyUniqueId=self.robot,
                jointIndex=joint,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=speed,
                force=self.max_torque,
                physicsClientId=self.client,
            )
