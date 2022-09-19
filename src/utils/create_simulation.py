from typing import Any

import pybullet as p
import pybullet_data

PHYSICS_CLIENT = Any


class Robot:
    def __init__(
            self,
            robot_urdf: str,
            initial_pos: list[float],
            initial_orientation: list[float]) -> None:
        self.id = p.loadURDF(robot_urdf, initial_pos, initial_orientation)

        self.__control_mode = p.POSITION_CONTROL
        self.__joint_ids = list(range(p.getNumJoints(self.id)))

    def send_command(self, command: list[float]) -> None:
        assert len(command) == len(self.__joint_ids)
        p.setJointMotorControlArray(
            self.id,
            self.__joint_ids,
            self.__control_mode,
            targetPositions=command)

    def get_states(self) -> list[Any]:
        return [p.getJointStates(self.id, self.__joint_ids)]

    def get_positions(self) -> list[float]:
        return [state[0] for state in self.get_states()]

    def get_velocities(self) -> list[float]:
        return [state[1] for state in self.get_states()]


def create_simulation() -> tuple[PHYSICS_CLIENT, Robot]:
    physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
    p.setGravity(0, 0, -9.81)
    startPos = [0., 0, 0]
    startOrientation = p.getQuaternionFromEuler([0, 0, 0])
    robot = Robot("kuka_iiwa/model.urdf", startPos, startOrientation)
    return physicsClient, robot
