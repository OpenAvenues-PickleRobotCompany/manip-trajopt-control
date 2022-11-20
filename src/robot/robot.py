import math
from typing import Any, Optional

import numpy as np
import pybullet as p

PI = math.pi
BOUNDS = tuple[float, float]
XY = tuple[float, float]


class Robot2R:
    """Implements Forward and Inverse Kinematics of 2R Robot"""

    CONFIG = tuple[float, float]

    def __init__(
        self,
        link_lengths: tuple[float, float],
        bounds: Optional[tuple[BOUNDS, BOUNDS]] = None,
    ) -> None:
        if link_lengths[0] > link_lengths[1] and link_lengths[1] > 0:
            self.l1 = link_lengths[0]
            self.l2 = link_lengths[1]
        else:
            print("Error: Invalid link_lengths")
            self.l1, self.l2 = None, None

        if bounds != None:
            (self.lb1, self.ub1) = bounds[0][0], bounds[0][1]
            (self.lb2, self.ub2) = bounds[1][0], bounds[1][1]
        else:
            self.lb1, self.ub1, self.lb2, self.ub2 = None, None, None, None

    def forward_kinematics(self, configuration: CONFIG) -> XY:
        l1, l2 = self.l1, self.l2
        a1, a2 = configuration[0], configuration[1]

        if not self.__within_bounds(a1, a2):
            print("Error: Impossible configuration.")
            return

        l1 = self.l1
        l2 = self.l2

        x = l1 * math.cos(a1) + l2 * math.cos(a1 + a2)
        y = l1 * math.sin(a1) + l2 * math.sin(a1 + a2)

        return (x, y)

    def inverse_kinematics(self, end_effector_position: XY) -> CONFIG:
        x, y = end_effector_position[0], end_effector_position[1]
        l1, l2 = self.l1, self.l2

        cos_a2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)

        if math.fabs(cos_a2) > 1:
            print(f"Error: Position {XY} is not within workspace")
            return

        elif math.fabs(cos_a2) == 1:
            a1 = math.atan2(y, x)
            a2 = math.acos(cos_a2)

            if self.__within_bounds(a1, a2):
                return (a1, a2)
            else:
                print(f"Error: Position {XY} is not within bounds")
                return

        else:
            aXY = [math.atan2(y, x)] * 2  # angles towards reference
            a2 = [math.acos(cos_a2), -math.acos(cos_a2)]  # elbow up or down
            a1 = aXY - np.arctan2(l2 * np.sin(a2), l1 + l2 * np.cos(a2))

            config = []

            for i in range(2):
                if self.__within_bounds(a1[i], a2[i]):
                    config.append((a1[i], a2[i]))

            if config:
                return config[0]  # Returns elbow-up solution first if valid
            else:
                print(f"Error: Position {XY} is not within bounds")
                return

    def __within_bounds(self, a1, a2):
        """Return whether congfiguration is within bounds"""
        within_bounds = True

        if self.lb1 and self.lb2 and self.ub1 and self.ub2:
            lb1, lb2, ub1, ub2 = self.lb1, self.lb2, self.ub1, self.ub2
            if not (lb1 <= a1 <= ub1 and lb2 <= a2 <= ub2):
                within_bounds = False

        return within_bounds


class Robot3R:
    CONFIG = tuple[float, float, float]

    def __init__(
        self,
        link_lengths: tuple[float, float, float],
        bounds: Optional[tuple[BOUNDS, BOUNDS, BOUNDS]] = None,
    ) -> None:
        raise NotImplementedError("3R Robot not implemented")
        ...

    def forward_kinematics(self, configuration: CONFIG) -> XY:
        ...

    def inverse_kinematics(self, end_effector_position: XY) -> CONFIG:
        ...


class RobotIiwa:
    def __init__(
        self,
        robot_urdf: str,
        initial_pos: list[float],
        initial_orientation: list[float],
    ) -> None:
        self.id = p.loadURDF(robot_urdf, initial_pos, initial_orientation)

        self.__control_mode = p.POSITION_CONTROL
        self.__joint_ids = list(range(p.getNumJoints(self.id)))

    def send_command(self, command: list[float]) -> None:
        assert len(command) == len(self.__joint_ids)
        p.setJointMotorControlArray(
            self.id, self.__joint_ids, self.__control_mode, targetPositions=command
        )

    def get_states(self) -> Any:
        return p.getJointStates(self.id, self.__joint_ids)

    def get_positions(self) -> list[float]:
        return [state[0] for state in self.get_states()]

    def get_velocities(self) -> list[float]:
        return [state[1] for state in self.get_states()]
