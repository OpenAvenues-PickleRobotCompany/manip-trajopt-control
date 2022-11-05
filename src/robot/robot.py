import math
from typing import Any, Optional

import pybullet as p

PI = math.pi
CONFIG = tuple[float, float]
BOUNDS = tuple[float, float]
XY = tuple[float, float]


def __init__(
    self,
    link_lengths: tuple[float, float],
    bounds: Optional[tuple[BOUNDS, BOUNDS]] = None,
) -> None:
    self.l1 = link_lengths[0]
    self.l2 = link_lengths[1]

    if bounds:
        (self.lb1, self.ub1) = bounds[0][0], bounds[0][1]
        (self.lb2, self.ub2) = bounds[1][0], bounds[1][1]
    else:
        self.lb1, self.ub1, self.lb2, self.ub2 = None, None, None, None


def forward_kinematics(self, configuration: CONFIG) -> XY:
    theta1 = configuration[0]
    theta2 = configuration[1]

    l1 = self.l1
    l2 = self.l2

    x = l1 * math.cos(theta1) + l2 * math.cos(theta1 + theta2)  
    y = l1 * math.sin(theta1) + l2 * math.sin(theta1 + theta2)

    return (x, y)


def inverse_kinematics(self, end_effector_position: XY) -> CONFIG:
    x, y = end_effector_position[0], end_effector_position[1]
    l1, l2 = self.l1, self.l2

    fraction = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)

    if -1 < fraction > 1:  # Out of working space
        print("No IK solution found")
        return

    elif fraction == 1:  # Both joints aligned
        return (math.atan2(y, x), 0)

    ### First solution
    a2 = math.acos(fraction)
    if x != 0:
        a1 = math.atan2(y, x) - math.atan2(l2 * math.sin(a2), l1 + l2 * math.cos(a2))
    solution1 = (a1, a2)

    ### Second solution
    second_a2 = -a2
    if x != 0:
        second_a1 = math.atan(y / x) - math.atan(
            (l2 * math.sin(second_a2)) / (l1 + l2 * math.cos(second_a2))
        )
    solution2 = (second_a1, second_a2)

    if self.lb1:
        lb1, lb2, ub1, ub2 = self.lb1, self.lb2, self.ub1, self.ub2
        ### Checking for solutions
        if lb1 <= a1 <= ub1 and lb2 <= a2 <= ub2:
            if lb1 <= second_a1 <= ub1 and lb2 <= second_a2 <= ub2:
                print("Two solutions found:", solution1, solution2)
            return solution1

        elif lb1 <= second_a1 <= ub1 and lb2 <= second_a2 <= ub2:
            return solution2

        else:
            print("No IK solution found")
            return
    else:
        return solution1


class Robot3R:
    CONFIG = tuple[float, float, float]

    def __init__(
        self,
        link_lengths: tuple[float, float, float],
        bounds: Optional[tuple[BOUNDS, BOUNDS, BOUNDS]] = None,
    ) -> None:
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
