import math
from typing import Any, Optional

import pybullet as p

BOUNDS = tuple[float, float]
XY = tuple[float, float]


class Robot2R:
    CONFIG = tuple[float, float]

    def __init__(self,
                 link_lengths: tuple[float,
                                     float],
                 bounds: Optional[tuple[BOUNDS,
                                        BOUNDS]] = None) -> None:
                                        self.l1 = link_lengths[0]
                                        self.l2 = link_lengths[1]

    def forward_kinematics(self, configuration: CONFIG) -> XY:
        l1, l2= self.l1, self.l2
        theta1, theta2 = configuration[0], configuration[1]
         
        x = l1*math.cos(theta1) + l2*math.cos(theta1 + theta2)
        y = l1*math.sin(theta1) + l2*math.sin(theta1 + theta2)

        return (x,y)

    def inverse_kinematics(self, end_effector_position: XY) -> CONFIG:
        # a2 and a1 are angles
        # arccosine is always positive
        x, y = end_effector_position[0], end_effector_position[1]
        l1, l2= self.l1, self.l2

        # Be careful about 0 division errors
        a2 = math.acos((x**2 + y**2 - l1**2 - l2**2) / (2*l1*l2))
        if x != 0:
            a1 = math.atan(y/x) - math.atan((l2*math.sin(a2)) / (l1 + l2 * math.cos(a2)))
        else: 
            a1 = math.pi/2 - math.atan((l2*math.sin(a2)) / (l1 + l2 * math.cos(a2)))

        # configuration: CONFIG = (a1,a2)
        return (a1, a2)


class Robot3R:
    CONFIG = tuple[float, float, float]

    def __init__(self,
                 link_lengths: tuple[float, float, float],
                 bounds: Optional[tuple[BOUNDS, BOUNDS, BOUNDS]] = None) -> None:
        ...

    def forward_kinematics(self, configuration: CONFIG) -> XY:
        # Get x,y
        l1, l2,l3  = self.l1, self.l2, self.l3
        theta1, theta2 , theta3 = configuration[0], configuration[1], configuration[2]
        
        x = l1 * math.cos(theta1) +  l2 * math.cos(theta1 + theta2) + l3 * math.cos(theta1 + theta2 + theta3)
        y = l1 * math.sin(theta1) +  l2 * math.sin(theta1 + theta2) + l3 * math.sin(theta1 + theta2 + theta3)

        return (x,y)

    def inverse_kinematics(self, end_effector_position: XY) -> CONFIG:
        # Set first position to be same direction as XY
        x,y = end_effector_position[0], end_effector_position[1]
        theta1 = math.atan(y/x)
        
        # Get new pose of end effector ere



class RobotIiwa:
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

    def get_states(self) -> Any:
        return p.getJointStates(self.id, self.__joint_ids)

    def get_positions(self) -> list[float]:
        return [state[0] for state in self.get_states()]

    def get_velocities(self) -> list[float]:
        return [state[1] for state in self.get_states()]
