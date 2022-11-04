import math

import numpy as np
from numpy import random
from pytest import approx
from src.robot.robot import Robot2R

pi = math.pi


def create_robot():
    link_lengths = (50, 40)
    test_robot = Robot2R(link_lengths)
    return test_robot


def test_forward_kinematics():
    robot = create_robot()

    # Edge cases
    assert approx(robot.forward_kinematics((0, 0))) == (90, 0)
    assert approx(robot.forward_kinematics((pi / 2, 0))) == (0, 90)
    assert approx(robot.forward_kinematics((-pi / 2, 0))) == (0, -90)
    assert approx(robot.forward_kinematics((pi / 6, 0))) == (math.sqrt(3) * 45, 45)

    # Testing using random samples
    positions = random.randint(5, high=80, size=(1000, 2))
    for xy in positions:
        magnitude = np.linalg.norm(xy)
        if 100 <= magnitude**2 <= 8100:
            assert approx(robot.forward_kinematics(robot.inverse_kinematics(xy))) == xy


def test_inverse_kinematics():
    robot = create_robot()

    # Edge cases
    assert approx(robot.inverse_kinematics((90, 0))) == (0, 0)
    assert approx(robot.inverse_kinematics((0, 90))) == (pi / 2, 0)
    assert approx(robot.inverse_kinematics((0, -90))) == (-pi / 2, 0)
    assert approx(robot.inverse_kinematics((math.sqrt(3) * 45, 45))) == (pi / 6, 0)

    # Testing using random samples
    a1 = random.uniform(low=-pi, high=pi, size=(100,))
    a2 = random.uniform(low=0.0, high=3 * pi / 4, size=(100,))
    configurations = np.zeros((100, 2))
    configurations[:, 0] = a1
    configurations[:, 1] = a2
    count = 0

    for i in range(configurations.shape[0]):
        try:
            assert (
                approx(
                    robot.inverse_kinematics(
                        robot.forward_kinematics(configurations[i])
                    )
                )
                == configurations[i]
            )
        except AssertionError:
            count += 1

    assert count / 100 < 0.2
