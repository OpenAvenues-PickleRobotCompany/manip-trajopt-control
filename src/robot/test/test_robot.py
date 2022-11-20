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


def create_robot_wbounds():
    link_lengths = (60.0, 40.0)
    bounds = ((-2 * pi, 2 * pi), (-3 / 4 * pi, 3 / 4 * pi))
    test_robot = Robot2R(link_lengths, bounds)
    return test_robot


def test_forward_kinematics():
    robot = create_robot()

    # Edge cases
    assert approx(robot.forward_kinematics((0, 0))) == (90, 0)
    assert approx(robot.forward_kinematics((pi / 2, 0))) == (0, 90)
    assert approx(robot.forward_kinematics((-pi / 2, 0))) == (0, -90)
    assert approx(robot.forward_kinematics((pi / 6, 0))) == (math.sqrt(3) * 45, 45)

    r = np.linspace(10, 89.99, num=100)
    t = np.linspace(-pi, pi, 100)
    for theta in t:
        for radius in r:
            (x, y) = (radius * math.cos(theta), radius * math.sin(theta))
            assert approx(
                robot.forward_kinematics(robot.inverse_kinematics((x, y)))
            ) == (x, y)


def test_inverse_kinematics():
    robot = create_robot()

    # Edge cases
    assert approx(robot.inverse_kinematics((90, 0))) == (0, 0)
    assert approx(robot.inverse_kinematics((0, 90))) == (pi / 2, 0)
    assert approx(robot.inverse_kinematics((0, -90))) == (-pi / 2, 0)
    assert approx(robot.inverse_kinematics((math.sqrt(3) * 45, 45))) == (pi / 6, 0)

    a1_ub = pi - math.tan(
        4 / 3
    )  # Restriction added to prevent: (x,y) in Q3 while a2 > 0
    a1 = random.uniform(low=-pi, high=a1_ub, size=(100,))
    a2 = random.uniform(low=0.0, high=pi, size=(100,))

    for angle1 in a1:
        for angle2 in a2:
            config = (angle1, angle2)
            assert (
                approx(robot.inverse_kinematics(robot.forward_kinematics(config)))
                == config
            )


def test_daniel():
    ## Testing ##
    link_lengths = (30, 20)
    bounds = (-2 * math.pi, 2 * math.pi)

    testclass = Robot2R(link_lengths)

    configuration_1 = (math.pi, math.pi / 2)
    test_1 = testclass.forward_kinematics(configuration_1)
    assert approx(test_1) == (-30, -20)

    test_2 = testclass.inverse_kinematics(testclass.forward_kinematics(configuration_1))
    test_2 = np.abs(test_2)
    assert approx(test_2) == configuration_1

    configuration_3 = (-math.pi / 2, 0)
    test_3 = testclass.forward_kinematics(configuration_3)
    assert approx(test_3) == (0, -50)


def test_bounds():
    test_robot = create_robot_wbounds()

    # Forward Kinematics
    configuration1 = (pi / 3, pi)
    test1 = test_robot.forward_kinematics(configuration1)
    assert test1 == None

    configuration2 = (3 * pi, 0)
    assert test_robot.forward_kinematics(configuration2) == None

    # Inverse Kinematics
    xRef_OutsidePositionalSpace = (110, 0)
    assert test_robot.inverse_kinematics(xRef_OutsidePositionalSpace) == None

    xRef_InsidePositionalSpace = (19, 0)
    assert test_robot.inverse_kinematics(xRef_InsidePositionalSpace) == None
