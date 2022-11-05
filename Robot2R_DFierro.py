import math
from typing import Any, Optional

import pybullet as p

BOUNDS = tuple[float , float ]
XY = tuple[float , float]


class Robot2R:
    CONFIG = tuple[float, float]

    def __init__(self,
                 link_lengths: tuple[float,
                                     float],
                 bounds: Optional[tuple[BOUNDS,
                                        BOUNDS]] = None) -> None:
                                      self.linklength = link_lengths[0]
                                      self.linklength2 = link_lengths[1]   

    def forward_kinematics(self, configuration: CONFIG) -> XY:
        theta1 = configuration[0]
        theta2 = configuration[1]
        l1 = self.linklength
        l2 = self.linklength2

        global x 
        x= l1*math.cos(theta1) + l2*math.cos(theta1+theta2) 
        global y 
        y= l1*math.sin(theta1) + l2*math.sin(theta1+theta2) 

        return(x,y) 



    def inverse_kinematics(self, end_effector_position: XY) -> CONFIG:
        enf_effec_posx = end_effector_position[0]
        end_effec_posy = end_effector_position[1] 
        l1 = self.linklength
        l2 = self.linklength2
        
        theta_2 = math.acos((l1**2 + l2**2 - x**2 - y**2)/2*l1*l2)
        theta_1 = math.atan(y/x) - math.atan((l2*math.sin(theta_2))/l1+l2*math.cos(theta_2))

        return(theta_1,theta_2) 


## Testing ## 
link_lengths = (30,20)
bounds = (-2*math.pi, 2*math.pi) 

testclass = Robot2R(link_lengths,bounds)

configuration_1 = (math.pi,math.pi/2) 
test_1 = testclass.forward_kinematics(configuration_1) 
print(test_1) 

configuration_2 = (-math.pi/2,0) 
test_2 = testclass.forward_kinematics(configuration_2) 
print(test_2) 

test_3 = testclass.inverse_kinematics(testclass.forward_kinematics(configuration_1)) 
print(test_3)
 