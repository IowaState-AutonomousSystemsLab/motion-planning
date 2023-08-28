import math

import numpy as np

from typing import Tuple

#how close to zero should equal zero
sigma = 0.0001

"""
Class implements forward kinematics for differential drive
"""
class DifferentialDriveKinematics():

    def __init__(self,
                 robot_width):
        self.width = robot_width

    def calculate_radius(self,
                         vel_left: float,
                         vel_right: float) -> float:
        """
        Calculates the radius of the turn

        Arguments:
            vel_left: the velocity of the left wheel
            vel_right: the velocity of the right wheel

        Returns:
            The radius of the turn. Returns math.inf of the ratio between
            the left and right wheel speeds is close to 1.
        """
        ratio = vel_left / vel_right

        #as the robot gets closer to moving straight the radius approaches 
        #infinity and rest of the math has a divide by 0 error. 
        if np.abs(ratio - 1) < sigma:
            return math.inf

        R = (0.5 * self.width * (1 + ratio)) / (1 - ratio)
        return R

    def calculate_angular_velocity(self,
                                   vel_left,
                                   vel_right):
        """
        Calculate the angular velocity of a turn

        Arguments:
            length: the distance between the wheels
            raidus: the radius of the turn
            vel_right: the velocity of the right wheel

        Returns:
            The angular velocity of the turn in radians/sec
        """
        R = self.calculate_radius(vel_left, vel_right)
        return vel_right / (R + 0.5 * self.width)
    
    def calculate_wheel_speeds_from_angular_velocity(self, vel_left, vel_right, angular_velocity:float):
        """
            if angular velocity is positive, it's a counter-clockwise/left turn, right wheel is faster
        """
        offset = self.width * angular_velocity / 2

        
        if angular_velocity > sigma:
            return (vel_left - offset, vel_right + offset)
        elif angular_velocity < -1 * sigma:
            return (vel_left + offset, vel_right - offset)
        elif abs(angular_velocity) <= sigma:
            return (0.5, 0.5)
        else:
            print("THIS CONDITION SHOULD NEVER BE REACHED")



    def calculate_ICC(self,
                      vel_left: float,
                      vel_right: float,
                      heading: float,
                      x: float, 
                      y: float) -> Tuple[float, float]:
        """
        Calculates the Instantaneous Center of Curvature 

        Arguments:
            radius: the radius of the turn
            heading: the heading of the robot in radians
            x: the x coordinate of the robot
            y: the y coordinate of the robot

        Returns:
            A tuple in the form of (ICCx, ICCy)
        """
        radius = self.calculate_radius(vel_left, vel_right)
        ICCx = x - radius * np.sin(heading)
        ICCy = y + radius * np.cos(heading)
        return ICCx, ICCy

    def calculate_new_pose(self,
                           x: float,
                           y: float,
                           heading: float,
                           vel_left: float,
                           vel_right: float,
                           dt: float) -> Tuple[float, float, float]:
        dr = vel_right * dt
        dl = vel_left * dt

        #not moving check
        if np.abs(dr) < sigma or np.abs(dl) < sigma:
            return (x, y, heading)
        
        radius = self.calculate_radius(vel_left, vel_right)

        #moving in an arc
        if radius != math.inf:
            dtheta = (dr - dl) / (self.width)
            ds = (dr + dl) / 2
            
            #if the period of time is small enough we can assume
            #the length of the arc is equal to the straightline distance
            x += ds * np.cos(heading + dtheta / 2)
            y += ds * np.sin(heading + dtheta / 2)
            heading += dtheta
        else: #moving forward
            # they should almost be the same, but avg for further accuracy
            ds = (dl + dr) / 2
            x += dl * np.cos(heading)
            y += dl * np.sin(heading)
        
        return (x, y, heading)