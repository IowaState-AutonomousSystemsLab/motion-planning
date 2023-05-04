from typing import List

from .point import Point
from .error_calculator import ErrorCalculator
from .distance_calculator import DistanceCalcualtor
from .controller import Controller

class PurePursuit:
    """
    Generalized implemenation of Pure Pursuit
    """

    def __init__(self,
                 look_ahead: float,
                 path: List[Point],
                 distance_calculator: DistanceCalcualtor,
                 error_calculator: ErrorCalculator,
                 controller: Controller):
        """
        Constructor

        Arguments:
            look_ahead: the look ahead distance
            path: the list of points to follow
            distance_calculator: the method for calculating distance between two 
                points
            error_calculator: the method for calcuating the error between
                the robots current pose and a goal point
            controller: A controller that takes in the error and attempts
                to drive it to 0
        """
        self.path = path
        self.look_ahead = look_ahead
        self.distance_calculator = distance_calculator
        self.error_calculator = error_calculator
        self.controller = controller

    def find_nearest_point(self, pos: Point) -> int:
        """
        Finds the nearest point on the path to the robot and returns
        the index of it
        """

        #arbitrary large number
        min_distance = 10000000
        min_idx = 0
        for i, point in enumerate(self.path):
            dist = self.distance_calculator.calculate_distance(pos, self.path[i])

            if dist < min_distance:
                min_distance = dist
                min_idx = i

        return min_idx

    def find_goal_point(self, pos: Point, nearest_idx: int) -> Point:
        
        #for all points ahead and including the nearest one on the path
        for i, point in enumerate(self.path[nearest_idx:]):
            dist = self.distance_calculator.calculate_distance(pos, point)
            
            #sigma for comparing floats
            if dist >= self.look_ahead - 0.00001:
                return point

    def update(self, pos: Point):
        """
        Updates pure pursuit with the latest position of the robot
        """
        nearest_idx = self.find_nearest_point(pos)
        goal_point = self.find_goal_point(pos, nearest_idx)
        error = self.error_calculator.calculate_error(pos, goal_point)
        self.controller.minimize(error)

    