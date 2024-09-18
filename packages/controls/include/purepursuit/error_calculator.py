from typing import List
from .point import Point

import numpy as np

class ErrorCalculator:
    """
    Interface for an error calculator that calculates the error between the current_point and goal point 
    for the pure pursuit algorithm
    """
    def calculate_error(current_point: Point, goal_point: Point):
        raise NotImplementedError()

class HeadingToPointErrorCalculator(ErrorCalculator):
    """
    Calculates the error in radians between the current heading and the heading needed to go
    straight towards the goal point. Positive means counter-clockwise, negative means clockwise
    """

    def calculate_error(current_point: Point, goal_point: Point):
        dx = goal_point.x - current_point.x
        dy = goal_point.y - current_point.y
        
        vector_to_goal = np.array((dx, dy))
        current_vector = np.array((current_point.x + np.cos(current_point.heading), 
                          current_point.y + np.sin(current_point.heading)))

        #angle between two vectors using dot product
        dot = np.dot(vector_to_goal, current_vector)
        mag_goal = np.linalg.norm(vector_to_goal)
        mag_current = np.linalg.norm(current_vector)
        error = dot / (mag_goal * mag_current)

        return error

