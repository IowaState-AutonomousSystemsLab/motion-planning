import math
from .point import Point

class DistanceCalcualtor:
    """
    Interface for calculating the distance between two points
    """

    def calculate_distance(point1: Point, point2: Point):
        raise NotImplementedError()

class EuclideanDistanceCalculator(DistanceCalcualtor):
    """
    Calculates the euclidean distance between two points
    """

    def calculate_distance(point1: Point, point2: Point):
        x2 = (point1.x - point2.x)**2
        y2 = (point1.y - point2.y)**2
        return math.sqrt(x2 + y2)