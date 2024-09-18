from packages.controls.include.purepursuit.pure_pursuit import PurePursuit
from packages.controls.include.purepursuit.point import Point
from packages.controls.include.purepursuit.distance_calculator import EuclideanDistanceCalculator
from packages.controls.include.purepursuit.error_calculator import ErrorCalculator
from packages.controls.include.purepursuit.controller import DuckiebotController

import matplotlib.pyplot as plt

distance_calculator = EuclideanDistanceCalculator()
error_calculator = ErrorCalculator()
controller = DuckiebotController()

p = PurePursuit(1, 
                [Point(0,1), Point(1,1), Point(2,1), Point(3,1), Point(4,1)], 
                distance_calculator,
                error_calculator,
                controller
                )
