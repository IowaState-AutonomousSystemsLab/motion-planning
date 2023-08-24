from packages.controls.include.purepursuit.pure_pursuit import PurePursuit
from packages.controls.include.purepursuit.point import Point
from packages.controls.include.purepursuit.distance_calculator import EuclideanDistanceCalcualtor
from packages.controls.include.purepursuit.error_calculator import ErrorCalculator

import matplotlib.pyplot as plt

p = PurePursuit(1, 
                [Point(0,1), Point(1,1), Point(2,1), Point(3,1), Point(4,1)], 
                EuclideanDistanceCalcualtor(),

                )
