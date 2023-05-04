import unittest
from point import Point
from error_calculator import ErrorCalculator
from distance_calculator import DistanceCalculator
from controller import Controller
from pure_pursuit import PurePursuit

class TestPurePursuit(unittest.TestCase):

    def setUp(self):
        # Define some test points
        self.points = [Point(0,0), Point(0,5), Point(5,5), Point(10,5), Point(10,0)]
        self.distance_calculator = DistanceCalculator()
        self.error_calculator = ErrorCalculator()
        self.controller = Controller(1.0)

    def test_find_nearest_point(self):
        # Test finding the nearest point to a given position
        pp = PurePursuit(1.0, self.points, self.distance_calculator, self.error_calculator, self.controller)
        pos = Point(2,2)
        idx = pp.find_nearest_point(pos)
        self.assertEqual(idx, 1)  # The nearest point is (0,5)

    def test_find_goal_point(self):
        # Test finding the goal point for a given position
        pp = PurePursuit(1.0, self.points, self.distance_calculator, self.error_calculator, self.controller)
        pos = Point(2,2)
        nearest_idx = pp.find_nearest_point(pos)
        goal_point = pp.find_goal_point(pos, nearest_idx)
        self.assertEqual(goal_point, self.points[1])  # The goal point should be (0,5)

    def test_update(self):
        # Test updating the pure pursuit controller with a given position
        pp = PurePursuit(1.0, self.points, self.distance_calculator, self.error_calculator, self.controller)
        pos = Point(2,2)
        pp.update(pos)
        # TODO: Add assertions for the controller output or other expected behavior

    def test_distance_calculator(self):
        # Test the distance calculator
        p1 = Point(0,0)
        p2 = Point(3,4)
        d = self.distance_calculator.calculate_distance(p1, p2)
        self.assertAlmostEqual(d, 5.0)

    def test_error_calculator(self):
        # Test the error calculator
        p1 = Point(0,0)
        p2 = Point(3,4)
        e = self.error_calculator.calculate_error(p1, p2)
        self.assertAlmostEqual(e, 5.0)

    def test_controller(self):
        # Test the controller
        c = Controller(1.0)
        out = c.minimize(5.0)
        self.assertAlmostEqual(out, -1.0)

if __name__ == '__main__':
    import rostest
    rostest.rosrun('controls', 'test_pure_pursuit', TestPurePursuit)

