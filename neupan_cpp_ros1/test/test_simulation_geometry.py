"""Offline checks for the simulator's independent sensors and collision geometry."""
import importlib.util
import math
from pathlib import Path
import unittest

module = Path(__file__).resolve().parents[1]/'scripts/simulation_test.py'
spec = importlib.util.spec_from_file_location('simulation', module)
sim = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sim)


class SimulationGeometry(unittest.TestCase):
    def test_ray_circle(self):
        self.assertAlmostEqual(sim.ray_range((0, 0), (1, 0), [(2, 0, .5)], [], 5), 1.5)
        self.assertTrue(math.isinf(sim.ray_range((0, 0), (-1, 0), [(2, 0, .5)], [], 5)))

    def test_ray_wall_and_range_limit(self):
        wall = [(2, -1, 2, 1)]
        self.assertAlmostEqual(sim.ray_range((0, 0), (1, 0), [], wall, 5), 2)
        self.assertTrue(math.isinf(sim.ray_range((0, 0), (1, 0), [], wall, 1)))

    def test_rotated_offset_sensor(self):
        x, y, yaw = sim.sensor_pose([1, 2, math.pi/2], [.2, .1, -.3])
        self.assertAlmostEqual(x, .9)
        self.assertAlmostEqual(y, 2.2)
        self.assertAlmostEqual(yaw, math.pi/2-.3)

    def test_footprint_not_just_centre(self):
        scene = dict(circles=[(1.2, 0, .3)], walls=[])
        self.assertAlmostEqual(sim.clearance([0, 0, 0], scene, 2, 1), -.1)
        self.assertAlmostEqual(sim.clearance([0, 0, math.pi/2], scene, 2, 1), .4)

    def test_wall_crosses_rectangle_without_endpoint_inside(self):
        scene = dict(circles=[], walls=[(-3, 0, 3, 0)])
        self.assertEqual(sim.clearance([0, 0, .3], scene, 2, 1), 0)
        scene['walls'] = [(2, -5, 2, 5)]
        self.assertAlmostEqual(sim.clearance([0, 0, 0], scene, 2, 1), 1)

    def test_arc_motion(self):
        x, y, yaw = sim.integrate([0, 0, 0], 1, 1, math.pi/2)
        self.assertAlmostEqual(x, 1)
        self.assertAlmostEqual(y, 1)
        self.assertAlmostEqual(yaw, math.pi/2)


if __name__ == '__main__':
    unittest.main()
