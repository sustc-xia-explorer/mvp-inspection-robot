"""Independent geometric checks for the interactive world's global paths."""
import importlib.util
import math
from pathlib import Path
import unittest

root = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('rviz_sim', root/'scripts/rviz_sim.py')
sim = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sim)


class RVizWorld(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.world = sim.World(root/'config/scenario_corridor.yaml', root/'config/fw_mini_sim.yaml')

    def test_global_path_clears_full_body_at_multiple_orientations(self):
        world = self.world
        path = world.route(world.config['initial_pose'], world.config['default_goal'])
        self.assertGreater(len(path), 2)
        self.assertEqual(tuple(path[-1]), tuple(world.config['default_goal']))
        for a, b in zip(path, path[1:]):
            for fraction in (0, .25, .5, .75, 1):
                x, y = (a[k]+(b[k]-a[k])*fraction for k in (0, 1))
                for yaw in (0, math.pi/4, math.pi/2, -math.pi/3):
                    self.assertGreater(world.clearance([x, y, yaw]), .03)

    def test_exact_ros2_map_is_not_silently_widened(self):
        original = sim.World(root/'config/scenario_corridor_original.yaml', root/'config/fw_mini_sim.yaml')
        self.assertTrue(any(abs(y-1) < 1e-9 for _, y, _, _, _ in original.rectangles))
        with self.assertRaisesRegex(ValueError, 'No route'):
            original.route(original.config['initial_pose'], original.config['default_goal'])

    def test_reject_goal_in_obstacle(self):
        with self.assertRaises(ValueError):
            self.world.route(self.world.config['initial_pose'], (-3, 0))

    def test_scan_and_solid_obstacle_interior(self):
        self.assertLessEqual(self.world.clearance([-3, 0, 0]), 0)
        ranges, points = self.world.scan(self.world.config['initial_pose'])
        self.assertEqual(len(ranges), self.world.config['sensor']['beams'])
        self.assertTrue(points)
        for distance in ranges:
            self.assertTrue(math.isnan(distance) or distance >= self.world.config['sensor']['range_min'])


class FixedHeadingPlanning(unittest.TestCase):
    def test_clearance_margin_for_fixed_heading_omni(self):
        import tempfile
        import yaml
        with tempfile.TemporaryDirectory() as directory:
            settings=yaml.safe_load((root/'config/scenario_corridor.yaml').read_text())
            settings['map_file']=str((root/'config/maps/corridor_fw_mini.yaml').resolve())
            settings['omni_global_planner']=dict(footprint='fixed_heading',safety_margin=.16,
                clearance_weight=6.,preferred_clearance=.28,corner_cut=.5,path_spacing=.04)
            file=Path(directory)/'scenario.yaml';file.write_text(yaml.safe_dump(settings))
            world=sim.World(file,root/'config/fw_mini_omni.yaml')
            path=world.route(settings['initial_pose'],settings['default_goal'])
            for point in path:
                self.assertGreater(world.clearance([*point,0.]),.16)
            backward=world.route([*settings['default_goal'],0.],settings['initial_pose'][:2])
            self.assertEqual(tuple(backward[-1]),tuple(settings['initial_pose'][:2]))


class ReferenceFixtures(unittest.TestCase):
    def test_hard_corner_is_preserved_and_unsafe_fixture_is_rejected(self):
        import tempfile
        import yaml
        with tempfile.TemporaryDirectory() as directory:
            settings=yaml.safe_load((root/'config/scenario_corridor.yaml').read_text())
            settings['map_file']=str((root/'config/maps/corridor_fw_mini.yaml').resolve())
            settings['initial_pose']=[0.,-.7,0.];settings['default_goal']=[.6,-.1]
            settings['test_reference_path']=[[0.,-.7],[.6,-.7],[.6,-.1]]
            file=Path(directory)/'scenario.yaml';file.write_text(yaml.safe_dump(settings))
            world=sim.World(file,root/'config/fw_mini_omni.yaml')
            path=world.route(settings['initial_pose'],settings['default_goal'])
            self.assertEqual(path, [tuple(p) for p in settings['test_reference_path']])
            world.config['test_reference_path']=[[0.,-.7],[-.5,.4],[.6,-.1]]
            with self.assertRaisesRegex(ValueError,'Unsafe test reference'):
                world.route(settings['initial_pose'],settings['default_goal'])


if __name__ == '__main__':
    unittest.main()
