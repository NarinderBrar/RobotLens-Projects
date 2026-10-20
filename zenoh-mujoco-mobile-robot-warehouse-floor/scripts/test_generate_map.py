import math
from pathlib import Path
import tempfile
import unittest

from generate_map import PROJECT, build_map


class GlobalMapTest(unittest.TestCase):
    def sample(self, grid, x, y):
        column = math.floor((x - grid["origin"]["x"]) / grid["resolution"])
        row = math.floor((y - grid["origin"]["y"]) / grid["resolution"])
        return grid["data"][row * grid["width"] + column]

    def build_fixture(self, geometry):
        with tempfile.TemporaryDirectory() as folder:
            world = Path(folder) / "world.xml"
            world.write_text('<mujoco><worldbody><body>'
                             '<geom type="box" size="2 2 .05" pos="0 0 -.05"/>'
                             f'{geometry}</body></worldbody></mujoco>')
            return build_map(world)

    def test_warehouse_bounds_and_obstacles(self):
        grid = build_map(PROJECT / "worlds/warehouse.xml")
        self.assertEqual((grid["width"], grid["height"]), (334, 256))
        self.assertEqual(grid["origin"], {"x": -8.35, "y": -6.4, "yaw": 0})
        self.assertEqual(len(grid["data"]), 334 * 256)
        self.assertEqual(self.sample(grid, -8.25, 0), 100)
        self.assertEqual(self.sample(grid, 8.25, 0), 100)
        self.assertEqual(self.sample(grid, -5.37, -2.2333333), 100)
        for x in (-6.5, -2, 0, 2, 6.5):
            self.assertEqual(self.sample(grid, x, 0), 0)

    def test_nested_rotated_collision_and_overhead_clearance(self):
        grid = self.build_fixture(
            '<body pos=".5 .5 0" quat=".7071067811865476 0 0 .7071067811865476">'
            '<geom type="box" size=".6 .1 .2" pos="0 0 .2"/></body>'
            '<geom type="box" size=".5 .5 .1" pos="-1 -1 2"/>')
        self.assertEqual(self.sample(grid, .5, .9), 100)
        self.assertEqual(self.sample(grid, .9, .5), 0)
        self.assertEqual(self.sample(grid, -1, -1), 0)

    def test_visual_only_geometry_is_excluded(self):
        grid = self.build_fixture(
            '<geom type="box" size="1 1 1" pos="0 0 1" contype="0" conaffinity="0"/>')
        self.assertEqual(self.sample(grid, 0, 0), 0)

    def test_dynamic_and_unsupported_geometry_are_rejected(self):
        for geometry in ('<body><freejoint/></body>',
                         '<geom type="sphere" size=".1"/>',
                         '<geom type="box" size=".1 .1 .1" euler="0 0 1"/>'):
            with self.subTest(geometry=geometry), self.assertRaises(ValueError):
                self.build_fixture(geometry)


if __name__ == "__main__":
    unittest.main()
