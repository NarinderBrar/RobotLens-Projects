from pathlib import Path
import shutil
import tempfile
import unittest
import xml.etree.ElementTree as ET

from asset_paths import robot_description


class AssetPathTests(unittest.TestCase):
    def test_fixture_resources_are_portable_and_present(self):
        fixture = Path(__file__).resolve().parent
        root = ET.parse(fixture / "pr2.urdf").getroot()
        resources = root.findall(".//mesh[@filename]") + root.findall(".//texture[@filename]")
        self.assertTrue(resources)
        for element in resources:
            value = element.get("filename")
            self.assertFalse(Path(value).is_absolute(), value)
            self.assertNotIn("://", value)
            self.assertTrue((fixture / value).is_file(), value)
        ET.fromstring(robot_description(fixture / "pr2.urdf"))

    def test_resources_resolve_after_moving_fixture(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            original = root / "old drive" / "fixture"
            original.mkdir(parents=True)
            (original / "shape.stl").write_bytes(b"mesh")
            (original / "texture.png").write_bytes(b"texture")
            urdf = '<robot><mesh filename="shape.stl"/><texture filename="texture.png"/></robot>'
            (original / "robot.urdf").write_text(urdf)
            moved = root / "new drive" / "fixture"
            moved.parent.mkdir()
            shutil.move(original, moved)

            description = ET.fromstring(robot_description(moved / "robot.urdf"))

            self.assertEqual(description.find("mesh").get("filename"), (moved / "shape.stl").as_uri())
            self.assertEqual(description.find("texture").get("filename"), (moved / "texture.png").as_uri())
            self.assertEqual((moved / "robot.urdf").read_text(), urdf)

    def test_missing_resource_is_reported_before_launch(self):
        with tempfile.TemporaryDirectory() as directory:
            urdf = Path(directory) / "robot.urdf"
            urdf.write_text('<robot><mesh filename="missing.stl"/></robot>')
            with self.assertRaisesRegex(FileNotFoundError, "missing.stl"):
                robot_description(urdf)


if __name__ == "__main__":
    unittest.main()
