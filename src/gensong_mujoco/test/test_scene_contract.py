import ast
import unittest
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).parents[2]
SCENE = ROOT / "ocs2_robotic_assets/resources/gensong_board/mjcf/gensong_scene.xml"
NODE = ROOT / "gensong_mujoco/gensong_mujoco/gensong_mujoco_node.py"


class SceneContractTest(unittest.TestCase):
    def test_scene_contains_objects_and_rgbd_cameras(self):
        root = ET.parse(SCENE).getroot()
        self.assertIsNotNone(root.find('.//body[@name="scene_box"]'))
        self.assertIsNotNone(root.find('.//body[@name="scene_table"]'))
        self.assertIsNotNone(root.find('.//camera[@name="gensong_rgb_camera"]'))
        self.assertIsNotNone(root.find('.//camera[@name="gensong_depth_camera"]'))
        base = root.find('.//body[@name="base_link"]')
        self.assertEqual(base.attrib["pos"], "0 0 0")
        box = root.find('.//body[@name="scene_box"]')
        self.assertEqual(box.attrib["pos"], "1.0 0 0.81")
        self.assertEqual(box.attrib["euler"], "1.5708 0 0")
        table = root.find('.//body[@name="scene_table"]')
        self.assertEqual(table.attrib["pos"], "1.5 -1.0 0")
        self.assertEqual(table.attrib["euler"], "0 1.5708 1.5708")

    def test_ros_bridge_declares_joint_and_velocity_interfaces(self):
        source = NODE.read_text()
        ast.parse(source)
        for topic in ("/gensong/joint_command", "/gensong/joint_states", "/cmd_vel"):
            self.assertIn(topic, source)
        self.assertIn("rgb8", source)
        self.assertIn("32FC1", source)
        self.assertIn('os.environ.setdefault("MUJOCO_GL", "egl")', source)


if __name__ == "__main__":
    unittest.main()
