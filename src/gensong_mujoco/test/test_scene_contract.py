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
        self.assertEqual(box.attrib["pos"], "0.8 0 0.81")
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

    def test_joint_states_are_decoupled_from_camera_rendering(self):
        source = NODE.read_text()
        self.assertIn('self.declare_parameter("joint_state_publish_hz", 500.0)', source)
        self.assertIn('self.publish_state()', source)
        self.assertNotIn('self.state_timer = self.create_timer(', source)
        self.assertIn('self.camera_thread = threading.Thread(', source)
        self.assertIn('mujoco.mj_copyData(', source)
        self.assertNotIn('joint_state_publish_interval', source)

    def test_scene_box_uses_primitive_collision_walls_for_thin_mesh(self):
        root = ET.parse(SCENE).getroot()
        box = root.find('.//body[@name="scene_box"]')
        collision_geoms = box.findall('./geom[@name]')
        names = {geom.attrib["name"] for geom in collision_geoms if geom.attrib["name"].startswith("scene_box_collision_")}

        self.assertEqual(
            names,
            {
                "scene_box_collision_bottom",
                "scene_box_collision_left",
                "scene_box_collision_right",
                "scene_box_collision_front",
                "scene_box_collision_back",
            },
        )
        self.assertIsNone(box.find('./geom[@name="scene_box_collision"]'))


if __name__ == "__main__":
    unittest.main()
