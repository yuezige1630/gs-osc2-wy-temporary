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
        self.assertEqual(box.attrib["pos"], "1.0 0 0.91")
        self.assertNotIn("euler", box.attrib)
        for cam_name in ("gensong_rgb_camera", "gensong_depth_camera"):
            cam = root.find('.//camera[@name="%s"]' % cam_name)
            self.assertIsNotNone(cam)
            self.assertNotIn("fovy", cam.attrib)
            self.assertEqual(cam.attrib["focalpixel"], "785.77 785.19")
            self.assertEqual(cam.attrib["principalpixel"], "2.89 9.74")
            self.assertEqual(cam.attrib["resolution"], "1280 800")
            self.assertEqual(cam.attrib["sensorsize"], "1.6 1")
        table = root.find('.//body[@name="scene_table"]')
        self.assertEqual(table.attrib["pos"], "1.5 -1.0 0.1")
        self.assertEqual(table.attrib["euler"], "0 1.5708 1.5708")

    def test_ros_bridge_declares_joint_and_velocity_interfaces(self):
        source = NODE.read_text()
        ast.parse(source)
        for topic in ("/gensong/joint_command", "/gensong/joint_states", "/cmd_vel"):
            self.assertIn(topic, source)
        self.assertIn("rgb8", source)
        self.assertIn("32FC1", source)
        self.assertIn("Berxel K", source)

    def test_joint_states_are_decoupled_from_camera_rendering(self):
        source = NODE.read_text()
        self.assertIn('self.declare_parameter("joint_state_publish_hz", 500.0)', source)
        self.assertIn('self.publish_state()', source)
        self.assertNotIn('self.state_timer = self.create_timer(', source)
        self.assertIn('self._camera_timer = self.create_timer(', source)
        self.assertIn('mujoco.mj_copyData(', source)
        self.assertNotIn('joint_state_publish_interval', source)

    def test_scene_box_visual_and_collision_geometry_share_pose(self):
        root = ET.parse(SCENE).getroot()
        box = root.find('.//body[@name="scene_box"]')
        visual = box.find('./geom[@name="scene_box_visual"]')

        self.assertIsNotNone(visual)
        self.assertEqual(visual.attrib["type"], "mesh")
        self.assertEqual(visual.attrib["mesh"], "scene_box")
        self.assertEqual(visual.attrib["contype"], "0")
        self.assertEqual(visual.attrib["conaffinity"], "0")
        self.assertEqual(visual.attrib["quat"], "0.70710678 0.70710678 0 0")
        collision = box.find('./geom[@name="scene_box_collision"]')
        self.assertIsNotNone(collision)
        self.assertEqual(collision.attrib["type"], "mesh")
        self.assertEqual(collision.attrib["mesh"], "scene_box")
        self.assertEqual(collision.attrib["quat"], "0.70710678 0.70710678 0 0")
        self.assertEqual(collision.attrib["contype"], "1")
        self.assertEqual(collision.attrib["conaffinity"], "1")
        self.assertEqual(collision.attrib["mass"], "3.0")
        self.assertEqual(collision.attrib["margin"], "0.003")
        for name in ("scene_box_slot_left_lip", "scene_box_slot_right_lip"):
            lip = box.find('./geom[@name="%s"]' % name)
            self.assertIsNotNone(lip)
            self.assertEqual(lip.attrib["type"], "box")
            self.assertEqual(lip.attrib["contype"], "1")
            self.assertEqual(lip.attrib["conaffinity"], "1")
            self.assertEqual(lip.attrib["mass"], "0")

    def test_scene_box_uses_real_mesh_collision_without_automatic_grasp_constraint(self):
        root = ET.parse(SCENE).getroot()
        box = root.find('.//body[@name="scene_box"]')
        collision = box.find('./geom[@name="scene_box_collision"]')
        self.assertEqual(collision.attrib["type"], "mesh")
        self.assertEqual(collision.attrib["mesh"], "scene_box")
        self.assertEqual(collision.attrib["contype"], "1")
        self.assertEqual(collision.attrib["conaffinity"], "1")

        table_top = root.find('.//geom[@name="scene_table_top_collision"]')
        self.assertEqual(table_top.attrib["pos"], "1.0 0.7 -0.5")
        self.assertIsNone(root.find('.//equality/weld[@name="scene_box_grasp_weld"]'))
        self.assertIsNone(root.find('.//body[@name="box_grasp_anchor"]'))

    def test_mujoco_bridge_reports_physical_grasp_state_without_forcing_it(self):
        source = NODE.read_text()
        self.assertIn('PoseStamped', source)
        self.assertIn('"/box_pose"', source)
        self.assertIn('mj_contactForce', source)
        self.assertIn('data.qpos[self.box_qpos_addr', source)
        self.assertNotIn('eq_active', source)
        self.assertNotIn('body_gravcomp', source)
        self.assertNotIn('mocap_pos', source)
        self.assertIn('box_grasped', source)
        self.assertIn('self.box_initial_z + 0.05', source)
        self.assertIn('box_speed <= 0.35', source)

    def test_scene_table_uses_primitive_collision_geometry(self):
        root = ET.parse(SCENE).getroot()
        table = root.find('.//body[@name="scene_table"]')
        visual = table.find('./geom[@name="scene_table_visual"]')
        collision_mesh = table.find('./geom[@name="scene_table_collision"]')
        primitive_collisions = table.findall('./geom[@name="scene_table_top_collision"]')

        self.assertIsNotNone(visual)
        self.assertEqual(visual.attrib["contype"], "0")
        self.assertIsNone(collision_mesh)
        self.assertEqual(len(primitive_collisions), 1)
        self.assertEqual(primitive_collisions[0].attrib["type"], "box")
        self.assertEqual(primitive_collisions[0].attrib["pos"], "1.0 0.7 -0.5")


if __name__ == "__main__":
    unittest.main()
