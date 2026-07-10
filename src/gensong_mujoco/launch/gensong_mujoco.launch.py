from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_model = str(Path(get_package_share_directory("ocs2_robotic_assets")) /
                        "resources/gensong_board/mjcf/gensong_scene.xml")
    return LaunchDescription([
        SetEnvironmentVariable("MUJOCO_GL", "egl"),
        DeclareLaunchArgument("model_path", default_value=default_model),
        DeclareLaunchArgument("viewer", default_value="true"),
        Node(
            package="gensong_mujoco",
            executable="gensong_mujoco_node.py",
            name="gensong_mujoco",
            output="screen",
            parameters=[{
                "model_path": LaunchConfiguration("model_path"),
                "viewer": LaunchConfiguration("viewer"),
            }],
        ),
    ])
