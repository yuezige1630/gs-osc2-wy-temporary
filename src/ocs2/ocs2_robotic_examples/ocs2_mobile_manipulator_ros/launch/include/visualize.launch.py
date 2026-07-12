import os
import launch
import launch_ros.actions
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='urdfFile',
            default_value=get_package_share_directory('ocs2_robotic_assets') + "/resources/mobile_manipulator/franka/urdf/panda.urdf"
        ),
        launch.actions.DeclareLaunchArgument(
            name='test',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='rviz',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='rvizconfig',
            default_value=get_package_share_directory('ocs2_mobile_manipulator_ros') + "/rviz/mobile_manipulator.rviz"
        ),
        launch.actions.DeclareLaunchArgument(
            name='joint_states_topic',
            default_value='/joint_states'
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": ParameterValue(Command(["cat ", LaunchConfiguration("urdfFile")]), value_type=str)}],
            remappings=[("joint_states", LaunchConfiguration("joint_states_topic"))],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            arguments=[LaunchConfiguration("urdfFile")],
            condition=IfCondition(LaunchConfiguration("test")),
            remappings=[("joint_states", LaunchConfiguration("joint_states_topic"))],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='mobile_manipulator',
            output='screen',
            condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=["-d", LaunchConfiguration("rvizconfig")]
        )
    ])
    return ld

if __name__ == '__main__':
    generate_launch_description()
