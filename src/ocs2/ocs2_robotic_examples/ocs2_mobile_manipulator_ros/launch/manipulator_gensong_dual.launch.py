import os

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
import rclpy


def _assert_no_existing_gensong_stack(context):
    rclpy.init(args=None)
    node = rclpy.create_node("_gensong_dual_launch_preflight")
    try:
        end_time = node.get_clock().now().nanoseconds + int(1.5e9)
        while node.get_clock().now().nanoseconds < end_time:
            rclpy.spin_once(node, timeout_sec=0.1)

        existing_names = {name for name, _namespace in node.get_node_names_and_namespaces()}
        conflicting_names = sorted(existing_names.intersection({
            "gensong_world_to_base_tf",
            "mobile_manipulator_mpc",
            "mobile_manipulator_mrt_node",
            "robot_state_publisher",
        }))
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if conflicting_names:
        raise RuntimeError(
            "[manipulator_gensong_dual.launch.py] Refusing to start because an existing gensong MPC stack is already "
            "running. Conflicting nodes: " + ", ".join(conflicting_names) +
            ". Stop the previous launch before starting a new one."
        )

    return []


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='rviz',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='debug',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='urdfFile',
            default_value=get_package_share_directory('ocs2_robotic_assets') +
                          '/resources/gensong_board/urdf/gensong_board.urdf'
        ),
        launch.actions.DeclareLaunchArgument(
            name='taskFile',
            default_value=get_package_share_directory('ocs2_mobile_manipulator') + '/config/gensong/task_dual_ee.info'
        ),
        launch.actions.DeclareLaunchArgument(
            name='libFolder',
            default_value='/tmp/ocs2_mobile_manipulator_auto_generated/gensong'
        ),
        OpaqueFunction(function=_assert_no_existing_gensong_stack),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gensong_world_to_base_tf',
            output='screen',
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'world', '--child-frame-id', 'base_link']
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ocs2_mobile_manipulator_ros'),
                             'launch/include/mobile_manipulator.launch.py')
            ),
            launch_arguments={
                'rviz': launch.substitutions.LaunchConfiguration('rviz'),
                'debug': launch.substitutions.LaunchConfiguration('debug'),
                'urdfFile': launch.substitutions.LaunchConfiguration('urdfFile'),
                'taskFile': launch.substitutions.LaunchConfiguration('taskFile'),
                'libFolder': launch.substitutions.LaunchConfiguration('libFolder'),
                'targetExecutable': 'mobile_manipulator_dual_target',
                'targetEnabled': 'false',
                'mrtExecutable': 'mobile_manipulator_hardware_mrt_node',
                'observation_timeout_sec': '0.3',
                # The previous values made the hardware bridge lag far behind
                # the MPC end-effector trajectory. Keep filtering enabled, but
                # allow a useful tracking bandwidth for the waypoint planner.
                'command_blend_alpha': '0.18',
                'max_joint_delta_per_command': '0.04',
                'joint_states_topic': '/gensong/joint_states',
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
