import os

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import OpaqueFunction
import rclpy


def _assert_no_existing_grasp_stack(context):
    rclpy.init(args=None)
    node = rclpy.create_node("_grasp_waypoint_launch_preflight")
    try:
        end_time = node.get_clock().now().nanoseconds + int(1.5e9)
        while node.get_clock().now().nanoseconds < end_time:
            rclpy.spin_once(node, timeout_sec=0.1)

        existing_names = {name for name, _namespace in node.get_node_names_and_namespaces()}
        conflicting_names = sorted(existing_names.intersection({
            "dual_arm_grasp_waypoint_planner",
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
            "[grasp_waypoint_planner.launch.py] Refusing to start because an existing grasp planner stack is already "
            "running. Conflicting nodes: " + ", ".join(conflicting_names) +
            ". Stop the previous launch before starting a new one."
        )

    return []


def generate_launch_description():
    manipulator_launch = os.path.join(
        get_package_share_directory('ocs2_mobile_manipulator_ros'),
        'launch',
        'manipulator_gensong_dual.launch.py',
    )
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='rviz',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='urdfFile',
            default_value=get_package_share_directory('ocs2_robotic_assets') +
                          '/resources/gensong_board/urdf/gensong_board.urdf'
        ),
        launch.actions.DeclareLaunchArgument(
            name='rvizconfig',
            default_value=get_package_share_directory('ocs2_mobile_manipulator_ros') + '/rviz/mobile_manipulator.rviz'
        ),
        launch.actions.DeclareLaunchArgument(
            name='debug',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='taskFile',
            default_value=get_package_share_directory('ocs2_mobile_manipulator') + '/config/gensong/task_dual_ee.info'
        ),
        launch.actions.DeclareLaunchArgument(
            name='libFolder',
            default_value='/tmp/ocs2_mobile_manipulator_auto_generated/gensong'
        ),
        launch.actions.DeclareLaunchArgument(
            name='box_pose_topic',
            default_value='box_pose'
        ),
        launch.actions.DeclareLaunchArgument(
            name='place_box_pose_topic',
            default_value='place_box_pose'
        ),
        launch.actions.DeclareLaunchArgument(
            name='box_size_x',
            default_value='0.45'
        ),
        launch.actions.DeclareLaunchArgument(
            name='box_size_y',
            default_value='0.72'
        ),
        launch.actions.DeclareLaunchArgument(
            name='box_size_z',
            default_value='0.12'
        ),
        launch.actions.DeclareLaunchArgument(
            name='grasp_edge_inset_y',
            # Open both arms slightly beyond the box edges so the handboards
            # approach from outside instead of colliding at the boundary.
            default_value='-0.03'
        ),
        launch.actions.DeclareLaunchArgument(
            name='grasp_x_offset',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='grasp_z_offset',
            default_value='-0.01'
        ),
        launch.actions.DeclareLaunchArgument(
            name='grasp_hold_sec',
            default_value='2.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='lift_distance',
            default_value='0.20'
        ),
        launch.actions.DeclareLaunchArgument(
            name='trajectory_time_scale',
            default_value='4.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='carry_home_after_grasp',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='dt_lift_to_carry_upright',
            default_value='0.8'
        ),
        launch.actions.DeclareLaunchArgument(
            name='dt_carry_upright_to_home',
            default_value='1.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='dt_post_release_retreat_to_initial',
            default_value='1.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='carry_home_box_x',
            default_value='0.6'
        ),
        launch.actions.DeclareLaunchArgument(
            name='carry_home_box_y',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='carry_home_box_z',
            default_value='1.10'
        ),
        launch.actions.DeclareLaunchArgument(
            name='carry_home_front_clearance',
            default_value='0.05'
        ),
        launch.actions.DeclareLaunchArgument(
            name='carry_home_table_clearance',
            default_value='0.05'
        ),
        launch.actions.DeclareLaunchArgument(
            name='grasp_frame_roll',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='grasp_frame_pitch',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='grasp_frame_yaw',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='left_wrist_roll',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='left_wrist_pitch',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='left_wrist_yaw',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='right_wrist_roll',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='right_wrist_pitch',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='right_wrist_yaw',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='enable_transport_stage',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='enable_place_stage',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='transport_offset_is_absolute',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='transport_offset_x',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='transport_offset_y',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='transport_offset_z',
            default_value='0.0'
        ),
        OpaqueFunction(function=_assert_no_existing_grasp_stack),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                manipulator_launch
            ),
            launch_arguments={
                'rviz': launch.substitutions.LaunchConfiguration('rviz'),
                'debug': launch.substitutions.LaunchConfiguration('debug'),
                'urdfFile': launch.substitutions.LaunchConfiguration('urdfFile'),
                'taskFile': launch.substitutions.LaunchConfiguration('taskFile'),
                'libFolder': launch.substitutions.LaunchConfiguration('libFolder'),
            }.items()
        ),
        launch_ros.actions.Node(
            package='ocs2_mobile_manipulator_ros',
            executable='dual_arm_grasp_waypoint_planner',
            name='dual_arm_grasp_waypoint_planner',
            output='screen',
            parameters=[
                {
                    'taskFile': launch.substitutions.LaunchConfiguration('taskFile')
                },
                {
                    'urdfFile': launch.substitutions.LaunchConfiguration('urdfFile')
                },
                {
                    'box_pose_topic': launch.substitutions.LaunchConfiguration('box_pose_topic')
                },
                {
                    'place_box_pose_topic': launch.substitutions.LaunchConfiguration('place_box_pose_topic')
                },
                {
                    'box_size_x': launch.substitutions.LaunchConfiguration('box_size_x')
                },
                {
                    'box_size_y': launch.substitutions.LaunchConfiguration('box_size_y')
                },
                {
                    'box_size_z': launch.substitutions.LaunchConfiguration('box_size_z')
                },
                {
                    'grasp_edge_inset_y': launch.substitutions.LaunchConfiguration('grasp_edge_inset_y')
                },
                {
                    'grasp_x_offset': launch.substitutions.LaunchConfiguration('grasp_x_offset')
                },
                {
                    'grasp_z_offset': launch.substitutions.LaunchConfiguration('grasp_z_offset')
                },
                {
                    'grasp_hold_sec': launch.substitutions.LaunchConfiguration('grasp_hold_sec')
                },
                {
                    'lift_distance': launch.substitutions.LaunchConfiguration('lift_distance')
                },
                {
                    'trajectory_time_scale': launch.substitutions.LaunchConfiguration('trajectory_time_scale')
                },
                {
                    'carry_home_after_grasp': launch.substitutions.LaunchConfiguration('carry_home_after_grasp')
                },
                {
                    'dt_lift_to_carry_upright': launch.substitutions.LaunchConfiguration('dt_lift_to_carry_upright')
                },
                {
                    'dt_carry_upright_to_home': launch.substitutions.LaunchConfiguration('dt_carry_upright_to_home')
                },
                {
                    'dt_post_release_retreat_to_initial': launch.substitutions.LaunchConfiguration(
                        'dt_post_release_retreat_to_initial'
                    )
                },
                {
                    'carry_home_box_x': launch.substitutions.LaunchConfiguration('carry_home_box_x')
                },
                {
                    'carry_home_box_y': launch.substitutions.LaunchConfiguration('carry_home_box_y')
                },
                {
                    'carry_home_box_z': launch.substitutions.LaunchConfiguration('carry_home_box_z')
                },
                {
                    'carry_home_front_clearance': launch.substitutions.LaunchConfiguration('carry_home_front_clearance')
                },
                {
                    'carry_home_table_clearance': launch.substitutions.LaunchConfiguration('carry_home_table_clearance')
                },
                {
                    'grasp_frame_rpy.roll': launch.substitutions.LaunchConfiguration('grasp_frame_roll')
                },
                {
                    'grasp_frame_rpy.pitch': launch.substitutions.LaunchConfiguration('grasp_frame_pitch')
                },
                {
                    'grasp_frame_rpy.yaw': launch.substitutions.LaunchConfiguration('grasp_frame_yaw')
                },
                {
                    'left_wrist_compensation_rpy.roll': launch.substitutions.LaunchConfiguration('left_wrist_roll')
                },
                {
                    'left_wrist_compensation_rpy.pitch': launch.substitutions.LaunchConfiguration('left_wrist_pitch')
                },
                {
                    'left_wrist_compensation_rpy.yaw': launch.substitutions.LaunchConfiguration('left_wrist_yaw')
                },
                {
                    'right_wrist_compensation_rpy.roll': launch.substitutions.LaunchConfiguration('right_wrist_roll')
                },
                {
                    'right_wrist_compensation_rpy.pitch': launch.substitutions.LaunchConfiguration('right_wrist_pitch')
                },
                {
                    'right_wrist_compensation_rpy.yaw': launch.substitutions.LaunchConfiguration('right_wrist_yaw')
                },
                {
                    'enable_transport_stage': launch.substitutions.LaunchConfiguration('enable_transport_stage')
                },
                {
                    'enable_place_stage': launch.substitutions.LaunchConfiguration('enable_place_stage')
                },
                {
                    'transport_offset_is_absolute': launch.substitutions.LaunchConfiguration('transport_offset_is_absolute')
                },
                {
                    'transport_offset_x': launch.substitutions.LaunchConfiguration('transport_offset_x')
                },
                {
                    'transport_offset_y': launch.substitutions.LaunchConfiguration('transport_offset_y')
                },
                {
                    'transport_offset_z': launch.substitutions.LaunchConfiguration('transport_offset_z')
                }
            ]
        )
    ])


if __name__ == '__main__':
    generate_launch_description()
