import os

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    manipulator_launch = os.path.join(
        get_package_share_directory('ocs2_mobile_manipulator_ros'),
        'launch',
        'manipulator_gensong_dual.launch.py',
    )
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='rviz',
            default_value='true'
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
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='grasp_x_offset',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='grasp_z_offset',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='grasp_hold_sec',
            default_value='2.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='trajectory_time_scale',
            default_value='1.0'
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
                    'trajectory_time_scale': launch.substitutions.LaunchConfiguration('trajectory_time_scale')
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
