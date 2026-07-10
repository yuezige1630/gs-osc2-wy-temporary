import os

import launch
import launch_ros.actions
from launch.conditions import IfCondition, UnlessCondition
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
            name='taskFile',
            default_value=get_package_share_directory('ocs2_mobile_manipulator') + '/config/gensong/task_dual_ee.info'
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
            name='sweep_mode',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='sweep_frame_id',
            default_value='base_link'
        ),
        launch.actions.DeclareLaunchArgument(
            name='sweep_csv_path',
            default_value='/tmp/gensong_grasp_reachability.csv'
        ),
        launch.actions.DeclareLaunchArgument(
            name='sweep_summary_path',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='sweep_publish_box_pose',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='sweep_pose_publish_settle_sec',
            default_value='0.05'
        ),
        launch.actions.DeclareLaunchArgument(
            name='sweep_service_timeout_sec',
            default_value='10.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='sweep_service_wait_timeout_sec',
            default_value='30.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='sweep_startup_wait_sec',
            default_value='2.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='sweep_x_min',
            default_value='0.65'
        ),
        launch.actions.DeclareLaunchArgument(
            name='sweep_x_max',
            default_value='1.15'
        ),
        launch.actions.DeclareLaunchArgument(
            name='sweep_x_step',
            default_value='0.05'
        ),
        launch.actions.DeclareLaunchArgument(
            name='sweep_y_min',
            default_value='-0.35'
        ),
        launch.actions.DeclareLaunchArgument(
            name='sweep_y_max',
            default_value='0.35'
        ),
        launch.actions.DeclareLaunchArgument(
            name='sweep_y_step',
            default_value='0.05'
        ),
        launch.actions.DeclareLaunchArgument(
            name='sweep_z_min',
            default_value='0.85'
        ),
        launch.actions.DeclareLaunchArgument(
            name='sweep_z_max',
            default_value='1.40'
        ),
        launch.actions.DeclareLaunchArgument(
            name='sweep_z_step',
            default_value='0.05'
        ),
        launch.actions.DeclareLaunchArgument(
            name='box_center_x',
            default_value='0.8994'
        ),
        launch.actions.DeclareLaunchArgument(
            name='box_center_y',
            default_value='-0.0327'
        ),
        launch.actions.DeclareLaunchArgument(
            name='box_center_z',
            default_value='1.1000'
        ),
        launch.actions.DeclareLaunchArgument(
            name='box_center_qx',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='box_center_qy',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='box_center_qz',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='box_center_qw',
            default_value='1.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='place_center_x',
            default_value='0.8994'
        ),
        launch.actions.DeclareLaunchArgument(
            name='place_center_y',
            default_value='-0.0327'
        ),
        launch.actions.DeclareLaunchArgument(
            name='place_center_z',
            default_value='1.1000'
        ),
        launch.actions.DeclareLaunchArgument(
            name='place_center_qx',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='place_center_qy',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='place_center_qz',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='place_center_qw',
            default_value='1.0'
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
            default_value='-0.01'
        ),
        launch.actions.DeclareLaunchArgument(
            name='grasp_hold_sec',
            default_value='2.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='trajectory_time_scale',
            default_value='0.7'
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
            default_value='0.65'
        ),
        launch.actions.DeclareLaunchArgument(
            name='carry_home_box_y',
            default_value='0.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='carry_home_box_z',
            default_value='1.20'
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
            name='place_trigger_delay_sec',
            default_value='8.0'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(manipulator_launch),
            launch_arguments={
                'rviz': launch.substitutions.LaunchConfiguration('rviz'),
                'urdfFile': launch.substitutions.LaunchConfiguration('urdfFile'),
                'taskFile': launch.substitutions.LaunchConfiguration('taskFile'),
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
                    'dry_run_mode': launch.substitutions.LaunchConfiguration('sweep_mode')
                },
                {
                    'box_pose_topic': launch.substitutions.LaunchConfiguration('box_pose_topic')
                },
                {
                    'place_box_pose_topic': launch.substitutions.LaunchConfiguration('place_box_pose_topic')
                },
                {
                    'box_center.x': launch.substitutions.LaunchConfiguration('box_center_x')
                },
                {
                    'box_center.y': launch.substitutions.LaunchConfiguration('box_center_y')
                },
                {
                    'box_center.z': launch.substitutions.LaunchConfiguration('box_center_z')
                },
                {
                    'box_center.qx': launch.substitutions.LaunchConfiguration('box_center_qx')
                },
                {
                    'box_center.qy': launch.substitutions.LaunchConfiguration('box_center_qy')
                },
                {
                    'box_center.qz': launch.substitutions.LaunchConfiguration('box_center_qz')
                },
                {
                    'box_center.qw': launch.substitutions.LaunchConfiguration('box_center_qw')
                },
                {
                    'place_center.x': launch.substitutions.LaunchConfiguration('place_center_x')
                },
                {
                    'place_center.y': launch.substitutions.LaunchConfiguration('place_center_y')
                },
                {
                    'place_center.z': launch.substitutions.LaunchConfiguration('place_center_z')
                },
                {
                    'place_center.qx': launch.substitutions.LaunchConfiguration('place_center_qx')
                },
                {
                    'place_center.qy': launch.substitutions.LaunchConfiguration('place_center_qy')
                },
                {
                    'place_center.qz': launch.substitutions.LaunchConfiguration('place_center_qz')
                },
                {
                    'place_center.qw': launch.substitutions.LaunchConfiguration('place_center_qw')
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
            ]
        ),
        launch_ros.actions.Node(
            package='ocs2_mobile_manipulator_ros',
            executable='preset_dual_arm_grasp_pose_publisher.py',
            name='preset_dual_arm_grasp_pose_publisher',
            output='screen',
            condition=UnlessCondition(launch.substitutions.LaunchConfiguration('sweep_mode')),
            parameters=[
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
                    'urdf_file': launch.substitutions.LaunchConfiguration('urdfFile')
                },
                {
                    'auto_trigger': False
                },
                {
                    'auto_publish_place': True
                },
                {
                    'place_trigger_delay_sec': launch.substitutions.LaunchConfiguration('place_trigger_delay_sec')
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
                }
            ]
        ),
        launch_ros.actions.Node(
            package='ocs2_mobile_manipulator_ros',
            executable='grasp_reachability_sweep.py',
            name='grasp_reachability_sweep',
            output='screen',
            condition=IfCondition(launch.substitutions.LaunchConfiguration('sweep_mode')),
            parameters=[
                {
                    'frame_id': launch.substitutions.LaunchConfiguration('sweep_frame_id')
                },
                {
                    'box_pose_topic': launch.substitutions.LaunchConfiguration('box_pose_topic')
                },
                {
                    'evaluate_service_name': 'evaluate_box_pose'
                },
                {
                    'csv_path': launch.substitutions.LaunchConfiguration('sweep_csv_path')
                },
                {
                    'summary_path': launch.substitutions.LaunchConfiguration('sweep_summary_path')
                },
                {
                    'publish_box_pose': launch.substitutions.LaunchConfiguration('sweep_publish_box_pose')
                },
                {
                    'pose_publish_settle_sec': launch.substitutions.LaunchConfiguration('sweep_pose_publish_settle_sec')
                },
                {
                    'service_timeout_sec': launch.substitutions.LaunchConfiguration('sweep_service_timeout_sec')
                },
                {
                    'service_wait_timeout_sec': launch.substitutions.LaunchConfiguration('sweep_service_wait_timeout_sec')
                },
                {
                    'startup_wait_sec': launch.substitutions.LaunchConfiguration('sweep_startup_wait_sec')
                },
                {
                    'box_qx': launch.substitutions.LaunchConfiguration('box_center_qx')
                },
                {
                    'box_qy': launch.substitutions.LaunchConfiguration('box_center_qy')
                },
                {
                    'box_qz': launch.substitutions.LaunchConfiguration('box_center_qz')
                },
                {
                    'box_qw': launch.substitutions.LaunchConfiguration('box_center_qw')
                },
                {
                    'x_min': launch.substitutions.LaunchConfiguration('sweep_x_min')
                },
                {
                    'x_max': launch.substitutions.LaunchConfiguration('sweep_x_max')
                },
                {
                    'x_step': launch.substitutions.LaunchConfiguration('sweep_x_step')
                },
                {
                    'y_min': launch.substitutions.LaunchConfiguration('sweep_y_min')
                },
                {
                    'y_max': launch.substitutions.LaunchConfiguration('sweep_y_max')
                },
                {
                    'y_step': launch.substitutions.LaunchConfiguration('sweep_y_step')
                },
                {
                    'z_min': launch.substitutions.LaunchConfiguration('sweep_z_min')
                },
                {
                    'z_max': launch.substitutions.LaunchConfiguration('sweep_z_max')
                },
                {
                    'z_step': launch.substitutions.LaunchConfiguration('sweep_z_step')
                }
            ]
        )
    ])


if __name__ == '__main__':
    generate_launch_description()
