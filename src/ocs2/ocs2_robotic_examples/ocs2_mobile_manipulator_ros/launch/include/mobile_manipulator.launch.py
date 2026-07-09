import os
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    default_task_file = get_package_share_directory('ocs2_mobile_manipulator') + '/config/franka/task.info'
    default_urdf_file = get_package_share_directory('ocs2_robotic_assets') + '/resources/mobile_manipulator/franka/urdf/panda.urdf'
    default_lib_folder = '/tmp/ocs2_mobile_manipulator_auto_generated'

    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='rviz',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='debug',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='gdb_prefix',
            default_value='gdb -ex run --args'
        ),
        launch.actions.DeclareLaunchArgument(
            name='terminal_prefix',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='taskFile',
            default_value=default_task_file
        ),
        launch.actions.DeclareLaunchArgument(
            name='urdfFile',
            default_value=default_urdf_file
        ),
        launch.actions.DeclareLaunchArgument(
            name='libFolder',
            default_value=default_lib_folder
        ),
        launch.actions.DeclareLaunchArgument(
            name='targetExecutable',
            default_value='mobile_manipulator_target'
        ),
        launch.actions.DeclareLaunchArgument(
            name='targetEnabled',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='mrtExecutable',
            default_value='mobile_manipulator_dummy_mrt_node'
        ),
        launch.actions.DeclareLaunchArgument(
            name='observation_timeout_sec',
            default_value='0.1'
        ),
        launch.actions.DeclareLaunchArgument(
            name='command_blend_alpha',
            default_value='0.15'
        ),
        launch.actions.DeclareLaunchArgument(
            name='max_joint_delta_per_command',
            default_value='0.03'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'ocs2_mobile_manipulator_ros'), 'launch/include/visualize.launch.py')
            ),
            launch_arguments={
                'urdfFile': LaunchConfiguration('urdfFile'),
                'rviz': LaunchConfiguration('rviz'),
                'joint_states_topic': LaunchConfiguration('joint_states_topic')
            }.items()
        ),
        launch.actions.DeclareLaunchArgument(
            name='joint_states_topic',
            default_value='/joint_states'
        ),
        launch_ros.actions.Node(
            package='ocs2_mobile_manipulator_ros',
            executable='mobile_manipulator_mpc_node',
            name='mobile_manipulator_mpc',
            prefix=LaunchConfiguration('gdb_prefix'),
            condition=launch.conditions.IfCondition(LaunchConfiguration("debug")),
            output='screen',
            parameters=[
                {
                    'taskFile': launch.substitutions.LaunchConfiguration('taskFile')
                },
                {
                    'urdfFile': launch.substitutions.LaunchConfiguration('urdfFile')
                },
                {
                    'libFolder': PathJoinSubstitution([
                        launch.substitutions.LaunchConfiguration('libFolder'),
                        'mpc',
                    ])
                }
            ]
        ),
        launch_ros.actions.Node(
            package='ocs2_mobile_manipulator_ros',
            executable='mobile_manipulator_mpc_node',
            name='mobile_manipulator_mpc',
            prefix=LaunchConfiguration('terminal_prefix'),
            condition=launch.conditions.UnlessCondition(LaunchConfiguration("debug")),
            output='screen',
            parameters=[
                {
                    'taskFile': launch.substitutions.LaunchConfiguration('taskFile')
                },
                {
                    'urdfFile': launch.substitutions.LaunchConfiguration('urdfFile')
                },
                {
                    'libFolder': PathJoinSubstitution([
                        launch.substitutions.LaunchConfiguration('libFolder'),
                        'mpc',
                    ])
                }
            ]
        ),
        launch_ros.actions.Node(
            package='ocs2_mobile_manipulator_ros',
            executable=LaunchConfiguration('mrtExecutable'),
            name='mobile_manipulator_mrt_node',
            prefix=LaunchConfiguration('terminal_prefix'),
            output='screen',
            parameters=[
                {
                    'taskFile': launch.substitutions.LaunchConfiguration('taskFile')
                },
                {
                    'urdfFile': launch.substitutions.LaunchConfiguration('urdfFile')
                },
                {
                    'libFolder': PathJoinSubstitution([
                        launch.substitutions.LaunchConfiguration('libFolder'),
                        'mrt',
                    ])
                },
                {
                    'observation_timeout_sec': launch.substitutions.LaunchConfiguration('observation_timeout_sec')
                },
                {
                    'command_blend_alpha': launch.substitutions.LaunchConfiguration('command_blend_alpha')
                },
                {
                    'max_joint_delta_per_command': launch.substitutions.LaunchConfiguration('max_joint_delta_per_command')
                }
            ]
        ),
        launch_ros.actions.Node(
            package='ocs2_mobile_manipulator_ros',
            executable=LaunchConfiguration('targetExecutable'),
            name='mobile_manipulator_target',
            prefix=LaunchConfiguration('terminal_prefix'),
            condition=launch.conditions.IfCondition(LaunchConfiguration('targetEnabled')),
            output='screen',
            parameters=[
                {
                    'taskFile': launch.substitutions.LaunchConfiguration('taskFile')
                },
                {
                    'urdfFile': launch.substitutions.LaunchConfiguration('urdfFile')
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
