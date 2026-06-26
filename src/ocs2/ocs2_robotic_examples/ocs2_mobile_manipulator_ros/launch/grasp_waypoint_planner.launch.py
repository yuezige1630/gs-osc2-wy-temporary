import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='urdfFile',
            default_value=get_package_share_directory('ocs2_robotic_assets') +
                          '/resources/gensong_wheel_outfit_cover/urdf/gensong_wheel_outfit.urdf'
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
            name='left_grasp_pose_topic',
            default_value='left_grasp_pose'
        ),
        launch.actions.DeclareLaunchArgument(
            name='right_grasp_pose_topic',
            default_value='right_grasp_pose'
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
                    'left_grasp_pose_topic': launch.substitutions.LaunchConfiguration('left_grasp_pose_topic')
                },
                {
                    'right_grasp_pose_topic': launch.substitutions.LaunchConfiguration('right_grasp_pose_topic')
                }
            ]
        )
    ])


if __name__ == '__main__':
    generate_launch_description()
