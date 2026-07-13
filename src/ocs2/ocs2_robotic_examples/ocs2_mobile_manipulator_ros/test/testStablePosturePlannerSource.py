from pathlib import Path


PLANNER_SOURCE = (
    Path(__file__).resolve().parents[5]
    / "src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/src/DualArmGraspWaypointPlanner.cpp"
)


def test_planner_uses_fixed_pre_transport_posture_for_return():
    source = PLANNER_SOURCE.read_text(encoding="utf-8")

    assert "initialState.arm" in source
    assert "stable_posture_hold_sec" in source
    assert "buildPostReleaseInitialReturnTrajectory" in source
    assert "stablePostureEndEffectorPoses" in source
    assert "Cached fixed pre-transport end-effector poses" in source


def test_launch_files_forward_stable_posture_hold_parameter():
    root = PLANNER_SOURCE.parents[1]
    for launch_name in ("grasp_waypoint_planner.launch.py", "grasp_waypoint_test.launch.py"):
        source = (root / "launch" / launch_name).read_text(encoding="utf-8")
        assert "stable_posture_hold_sec" in source
        assert (
            "stable_posture_hold_sec': launch.substitutions.LaunchConfiguration('stable_posture_hold_sec')"
            in source
        )


def test_grasp_approach_expands_along_y_before_descent():
    source = PLANNER_SOURCE.read_text(encoding="utf-8")

    assert "const double yExpansion = 0.5 * boxSizeY_ + approachBoxClearance_;" in source
    assert "PoseData yExpanded" in source
    assert "yExpanded.position.y() = snapshot.boxPose.position.y()" in source
    assert "armIndex == 0 ? yExpansion : -yExpansion" in source
    assert "clearanceLift, yExpanded, via, preGrasp" in source
