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
    assert "yExpanded.position.y() = currentPoses[armIndex].position.y()" in source
    assert "snapshot.boxPose.position.y()" not in source.split("yExpanded.position.y()", 1)[1].split(";", 1)[0]
    assert "armIndex == 0 ? yExpansion : -yExpansion" in source
    assert "PoseData via = yExpanded;" in source
    assert "via.position.z() = preGrasp.position.z();" in source
    assert "clearanceLift, yExpanded, yExpanded, via, via, preGrasp" in source


def test_grasp_approach_uses_larger_default_y_clearance():
    source = PLANNER_SOURCE.read_text(encoding="utf-8")
    assert 'declare_parameter<double>("approach_box_clearance", 0.10)' in source

    root = PLANNER_SOURCE.parents[1]
    for launch_name in ("grasp_waypoint_planner.launch.py", "grasp_waypoint_test.launch.py"):
        launch_source = (root / "launch" / launch_name).read_text(encoding="utf-8")
        parameter_block = launch_source.split("name='approach_box_clearance'", 1)[1].split("),", 1)[0]
    assert "default_value='0.10'" in parameter_block


def test_lift_distance_default_increases_by_ten_centimeters():
    source = PLANNER_SOURCE.read_text(encoding="utf-8")
    assert 'declare_parameter<double>("lift_distance", 0.30)' in source
    assert "double liftDistance_ = 0.30;" in source

    launch_source = (PLANNER_SOURCE.parents[1] / "launch" / "grasp_waypoint_planner.launch.py").read_text(
        encoding="utf-8"
    )
    parameter_block = launch_source.split("name='lift_distance'", 1)[1].split("),", 1)[0]
    assert "default_value='0.30'" in parameter_block


def test_grasp_approach_has_settle_waypoints_before_next_motion():
    source = PLANNER_SOURCE.read_text(encoding="utf-8")
    assert 'declare_parameter<double>("y_expanded_settle_sec", 1.0)' in source
    assert 'declare_parameter<double>("via_settle_sec", 1.0)' in source
    assert "yExpanded, yExpanded, via, via, preGrasp" in source
    assert "dtYExpandedSettle" in source
    assert "dtViaSettle" in source


def test_post_release_retreat_defaults():
    source = PLANNER_SOURCE.read_text(encoding="utf-8")
    assert 'declare_parameter<double>("post_release_retreat_distance", 0.10)' in source
    assert 'declare_parameter<double>("post_release_retreat_height", 0.10)' in source
    assert "double postReleaseRetreatDistance_ = 0.10;" in source
    assert "double postReleaseRetreatHeight_ = 0.10;" in source
