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

