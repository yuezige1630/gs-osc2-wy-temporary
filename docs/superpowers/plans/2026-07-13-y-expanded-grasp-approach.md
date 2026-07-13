# Y-Expanded Grasp Approach Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add automatic y-axis expansion before the dual-arm grasp descent.

**Architecture:** Extend the existing `buildGraspHoldWaypoints()` sequence with a high-clearance y-expanded pose for each arm. Compute the pose from `box_size_y`, `approach_box_clearance`, and the arm side; keep the existing target trajectory publisher and launch parameters unchanged.

**Tech Stack:** C++17, Eigen, OCS2, ROS 2, pytest/source-contract tests.

## Global Constraints

- Preserve the user's existing changes in `task_dual_ee.info`.
- Do not change the final grasp, lift, place, or return target poses.
- Keep waypoint and time-offset arrays the same length.

### Task 1: Add regression coverage for y expansion

**Files:**
- Modify: `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/test/testStablePosturePlannerSource.py`
- Test: `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/test/testStablePosturePlannerSource.py`

- [ ] Add a test asserting the planner source contains automatic expansion based on `boxSizeY_`, `approachBoxClearance_`, and side-dependent y signs, plus a `yExpanded` waypoint in both grasp branches.
- [ ] Run the focused pytest and confirm it fails because production code does not yet contain the new behavior.

### Task 2: Implement the expanded approach waypoint

**Files:**
- Modify: `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/src/DualArmGraspWaypointPlanner.cpp:663-720`

- [ ] Create `yExpanded` from the high-clearance approach pose, set its y coordinate to the box center y plus/minus `boxSizeY_ / 2 + approachBoxClearance_`, and preserve the existing orientation and high z.
- [ ] Insert `yExpanded` before `via` in both `carryHomeAfterGrasp_` and non-carry waypoint arrays.
- [ ] Add one matching time offset using the existing approach timing and retain strict increasing timestamps.
- [ ] Run the focused test and confirm it passes.

### Task 3: Verify the planner package

**Files:**
- No additional files.

- [ ] Run the planner source tests and the existing grasp/lift helper tests available in the workspace.
- [ ] Inspect the diff to ensure only the intended planner/test/docs changes are present and the pre-existing task config edit remains untouched.
