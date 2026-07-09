# Carry-Home Stage Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** After grasping and lifting the box, move the carried box into a predefined upright/home pose whose orientation is aligned with `base_link`, while keeping a conservative clearance that accounts for the box length.

**Architecture:** The carry-home behavior stays inside `dual_arm_grasp_waypoint_planner`, so the existing topic-driven state machine and `TargetTrajectoriesRosPublisher` path remain unchanged. A tiny helper header provides the reusable upright-orientation and clearance check logic, which keeps the planner readable and gives us a unit-test seam for the box-length safety rule.

**Tech Stack:** C++17, ROS 2 launch files, `ament_cmake`, `ament_cmake_gtest`, Eigen, OCS2.

## Global Constraints

- Keep the existing two-stage grasp/place workflow intact.
- Preserve the `box_pose` and `place_box_pose` topic semantics.
- Keep `trajectory_time_scale` as a runtime-overridable launch parameter.
- Use `base_link`-aligned box orientation for the new carry-home pose.
- Reject any carry-home pose whose front face would violate the configured box-length clearance.

---

### Task 1: Add reusable carry-home helpers and unit tests

**Files:**
- Create: `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/include/ocs2_mobile_manipulator_ros/CarryHomePoseHelpers.h`
- Create: `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/test/testCarryHomePoseHelpers.cpp`
- Modify: `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/CMakeLists.txt`
- Modify: `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/package.xml`

**Interfaces:**
- Consumes: Eigen vector/quaternion types, box size, front-clearance, and table-clearance values.
- Produces: `baseLinkAlignedBoxOrientation()` and `isCarryHomeBoxPoseSafe(...)` for the planner to call.

- [ ] **Step 1: Write the failing test**

Add a gtest that verifies both behaviors:

```cpp
TEST(CarryHomePoseHelpers, BaseLinkAlignedOrientationIsIdentity) {
  const auto q = ocs2::mobile_manipulator::baseLinkAlignedBoxOrientation();
  EXPECT_NEAR(q.x(), 0.0, 1e-12);
  EXPECT_NEAR(q.y(), 0.0, 1e-12);
  EXPECT_NEAR(q.z(), 0.0, 1e-12);
  EXPECT_NEAR(q.w(), 1.0, 1e-12);
}

TEST(CarryHomePoseHelpers, RejectsHomePoseWhenBoxLengthWouldHitFrontClearance) {
  std::string error;
  const bool ok = ocs2::mobile_manipulator::isCarryHomeBoxPoseSafe(
      Eigen::Vector3d(0.20, 0.0, 1.10), 0.45, 0.12, 0.05, 0.05, &error);
  EXPECT_FALSE(ok);
  EXPECT_NE(error.find("front face"), std::string::npos);
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run:

```bash
colcon test --packages-select ocs2_mobile_manipulator_ros --ctest-args -R testCarryHomePoseHelpers -V
```

Expected: the test target is missing or the helper functions are undefined.

- [ ] **Step 3: Write the minimal implementation**

Implement the helper functions in the new header so the test can compile and pass:

```cpp
namespace ocs2::mobile_manipulator {

inline Eigen::Quaterniond baseLinkAlignedBoxOrientation() {
  return Eigen::Quaterniond::Identity();
}

inline bool isCarryHomeBoxPoseSafe(const Eigen::Vector3d& boxCenter, double boxSizeX, double boxSizeZ,
                                   double frontClearance, double tableClearance, std::string* errorMessage) {
  if (boxCenter.x() - 0.5 * boxSizeX < frontClearance) {
    if (errorMessage != nullptr) {
      *errorMessage = "Carry-home box front face violates the front clearance.";
    }
    return false;
  }
  if (boxCenter.z() - 0.5 * boxSizeZ < tableClearance) {
    if (errorMessage != nullptr) {
      *errorMessage = "Carry-home box bottom violates the table clearance.";
    }
    return false;
  }
  return true;
}

}  // namespace ocs2::mobile_manipulator
```

- [ ] **Step 4: Run the test to verify it passes**

Run the same `colcon test` command and confirm the new gtest passes.

- [ ] **Step 5: Commit**

```bash
git add src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/include/ocs2_mobile_manipulator_ros/CarryHomePoseHelpers.h
git add src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/test/testCarryHomePoseHelpers.cpp
git add src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/CMakeLists.txt
git add src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/package.xml
git commit -m "test(mobile_manipulator_ros): add carry-home pose helpers"
```

### Task 2: Extend the grasp planner with a carry-home stage

**Files:**
- Modify: `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/src/DualArmGraspWaypointPlanner.cpp`

**Interfaces:**
- Consumes: the helper functions from Task 1 and new launch parameters for the carry-home box center and clearance.
- Produces: an extra upright/home segment in the published `TargetTrajectories`, with the final box pose aligned to `base_link`.

- [ ] **Step 1: Write the failing integration check**

Add a small planner-level test or compile-time check that exercises the new carry-home path and expects an extra waypoint after the lift stage.

- [ ] **Step 2: Run the check to verify it fails**

Run:

```bash
colcon build --packages-select ocs2_mobile_manipulator_ros --cmake-args -DBUILD_TESTING=ON
```

Expected: the build fails because the planner does not yet define the carry-home inputs or extra stage.

- [ ] **Step 3: Write the minimal implementation**

Update `buildGraspHoldWaypoints(...)` so it publishes:

```cpp
current -> via -> pregrasp -> grasp -> hold -> retreat -> lift -> carry_upright -> carry_home
```

Use a `base_link`-aligned quaternion for the upright/home box poses, keep the lifted pose as the transition into carry-home, and compute the home arm poses by reusing the rigid grasp session.

- [ ] **Step 4: Run the build again**

Run the same `colcon build` command and confirm `ocs2_mobile_manipulator_ros` builds successfully.

- [ ] **Step 5: Commit**

```bash
git add src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/src/DualArmGraspWaypointPlanner.cpp
git commit -m "feat(mobile_manipulator_ros): add carry-home stage"
```

### Task 3: Expose carry-home parameters in launches and update the user docs

**Files:**
- Modify: `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/launch/grasp_waypoint_planner.launch.py`
- Modify: `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/launch/grasp_waypoint_test.launch.py`
- Modify: `src/README_gensong_wheel_end_effector_tracking.md`

**Interfaces:**
- Consumes: the new carry-home planner parameters.
- Produces: launch-time knobs for the upright/home box center and clearance, plus updated usage notes for the new end pose.

- [ ] **Step 1: Write the failing launch/documentation check**

Add the new `DeclareLaunchArgument` entries and parameter forwarding blocks to both launch files, then update the README example that currently describes the post-grasp lift-only behavior.

- [ ] **Step 2: Run a syntax/build check to verify it fails before the code exists**

Run:

```bash
python3 -m py_compile src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/launch/grasp_waypoint_planner.launch.py
python3 -m py_compile src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/launch/grasp_waypoint_test.launch.py
```

Expected: the launch files are still missing the new carry-home arguments until the planner is updated.

- [ ] **Step 3: Write the minimal implementation**

Add the new launch arguments, forward them into `dual_arm_grasp_waypoint_planner`, and document the new carry-home behavior and safety parameters in the README.

- [ ] **Step 4: Run the verification commands again**

Run:

```bash
python3 -m py_compile src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/launch/grasp_waypoint_planner.launch.py
python3 -m py_compile src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/launch/grasp_waypoint_test.launch.py
colcon build --packages-select ocs2_mobile_manipulator_ros
```

Expected: both launch files parse and the package builds.

- [ ] **Step 5: Commit**

```bash
git add src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/launch/grasp_waypoint_planner.launch.py
git add src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/launch/grasp_waypoint_test.launch.py
git add src/README_gensong_wheel_end_effector_tracking.md
git commit -m "docs(mobile_manipulator_ros): document carry-home stage"
```
