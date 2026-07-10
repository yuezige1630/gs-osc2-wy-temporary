# Box-Center Grasp and Independent Lift Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `/box_pose` represent a box center whose z coordinate is the contact-grasp height, then always append a lift stage before optional carry-home motion.

**Architecture:** Keep the topic interface and existing waypoint builder. Separate `liftAfterGrasp` from `carryHomeAfterGrasp`; the former appends the contact-to-lift waypoint while the latter only appends upright/home waypoints. Set the default z offset to `-0.01` to cancel the existing fixed local `+0.01` contact offset.

**Tech Stack:** ROS 2 Jazzy, C++, Eigen, GoogleTest, colcon.

## Global Constraints

- `/box_pose` remains a box-center pose in `base_link`.
- Contact grasp height equals `box_pose.position.z()` when the default parameters are used.
- The grasp sequence is current, via, pregrasp, contact grasp, hold, lift.
- `carry_home_after_grasp` only controls upright/home waypoints after lift.

---

### Task 1: Isolate the grasp/lift waypoint policy

**Files:**
- Modify: `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/src/DualArmGraspWaypointPlanner.cpp`
- Modify: `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/launch/grasp_waypoint_planner.launch.py`

**Interfaces:**
- Consumes: `box_pose`, `lift_distance`, `carry_home_after_grasp`.
- Produces: a target trajectory containing a contact-grasp waypoint at `box_pose.z` and a subsequent lift waypoint.

- [ ] **Step 1: Add a `lift_after_grasp` launch argument and forward it to the planner**

```python
launch.actions.DeclareLaunchArgument(name='lift_after_grasp', default_value='true')
```

- [ ] **Step 2: Add planner parameters and compute contact/lift box poses**

```cpp
liftAfterGrasp_ = node_->declare_parameter<bool>("lift_after_grasp", true);
PoseData liftBoxPose = snapshot.boxPose;
liftBoxPose.position.z() = std::max(snapshot.boxPose.position.z() + liftDistance_, clearanceBaseZ);
```

- [ ] **Step 3: Build the non-carry waypoint sequence with a lift stage**

```cpp
armWaypoints[armIndex] = {currentPoses[armIndex], via, preGrasp, graspPose, hold, lift};
timeOffsets = {0.0, scaledTime(dtCurrentToVia_), scaledTime(dtCurrentToVia_ + dtViaToPregrasp_),
               scaledTime(dtCurrentToVia_ + dtViaToPregrasp_ + dtPregraspToGrasp_),
               scaledTime(dtCurrentToVia_ + dtViaToPregrasp_ + dtPregraspToGrasp_ + graspHoldSec_),
               scaledTime(dtCurrentToVia_ + dtViaToPregrasp_ + dtPregraspToGrasp_ + graspHoldSec_ + dtRetreatToLift_)};
```

- [ ] **Step 4: Set default `grasp_z_offset` to `-0.01` in the launch file**

```python
launch.actions.DeclareLaunchArgument(name='grasp_z_offset', default_value='-0.01')
```

- [ ] **Step 5: Build the package**

Run: `source /opt/ros/jazzy/setup.bash && colcon build --packages-select ocs2_mobile_manipulator_ros`

Expected: `Summary: 1 package finished`.

### Task 2: Add regression coverage and online validation

**Files:**
- Create: `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/test/testGraspLiftWaypointHelpers.cpp`
- Modify: `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/CMakeLists.txt`

**Interfaces:**
- Consumes: a box center z of `0.91074` and default `grasp_z_offset=-0.01`.
- Produces: an assertion that contact waypoint z is `0.91074` and lift waypoint z is `0.91074 + lift_distance` subject to clearance.

- [ ] **Step 1: Write a failing waypoint test**

```cpp
EXPECT_DOUBLE_EQ(contactPose.position.z(), 0.91074);
EXPECT_DOUBLE_EQ(liftPose.position.z(), 1.06074);
```

- [ ] **Step 2: Run the focused test and verify it fails before the behavior is implemented**

Run: `colcon test --packages-select ocs2_mobile_manipulator_ros --ctest-args -R testGraspLiftWaypointHelpers`

Expected: failure because the current non-carry sequence has no independent lift stage.

- [ ] **Step 3: Run package tests after implementation**

Run: `colcon test --packages-select ocs2_mobile_manipulator_ros --event-handlers console_direct+ && colcon test-result --verbose`

Expected: all package tests pass.

- [ ] **Step 4: Perform online verification with the requested box center**

Run:

```bash
ros2 launch ocs2_mobile_manipulator_ros grasp_waypoint_planner.launch.py
ros2 topic pub --once /box_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: base_link}, pose: {position: {x: 0.82795, y: 0.0, z: 0.91074}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

Expected: TF samples show both hands first reach z≈`0.91074`, then increase by `lift_distance` without sustained oscillation.
