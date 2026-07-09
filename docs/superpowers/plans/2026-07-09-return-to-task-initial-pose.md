# Return To Task Initial Pose Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** After the release phase, the dual-arm grasp waypoint planner first reorients both arms to a safe upright pose, then returns them to the task-start end-effector poses captured from the first valid observation, and exposes that tail sequence behind a ROS 2 `SetBool` service switch.

**Architecture:** Keep the existing two-stage grasp/place workflow intact, but add a small task-start pose cache inside the planner. The cache is populated once from the first valid MPC observation and reused when the place stage appends a final upright-to-initial return sequence. A `std_srvs/srv/SetBool` service toggles whether that tail sequence is appended, so the default behavior stays conservative and the README explains the distinction between carried-box home and task-start initial pose.

**Tech Stack:** C++17, ROS 2 launch, Eigen, Pinocchio, gtest, ament_cmake.

## Global Constraints

- Preserve the current topic-driven planner state machine and `TargetTrajectoriesRosPublisher` flow.
- Do not change the existing carried-box `home` semantics; the new return target is the task-start initial end-effector pose.
- Keep all new code ASCII-only and follow the repository's existing ROS 2 naming conventions.

---

### Task 1: Add task-start pose caching and the final return waypoint

**Files:**
- Modify: `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/src/DualArmGraspWaypointPlanner.cpp`

**Interfaces:**
- Consumes: `computeCurrentEndEffectorPoses()`, `buildPlaceWaypoints()`, `handleObservationMsg()`
- Produces: a cached first-observation end-effector pose pair and an extra post-release upright + return waypoint pair

- [ ] **Step 1: Update the place-stage waypoint builder**

```cpp
// Append a final upright waypoint before returning both hands to the cached task-start poses.
armWaypoints[armIndex] = {currentPoses[armIndex], transport, prePlace, place, release, postReleaseRetreat, postReleaseUpright, returnInitial};
```

- [ ] **Step 2: Cache the first valid task-start poses**

```cpp
if (!hasTaskStartPose_) {
  taskStartEndEffectorPoses_ = computeCurrentEndEffectorPoses(latestObservation_);
  hasTaskStartPose_ = true;
}
```

- [ ] **Step 3: Extend the place-stage timing vector**

```cpp
timeOffsets = {0.0,
               scaledTime(dtLiftToTransport_),
               scaledTime(dtLiftToTransport_ + dtTransportToPrePlace_),
               scaledTime(dtLiftToTransport_ + dtTransportToPrePlace_ + dtPrePlaceToPlace_),
               scaledTime(dtLiftToTransport_ + dtTransportToPrePlace_ + dtPrePlaceToPlace_ + dtPlaceToRelease_),
               scaledTime(dtLiftToTransport_ + dtTransportToPrePlace_ + dtPrePlaceToPlace_ + dtPlaceToRelease_ +
                          dtReleaseToPostReleaseRetreat_),
               scaledTime(dtLiftToTransport_ + dtTransportToPrePlace_ + dtPrePlaceToPlace_ + dtPlaceToRelease_ +
                          dtReleaseToPostReleaseRetreat_ + 0.5 * dtPostReleaseRetreatToInitial_),
               scaledTime(dtLiftToTransport_ + dtTransportToPrePlace_ + dtPrePlaceToPlace_ + dtPlaceToRelease_ +
                          dtReleaseToPostReleaseRetreat_ + dtPostReleaseRetreatToInitial_)};
```

- [ ] **Step 4: Run the planner compilation target**

Run: `colcon build --packages-select ocs2_mobile_manipulator_ros`
Expected: `dual_arm_grasp_waypoint_planner` builds successfully with the new cache and upright return stage.

### Task 2: Expose the new return timing in launch files

**Files:**
- Modify: `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/launch/grasp_waypoint_planner.launch.py`
- Modify: `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/launch/grasp_waypoint_test.launch.py`

**Interfaces:**
- Consumes: the planner parameter name `dt_post_release_retreat_to_initial`
- Produces: launch-time control over the total time spent after post-release retreat before the upright-to-initial return finishes

- [ ] **Step 1: Add the launch argument**

```python
launch.actions.DeclareLaunchArgument(
    name='dt_post_release_retreat_to_initial',
    default_value='1.0'
),
```

- [ ] **Step 2: Forward the launch argument into the planner node**

```python
{
    'dt_post_release_retreat_to_initial': launch.substitutions.LaunchConfiguration('dt_post_release_retreat_to_initial')
},
```

- [ ] **Step 3: Verify the launch files still parse**

Run: `python3 -m py_compile src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/launch/grasp_waypoint_planner.launch.py src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/launch/grasp_waypoint_test.launch.py`
Expected: no syntax errors.

### Task 3: Add the service switch for the final return sequence

**Files:**
- Modify: `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/src/DualArmGraspWaypointPlanner.cpp`

**Interfaces:**
- Consumes: `std_srvs/srv/SetBool`
- Produces: a `set_post_release_initial_return_enabled` service that toggles whether `postReleaseUpright -> returnInitial` is appended to the place trajectory

- [ ] **Step 1: Add the service callback and state**

```cpp
postReleaseInitialReturnService_ = node_->create_service<std_srvs::srv::SetBool>(
    "set_post_release_initial_return_enabled",
    [this](const std::shared_ptr<std_srvs::srv::SetBool::Request>& request,
           std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
      handleSetPostReleaseInitialReturnEnabled(*request, *response);
    });
```

- [ ] **Step 2: Gate the final two waypoints behind the service flag**

```cpp
if (postReleaseInitialReturnEnabled_) {
  armWaypoints[armIndex] = {currentPoses[armIndex], transport, prePlace, place, release, postReleaseRetreat,
                            postReleaseUpright, returnInitial};
} else {
  armWaypoints[armIndex] = {currentPoses[armIndex], transport, prePlace, place, release, postReleaseRetreat};
}
```

- [ ] **Step 3: Verify the planner still compiles**

Run: `colcon build --packages-select ocs2_mobile_manipulator_ros`
Expected: the new service compiles and the default-disabled path still works.

### Task 4: Document the service switch and updated tail behavior

**Files:**
- Modify: `src/README_gensong_wheel_end_effector_tracking.md`

**Interfaces:**
- Consumes: the updated planner behavior and launch argument name
- Produces: user-facing notes that `home` remains the carried-box waiting pose, while the post-release return sequence is controlled by `set_post_release_initial_return_enabled`

- [ ] **Step 1: Update the behavior description**

```md
第一段结束后，机器人会先回到“直立过渡位姿”，再回到“任务初始末端位姿”并结束放置段；这一整段只有在调用 `set_post_release_initial_return_enabled(true)` 后才会启用，这里的初始位姿是任务启动时第一份有效 observation 锁存下来的双臂末端位姿，不是 carried-box 的 home 点。
```

- [ ] **Step 2: Add the new tuning knob**

```md
ros2 launch ocs2_mobile_manipulator_ros grasp_waypoint_planner.launch.py \
  dt_post_release_retreat_to_initial:=1.0
```

- [ ] **Step 3: Re-read the updated README section**

Run: `sed -n '1,220p' src/README_gensong_wheel_end_effector_tracking.md`
Expected: the new semantics are unambiguous and do not redefine `home`.
