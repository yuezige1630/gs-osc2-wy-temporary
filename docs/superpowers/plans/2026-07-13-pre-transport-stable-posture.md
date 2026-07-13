# Pre-Transport Stable Posture Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make launch startup and `/continue_return_to_initial_pose` converge to one fixed, explicitly configured 19-joint pre-transport posture instead of only holding end-effector pose.

**Architecture:** Keep the existing 14-dimensional dual-end-effector target message. Add an OCS2 state-input cost that tracks the fixed joint posture only when the active end-effector reference is the derived stable end-effector pose. Use `initialState.arm` in the task file as the single source of truth, so hardware MRT startup and the waypoint planner derive the same stable posture.

**Tech Stack:** C++17, Eigen, OCS2 `StateInputCost`, Pinocchio, ROS 2, ament CMake, GoogleTest, Python launch files.

## Global Constraints

- The stable posture is exactly 19 joint angles in the current Gensong model order: 5 torso joints, 7 left-arm joints, and 7 right-arm joints.
- `initialState.arm` in `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/config/gensong/task_dual_ee.info` is the only stable-posture data source.
- Do not change the existing 14-dimensional end-effector target message format.
- Do not publish direct joint commands from the planner; MPC remains the only command generator.
- Stable-posture cost must be inactive for grasp, lift, transport, and place references whose end-effector target is not the stable pose.
- New code must validate finite values, dimensions, positive timing, and URDF/Pinocchio joint limits before use.
- Preserve unrelated user work already present in the working tree.

## File Map

- Create `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/include/ocs2_mobile_manipulator/cost/StablePostureCost.h`: fixed joint posture cost and stable-target activation predicate.
- Modify `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/src/MobileManipulatorInterface.cpp`: parse stable-posture settings, derive stable end-effector pose, and register the cost.
- Modify `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/CMakeLists.txt`: register the focused stable-cost gtest.
- Create `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/test/testStablePostureCost.cpp`: cost, activation, and dimension regression tests.
- Modify `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/include/ocs2_mobile_manipulator_ros/PostReleaseInitialReturnHelpers.h`: add a stable-pose hold waypoint.
- Modify `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/test/testPostReleaseInitialReturnHelpers.cpp`: verify the final stable pose and strictly increasing hold timing.
- Modify `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/src/DualArmGraspWaypointPlanner.cpp`: load the same 19-joint posture, derive its end-effector targets, and use them for return planning.
- Modify `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/launch/grasp_waypoint_planner.launch.py`: expose stable hold duration and pass it to the planner.
- Modify `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/launch/grasp_waypoint_test.launch.py`: pass the same stable hold duration in the test launch path.
- Modify `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/config/gensong/task_dual_ee.info`: define the 19 joint values and stable-cost parameters.
- Modify `src/README_gensong_wheel_end_effector_tracking.md`: document that both startup and the return service target the fixed joint posture.

---

### Task 1: Add and test the stable joint-posture cost

**Files:**
- Create: `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/include/ocs2_mobile_manipulator/cost/StablePostureCost.h`
- Create: `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/test/testStablePostureCost.cpp`
- Modify: `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/CMakeLists.txt`

**Interfaces:**
- Consumes: a fixed state-space posture vector, a 14-dimensional stable dual-end-effector target, a state-weight matrix, and position/orientation activation tolerances.
- Produces: `StablePostureCost`, a cloneable `ocs2::StateInputCost` with `isStableEndEffectorTarget()` exposed as a testable static/helper function.

- [ ] **Step 1: Write the failing tests**

Add tests covering exactly these behaviors:

```cpp
TEST(StablePostureCost, ActivatesOnlyForTheStableDualEndEffectorTarget) {
  vector_t stableEe = vector_t::Zero(14);
  stableEe(6) = 1.0;
  stableEe(13) = 1.0;
  const vector_t exactTarget = stableEe;
  vector_t movedTarget = stableEe;
  movedTarget(0) = 0.05;

  EXPECT_TRUE(StablePostureCost::isStableEndEffectorTarget(exactTarget, stableEe, 0.01, 0.05));
  EXPECT_FALSE(StablePostureCost::isStableEndEffectorTarget(movedTarget, stableEe, 0.01, 0.05));
}

TEST(StablePostureCost, ProducesQuadraticJointDeviationWhenActive) {
  vector_t stableEe = vector_t::Zero(14);
  stableEe(6) = 1.0;
  stableEe(13) = 1.0;
  vector_t stateTarget = vector_t::Zero(19);
  stateTarget(3) = 1.0;
  stateTarget(10) = 1.0;
  vector_t state = stateTarget;
  state(5) = 0.2;

  matrix_t Q = matrix_t::Identity(19, 19);
  StablePostureCost cost(stateTarget, stableEe, Q, 0.01, 0.05);
  TargetTrajectories target({0.0}, {stableEe}, {vector_t::Zero(19)});
  PreComputation preComputation;

  EXPECT_GT(cost.getValue(0.0, state, vector_t::Zero(19), target, preComputation), 0.0);
  const auto approximation = cost.getQuadraticApproximation(
      0.0, state, vector_t::Zero(19), target, preComputation);
  EXPECT_NEAR(approximation.dfdx(5), 0.2, 1e-12);
  EXPECT_DOUBLE_EQ(approximation.dfdxx(5, 5), 1.0);
}

TEST(StablePostureCost, ReturnsZeroWhenReferenceIsNotStable) {
  vector_t stableEe = vector_t::Zero(14);
  stableEe(6) = 1.0;
  stableEe(13) = 1.0;
  vector_t taskTarget = stableEe;
  taskTarget(0) = 0.2;
  const vector_t posture = vector_t::Zero(19);
  StablePostureCost cost(posture, stableEe, matrix_t::Identity(19, 19), 0.01, 0.05);
  TargetTrajectories target({0.0}, {taskTarget}, {vector_t::Zero(19)});
  PreComputation preComputation;

  EXPECT_DOUBLE_EQ(cost.getValue(0.0, vector_t::Ones(19), vector_t::Zero(19), target, preComputation), 0.0);
}
```

Add `add_ocs2_test(StablePostureCostTest test/testStablePostureCost.cpp)` to the mobile manipulator test section.

- [ ] **Step 2: Run the focused test and verify the expected failure**

Run:

```bash
colcon test --packages-select ocs2_mobile_manipulator --ctest-args -R StablePostureCostTest --event-handlers console_direct+
```

Expected: configuration or compilation fails because `StablePostureCost.h` and the class do not exist yet. If the test is not discovered, inspect the package build output and correct only the test registration before proceeding.

- [ ] **Step 3: Implement the minimal cost**

Implement the class with this contract:

```cpp
class StablePostureCost final : public ocs2::StateInputCost {
 public:
  StablePostureCost(vector_t posture, vector_t stableEndEffectorTarget, matrix_t Q,
                    scalar_t positionTolerance, scalar_t orientationTolerance);
  StablePostureCost* clone() const override { return new StablePostureCost(*this); }

  scalar_t getValue(scalar_t time, const vector_t& state, const vector_t& input,
                    const TargetTrajectories& target, const PreComputation&) const override;
  ScalarFunctionQuadraticApproximation getQuadraticApproximation(
      scalar_t time, const vector_t& state, const vector_t& input,
      const TargetTrajectories& target, const PreComputation&) const override;

  static bool isStableEndEffectorTarget(const vector_t& target, const vector_t& stableTarget,
                                        scalar_t positionTolerance, scalar_t orientationTolerance);
};
```

Use the arm portion of `state` by storing the full state target and a full-state `Q`; for this task `Q` has zero base entries and positive diagonal entries for the 19 arm states. The active cost is `0.5 * deviation.transpose() * Q * deviation`; the inactive approximation is a zero `ScalarFunctionQuadraticApproximation` sized to the supplied state/input dimensions. Compare each end-effector position with Euclidean norm and each quaternion by the shortest-angle distance, using normalized quaternions and `abs(dot)` so sign-equivalent quaternions match. Reject constructor inputs with non-finite values, wrong dimensions, non-positive tolerances, or a non-positive-semidefinite diagonal weight matrix.

- [ ] **Step 4: Run the focused test and verify it passes**

Run the same `colcon test` command. Expected: `StablePostureCostTest` passes with zero failures.

- [ ] **Step 5: Commit the focused component**

```bash
git add src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/include/ocs2_mobile_manipulator/cost/StablePostureCost.h \
        src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/test/testStablePostureCost.cpp \
        src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/CMakeLists.txt
git commit -m "feat: add gated stable posture cost"
```

---

### Task 2: Register the cost and make the task file the source of truth

**Files:**
- Modify: `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/src/MobileManipulatorInterface.cpp`
- Modify: `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/config/gensong/task_dual_ee.info`

**Interfaces:**
- Consumes: `initialState.arm`, the Pinocchio model and `stablePosture` configuration values.
- Produces: an MPC problem containing `StablePostureCost`, with the stable dual-end-effector target derived from exactly the same initial arm vector used by MRT startup.

- [ ] **Step 1: Add configuration assertions/tests before implementation**

Extend the stable-cost test with a constructor rejection case for a 18-dimensional posture and a 13-dimensional end-effector target. The test must assert `std::invalid_argument`.

Run:

```bash
colcon test --packages-select ocs2_mobile_manipulator --ctest-args -R StablePostureCostTest --event-handlers console_direct+
```

Expected: the new test fails because constructor validation is not yet present.

- [ ] **Step 2: Implement configuration and cost registration**

Change `task_dual_ee.info` so `initialState.arm` contains, in order:

```text
0.05, 0.10, 0.00, 0.00, 0.00,
0.00, 0.05, 0.00, 0.00, 0.00, 0.00, 0.00,
0.00, -0.05, 0.00, 0.00, 0.00, 0.00, 0.00
```

Add:

```text
stablePosture
{
  activate true
  jointWeight 8.0
  activationPositionTolerance 0.02
  activationOrientationTolerance 0.0872664626
}
```

After `initialState_` is loaded in `MobileManipulatorInterface`, parse the block. When active, validate that `initialState_.tail(armDim)` has 19 entries for this Gensong task, every value is finite, and every corresponding `model.lowerPositionLimit`/`upperPositionLimit` check leaves at least `1e-3` rad of margin. Compute the two stable end-effector poses from `initialState_` using the existing Pinocchio interface and package them as the 14-dimensional stable target `[left position, left quaternion.coeffs(), right position, right quaternion.coeffs()]`. Build a state weight matrix with zeros in any non-arm state entries and `jointWeight` on the arm diagonal, then add `StablePostureCost` to `problem_.costPtr` after the existing input cost.

Reject non-finite or non-positive `jointWeight`, non-finite/non-positive tolerances, incorrect 19-dimensional Gensong posture, and invalid limits with a descriptive `std::invalid_argument`.

- [ ] **Step 3: Run tests and build the interface package**

Run:

```bash
colcon build --packages-select ocs2_mobile_manipulator --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select ocs2_mobile_manipulator --ctest-args -R 'StablePostureCostTest|EndEffectorConstraintTest' --event-handlers console_direct+
```

Expected: build exits 0 and both selected tests pass.

- [ ] **Step 4: Commit the configuration/integration component**

```bash
git add src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/src/MobileManipulatorInterface.cpp \
        src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/config/gensong/task_dual_ee.info
git commit -m "feat: configure Gensong stable joint posture"
```

---

### Task 3: Make post-release return target the fixed posture and hold it

**Files:**
- Modify: `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/include/ocs2_mobile_manipulator_ros/PostReleaseInitialReturnHelpers.h`
- Modify: `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/test/testPostReleaseInitialReturnHelpers.cpp`

**Interfaces:**
- Consumes: current end-effector poses, post-release retreat poses, fixed stable end-effector poses, scaled transition duration, and positive stable hold duration.
- Produces: four waypoint stages per arm: current, aligned retreat, stable pose, stable pose hold.

- [ ] **Step 1: Write the failing hold-waypoint test**

Change the helper test call to pass `0.8` seconds of hold time and assert:

```cpp
const auto trajectory = buildPostReleaseInitialReturnTrajectory(
    currentPoses, retreatPoses, taskStartPoses, 1.2, 0.5, 0.8);
ASSERT_EQ(trajectory.armWaypoints[0].size(), 4u);
ASSERT_EQ(trajectory.timeOffsets.size(), 4u);
EXPECT_DOUBLE_EQ(trajectory.timeOffsets[0], 0.0);
EXPECT_DOUBLE_EQ(trajectory.timeOffsets[1], 0.3);
EXPECT_DOUBLE_EQ(trajectory.timeOffsets[2], 0.6);
EXPECT_DOUBLE_EQ(trajectory.timeOffsets[3], 1.0);
EXPECT_TRUE(trajectory.armWaypoints[0][2].position.isApprox(taskStartPoses[0].position));
EXPECT_TRUE(trajectory.armWaypoints[0][3].position.isApprox(taskStartPoses[0].position));
for (size_t i = 1; i < trajectory.timeOffsets.size(); ++i) {
  EXPECT_LT(trajectory.timeOffsets[i - 1], trajectory.timeOffsets[i]);
}
```

Run:

```bash
colcon test --packages-select ocs2_mobile_manipulator_ros --ctest-args -R PostReleaseInitialReturnHelpers --event-handlers console_direct+
```

Expected: compilation fails because the helper still accepts six arguments and returns three stages.

- [ ] **Step 2: Implement the hold stage**

Change the helper signature to:

```cpp
buildPostReleaseInitialReturnTrajectory(
    const std::array<PoseT, 2>& currentPoses,
    const std::array<PoseT, 2>& retreatPoses,
    const std::array<PoseT, 2>& stablePoses,
    double dtPostReleaseRetreatToInitial,
    double trajectoryTimeScale,
    double stablePoseHoldSec)
```

Compute `transitionDuration = trajectoryTimeScale * dtPostReleaseRetreatToInitial` and `holdDuration = trajectoryTimeScale * stablePoseHoldSec`. Set offsets to `{0.0, 0.5 * transitionDuration, transitionDuration, transitionDuration + holdDuration}`. Use `makeBaseLinkAlignedPose(retreatPoses[armIndex])` only for the retreat waypoint; both final waypoints must preserve the fixed stable pose orientation exactly. Throw `std::invalid_argument` for non-finite or non-positive transition/hold inputs.

- [ ] **Step 3: Run the helper test and verify it passes**

Run the same `colcon test` command. Expected: `PostReleaseInitialReturnHelpers` passes with zero failures.

- [ ] **Step 4: Commit the helper component**

```bash
git add src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/include/ocs2_mobile_manipulator_ros/PostReleaseInitialReturnHelpers.h \
        src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/test/testPostReleaseInitialReturnHelpers.cpp
git commit -m "feat: hold fixed stable pose after return"
```

---

### Task 4: Load the fixed posture in the waypoint planner

**Files:**
- Modify: `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/src/DualArmGraspWaypointPlanner.cpp`

**Interfaces:**
- Consumes: `initialState.arm` loaded in `main`, the Pinocchio interface, and a `stable_posture_hold_sec` node parameter.
- Produces: planner state with fixed stable joint/end-effector targets and return waypoints matching the OCS2 cost activation target.

- [ ] **Step 1: Add a failing source-level regression test**

Add a small Python test under `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/test/testStablePosturePlannerSource.py` that reads the planner source and asserts the implementation contains all of these behavior anchors:

```python
assert "initialState.arm" in source
assert "stable_posture_hold_sec" in source
assert "buildPostReleaseInitialReturnTrajectory" in source
assert "stablePostureEndEffectorPoses" in source
assert "Cached fixed pre-transport end-effector poses" in source
```

Run it with `python3 -m pytest ...`; it must fail because the new symbols/messages do not exist.

- [ ] **Step 2: Load and validate `initialState.arm`**

Extend the planner constructor to receive `vector_t stablePosture`. In `main`, load it with:

```cpp
vector_t stablePosture = vector_t::Zero(pinocchioInterface.getModel().nq);
ocs2::loadData::loadEigenMatrix(taskFile, "initialState.arm", stablePosture);
```

Pass it into the planner. Validate its dimension against `pinocchioInterface_.getModel().nq`, require exactly 19 values for this Gensong planner, require finite values, and check each value against the Pinocchio model limits with `1e-3` rad margin. Declare `stable_posture_hold_sec` with default `1.0` and require it to be finite and positive.

- [ ] **Step 3: Derive stable end-effector poses from the same joint vector**

Add a Pinocchio helper next to `computeCurrentEndEffectorPoses()`:

```cpp
std::array<PoseData, 2> computeEndEffectorPosesFromState(const vector_t& state);
```

Use `pinocchioMapping_.getPinocchioJointPosition(state)`, `forwardKinematics`, `updateFramePlacements`, and the existing `eeFrameIds_`. Store the normalized result in `stablePostureEndEffectorPoses_`.

Replace the first-observation cache behavior with a stable-target cache: `cacheTaskStartPoseIfNeededUnlocked()` should copy `stablePostureEndEffectorPoses_` into the existing return-target storage on first observation and log `Cached fixed pre-transport end-effector poses`. Do not use `latestObservation_` to define the final return target.

- [ ] **Step 4: Use the fixed stable target and hold duration in return planning**

Call the updated helper from `buildInitialReturnWaypoints()` with the fixed stable pose storage, `dtPostReleaseRetreatToInitial_`, `trajectoryTimeScale_`, and `stablePostureHoldSec_`. Keep the existing current and post-release retreat poses as the first two stages. Update the service response text from `return-initial` to `return-stable-posture`, including the fixed 19-joint target vector in the log/message so the operator can verify which target was used.

- [ ] **Step 5: Run planner source test and compile the ROS package**

Run:

```bash
python3 -m pytest src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/test/testStablePosturePlannerSource.py -q
python3 -m py_compile src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/launch/grasp_waypoint_planner.launch.py
colcon build --packages-select ocs2_mobile_manipulator_ros --cmake-args -DBUILD_TESTING=ON
```

Expected: the source test, Python syntax check, and ROS package build all exit 0.

- [ ] **Step 6: Commit the planner component**

```bash
git add src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/src/DualArmGraspWaypointPlanner.cpp \
        src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/test/testStablePosturePlannerSource.py
git commit -m "feat: return grasp planner to fixed stable posture"
```

---

### Task 5: Wire launch parameters and update operator documentation

**Files:**
- Modify: `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/launch/grasp_waypoint_planner.launch.py`
- Modify: `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/launch/grasp_waypoint_test.launch.py`
- Modify: `src/README_gensong_wheel_end_effector_tracking.md`

**Interfaces:**
- Consumes: planner parameter `stable_posture_hold_sec`.
- Produces: both supported launch files pass the same positive hold duration and docs describe fixed-joint behavior.

- [ ] **Step 1: Add launch-source regression assertions**

Extend the launch source test to load both launch files and assert each has:

```python
assert "stable_posture_hold_sec" in source
assert "stable_posture_hold_sec': launch.substitutions.LaunchConfiguration('stable_posture_hold_sec')" in source
```

Run the test before changing launch files; expected: failure for both files.

- [ ] **Step 2: Wire the parameter**

In both launch files, declare:

```python
launch.actions.DeclareLaunchArgument(
    name='stable_posture_hold_sec',
    default_value='1.0'
),
```

and add the matching planner node parameter:

```python
{
    'stable_posture_hold_sec': launch.substitutions.LaunchConfiguration('stable_posture_hold_sec')
},
```

Do not add a second copy of the 19 joint vector to either launch file; it remains in `task_dual_ee.info`.

- [ ] **Step 3: Update the Chinese operator documentation**

Document that:

- the 19 values in `initialState.arm` are the fixed pre-transport posture;
- launch startup uses that posture through the hardware MRT initial state;
- `/continue_return_to_initial_pose` returns to that posture and holds it;
- the service no longer means “return to the first observed end-effector position”.

Include the command:

```bash
ros2 service call /continue_return_to_initial_pose std_srvs/srv/Trigger "{}"
```

and state that joint-angle verification, not only end-effector position verification, is required.

- [ ] **Step 4: Run launch syntax and source tests**

Run:

```bash
python3 -m py_compile src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/launch/grasp_waypoint_planner.launch.py \
    src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/launch/grasp_waypoint_test.launch.py
python3 -m pytest src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/test/testStablePosturePlannerSource.py -q
```

Expected: all commands exit 0.

- [ ] **Step 5: Commit launch and documentation changes**

```bash
git add src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/launch/grasp_waypoint_planner.launch.py \
        src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/launch/grasp_waypoint_test.launch.py \
        src/README_gensong_wheel_end_effector_tracking.md
git commit -m "docs: expose fixed stable posture return behavior"
```

---

### Task 6: Full verification and runtime acceptance checks

**Files:**
- Verify all files changed by Tasks 1–5; do not modify unrelated dirty files.

- [ ] **Step 1: Inspect the complete diff and run whitespace checks**

Run:

```bash
git diff --check
git status --short
```

Confirm the new commits contain only the planned files and the pre-existing unrelated work remains untouched.

- [ ] **Step 2: Build both packages with tests enabled**

Run:

```bash
colcon build --packages-select ocs2_mobile_manipulator ocs2_mobile_manipulator_ros --cmake-args -DBUILD_TESTING=ON
```

Expected: exit code 0 for both packages.

- [ ] **Step 3: Run all focused and existing tests**

Run:

```bash
colcon test --packages-select ocs2_mobile_manipulator ocs2_mobile_manipulator_ros --event-handlers console_direct+
colcon test-result --verbose
```

Expected: no failed tests. Specifically verify `StablePostureCostTest`, `PostReleaseInitialReturnHelpers`, `GraspLiftPoseHelpers`, `CarryHomePoseHelpers`, and `JointStateHardwareBridge`.

- [ ] **Step 4: Run the available runtime smoke test**

With exactly one Gensong/MuJoCo stack running, start:

```bash
ros2 launch ocs2_mobile_manipulator_ros grasp_waypoint_planner.launch.py
```

After the initial MPC policy is active, inspect `/gensong/joint_states` and verify the 19 controlled joints move toward the configured vector. After the place phase reaches `WAITING_FOR_INITIAL_RETURN`, run:

```bash
ros2 service call /continue_return_to_initial_pose std_srvs/srv/Trigger "{}"
```

Verify the service succeeds, the planner logs four return stages including a stable hold, and all 19 controlled joint errors remain within the chosen runtime tolerance during the hold. If the hardware/MuJoCo stack is unavailable, report that runtime check as not run rather than claiming completion.

- [ ] **Step 5: Perform final verification before reporting completion**

Re-read the design checklist and report concrete command output for build, tests, launch syntax, and runtime checks. Do not claim that the robot reaches the posture unless the joint-state runtime observation confirms it.
