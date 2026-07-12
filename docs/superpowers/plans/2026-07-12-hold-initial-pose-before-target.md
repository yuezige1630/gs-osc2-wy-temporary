# Hold Initial Pose Before Grasp Target Implementation Plan

> **For agentic workers:** Inline execution in this session; preserve unrelated existing worktree changes and do not reset or commit them.

**Goal:** Keep the Gensong robot at its first observed joint pose until a real MPC target trajectory is published, preventing the bootstrap MPC policy from moving to an arbitrary redundant posture.

**Architecture:** The hardware MRT node subscribes to the existing `mobile_manipulator_mpc_target` target topic. Before the first target callback, it publishes the latest complete observed arm pose instead of the bootstrap MPC rollout; after the callback, it returns to normal MPC command generation. A small helper makes the gate behavior unit-testable.

**Tech Stack:** ROS 2 rclcpp, `ocs2_msgs/msg/MpcTargetTrajectories`, Eigen, existing GoogleTest target.

## Global Constraints

- Preserve all unrelated uncommitted user changes.
- Do not change the MuJoCo scene geometry or gravity to mask the MPC bootstrap behavior.
- Keep the existing command filtering and target-trajectory topic unchanged.

---

### Task 1: Add a testable initial-pose gate

**Files:**
- Modify: `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/include/ocs2_mobile_manipulator_ros/JointStateHardwareBridgeHelpers.h`
- Test: `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/test/testJointStateHardwareBridge.cpp`

- [x] Add a helper that returns true only while no external target has been received.
- [x] Add a focused test showing the gate holds before a target and opens after one.
- [x] Run the focused test and confirm the pre-change failure.

### Task 2: Gate MRT commands until a target trajectory arrives

**Files:**
- Modify: `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/src/MobileManipulatorHardwareMRT.cpp`

- [x] Include `ocs2_msgs/msg/mpc_target_trajectories.hpp`.
- [x] Subscribe to `mobile_manipulator_mpc_target` with the same topic prefix used by the MPC interface.
- [x] Initialize the held pose from the first complete hardware observation.
- [x] While the gate is closed, publish that observed arm pose; after a target callback, use the existing MPC rollout and filtering path.
- [x] Run the focused test and build the affected ROS package.

### Task 3: Verify the original symptom and regression behavior

- [x] Start MuJoCo and the grasp planner with ROS logging redirected into the workspace.
- [x] Confirm the first command remains at the initial pose before any target.
- [x] Publish a test target and confirm command output is no longer held by the gate.
- [x] Stop diagnostic processes and report exact verification evidence.
