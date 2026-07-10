# MuJoCo Stable Box Lift Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a tested MuJoCo box-center parser and a stable staged dual-arm lift controller while preserving 500 Hz joint-state publication.

**Architecture:** Keep the existing ROS 2 MuJoCo bridge and add focused helpers for XML box geometry, lift phases, and bounded joint control. The simulation loop remains 500 Hz; camera output stays slower and is used for visual validation.

**Tech Stack:** Python 3, ROS 2, MuJoCo Python bindings, NumPy, unittest/pytest.

## Global Constraints

- `/gensong/joint_states` publishes at 500 Hz.
- `scene_box` position comes from `gensong_scene.xml`, not a duplicated hard-coded target.
- The lift sequence is approach, contact, grasp, lift, hold.
- Joint commands are velocity-damped and torque-limited.
- Existing uncommitted user changes must be preserved.

### Task 1: Add pure control-policy helpers

**Files:**
- Create: `src/gensong_mujoco/gensong_mujoco/box_lift_policy.py`
- Create: `src/gensong_mujoco/test/test_box_lift_policy.py`

**Interfaces:**
- `load_box_center(xml_path) -> np.ndarray`
- `LiftPhase` enum with `APPROACH`, `CONTACT`, `GRASP`, `LIFT`, `HOLD`
- `BoxLiftPolicy.next_target(now, box_center, current_target) -> (LiftPhase, np.ndarray)`
- `is_stable(box_z, initial_z, linear_speed, attitude_error) -> bool`

- [ ] Write failing tests for XML center parsing, monotonic lift target, phase progression, and stable-success thresholds.
- [ ] Run `python3 -m pytest src/gensong_mujoco/test/test_box_lift_policy.py -q` and confirm failure for missing helpers.
- [ ] Implement the smallest pure NumPy/XML helpers with explicit thresholds and no ROS imports.
- [ ] Re-run the focused tests and then the existing scene contract test.

### Task 2: Integrate bounded staged control into the MuJoCo node

**Files:**
- Modify: `src/gensong_mujoco/gensong_mujoco/gensong_mujoco_node.py`
- Modify: `src/gensong_mujoco/config/gensong_mujoco.yaml`
- Modify: `src/gensong_mujoco/README.md`

**Interfaces:**
- Preserve `/gensong/joint_command`, `/gensong/joint_states`, `/cmd_vel`, and RGB-D topics.
- Add parameters for box XML path, lift height, phase durations, proportional/derivative gains, and torque limit.

- [ ] Add a test-level static check that simulation frequency defaults to 500 Hz and the node loads `scene_box` from XML.
- [ ] Implement XML loading at startup and expose the resolved center in a log message.
- [ ] Replace unbounded fixed torque with clipped position/velocity control, while preserving named joint command overrides.
- [ ] Add staged dual-arm target generation and HOLD stabilization, with contact/grasp transitions driven by measured state.
- [ ] Run unit tests and syntax checks before launching ROS 2.

### Task 3: Run online verification

**Files:**
- Modify: `src/gensong_mujoco/test/test_scene_contract.py` only if coverage needs a narrow regression assertion.

- [ ] Build/source the package and launch headless MuJoCo at 500 Hz.
- [ ] Measure `/gensong/joint_states` with `ros2 topic hz` for at least 10 seconds; require measured rate near 500 Hz.
- [ ] Capture RGB frames and inspect at least the initial, contact, lift, and hold states.
- [ ] Record box center height and velocity from MuJoCo state; require height increase and bounded motion during hold.
- [ ] Report any environment limitations separately from code/test results.
