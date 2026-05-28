# OCS2 Toolbox (ROS 2)

<<<<<<< HEAD
This branch (`ros2`) ports OCS2 from ROS 1/catkin to **ROS 2 (colcon/ament)** and currently targets **Ubuntu 24.04 + ROS 2 Jazzy**.
For the ROS 1 version, see branch `main`.

![legged-robot](https://leggedrobotics.github.io/ocs2/_static/gif/legged_robot.gif)

OCS2 (**O**ptimal **C**ontrol for **S**witched **S**ystems) is a C++ toolbox for formulating and solving nonlinear optimal control problems, with an emphasis on real-time **Model Predictive Control (MPC)** for robotics.

OCS2 handles general path constraints through Augmented Lagrangian or relaxed barrier methods. To facilitate the application of OCS2 in robotic tasks, it provides tools to set up the system dynamics (such as kinematic or dynamic models) and cost/constraints (such as self-collision avoidance and end-effector tracking) from a URDF model. The library also provides an automatic differentiation tool to calculate derivatives of the system dynamics, constraints, and cost. To facilitate deployment on robotic platforms, OCS2 provides tools for ROS interfaces.

## What’s included

- **Optimal control solvers**
  - **SLQ**: continuous-time constrained DDP
=======
OCS2 (**O**ptimal **C**ontrol for **S**witched **S**ystems) is a C++ toolbox for formulating and solving nonlinear optimal control problems, with an emphasis on real-time **Model Predictive Control (MPC)** for robotics.

![legged-robot](https://leggedrobotics.github.io/ocs2/_static/gif/legged_robot.gif)

> **ROS 2 Jazzy port available:** use branch `ros2` (Ubuntu 24.04, colcon/ament). See **ROS 1 vs ROS 2** below.

## Highlights

- **Multiple solvers**
  - **SLQ**: continuous-time constrained DDP (SLQ/DDP family)
>>>>>>> 97a6bac1c (docs: expand README and note ROS 2 port)
  - **iLQR**: discrete-time constrained DDP
  - **SQP**: multiple-shooting SQP (QP subproblems via HPIPM/BLASFEO)
  - **SLP**: sequential linear programming (PIPG)
  - **IPM**: multiple-shooting nonlinear interior-point method
<<<<<<< HEAD
- **Robotics tooling**: URDF/Pinocchio integration, kinematics, centroidal models, and self-collision constraints (HPP-FCL).
- **ROS integration**: messages, nodes, and visualization/plotting tools for deploying MPC on robots.
- **Robotic examples**: end-to-end MPC examples (and ROS wrappers) for double integrator, cartpole, ballbot, quadrotor, mobile manipulator, legged robot, and more.

## Port status (ROS 2 Jazzy)

- This branch replaces catkin with ament/colcon across the workspace.
- `blasfeo_catkin` / `hpipm_catkin` keep their names but are ROS 2 `ament_cmake` packages.
- `ocs2_mpcnet` and `ocs2_raisim` are currently not ported and are ignored via `COLCON_IGNORE`.
- `rqt_multiplot` is not released for Jazzy on Ubuntu 24.04; multiplot launch files are optional.

## Installation

Follow the branch-specific instructions in `installation.md` (dependencies, Pinocchio on Jazzy, and colcon build):
- [`installation.md`](installation.md)

Optional Docker environment:
- [`docker/README.md`](docker/README.md)

## Run an example

After building and sourcing your workspace:

```bash
ros2 launch ocs2_ballbot_ros ballbot_mpc_mrt.launch.py
```

## Documentation

Project documentation is hosted at:
- https://leggedrobotics.github.io/ocs2/

Note: the online docs still describe the ROS 1/catkin workflow; the solver concepts and APIs are largely identical, but build/launch instructions differ on this branch.
=======
- **Switched-system OCP support**: mode schedules and jump maps (single- and multi-domain problems).
- **Constraints**: hard/soft constraints with Augmented Lagrangian and relaxed barrier methods.
- **Derivatives**: analytic derivative interfaces + automatic differentiation (CppAD) and optional code generation (CppADCodeGen).
- **Robotics tooling**: URDF→model helpers (Pinocchio), kinematics, centroidal models, self-collision constraints (HPP-FCL).
- **ROS integration**: messages, nodes, and visualization/plotting tools for deploying MPC on robots.
- **Robotic examples**: end-to-end MPC examples (and ROS wrappers) for double integrator, cartpole, ballbot, quadrotor, mobile manipulator, legged robot, and more.

## Repository structure (high level)

- `ocs2_core`: core data types, math utilities, rollout interfaces, AD utilities.
- `ocs2_oc`: optimal control problem building blocks (costs, constraints, dynamics, pre-computation, reference management).
- `ocs2_ddp`, `ocs2_mpc`: DDP-family solvers + MPC infrastructure and runtime interfaces.
- `ocs2_sqp`, `ocs2_slp`, `ocs2_ipm`: alternative solvers (SQP / SLP / IPM).
- `ocs2_pinocchio/*`: URDF/Pinocchio tooling (centroidal model, kinematics, self-collision, visualization).
- `ocs2_ros_interfaces`, `ocs2_msgs`: ROS interfaces and message definitions.
- `ocs2_robotic_examples/*`: robotic examples and their ROS nodes.
- `ocs2_python_interface`: Python bindings for selected components.
- `ocs2_mpcnet`: MPC-Net tooling for learned policy deployment/training.
- `ocs2_doc`: documentation sources (Sphinx/Doxygen).

## Documentation

- Online documentation: https://leggedrobotics.github.io/ocs2/

## ROS 1 vs ROS 2

OCS2 historically targets **ROS 1 / catkin** (documented/tested primarily on **Ubuntu 20.04 + ROS Noetic**).

A **ROS 2 Jazzy** port (**Ubuntu 24.04**, **colcon/ament**) is available in this repository on branch `ros2`.

### ROS 2 Jazzy (branch `ros2`)

- Build instructions: https://github.com/leggedrobotics/ocs2/blob/ros2/installation.md
- Optional Jazzy Docker environment: https://github.com/leggedrobotics/ocs2/tree/ros2/docker
- Robotic assets: use the `ros2` branch of https://github.com/leggedrobotics/ocs2_robotic_assets

### ROS 1 Noetic (branch `main`)

Follow the installation instructions in the documentation:
- https://leggedrobotics.github.io/ocs2/ (Installation page)

## Try an example

After building and sourcing your workspace:

- **ROS 1 (Noetic, `main`)**
  ```bash
  roslaunch ocs2_cartpole_ros cartpole.launch
  ```

- **ROS 2 (Jazzy, `ros2`)**
  ```bash
  ros2 launch ocs2_cartpole_ros cartpole.launch.py
  ```
>>>>>>> 97a6bac1c (docs: expand README and note ROS 2 port)

## Citing OCS2

```latex
@misc{OCS2,
  title  = {{OCS2}: An open source library for Optimal Control of Switched Systems},
  note   = {[Online]. Available: \url{https://github.com/leggedrobotics/ocs2}},
  author = {Farbod Farshidian and others}
}
```

## License

BSD 3-Clause, see `LICENCE.txt`.
