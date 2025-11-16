# ROS 2 Workspace Onboarding Guide

## Overview
This repository provides a pre-configured [ROS 2 Humble](https://docs.ros.org/en/humble/) workspace that is managed with [pixi](https://prefix.dev/). It is meant to give robotics teams a consistent, reproducible starting point for simulation, perception, and motion-planning projects. The workspace bundles commonly used desktop tooling (RViz, Gazebo), navigation stacks, MoveIt, and developer utilities so that new contributors can immediately focus on building nodes instead of configuring their environment.

Key capabilities include:
- Deterministic environments powered by pixi lockfiles and conda-forge/robostack channels.
- Ready-to-use build tasks for both Python (`ament_python`) and C++ (`ament_cmake`) ROS 2 packages.
- Helper scripts for importing repositories, building with `colcon`, and managing workspace state.

## Installation
1. **Install pixi** (if you have not already) by following the instructions at <https://prefix.dev/docs/pixi/getting_started>. Ensure pixi is on your `PATH`.
2. **Create the environment** from the project root:
   ```bash
   pixi install
   ```
   This resolves every dependency declared in `pixi.toml`/`pixi.lock`, including ROS 2 Humble desktop packages, Gazebo, MoveIt, and development tools such as `colcon` and `clang-format`.
3. **Activate the workspace**:
   ```bash
   pixi shell
   ```
   Activation runs `install/setup.sh` and `env.sh`, exporting variables like `RMW_IMPLEMENTATION` so ROS nodes communicate through Cyclone DDS.
4. **Initialize or import ROS packages**:
   - To start with an empty workspace: `pixi run ws_init`
   - To import repositories listed in `ws.repos`: `pixi run ws_import`
   These commands populate the `src/` folder using `vcs`.
   - The import task now passes `--recursive` so that submodules are fetched automatically. This is required for repositories
     such as `moveit_task_constructor` (pulls in `pybind11` and `scope_guard`) and `xarm_ros2` (pulls in the `xArm-CPLUS-SDK`).
5. **Build the workspace**:
   ```bash
   pixi run build
   ```
   The default task wraps `colcon build --symlink-install`. You can also run `pixi run ws_build` to build from the project root or pass custom CMake arguments (e.g., `pixi run build --cmake-args -G Ninja`).
6. **Run ROS demos** (optional): for example, launch the turtlesim demo with `pixi run turtle`.

> **Tip:** If you need CUDA-enabled packages, target the `cuda` pixi environment (`pixi shell -e cuda`) which installs additional dependencies such as `jax` and `roboreg`.

## Data
The workspace expects all ROS packages inside the `src/` directory. Repository sets are managed through `.repos` manifests:
- `ws.repos` – default collection of packages to import.
- `ws.pinned.repos` – exact commit pins generated via `pixi run ws_pin`.

Use the bundled tasks to keep the workspace synchronized:
- `pixi run ws_status` to check repository states.
- `pixi run ws_pull` to update imported packages.
- `pixi run ws_export` to snapshot current sources into a `.repos` file.

Simulation assets, bag files, and other large datasets should be stored outside the git repository (e.g., under `~/ros_data`) and referenced from package configuration. Document any additional data requirements in the relevant package README.

## Contributing
- Create feature branches from `main` using the format `feature/<short-description>` or `fix/<short-description>`.
- Keep commits focused and descriptive. Use conventional commit prefixes when possible (e.g., `feat:`, `fix:`, `docs:`).
- Before opening a pull request, ensure the workspace builds cleanly: `pixi run build` and run any package-specific tests.
- Format C++ code with `ament_clang_format` and Python code with `ruff` if available in the package; ROS launch files should remain readable and well-commented.

When submitting a PR:
1. Rebase onto the latest `main` and resolve conflicts locally.
2. Provide context in the PR description—include affected packages, testing steps, and any data requirements.
3. Request review from another team member. Reviews should check for build success, adherence to ROS best practices, and updated documentation.

For new contributors, start by reading the `setup/` scripts and `pixi.toml` to understand environment configuration, then try building and running the sample tasks above. Do not hesitate to ask for access to any repositories referenced in `ws.repos`.
