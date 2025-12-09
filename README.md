# **Project MARS – Multi-Agent Robotic SLAM**

**ENPM700 Final Project – University of Maryland**

![CICD Workflow Status](https://github.com/GraysonGilbert/project_mars/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg)
[![codecov](https://codecov.io/gh/GraysonGilbert/project_mars/branch/main/graph/badge.svg?token=YOURTOKEN)](https://codecov.io/gh/GraysonGilbert/project_mars)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

**Team Members:**

* Marcus Hurt
* Grayson Gilbert

**Project Code Name:** *Project MARS* (Multi-Agent Robotic SLAM)

---
## Table of Contents

1. [Overview](#overview)
2. [Repository Layout](#repository-layout)
3. [System Architecture](#system-architecture)
   - [Per-Robot SLAM Stack](#1-per-robot-slam-stack)
   - [Overseer Node (`mars_overseer`)](#2-overseer-node-mars_overseer)
   - [Exploration (`mars_exploration`)](#3-exploration-mars_exploration)
   - [Bringup (`mars_fleet_bringup`)](#4-bringup-mars_fleet_bringup)
4. [Build Instructions](#build-instructions)
5. [Running the System](#running-the-system)
6. [Testing](#testing)
7. [Documentation](#documentation)
8. [Development Approach](#development-approach)
9. [Deliverables](#deliverables)
   - [Phase 1](#phase-1)
   - [Phase 2](#phase-2)
10. [License](#license)


## **Overview**

Project MARS is a scalable multi-robot mapping and exploration system that uses **ROS 2 Humble**, **Webots**, and **slam_toolbox** to generate a unified global map of a large indoor environment.

The system simulates **2 TurtleBot3 Burger robots**, each performing SLAM inside its own ROS namespace. A central **Overseer Node** fuses each robot’s `map` topic into a single `/global_map`, enabling rapid, facility-scale mapping and digital-twin generation.

This repository contains all code, documentation, test pipelines, and CI systems required for the ENPM700 Final Project.

---

## **Repository Layout**

The repository follows the colcon workspace pattern:

```
project_mars/
├── src/
│   ├── mars_overseer/         # Map fusion and global SLAM node
│   ├── mars_fleet_bringup/    # Launch files, configuration, multi-robot simulation
│   ├── mars_exploration/      # Sector-based and frontier-based exploration
├── .github/                   # CI, test, docs build configurations
├── docs/                      # Auto-generated documentation
├── UML/                       # Initial and revised UML diagrams
├── README.md                  # This file
└── this-is-a-colcon-workspace
```

This structure is designed to be compatible with GitHub Actions, CodeCov, and documentation generation, per the course requirements.

---

## **System Architecture**

### **1. Per-Robot SLAM Stack**

Each TurtleBot runs:

* `/robot_i/scan` (LiDAR)
* `/root_i/scan_corrected` (required to workaround LiDAR frame header bug)
* `/robot_i/odom`
* `/robot_i/map` (via slam_toolbox)
* REP-105 compliant TF tree
* Independent namespace (`/robot_1`, `/robot_2`, …)

### **2. Overseer Node (`mars_overseer`)**

Responsible for:

* Subscribing to all `/robot_i/map` topics
* Applying known initial transforms
* Deterministic occupancy-grid fusion
* Publishing a unified `/global_map`

### **3. Exploration (`mars_exploration`)**

Implements:

* Sector-based waypoint exploration (MVP)
* Optional frontier exploration (stretch goal)

### **4. Bringup (`mars_fleet_bringup`)**

Manages:

* Multi-robot Webots simulation
* slam_toolbox bringup for each robot
* RViz2 visualization
* Launch files and parameters

---

## **Build Instructions**

Install necessary dependencies with rosdep:

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Make sure ROS 2 Humble is sourced:

```bash
source /opt/ros/humble/setup.bash
```

Then build the workspace:

```bash
colcon build
source install/setup.bash
```

Optionally you can build individual packages in this workspace:

```bash
# Build only the mars_exploration package
colcon build --packages-select mars_exploration
source install/setup.bash

# Build only the mars_overseer package
colcon build --packages-select mars_overseer
source install/setup.bash

# Build only the mars_fleet_bringup package
colcon build --packages-select mars_fleet_bringup
source install/setup.bash
```

---

## **Running the System**
To run the default project demonstration, run the following command in the root of the project workspace:

```bash
# Launch Webots multi-robot demonstration
ros2 launch mars_fleet_bringup mars_fleet_bringup.launch.py
```

Optionallly, you can run other simulations with more or less robots with the folllowing:

```bash
# Run the 1 robot SLAM demo
ros2 launch mars_exploration mars_fleet_bringup.launch.py num_robots:1 world_file:='mars_1_robots.wbt'

# Run the 3 robot SLAM demo (Currently the 3 robot demo is not fully functional)
ros2 launch mars_exploration mars_fleet_bringup.launch.py num_robots:3 world_file:='mars_3_robots.wbt'

# Run the mars_overseer node by itself
ros2 run mars_overseer overseer_node --ros-args -p use_sim_time:=true
```

---

## **Testing**

Project MARS uses **TDD**, **GoogleTest**, and **GitHub CI**.

Run tests locally:

```bash
colcon test
colcon test-result --verbose
```

Optionally run tests for individual packages in this workspace:

```bash
# Run the mars_exploration tests
colcon test --packages-select mars_exploration
colcon test-result --verbose

# Run the mars_overseer tests
colcon test --packages-select mars_overseer
colcon test-result --verbose

# Run the mars_fleet_bringup tests
colcon test --packages-select mars_fleet_bringup
colcon test-result --verbose
```

View coverage:

```bash
./do-tests-and-coverage.bash
```

CI runs:

* Build & Test
* CodeCov upload
* Documentation generation (Doxygen)

---

## **Documentation**

Auto-generated documentation will be placed under:

```
docs/html/index.html
```

Generate locally:

```bash
./do-docs.bash
```

UML diagrams will live under:

```
docs/uml/
```

---

## **Development Approach**

* **AIP (Agile Iterative Process)**
* **Pair Programming** (driver/navigator)
* **TDD**
* **Continuous Integration**
* **One week sprint cycles**

All changes correspond to the proposal submitted to Acme Robotics for ENPM700.

---

## **Deliverables**

### Phase 1

1) [AIP Backlog](https://docs.google.com/spreadsheets/d/1VFT9h6v-TJoIZZqw14fOZrGEaPpVfkw-nTQ9vnsXJHE/edit?usp=sharing)

2) [Sprint 1 Notes](https://docs.google.com/document/d/1vtnkgUcWeYFP_rzQFlWj7m7FjFuriQDOsglt_6psIaA/edit?tab=t.0)

### Phase 2

1) [Presentation](https://youtu.be/4LAzF54b1SQ)

2) [Sprint 2 Notes](https://docs.google.com/document/d/1RzEs2etGg3aeXfAYIdBvH6DnQ7stsbbt5ecsiO3mHLo/edit?tab=t.0)

3) [Single Robot SLAM Demo](https://youtu.be/BxGX-qR2iZE)

4) [Two Robot SLAM Demo](https://youtu.be/aSOs7f2JbLM)

5) [AIP Backlog](https://docs.google.com/spreadsheets/d/1VFT9h6v-TJoIZZqw14fOZrGEaPpVfkw-nTQ9vnsXJHE/edit?gid=0#gid=0)

---

## **License**

This project is licensed under the **MIT License**.

Starter template from:
[https://github.com/TommyChangUMD/ENPM700-final-project-boilerplate](https://github.com/TommyChangUMD/ENPM700-final-project-boilerplate)