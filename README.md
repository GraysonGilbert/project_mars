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

## **Overview**

Project MARS is a scalable multi-robot mapping and exploration system that uses **ROS 2 Humble**, **Webots**, and **slam_toolbox** to generate a unified global map of a large indoor environment.

The system simulates **10 TurtleBot3 Waffle robots**, each performing SLAM inside its own ROS namespace. A central **Overseer Node** fuses each robot’s `map` topic into a single `/global_map`, enabling rapid, facility-scale mapping and digital-twin generation.

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
│   ├── my_controller/         # (From boilerplate) Example controller / pattern
│   └── my_model/              # (From boilerplate) Example model library
├── .github/                   # CI, test, docs build configurations
├── docs/                      # (Will contain UML + documentation)
├── README.md                  # This file
└── this-is-a-colcon-workspace
```

This structure is designed to be compatible with GitHub Actions, CodeCov, and documentation generation, per the course requirements.

---

## **System Architecture**

### **1. Per-Robot SLAM Stack**

Each TurtleBot runs:

* `/robot_i/scan` (LiDAR)
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
```

---

## **Running the System**

*(Sprint 2 - placeholder for now)*

Expected final commands:

```bash
# Launch Webots multi-robot world
ros2 launch mars_fleet_bringup multi_robot_webots.launch.py

# Visualize fused map
ros2 launch mars_fleet_bringup rviz_global_map.launch.py
```

Optionallly you can run individual packages in this workspace:

```bash
# Run the mars_exploration single robot SLAM demo
ros2 launch mars_exploration single_robot.launch.py

# Run the mars_overseer node
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

# Phase 1

1) [AIP Backlog](https://docs.google.com/spreadsheets/d/1VFT9h6v-TJoIZZqw14fOZrGEaPpVfkw-nTQ9vnsXJHE/edit?gid=241005242#gid=241005242)

2) [Sprint 1 Notes](https://docs.google.com/document/d/1vtnkgUcWeYFP_rzQFlWj7m7FjFuriQDOsglt_6psIaA/edit?tab=t.0)

---

## **License**

This project is licensed under the **MIT License**.

Starter template from:
[https://github.com/TommyChangUMD/ENPM700-final-project-boilerplate](https://github.com/TommyChangUMD/ENPM700-final-project-boilerplate)