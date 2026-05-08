# Project Brief

## Project name
ROS 2 simulation and control workflow for a carnation harvesting robot based on YOLOv8

## Project objective
This project is a graduation design focused on software design and simulation only.
It does NOT require physical hardware implementation.
The goal is to build a ROS 2 based simulation pipeline for a carnation harvesting robot, including:
1. flower detection with YOLOv8
2. target position acquisition
3. pixel-to-3D coordinate conversion under defined assumptions
4. end-effector motion planning
5. simulation of grasping / cutting / reset workflow

## Academic scope and constraints
- This is a digital prototype validation project.
- Main outputs are:
  - YOLOv8 detection results and metrics
  - 3D digital model of the end-effector
  - ROS 2 simulation workflow
  - motion planning and state-machine logic
  - optional simplified scene in Gazebo / RViz
- No real robot hardware
- No real controller deployment
- No real actuator commissioning
- No physical harvesting test

## System chain
Standard software chain:
YOLOv8 -> pixel coordinates -> spatial coordinates -> target pose -> motion planning -> joint / trajectory commands

## Recommended simulation strategy
Use a simplified target scene instead of full realistic plants.
Represent carnation targets with simplified stems and flower heads if needed.

## Robot concept
- dual-arm harvesting structure
- upper arm: stabilizing / holding
- lower arm: cutting
- focus on action sequence feasibility rather than full agricultural robustness

## Technical priorities
1. ROS 2 package architecture
2. camera topic processing
3. detection result subscription / publication
4. 2D-to-3D conversion
5. TF tree and target pose publishing
6. MoveIt motion planning
7. state machine for approach / hold / cut / retreat
8. Gazebo or RViz-based validation

## Preferred stack
- Ubuntu
- ROS 2
- Python for perception nodes
- C++ or Python for motion / coordination nodes
- OpenCV / NumPy for coordinate computation
- YOLOv8 for detection
- MoveIt for motion planning
- Gazebo and RViz for simulation / visualization

## Current status
- main 3D structure is mostly modeled
- need to organize the ROS 2 software architecture and development workflow
- need Codex to help scaffold packages, nodes, launch files, configs, message flow, and debug issues incrementally

## Deliverable style
Prioritize engineering clarity, modular ROS 2 structure, and simulation feasibility.
Avoid overengineering.
Build a minimum viable simulation pipeline first, then refine.