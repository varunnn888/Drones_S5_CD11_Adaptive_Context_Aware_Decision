
<img width="1948" height="568" alt="logo-branding-amrita-universiy-2024" src="https://github.com/user-attachments/assets/db019883-cfb1-46ad-9811-7f7ec523249e" />

<h1> Adaptive Context-Aware Multi-Objective Flight Decision Algorithm</h1>

## Team Members

| S. No. | Name | Roll Number | Email |
|--------|------|-------------|-------|
| 1 | Deepika Reddy | cb.sc.u4aie24215 | [cb.sc.u4aie24215@cb.students.amrita.edu](mailto:cb.sc.u4aie24215@cb.students.amrita.edu) |
| 2 | Gade Varshini | cb.sc.u4aie24216 | [cb.sc.u4aie24216@cb.students.amrita.edu](mailto:cb.sc.u4aie24216@cb.students.amrita.edu) |
| 3 | Jovika N B | cb.sc.u4aie24223 | [cb.sc.u4aie24223@cb.students.amrita.edu](mailto:cb.sc.u4aie24223@cb.students.amrita.edu) |
| 4 | Poojitha P | cb.sc.u4aie24237 | [cb.sc.u4aie24237@cb.students.amrita.edu](mailto:cb.sc.u4aie24237@cb.students.amrita.edu) |
| 5 | Potnuru Varun | cb.sc.u4aie24244 | [cb.sc.u4aie24244@cb.students.amrita.edu](mailto:cb.sc.u4aie24244@cb.students.amrita.edu) |

## Abstract

This project proposes an Adaptive Contextual Multi-Factor Decision System (ACMFDS) for autonomous UAV navigation in dynamic environments. The system continuously evaluates battery level, wind conditions, obstacle distance, mission priority, safety, energy, time, and tracking requirements to determine appropriate flight behavior. Based on the changing context, it dynamically adjusts decision weights, flight speed, operating mode, and navigation strategy. When obstacles block the direct path, an adaptive path planner evaluates alternative routes based on distance, clearance, and cost, while unsafe routes are rejected. The system also responds to environmental changes such as nearby obstacles and high wind by switching to appropriate safety-oriented modes and replanning the mission. A simulation demonstrates successful context detection, adaptive decision-making, obstacle avoidance, dynamic path planning, waypoint navigation, and mission completion while maintaining energy efficiency and safety. Thus, ACMFDS provides a unified framework for intelligent, context-aware, and adaptive UAV mission planning and autonomous navigation.



## Introduction

ACMFDS (Adaptive Contextual Multi-Factor Decision System) is a MATLAB/Simulink-based framework for autonomous UAV navigation in dynamic environments. It considers battery level, wind speed, obstacle distance, and mission priority to identify the current context and adapt the UAV's speed, flight mode, and navigation action. The system uses five factors—Energy, Safety, Time, Tracking, and Mission—with context-dependent weights to make balanced decisions.

The overall decision cost is represented as:

$$C = \sum_i w_i f_i$$

| Symbol | Description |
|--------|-------------|
| **C** | Overall decision / path cost |
| **w_i** | Importance (weight) of each factor |
| **f_i** | Value / cost of that factor |

When the direct path is blocked, the planner evaluates alternative routes using distance, clearance, and cost, rejects unsafe routes, and selects a feasible detour.

The project integrates **MATLAB**, **Simulink**, and **Stateflow** to demonstrate context detection, adaptive decision-making, obstacle avoidance, dynamic path planning, waypoint navigation, changing environmental conditions, and successful mission completion.
