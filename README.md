
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

---

## Introduction

ACMFDS (Adaptive Contextual Multi-Factor Decision System) is a decision-making system designed to help a UAV (Unmanned Aerial Vehicle) fly safely and efficiently in changing environments. Instead of following one fixed flight strategy, the system continuously observes factors such as battery level, wind speed, obstacle distance, mission priority, safety, time, and tracking requirements. Based on these conditions, it identifies the current context, such as Normal, Obstacle Nearby, or High Wind, and changes the UAV's behavior accordingly.

The main idea of ACMFDS is to give different importance (weights) to different factors depending on the situation. For example, when an obstacle is nearby, safety receives a higher weight, while during normal flight the UAV can give more importance to time or energy efficiency. A simple mathematical representation of the decision process is:

$$D = \sum_{i=1}^{n} w_i f_i$$

where **D** is the overall decision score, **w_i** is the weight assigned to a factor, and **f_i** is the value of that factor. The weights are adapted according to the current context. In the simulation, the weights for Energy, Safety, Time, Tracking, and Mission were **0.09, 0.52, 0.05, 0.14, and 0.20**, respectively.

When the direct path is blocked, ACMFDS evaluates possible directions such as East, West, North, and South using their distance, obstacle clearance, and cost, and selects a safe detour. It can also change the UAV's speed and flight mode when conditions change, such as switching to **Safety Mode** during high wind and returning to **Normal Flight** when conditions improve. Thus, the project demonstrates how a UAV can observe its environment, make adaptive decisions, avoid obstacles, manage changing conditions, and complete its mission autonomously.
