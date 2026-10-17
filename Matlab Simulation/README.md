ACMFDS Basic understanding :
==========================

ABOUT
-----
ACMFDS (Adaptive Context-Aware Multi-Objective Flight Decision System) is a
MATLAB simulation prototype for studying drone path planning under obstacles,
wind, battery constraints, and mission deadlines.

REQUIREMENTS
------------
- MATLAB R2024a or a compatible newer version.
- Keep the supplied MATLAB source and any companion files in the same folder.
- This is a simulation prototype, not flight-ready autopilot software.

HOW TO RUN
----------
1. Copy the project files into your project folder.
2. Open MATLAB and set the Current Folder to that location, or run:

   cd('file_path')

3. Start the application:

   ACMFDS_Scientific_Reviewed

CONTROLS
--------
- NEW WORLD: Generate a new obstacle layout.
- REPLAN: Calculate a route with the current settings.
- RUN MISSION: Simulate a mission using the selected planner.
- STOP: Stop the current simulation.
- COMPARE ROUTES: Compare planning strategies in the same environment.
- VEHICLE / MISSION PARAMETERS: Edit the assumed vehicle and mission values.
- Sliders: Adjust obstacle count, moving obstacles, wind, initial battery,
  deadline, and safety margin.

WHAT IT MODELS
--------------
- Grid-based A* route planning.
- Stationary and moving obstacles with route replanning.
- Wind-dependent ground-speed and steady-flight energy estimates.
- Battery-reserve and mission-deadline checks.
- Recovery decisions such as returning home or selecting a simulated landing site.

LIMITATIONS
-----------
- The planner uses a 2D horizontal flight layer; it is not a true 3D path planner.
- The power model is a steady-flight research model and is not calibrated to a
  specific drone unless validated with suitable vehicle data.
- Battery accounting is an energy/SOC estimate, not a full electrochemical or
  equivalent-circuit battery model.
- Moving obstacle paths and sensing are simulated; real sensor uncertainty,
  communication failures, and flight-controller dynamics are not fully modelled.
- Results from the simulation must not be represented as real-flight performance.

