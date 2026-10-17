# ACMFDS — Autonomous Quadrotor Simulation

A reference-inspired PySide6 dashboard for a MuJoCo-backed quadrotor demo with a larger arena and a fully orbitable 3D camera. The project includes local-world coordinates, simulated GPS telemetry, adaptive risk/weights, candidate route planning, static hazards, and moving dynamic hazards.

## What's included

- **Restart mission:** When the drone reaches the destination, the footer button changes to **START AGAIN ↻**. Press it to reset the quadrotor to the blue start marker, reset battery/history/time, and run the mission again.
- **Route profile options:** The `PATH PROFILE` dropdown lists available candidate routes with profile name, length, and score. Select one and press **USE ROUTE** to plan that profile from the drone's current position to the destination.
- **Customizable static and dynamic obstacles:** Drag red static (S1–S6) or orange dynamic (D1–D2) hazards directly in the top-view XY map. The **CUSTOMIZE OBSTACLE** controls also let you select either type and edit X/Y coordinates precisely. Releasing or applying changes updates MuJoCo and planner state, then recalculates a collision-aware route. Dynamic hazards additionally let you customize patrol axis (X/Y sweep), travel range (± metres), and angular rate (rad/s), while retaining their moving behavior.
- **Movable destination:** Drag the GREEN target marker in the same top-view XY map to change the endpoint. The 3D target pad and mission coordinates update, the route replans to the new destination, and distance/progress telemetry follows the new target. Locations inside the planner safety margin around obstacles or too close to the drone are rejected. The blue start point remains fixed.
- **Dynamic obstacles:** Orange cylinders move along user-customizable repeatable paths. The **Dynamic obstacles** checkbox enables/disables them; while enabled, the simulation periodically replans to account for their moving positions. The quadrotor pauses while any obstacle or the green destination marker is being dragged, preventing it from continuing along an outdated path during the edit.
- **Flight conditions (wind + battery):** The **FLIGHT CONDITIONS** row lets you set simulated wind speed (0–12 m/s), wind direction (0–359°), and current battery (0–100%). Press **APPLY CONDITIONS** to apply them live. Wind is a world-frame external force (0° pushes +X/east; 90° pushes +Y/north), increases risk/energy use, slows the commanded flight speed in stronger winds, and causes safety-weighted replanning. Battery below 35% favours energy conservation and slower flight; ACMFDS routes back to the blue home/start pad when battery reaches a wind-adjusted reserve threshold (25% in calm conditions, increasing to a maximum of 43% in severe wind), then descends to land. At 8% or below, it performs a simulated emergency landing at its current XY location. These are demonstration policies, not a real-drone safety guarantee.
- **Manual replan:** Press **REPLAN** to rank routes against the currently enabled obstacles.
- **360-degree view:** Click **AUTO 360° ORBIT** for continuous camera rotation; drag with the left mouse button to orbit manually, scroll to zoom, and use **OVERVIEW**, **TOP**, or **SIDE** for camera presets.
- **Larger arena:** The environment is expanded to about 16.5 m × 14.5 m, with the target moved to (12, 10, 2) m and hazards/buildings spread out to give the drone more room to navigate.
- **Route menu:** Static+dynamic is the default; switching either checkbox filters the planning obstacles and refreshes available route profiles.
- **Clear dashboard layout:** The top-view XY map now gets its own tall panel beside the 3D view. Its opaque legend and active-dynamic badge were removed so they do not cover hazards; static and dynamic obstacle IDs remain visible. Adaptive weights are placed alongside the three time-series plots instead of being stacked on top of the map. Objective, phase and route information remain available through Simulation Status, Mission Progress and the PATH PROFILE selector.
- **Graphs and GPS:** Time-series graphs show units and labelled axes. GPS is simulated by converting local X/Y coordinates to latitude/longitude using the origin in `config.py`; it is not hardware GPS data.

## Run on macOS (Apple Silicon)

1. Extract the ZIP and open this folder in VS Code.
2. Open the terminal in this folder and run:

```bash
bash run_mac.sh
```

The launcher uses Python 3.12 when available (otherwise `python3`), creates this folder's `.venv` on first run, installs `requirements.txt`, and launches the dashboard. Package installation requires an internet connection on the first run. The next time, run `bash run_mac.sh` again; it reuses the environment.

If the dashboard fails, run the following and send the full output for troubleshooting:

```bash
source .venv/bin/activate
python app.py
```

## Main files

- `app.py` — application entry point
- `simulation.py` — MuJoCo world, quadrotor controller, GPS telemetry, static/dynamic hazards, reset and replanning
- `planner.py` — collision-aware A* route families and route evaluation
- `dashboard.py` — PySide6 UI, route selector, obstacle toggles, restart button, XY map and graphs
- `data_model.py` — data-driven risk prediction and adaptive objective weights
- `config.py` — start/destination, simulation rate and simulated GPS origin

## Note

This is a simulation demo, not a real-flight control system. Dynamic obstacle motion and GPS telemetry are simulated. Validate the project in your local MuJoCo/PySide6 environment before relying on the GUI output.

## Reposition static obstacles (interactive)

The **Top View · Drag Red Obstacles / Green Goal** map supports direct mouse editing:

1. Start the app and find a red static obstacle in the top-view XY map.
2. Click and drag the red obstacle to a new position. The drone pauses while you drag, so it does not keep flying on a stale route during the edit.
3. Release the mouse. The MuJoCo obstacle is moved, the planner's obstacle map is updated, and ACMFDS calculates a new collision-aware route from the drone's current position to the destination.
4. The cyan route and 3D world update to reflect the new obstacle position. Use **REPLAN** if you want to calculate the route again manually.

Static obstacles remain at their edited positions when you press **RESTART MISSION** during the current app session. The orange dynamic obstacles continue moving and the planner continues periodically replanning around them.


## Move the destination (goal) interactively

1. Find the GREEN target marker in the top-view XY map. The BLUE marker is the fixed start location.
2. Click and drag the green marker to a new goal position. The drone pauses during editing.
3. Drop the target in a free area. The target pad in the 3D world and the displayed target coordinates update together.
4. Release the mouse to replan a collision-aware route from the drone's current position to the new target. Distance-to-target and mission progress use the updated destination.

The goal cannot be placed within an obstacle's inflated safety margin or within 0.55 m of the drone. A destination moved after successful mission completion resumes the mission when battery remains above 5%. The selected goal is retained when **RESTART MISSION** is pressed during the current app session.


## Customize obstacles and destination

Static obstacles are the red markers labelled **S1–S6** and dynamic obstacles are the orange markers labelled **D1–D2** in the **Top View** panel. Drag either type directly in that map (not in the 3D viewport). Releasing the marker updates the MuJoCo world and recalculates the path. Dynamic hazards keep moving along their configured patrol tracks after repositioning.

For precise placement, use the **CUSTOMIZE OBSTACLE** controls below the path options. The controls are now split into two readable rows: the first row has obstacle type, selected obstacle, X/Y coordinates and **APPLY CUSTOM**; the second row has dynamic patrol axis, range and movement rate plus feedback. Choose Static or Dynamic, select an obstacle, edit its coordinates, then apply. A position too close to the drone or target is rejected for safety.

The green target marker can also be dragged in the Top View to change the destination.


## Wind and battery experiments

1. Start at 100% battery and 0 m/s wind; note the normal route and speed.
2. Try wind at 6 m/s and 90°. The simulation applies a sideways world-frame force, raises the environmental risk contribution, reduces commanded speed, and increases battery use. Try changing the direction to 180° to see the disturbance reverse.
3. Set battery to 35% to test energy-conservation behaviour.
4. Set battery at or below the wind-adjusted return reserve (25% in calm conditions, higher in wind) to trigger automatic return-to-home. At 8% or below, the simulation switches to emergency landing at the current location.
5. Press **RESTART MISSION** to begin from the blue start marker using the currently selected test conditions. The selected wind and restart battery level are preserved.

All wind and battery values are simulated inputs. They do not connect to a weather service, physical battery sensor, or actual aircraft.


## Customize static and dynamic obstacles

Use **CUSTOMIZE OBSTACLE** below the route options. Choose **Static** or **Dynamic**, select S1–S6 or D1–D2, edit X/Y, and press **APPLY CUSTOM**. For a dynamic obstacle, X/Y define the centre of its patrol track; choose **X sweep** or **Y sweep**, set the travel range (± metres), and adjust its angular rate. The dynamic marker can also be dragged directly in the Top View: the track centre shifts so the live orange marker follows your drop location, and the original patrol motion continues from there. The app rejects edits that put the live obstacle directly on the drone/goal or push its full patrol outside the arena.
