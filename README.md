# RoboMaster Autonomous Maze Navigation

An autonomous fixed-grid maze navigation and object delivery system for the DJI RoboMaster EP. The robot uses A* path planning, wheel odometry, yaw feedback, a front ToF sensor, and left/right Sharp IR sensors to pick up an object, navigate to a drop-off location, place the object, and exit the maze safely.

## Project Overview

The maze is configured through a Tkinter GUI before the mission begins. The operator can define the grid dimensions, physical cell size, known walls, and the `START`, `DROP`, and `EXIT` cells. The system validates the mission route, controls the physical robot, updates wall evidence from live sensor readings, and exports the final map and sensor data after each run.

The project is designed for a fixed maze with known cell dimensions. It also includes an exploration fallback for situations where the planned route becomes unavailable.

## Mission Workflow

1. Configure the maze, mission markers, and robot parameters in the GUI.
2. Validate that routes exist from `START` to `DROP` and from `DROP` to `EXIT`.
3. Detect, approach, grasp, and verify the object using the front ToF sensor.
4. Generate an orientation-aware A* route to the drop-off cell.
5. Follow the route using odometry, yaw feedback, and side-wall correction.
6. Align the robot with the front and side walls before releasing the object.
7. Compensate for the robot's post-drop position and plan a route to the exit.
8. Drive outside the maze and export the mission results.

## Key Features

- GUI-based fixed-grid maze configuration
- Configurable `START`, `DROP`, and `EXIT` positions
- Route validation before starting the mission
- Orientation-aware A* path planning with turn cost
- Automatic replanning when new obstacles are detected
- Trémaux-style exploration fallback
- Wheel-odometry-based cell movement
- Yaw-feedback heading control and closed-loop turning
- Front ToF obstacle detection and emergency stopping
- Left and right Sharp IR wall following
- Robotic arm and gripper pickup-and-drop control
- ToF-based pickup verification
- Sensor-offset compensation for accurate object placement
- Persistent wall evidence to reduce false or disappearing walls
- Post-drop re-anchoring to prevent incorrect map updates
- JSON, final-map SVG, and sensor-graph SVG output
- Simulation mode and hardware-independent unit tests

## Hardware

- DJI RoboMaster EP
- RoboMaster robotic arm and gripper
- Front Time-of-Flight distance sensor
- Left and right Sharp IR distance sensors
- Front-left and front-right digital IR sensors
- RoboMaster wheel odometry and attitude/yaw feedback

The current sensor-adapter configuration is defined in `robomaster_mission/mission.py`:

| Sensor | Adapter ID | Port |
|---|---:|---:|
| Front-left digital IR | 1 | 1 |
| Front-right digital IR | 4 | 1 |
| Left Sharp IR | 2 | 1 |
| Right Sharp IR | 3 | 1 |

These values must be verified against the physical robot before running the mission.

## Project Structure

```text
robomaster-autonomous-maze-navigation/
├── main.py
├── README.md
├── requirements.txt
├── .gitignore
├── robomaster_mission/
│   ├── __init__.py
│   ├── configuration.py
│   ├── grid_map.py
│   ├── planning.py
│   ├── mission.py
│   ├── reporting.py
│   ├── version.py
│   └── gui.py
└── tests/
    └── test_wall_evidence.py
```

### Module Responsibilities

| Module | Responsibility |
|---|---|
| `main.py` | Application entry point and command-line mode selection |
| `configuration.py` | Mission configuration, validation, and drop target calculation |
| `grid_map.py` | Grid representation and persistent wall evidence |
| `planning.py` | Orientation-aware A* and topological route planning |
| `mission.py` | Sensors, motion control, pickup, navigation, drop, and exit |
| `reporting.py` | JSON, final-map SVG, and sensor-graph SVG generation |
| `gui.py` | Tkinter maze editor and mission monitor |
| `version.py` | Program and saved-result format version |

## Requirements

- Windows 10 or Windows 11
- Python 3.10 recommended
- DJI RoboMaster EP for real-robot operation
- Network connection to the robot using AP, STA, or RNDIS mode

The `requirements.txt` file should contain:

```text
robomaster @ https://github.com/dji-sdk/RoboMaster-SDK/archive/refs/heads/master.zip
```

## Installation

Clone the repository and enter the project directory:

```cmd
git clone https://github.com/nattannsra18/robomaster-autonomous-maze-navigation.git
cd robomaster-autonomous-maze-navigation
```

Create a virtual environment using Python 3.10:

```cmd
py -3.10 -m venv .venv
.venv\Scripts\activate.bat
```

Upgrade the installation tools and install the dependencies:

```cmd
python -m pip install --upgrade pip setuptools wheel
python -m pip install -r requirements.txt
```

Verify the RoboMaster SDK installation:

```cmd
python -c "from robomaster import robot; print('RoboMaster SDK OK')"
```

## Running the Application

Start the GUI:

```cmd
python main.py
```

Before starting a real mission:

1. Verify the sensor IDs and ports.
2. Confirm the Sharp IR calibration values.
3. Configure the maze and all three mission markers.
4. Check the pickup, drop, motion, and safety parameters.
5. Validate the route in the GUI.
6. Keep the emergency stop control accessible.

## Simulation Mode

The planner and GUI can be tested without connecting to a physical robot:

1. Run `python main.py`.
2. Enable `Simulation` in the GUI.
3. Configure the grid, walls, and mission markers.
4. Start the mission.

Simulation mode validates the route and generates result files, but it does not simulate robot physics, wheel slip, or sensor noise.

## Legacy Mode

The original exploration mode can be started without the GUI:

```cmd
python main.py --legacy
```

Legacy mode requires the RoboMaster SDK and a connected physical robot.

## Wall Evidence System

The map stores operator-defined walls separately from walls detected by the sensors. A blocked sensor reading adds a wall immediately for safety, while an open reading must be observed three consecutive times before an existing sensor wall is removed.

An edge physically crossed by the robot is stored as strong open-space evidence. This prevents a later angled Sharp IR reading near a corner from incorrectly closing a route that the robot has already traversed.

## Sharp IR Calibration

The built-in calibration table for both Sharp sensors is:

| ADC | Distance |
|---:|---:|
| 450 | 10 cm |
| 360 | 20 cm |
| 300 | 30 cm |
| 240 | 40 cm |
| 200 | 50 cm |

Optional left and right calibration JSON files can be selected in the GUI. Calibration distances must be measured from the Sharp sensor lens directly to the wall, not from the center of the robot.

## Generated Results

After a simulation or real mission, the program generates:

- `*_run_YYYYMMDD_HHMMSS.json` — configuration, map data, wall evidence, and sensor history
- `*_map.svg` — final map and actual robot trajectory
- `*_sensor_graph.svg` — front ToF, left Sharp, and right Sharp distance history

Generated run files are excluded by `.gitignore`. Selected results can be added to the repository later for documentation and portfolio presentation.

## Sample Mission Result

The following artifacts were generated from a completed mission run on August 27, 2026.

### Final Maze and Robot Trajectory

![Final maze and robot trajectory](docs/results/run-20260827-111035/robomaster_basic_maze_run_20260827_111035_map.svg)

### Sensor Distance History

![Front ToF and Sharp IR sensor history](docs/results/run-20260827-111035/robomaster_basic_maze_run_20260827_111035_sensor_graph.svg)

### Raw Mission Data

[Download the JSON mission report](docs/results/run-20260827-111035/robomaster_basic_maze_run_20260827_111035.json)

## Tests

Run the hardware-independent unit tests from the project root:

```cmd
python -m unittest discover -s tests -v
```

The current tests verify:

- A single open reading does not remove a confirmed sensor wall.
- Three consecutive open readings remove a sensor wall.
- A blocked reading resets the open-reading streak.
- A physically traversed edge is treated as strong open-space evidence.
- Mission artifacts include a sensor graph.

## Current Limitations

- The primary navigation mode assumes a fixed grid and known cell dimensions.
- Localization accuracy depends on wheel odometry and can be affected by wheel slip.
- Sharp IR performance depends on calibration, wall angle, and surface properties.
- Simulation mode does not reproduce physical motion or sensor behavior.
- The system does not currently use a camera, SLAM, ROS 2, or absolute localization.
- Real-world performance must be validated on the target robot and maze.

## Safety

This project controls a physical robot. Test at low speed, keep the operating area clear, verify emergency-stop behavior, and remain ready to stop the robot during every real-world test.

## Project Version

```text
BASIC_FIXED_GRID_ASTAR_PICKUP_DROP_V7_PERSISTENT_WALL_EVIDENCE
```
