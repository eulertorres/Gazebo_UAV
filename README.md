# Gazebo UAV Simulation Suite

This repository bundles a full ROS Noetic + Gazebo workflow for running ArduPilot-powered multirotor simulations with custom aerodynamics, motor, and battery plugins. It also ships a PyQt-based desktop launcher that orchestrates Gazebo worlds, ArduPilot SITL, MAVROS, plotting utilities, and mission upload helpers from a single window.

## Repository layout

- **`Super_simulador.py`** — PyQt5 control panel that starts and monitors Gazebo (`roslaunch simulacao simulacao.launch`), ArduPilot SITL (`sim_vehicle.py`), Mission Planner (via Mono), live plotting with `rqt`, avionics helpers, mission upload, and plugin compilation. Tabs expose ArduPilot CLI commands (arming, mode switching, PID tuning, takeoff altitude), joystick loading, and Gazebo physics rate shortcuts.【F:Super_simulador.py†L1-L246】【F:Super_simulador.py†L247-L524】
- **`controle/euler_drone_pkgs`** — Catkin package (`euler`) containing C++ Gazebo plugins for aerodynamics, battery modeling, motor dynamics, and ArduPilot interfacing, plus Python utilities for joystick->RC override and MAVLink mission helpers. Build configuration links against `gazebo_ros`, `roscpp`, `sensor_msgs`, and Boost.【F:controle/euler_drone_pkgs/CMakeLists.txt†L1-L72】【F:controle/euler_drone_pkgs/Control.py†L1-L23】
- **`simulacao`** — Catkin package defining Gazebo worlds and vehicle models (T30, EASy variants, etc.) with MAVROS integration. The `simulacao.launch` file accepts `model_name` and `world` arguments to spawn the selected SDF model and start MAVROS against ArduPilot’s UDP endpoint.【F:simulacao/launch/simulacao.launch†L1-L46】【F:simulacao/launch/simulacao.launch†L47-L61】
- **`utilidades`** — Helper assets such as plugin templates, joystick profiles (`frsky_x18.yaml`, `sony_PS5.yaml`), simulation requirements, and documentation/figures used during modeling.【F:utilidades/plugin_template.cpp†L1-L72】【F:utilidades/requirements.txt†L1-L11】

## Prerequisites

- Ubuntu with ROS Noetic, Gazebo (Ignition libraries where noted), MAVROS, and typical ROS build tools (`catkin`, `python3-rosinstall`, etc.).
- ArduPilot source tree compiled for SITL (`./waf configure --board sitl && ./waf copter`) with `sim_vehicle.py` available in `PATH`.
- Mono and Wine for running Mission Planner from Linux, plus joystick support (e.g., MAVProxy joystick module and profiles in `utilidades`).
- Python dependencies for the launcher and mission scripts: `PyQt5`, `psutil`, and those listed in `utilidades/requirements.txt`.

## Building the workspace

1. Create a catkin workspace if you do not already have one:
   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws
   catkin_make
   echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
   ```
2. Place this repository inside `~/catkin_ws/src/Euler_Drone_Sim/` and ensure ArduPilot is cloned at `~/ardupilot/` with submodules initialized.
3. Build the plugins and packages:
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

## Running the simulator

1. Launch the desktop controller:
   ```bash
   python3 Super_simulador.py
   ```
2. In the GUI:
   - Select a Gazebo world (`pista_vazia`, `cidade_pequena`, etc.) and aircraft (`T30`, `T30_barra`, `T30_estavel`, `EASy`, or `nenhum`).
   - Click **"Abrir ambiente Gazebo"** to spawn the world and optional model; **"Iniciar Ardupilot"** starts SITL in the corresponding model directory with optional console/map views and configurable cores/coordinates.【F:Super_simulador.py†L247-L335】【F:simulacao/launch/simulacao.launch†L1-L61】
   - Use the **Comandos Ardupilot** tab for arming, mode changes, takeoff altitude, PID inspection/tuning, and joystick loading (enables mission upload).【F:Super_simulador.py†L247-L429】
   - Optional buttons open Mission Planner, live ROS plotting (`rqt`), avionics and mission sender scripts, Excel plotting, or recompile the catkin workspace. Process output appears in per-task tabs, and “Encerrar” controls terminate individual or all processes safely.【F:Super_simulador.py†L247-L524】

## Models and assets

- Gazebo models for T30 and EASy airframes (including inertial and stability variants) live under `simulacao/models/` with associated ArduPilot parameters, EEPROM snapshots, terrain tiles, and visualization assets.【F:simulacao/models/T30/model.sdf†L1-L40】【F:simulacao/models/EASy_estavel/easy.xacro†L1-L40】
- World definitions are stored in `simulacao/worlds/`, with example `pista_vazia.world` referenced by the launcher.【F:simulacao/launch/simulacao.launch†L1-L38】
- Mission planning helpers and sample routes reside in `controle/euler_drone_pkgs/src/scripts/Dados/`, alongside plotting and MAVLink utilities invoked from the GUI.【F:controle/euler_drone_pkgs/src/scripts/Mission_sender.py†L1-L40】【F:controle/euler_drone_pkgs/src/scripts/Plot.py†L1-L40】
- Joystick configuration YAMLs in `utilidades` can be copied to `~/.mavproxy/joysticks` for MAVProxy’s joystick module.【F:utilidades/frsky_x18.yaml†L1-L18】【F:utilidades/sony_PS5.yaml†L1-L19】

## Notes

- Environment variables such as `GAZEBO_MODEL_PATH`, `GAZEBO_RESOURCE_PATH`, and `GAZEBO_PLUGIN_PATH` should include the catkin workspace and the ArduPilot Gazebo models/plugins to ensure Gazebo can locate assets.
- When adjusting Gazebo physics update rates or stopping processes from the GUI, the launcher uses `gz physics`, QProcess, and `psutil` to manage subprocesses gracefully.【F:Super_simulador.py†L1-L130】【F:Super_simulador.py†L430-L524】

