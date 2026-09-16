# Simulator command reference

This guide contains copy-and-paste commands for setting up and running every
simulator in this repository on Windows. All paths are relative to the
repository root, so the commands work regardless of where the repository was
cloned.

## Prerequisites

- Python 3.12
- [uv](https://docs.astral.sh/uv/getting-started/installation/)
- [Git LFS](https://git-lfs.com/)

Open PowerShell in the cloned `AutoNav-Simulated-Universe` directory, then
verify the tools and download the LFS-managed assets:

```powershell
uv --version
git lfs install
git lfs pull
$env:PYTHONUTF8 = "1"
```

The `PYTHONUTF8` setting applies only to the current PowerShell session and
prevents Windows console-encoding failures when a simulator prints symbols
such as degrees or theta.

Each simulator has an independent virtual environment under
`<simulator>/simulated_world/.venv`. Setup commands are safe to repeat: they
create the environment only when it is missing and then bring its dependencies
up to date.

## SPEED simulator

One-time setup:

```powershell
Push-Location .\SPEED_Sim\simulated_world
if (-not (Test-Path ".\.venv\Scripts\python.exe")) {
    uv venv --python 3.12 .venv
}
uv pip install --python ".\.venv\Scripts\python.exe" --link-mode=copy -r ".\requirements.txt"
Pop-Location
```

Run:

```powershell
.\SPEED_Sim\simulated_world\.venv\Scripts\python.exe .\SPEED_Sim\simulated_world\speed_sim.py
```

Run with explicit PID gains:

```powershell
.\SPEED_Sim\simulated_world\.venv\Scripts\python.exe .\SPEED_Sim\simulated_world\speed_sim.py --kp 120 --ki 55 --kd 28
```

Use the arrow keys to drive. The number row selects half-MPH setpoints, and
`[` / `]` adjust the setpoint by 0.1 MPH.

## GPS simulator

One-time setup:

```powershell
Push-Location .\GPS_Sim\simulated_world
if (-not (Test-Path ".\.venv\Scripts\python.exe")) {
    uv venv --python 3.12 .venv
}
uv pip install --python ".\.venv\Scripts\python.exe" --link-mode=copy -r ".\requirements.txt"
Pop-Location
```

Open the configuration launcher:

```powershell
.\GPS_Sim\Run_GPS_SIM.bat
```

Recommended single-agent run:

```powershell
.\GPS_Sim\Run_GPS_SIM_direct.bat --real --single --seed 7
```

Additional scenarios:

```powershell
# Default scripted scenario
.\GPS_Sim\Run_GPS_SIM_direct.bat --single --seed 7

# Random environment
.\GPS_Sim\Run_GPS_SIM_direct.bat --random --single --seed 42

# Extreme GPS interference
.\GPS_Sim\Run_GPS_SIM_direct.bat --crazy --single --seed 42

# Three-waypoint mission
.\GPS_Sim\Run_GPS_SIM_direct.bat --real --single --mission three-waypoint

# Multiple agents
.\GPS_Sim\Run_GPS_SIM_direct.bat --real --agents 25 --seed 42

# Headless verification
.\GPS_Sim\Run_GPS_SIM_direct.bat --real --single --headless --headless-steps 500
```

To bypass the batch file while troubleshooting:

```powershell
Push-Location .\GPS_Sim\simulated_world
.\.venv\Scripts\python.exe .\gps_sim_gui.py --real --single --seed 7
Pop-Location
```

## LiDAR simulator

One-time setup:

```powershell
Push-Location .\LiDAR_Sim\simulated_world
if (-not (Test-Path ".\.venv\Scripts\python.exe")) {
    uv venv --python 3.12 .venv
}
uv pip install --python ".\.venv\Scripts\python.exe" --link-mode=copy -r ".\requirements.txt"
Pop-Location
```

Open the configuration launcher:

```powershell
.\LiDAR_Sim\Run_LIDAR_SIM.bat
```

Recommended Windows run:

```powershell
.\LiDAR_Sim\Run_LIDAR_SIM_direct.bat --threshold 20 --rays 2000
```

Grade-threshold examples:

```powershell
# Approximately 5-degree ramps are passable
.\LiDAR_Sim\Run_LIDAR_SIM_direct.bat --threshold 15 --rays 2000

# Approximately 5- and 10-degree ramps are passable
.\LiDAR_Sim\Run_LIDAR_SIM_direct.bat --threshold 20 --rays 2000

# Approximately 5-, 10-, and 15-degree ramps are passable
.\LiDAR_Sim\Run_LIDAR_SIM_direct.bat --threshold 30 --rays 2000

# Full LiDAR resolution
.\LiDAR_Sim\Run_LIDAR_SIM_direct.bat --threshold 20 --rays 11520
```

Run the ramp benchmark:

```powershell
Push-Location .\LiDAR_Sim\simulated_world
.\.venv\Scripts\python.exe .\scripts\test_ramp_crossing.py --threshold 20 --rays 2000 --agents 5 --no-video
Pop-Location
```

In the interactive simulator, click to place the robot, Shift-click to place
the goal, press `G` to begin navigation, and press `R` to reset.

## Behavior Tree simulator

One-time setup:

```powershell
Push-Location .\BEHAVIOR_TREE_Sim\simulated_world
if (-not (Test-Path ".\.venv\Scripts\python.exe")) {
    uv venv --python 3.12 .venv
}
uv pip install --python ".\.venv\Scripts\python.exe" --link-mode=copy -r ".\requirements.txt"
Pop-Location
```

Open the configuration launcher:

```powershell
.\BEHAVIOR_TREE_Sim\Run_BT_SIM.bat
```

Direct runs:

```powershell
# Default layout
.\BEHAVIOR_TREE_Sim\Run_BT_SIM_direct.bat

# Corridor track
.\BEHAVIOR_TREE_Sim\Run_BT_SIM_direct.bat --layout track --seed 7

# Generated maze
.\BEHAVIOR_TREE_Sim\Run_BT_SIM_direct.bat --layout maze --maze-cells 7 --obstacles 6 --seed 42

# Headless test
.\BEHAVIOR_TREE_Sim\Run_BT_SIM_direct.bat --headless --headless-steps 20000

# Legacy Matplotlib renderer
.\BEHAVIOR_TREE_Sim\Run_BT_SIM_direct.bat --renderer matplotlib
```

## AutoNav GUI/HUD simulator

One-time setup:

```powershell
Push-Location .\GUI_Sim\simulated_world
if (-not (Test-Path ".\.venv\Scripts\python.exe")) {
    uv venv --python 3.12 .venv
}
uv pip install --python ".\.venv\Scripts\python.exe" --link-mode=copy -r ".\requirements.txt"
Pop-Location
```

Run the HUD with synthetic ROS sensor data:

```powershell
.\GUI_Sim\Run_GUI.bat
```

Run the Python entry point directly while troubleshooting:

```powershell
Push-Location .\GUI_Sim\simulated_world
.\.venv\Scripts\python.exe .\runner.py
Pop-Location
```

Convert a playback CSV or ROS bag to MP4. The command opens a file picker:

```powershell
.\GUI_Sim\Bake_Bag_Video.bat
```

## Set up every simulator at once

Run this block from the repository root:

```powershell
$simulators = @(
    "SPEED_Sim",
    "GPS_Sim",
    "LiDAR_Sim",
    "BEHAVIOR_TREE_Sim",
    "GUI_Sim"
)

foreach ($simulator in $simulators) {
    Push-Location ".\$simulator\simulated_world"
    if (-not (Test-Path ".\.venv\Scripts\python.exe")) {
        uv venv --python 3.12 .venv
    }
    uv pip install --python ".\.venv\Scripts\python.exe" --link-mode=copy -r ".\requirements.txt"
    Pop-Location
}
```

## Mock course asset

`Mock_Course_Asset/Course.blend` is not a standalone executable. It is the
source Blender scene for the full-stack Gazebo Fortress simulation. See
[`Mock_Course_Asset/README.md`](Mock_Course_Asset/README.md) for its Blender,
SDF, Gazebo, and ROS 2 workflow.
