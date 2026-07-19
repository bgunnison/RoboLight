# RoboLight

RoboLight is a Windows-based MuJoCo prototype for a geared, multi-axis robotic spotlight. The simulation includes six selectable follower gears, two articulated arms, spool-driven plate tilt, a lazy-Susan turntable, checkerboard room targets, a plate-mounted spotlight, and a live picture-in-picture camera aligned with the beam.

## Quick start

Requirements:

- Windows 10 or 11
- 64-bit Python 3.10 or newer, including Tkinter ([Python for Windows](https://www.python.org/downloads/windows/))
- An OpenGL-capable graphics driver

From Command Prompt or PowerShell in the repository root:

```bat
build.bat
runsim.bat
```

`build.bat` creates a local `.venv`, installs the pinned packages in `requirements.txt`, checks the launcher, compiles `sim/simple_motor_gear.xml` to `sim/simple_motor_gear.mjb`, and reloads the binary model as a validation step. The generated environment and `.mjb` are intentionally not committed.

## Manual installation and build

The automated build is equivalent to:

```powershell
py -3 -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe -m py_compile .\sim\launch_simple_motor_gear_controls.py
.\.venv\Scripts\python.exe .\sim\build_model.py
```

MuJoCo is installed from PyPI by the build; a separate SDK download is not required. If you want the standalone MuJoCo command-line binaries, download the matching Windows archive from the [official MuJoCo releases](https://github.com/google-deepmind/mujoco/releases), extract it under `tools/`, and keep that local directory uncommitted.

## Running

Start the full controls and viewer:

```bat
runsim.bat
```

Or launch it directly:

```powershell
.\.venv\Scripts\python.exe .\sim\launch_simple_motor_gear_controls.py
```

To open only the static MJCF model in MuJoCo's Python viewer:

```powershell
.\.venv\Scripts\python.exe -m mujoco.viewer --mjcf .\sim\simple_motor_gear.xml
```

The motor starts at `100 deg/s` with a requested rotation of `0 degrees`. Enter a move from `-360` to `360` degrees and press **Start**. The Arm 1, Arm 2, G4/Y tilt, G5/X tilt, and G6/turntable selectors independently engage their driven mechanisms. The spotlight-camera PIP follows the plate and shows the illuminated room surface.

## Repository layout

- `sim/simple_motor_gear.xml` — MuJoCo scene and mechanism geometry
- `sim/launch_simple_motor_gear_controls.py` — kinematic controls, UI, viewer, and spotlight PIP
- `sim/assets/` — checkerboard wall textures
- `sim/build_model.py` — deterministic model compile and validation step
- `sim/README.md` — detailed mechanism and control behavior
- `single_joint_prototype.md` — mechanical prototype notes and BOM
- `build.bat` / `runsim.bat` — Windows build and launch entry points
