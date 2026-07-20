# RoboLight

RoboLight is a robotic flashlight. Its goal is to shine light where the user is
pointing. It is intended to be as inexpensive as possible while retaining
mechanical character and being entertaining to watch.

## Project concept

A camera looks in the same direction as the flashlight. The planned tracking
system will use that image to find a pointer and center the light on it. One
possible pointer is a visually distinctive ring worn by the user, although the
final pointer and vision algorithm have not yet been selected.

The user may also move an arm by hand. On physical hardware, that displacement
will trigger **Hijacked** mode: automatic tracking stops and a status LED blinks
until the arm is reset. An axis that is not moving is held by friction rather
than a rigid powered lock. This makes the mechanism compliant and allows the
user to reposition it without fighting a continuously energized motor.

The camera-based pointer tracker, physical displacement detection, Hijacked
state machine, and blinking LED are design goals. They are not yet implemented
by this simulation or Python API.

## Mechanical architecture

The mechanism uses one motor and a selectable transmission instead of one motor
per axis. G1 is the common drive gear. A selector couples G1 to one of the
follower paths, moves that axis, and then releases it so friction can hold the
new position. Real hardware will therefore move one axis at a time in the normal
case. The simulation and API can select multiple paths together for mechanism
experiments and testing.

| Selector | Transmission | Mechanical result |
| --- | --- | --- |
| G1 / motor | Motor shaft drives G1 directly | Rotates the shared drive input without engaging an output |
| Arm 1 / G2 | G2 and its timing-belt stage | Rotates the first arm link |
| Arm 2 / G3 | G3 and its timing-belt stages | Rotates the second arm link |
| Y tilt / G4 | G4 and its cable spool | Tilts the flashlight plate about Y |
| X tilt / G5 | G5 and its cable spool | Tilts the flashlight plate about X |
| Turntable / G6 | G6 and the lazy-Susan disk | Rotates the complete mechanism about the vertical axis |

G4 and G5 pull virtual cables attached to the tilt plate. Spool rotation sets
cable travel, so changing the spool diameter changes the tilt produced by a
given motor rotation. The cables are intentionally not rendered. G6 rotates the
disk under the mechanism, carrying the motor, gears, arms, plate, flashlight,
and camera together.

## Current simulation

RoboLight is currently a Windows-based MuJoCo prototype of that architecture.
It includes six selectable follower gears, two articulated arms, spool-driven
plate tilt, a lazy-Susan turntable, checkerboard room targets, a plate-mounted
spotlight, and a live picture-in-picture camera aligned with the beam.

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

## Python API

The `scripts` package provides a deterministic API for commanding the simulated
transmission. It is headless by default, or it can open a synchronized MuJoCo
viewer so each API action is visible. It uses the same gear ratios, selectors,
spool-driven tilt, limits, and reset behavior as the UI. Import the public types
directly from `scripts`:

```python
from scripts import HWDesc, RoboLight, Selector

light = RoboLight(realtime=True)
light.SetHW(HWDesc(g1_diameter_mm=64, follower_diameter_mm=100, spool_diameter_mm=10))
light.open_viewer()
light.open_pip()

light.move(Selector.ARM1, velocity=100, degrees=90)
state = light.move([Selector.Y_TILT, Selector.TURNTABLE], velocity=100, degrees=-45)
print(state.to_dict())

light.reset()
light.close_pip()
light.close_viewer()
```

### API types

- `RoboLight` owns one MuJoCo model and its current mechanism state.
- `HWDesc` describes the adjustable G1, G2-G6 follower, and G4/G5 spool
  diameters in millimeters.
- `Selector` names the transmission path to engage for a move.
- `RoboLightState` is an immutable snapshot containing motor, gear, arm, tilt,
  turntable, simulation-time, and last-selector values.

### Configuring hardware

`set_hw(...)` is the Python-style spelling of `SetHW(...)`; both perform the
same operation. Hardware changes preserve the current pose and affect later
moves. A complete `HWDesc` is the clearest form, but a dictionary can update a
subset:

```python
light.set_hw({"spool_diameter_mm": 16})
print(light.hwdesc)
```

G1 and follower diameters accept 35-140 mm. Spool diameter accepts 5-50 mm.
Invalid field names, non-finite values, and out-of-range dimensions raise an
exception instead of silently changing the model.

### Moving an axis

`move(selector, velocity, degrees)` commands a relative motor move:

- `selector` chooses which output follows G1. It can be a `Selector`, an
  API name such as `"arm1"`, a UI label such as `"G4 -> Y tilt"`, or an iterable
  of selectors. `"all"` engages every simulated output.
- `velocity` is positive motor speed in degrees per second, from greater than 0
  through 720.
- `degrees` is signed relative motor rotation from -360 through +360. Its sign
  controls direction; it is not an absolute axis target.
- G1 always turns. Selected outputs follow their mechanical ratios while
  unselected outputs hold their previous simulated positions.
- The call blocks until the move is complete and returns a `RoboLightState`.

Moves run without wall-clock delay by default, but MuJoCo simulation time still
advances according to distance and velocity. Construct with
`RoboLight(realtime=True)` to pace calls in real time. That option is useful for
demonstrations, especially with `open_viewer()`. The viewer is synchronized
after every motion step and after hardware, direct-tilt, and reset operations.
`open_pip()` adds a separate 320×240 view from the camera aligned with the
spotlight; it follows every arm, tilt, and turntable move. `sync_visuals()` keeps
both windows responsive during application-defined pauses. Use `close_pip()` and
`close_viewer()` when finished. Fast deterministic mode without either window
is better for control development and automated tests.

`set_tilt(x_degrees=..., y_degrees=...)` provides API-only direct positioning
for tests and initial poses; the UI intentionally exposes tilt through G4/G5
moves rather than manual sliders. `reset()` returns all positions and simulation
time to zero without changing the hardware dimensions. The current snapshot is
always available as `light.state`, and any returned snapshot can be converted
with `state.to_dict()`.

The API currently controls the kinematic mechanism only. It does not yet detect
a physical manual displacement, enter Hijacked mode, blink an LED, find a
wearable pointer, or close the camera-tracking loop.

Run the visible API demonstration with:

```powershell
.\.venv\Scripts\python.exe .\scripts\test_api.py
```

The script opens the main MuJoCo viewer plus a separate spotlight-camera PIP,
moves G1, Arm 1, Arm 2, Y tilt, X tilt, and the turntable separately, returns
each axis, resets the mechanism, and then repeats the sequence. Close the main
viewer or press Ctrl+C to stop. Each motion also asserts the expected gear
ratio, cable-spool behavior, and friction hold of unselected axes.

For a finite test without a window:

```powershell
.\.venv\Scripts\python.exe .\scripts\test_api.py --headless --cycles 1
```

Additional options such as `--velocity`, `--degrees`, `--pause`, and `--cycles`
adjust the demonstration; run with `--help` for details.

## Repository layout

- `sim/simple_motor_gear.xml` — MuJoCo scene and mechanism geometry
- `sim/launch_simple_motor_gear_controls.py` — kinematic controls, UI, viewer, and spotlight PIP
- `sim/assets/` — checkerboard wall textures
- `sim/build_model.py` — deterministic model compile and validation step
- `scripts/robolight.py` — reusable `RoboLight` control API
- `scripts/test_api.py` — API usage example and smoke test
- `sim/README.md` — detailed mechanism and control behavior
- `single_joint_prototype.md` — mechanical prototype notes and BOM
- `build.bat` / `runsim.bat` — Windows build and launch entry points
