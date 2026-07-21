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
experiments and testing through the low-level `move_motor()` command.

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

A second disk duplicates the lower turntable's center, diameter, material, and
radial marker. It is attached to the top of the motor case and resizes with the
lower disk. This upper plate forms the physical constraint plane beneath the arm
pivots, sandwiching the motor and follower hardware between the two turntable
plates.

For motion limits, the top surface of that plate is treated as an infinite
horizontal plane. Arm 1 has a configurable symmetric travel range that defaults
to -80 through +80 degrees; the full configured range is available when Arm 2
is at reset. At other Arm 2 angles, the controller uses the current two-link
pose and configured arm lengths to determine whether Arm 1, Arm 2, or the tilt
plate would cross the platform. It checks the complete motion path and rejects
the command before the motor turns if any point would collide.

Arm 1 and Arm 2 both default to 150 mm and are independently adjustable from
75 to 300 mm. Resizing Arm 1 moves its end pivot and belt-driven TG6 assembly;
resizing Arm 2 moves the tilt plate, flashlight, and camera endpoint.

The hardware assumes every selectable follower gear, G2 through G6, has an
absolute encoder with a calibrated zero position. A physical reset reads those
encoders and moves one selected path at a fixed 100 motor-degrees/s. Cyclic Arm
1, Arm 2, and turntable errors wrap to the nearest equivalent zero, so reset
takes the shorter direction and does not unwind complete revolutions. G4/G5
retain multi-turn spool position because a full spool turn changes cable length
and is not an equivalent tilt reset. Reset order is G5/X tilt, G4/Y tilt,
G3/Arm 2, G2/Arm 1, then G6/turntable. G1 and the motor turn as required;
outputs do not teleport to zero.

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

The motor starts at `100 motor-deg/s` with a requested rotation of `0 degrees`.
By default, **Rotation targets selected output** is checked: choose exactly one
axis, enter a move from `-360` to `360` degrees, and press **Start**. A `+90°`
Arm 2 command therefore moves Arm 2 exactly `+90°`; with the default 64:100
gears the controller obtains that output by rotating G1 `-140.625°`. Motor V
remains the direct G1 speed.

Uncheck **Rotation targets selected output** for direct-G1 testing. In that mode
the same `+90°` value rotates G1 itself by `+90°`, and a selected arm follows the
external mesh by `-90 × 64 / 100 = -57.6°`. Direct-G1 mode also permits multiple
selectors for mechanism experiments. The spotlight-camera PIP follows the plate
and shows the illuminated room surface.

The UI also includes independent **Arm 1 length** and **Arm 2 length** entries
and sliders. Both default to 150 mm and accept 75-300 mm.

Arm commands are checked against the -80 to +80 degree Arm 1 limit and the
Arm 2-dependent upper-platform clearance. A rejected UI move leaves the
mechanism stationary and reports the reason in the status line.

## Python API

The `scripts` package provides a deterministic API for commanding the simulated
transmission. It is headless by default, or it can open a synchronized MuJoCo
viewer so each API action is visible. It uses the same gear ratios, selectors,
spool-driven tilt, limits, and reset behavior as the UI. Import the public types
directly from `scripts`:

```python
from scripts import HWDesc, MoveError, RoboLight, Selector

light = RoboLight(realtime=True)
light.SetHW(
    HWDesc(
        g1_diameter_mm=64,
        follower_diameter_mm=100,
        spool_diameter_mm=10,
        arm1_length_mm=180,
        arm2_length_mm=140,
        arm1_limit_degrees=80,
    )
)
light.open_viewer()
light.open_pip()

result = light.move(Selector.ARM1, velocity=30, degrees=20)
if result is not MoveError.OK:
    raise RuntimeError(f"Arm 1 move rejected: {result.value}")

result = light.move(Selector.Y_TILT, velocity=30, degrees=-15)
if result is not MoveError.OK:
    raise RuntimeError(f"Y tilt move rejected: {result.value}")
state = light.state
print(state.to_dict())

# Low-level direct-G1 command: rotate G1 +90 degrees while engaging Arm 1.
light.move_motor(Selector.ARM1, velocity=100, degrees=90)

light.reset()
light.close_pip()
light.close_viewer()
```

### API types

- `RoboLight` owns one MuJoCo model and its current mechanism state.
- `HWDesc` describes the adjustable G1, G2-G6 follower, and G4/G5 spool
  diameters, both arm lengths, and the symmetric Arm 1 travel limit.
- `Selector` names the transmission path to engage for a move.
- `MoveError` reports whether a high-level move completed or why it was
  rejected before motion.
- `RoboLightState` is an immutable snapshot containing motor, gear, arm, tilt,
  turntable, simulation-time, and last-selector values.

Motor/G1 values are cumulative. Cyclic G2/G3/G6, Arm 1, Arm 2, and turntable
positions are reported in the signed -180° through +180° range, so four
successive `+90°` Arm 2 output moves report the equivalent `0°` reset
orientation. G4/G5 remain multi-turn because their spools change cable length.

### Configuring hardware

`set_hw(...)` is the Python-style spelling of `SetHW(...)`; both perform the
same operation. Hardware changes preserve the current pose and affect later
moves. A complete `HWDesc` is the clearest form, but a dictionary can update a
subset:

```python
light.set_hw({"spool_diameter_mm": 16, "arm1_length_mm": 200})
print(light.hwdesc)
```

G1 and follower diameters accept 35-140 mm. Spool diameter accepts 5-50 mm.
Arm 1 and Arm 2 lengths independently accept 75-300 mm and default to 150 mm.
`arm1_limit_degrees` accepts a symmetric limit magnitude from 1-180 degrees and
defaults to 80 degrees.
Length changes preserve all current joint angles while moving the downstream
linkage, plate, flashlight, and camera geometry. Invalid field names, non-finite
values, and out-of-range dimensions raise an exception instead of silently
changing the model.

### Moving an axis

`move(selector, velocity, degrees)` commands one selected output in mechanism
coordinates:

- `selector` chooses exactly one output. It can be a `Selector`, an API name
  such as `"arm1"`, or a UI label such as `"G4 -> Y tilt"`.
- `velocity` is the positive speed of that selected output in degrees per
  second.
- `degrees` is the selected output's signed relative displacement from -360
  through +360. Its sign controls output direction; it is not an absolute
  target. X/Y tilt moves must also leave the plate within its -45 to +45 degree
  joint range.
- The API translates output angle and speed into G1 motor commands. It accounts
  for external-mesh direction reversal, the configured G1-to-follower diameter
  ratio, and—for X/Y tilt—the spool radius and 25 mm cable attachment lever.
- If the required motor angle exceeds 360 degrees, the controller transparently
  executes adjacent legal motor-move chunks. A request that would require more
  than 720 motor-deg/s is rejected.
- Arm 1 must remain within its configured symmetric travel limit, which
  defaults to -80 through +80 degrees. Arm 1 and Arm 2 moves are sampled across
  their full swept path against the infinite upper-platform plane using both
  configured arm lengths and the current Arm 2 position.
- The call blocks until a legal move completes and returns `MoveError.OK`.
  Any other `MoveError` means no part of the command moved. Read `light.state`
  after success to obtain a `RoboLightState`.

For example, `move(Selector.ARM1, velocity=30, degrees=20)` moves Arm 1 by
positive 20 degrees at 30 output-deg/s. With the default 64 mm G1 and 100 mm
follower, the hidden translation commands G1 by -31.25 degrees at
46.875 motor-deg/s.

`MoveError` values are `OK`, `INVALID_SELECTOR`, `INVALID_DEGREES`,
`INVALID_VELOCITY`, `MOTOR_SPEED_LIMIT`, `TILT_LIMIT`, `ARM1_LIMIT`,
`PLATFORM_COLLISION`, `LOST_STEPS`, and `HIJACKED`. The last two are reserved
for encoder/manual-movement feedback from the physical robot; the simulation
defines them for API compatibility but does not emit them yet.

`get_position(selector)` returns the current selected-output angle in degrees.
G1 is cumulative; Arm 1, Arm 2, and turntable positions are cyclic signed
angles; X/Y selectors return plate tilt. It requires exactly one selector.

`move_motor(selector, velocity, degrees)` is the lower layer. Its `degrees` and
`velocity` are direct G1/motor units, matching the UI when **Rotation targets
selected output** is unchecked. It also accepts selector iterables or `"all"`
for simulation-only multi-path experiments. Selected outputs then follow their
raw mechanical ratios, while unselected outputs hold their positions. This
low-level method intentionally bypasses the Arm 1 and platform guards; use
`move()` for normal controller commands.

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

`get_camera()` captures that same spotlight-aligned view without requiring the
PIP or main viewer. It returns a fresh RGB NumPy array with shape
`(240, 320, 3)` and `uint8` pixels, ready to pass to a later Pillow, OpenCV, or
other Python image-processing package.

`set_tilt(x_degrees=..., y_degrees=...)` provides API-only direct positioning
for tests and initial poses; the UI intentionally exposes tilt through G4/G5
moves rather than manual sliders. It also updates the corresponding absolute
spool-gear encoder, as a hand-moved cable plate would.

`reset()` performs the physical encoder-driven sequence at its fixed 100°/s
motor speed: X tilt, Y tilt, Arm 2, Arm 1, and turntable. Only one selector is
engaged at a time. Arm 1, Arm 2, and turntable encoder readings are wrapped to
the signed shortest error in the range -180° through +180°, so accumulated
complete revolutions are not unwound. G4/G5 use their unique multi-turn
cable-spool zero. G2-G6 and their outputs finish at an
equivalent zero, while G1/motor angle and simulation time reflect the actual
reset moves instead of being cleared. Hardware dimensions remain unchanged.
The current snapshot is always available as `light.state`, and any returned
snapshot can be converted with `state.to_dict()`.

The API currently controls the kinematic mechanism only. It does not yet detect
a physical manual displacement, enter Hijacked mode, blink an LED, find a
wearable pointer, or close the camera-tracking loop.

Run the visible API demonstration with:

```powershell
.\.venv\Scripts\python.exe .\scripts\test_api.py
```

For a shorter fixed-configuration example with no command-line options, run:

```powershell
.\.venv\Scripts\python.exe .\scripts\simple_test_api.py
```

`simple_test_api.py` contains one explicit `HWDesc`, opens the viewer and PIP,
moves every axis once, reads positions and a camera image, checks each
`MoveError`, and performs the physical reset. Edit that file directly to
experiment with different hardware or moves.

The script opens the main MuJoCo viewer plus a separate spotlight-camera PIP,
moves G1 and all five outputs separately, leaves the outputs displaced, then
visibly performs the fixed-order physical reset and repeats the sequence. Close
the main viewer or press Ctrl+C to stop. The test asserts requested output
displacement and speed, translated G1 angles, gear ratios, cable-spool behavior,
friction hold, Arm 1/platform rejections, atomic `MoveError` behavior, reset
order, encoder zeros, fixed reset speed, and reset duration.

For a finite test without a window:

```powershell
.\.venv\Scripts\python.exe .\scripts\test_api.py --headless --cycles 1
```

Additional options such as `--velocity`, `--degrees`, `--pause`, and `--cycles`
adjust the demonstration; run with `--help` for details.

## Repository layout

- `sim/simple_motor_gear.xml` — MuJoCo scene and mechanism geometry, including the lower turntable and upper arm-constraint plate
- `sim/launch_simple_motor_gear_controls.py` — kinematic controls, UI, viewer, and spotlight PIP
- `sim/assets/` — checkerboard wall textures
- `sim/build_model.py` — deterministic model compile and validation step
- `scripts/robolight.py` — reusable `RoboLight` control API
- `scripts/simple_test_api.py` — minimal fixed-configuration visible API example
- `scripts/test_api.py` — API usage example and smoke test
- `sim/README.md` — detailed mechanism and control behavior
- `single_joint_prototype.md` — mechanical prototype notes and BOM
- `build.bat` / `runsim.bat` — Windows build and launch entry points
