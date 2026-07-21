# Simple MuJoCo Starter Simulation

The current starter model is intentionally minimal:

- motor block fixed to the rotating turntable assembly
- one visible shaft
- one lengthened drive gear `G1` spanning five follower planes for `G2` through `G6`
- scripted kinematic motor rotation
- named motor `M`, drive/follower gears `G1` through `G6`, and timing gears `TG1` through `TG6`
- default `G1` diameter is 64 mm
- default `G2-G6` diameter is 100 mm, making each follower larger than `G1`
- `G3` through `G6` duplicate the `G2` follower geometry in separate planes; `G6` is the frontmost plane
- `G4` and `G5` each have a front spool; one `Spool diameter` control adjusts both from 5 to 50 mm and defaults to 10 mm
- default `Motor V` is 100 deg/s and default `Rotation` is 0 degrees, so startup remains stationary
- base is 10 mm thick, white, and extends to the room walls
- a 20 mm-thick lazy-Susan disk rests on the base, centered below the Arm 1 pivot, and raises the complete mechanism by 20 mm
- the disk radius automatically grows or shrinks with the gear diameters so it extends beneath the mechanism's full footprint
- a duplicate 20 mm-thick disk is centered above it with its lower face attached to the motor-case top; it matches the lower disk's radius and marker and provides the arm constraint plate
- the motor and follower hardware are sandwiched between the lower lazy-Susan disk and upper arm constraint plate, while the arm pivots sit above the upper plate
- Arm 1 is limited to -80 through +80 degrees; when Arm 2 is away from reset, the controller checks the two-link swept path and tilt plate against the upper plate, treated as an infinite horizontal plane
- blue and warm-colored 8 x 8 checkerboard side walls sit about one original mechanism/base width from the mechanism, giving the room a 780 mm clear width
- a green 8 x 8 checkerboard back wall closes the room, while the 900 mm-high ceiling uses a neutral 8 x 8 checkerboard
- the checkerboard walls and ceiling provide visible, directional surfaces for the plate-mounted spotlight and camera
- the shared axle height places a 140 mm `G1` at the turntable top without intersecting the fixed base
- each follower has an inboard support post from the turntable disk to its axle
- the `G2` axle stops at the front face of `G2`; a front-mounted timing gear `TG1` is coaxial with `G2`
- `TG1` is 35 mm diameter, matching the smallest configurable `G2` size
- `TG1` drives an equal-size `TG2` directly above it through a 100 mm vertical belt run
- `TG2` has an upper support extension from the `G2` support post
- `arm1` defaults to 150 mm, is adjustable from 75 to 300 mm, and is fixed to the front face of `TG2`
- `TG3` is coaxial with `G3` and drives an equal-size `TG4` directly above it through a second 100 mm vertical belt run
- `TG5` is coaxial with `TG4` and spins with it; `TG6` is at the end of `arm1`, with belt 3 mounted parallel to `arm1`
- green `arm2` defaults to 150 mm, is independently adjustable from 75 to 300 mm, and is fixed to `TG6`
- a 50 mm x 50 mm tilt plate is mounted at the end of `arm2` and faces up at reset
- a small spotlight is mounted at the center of the plate and follows the plate tilt
- a camera at the spotlight lens follows the same pose and uses a 50 degree field of view matching the spotlight cone
- the control window includes a live 320 x 240 picture-in-picture feed from the spotlight camera
- the control UI has Arm 1 and Arm 2 selectors; selected-output mode keeps exactly one path engaged, starts with Arm 1, and translates requested output angle into G1 motion
- unchecking `Rotation targets selected output` enables direct-G1 angle commands and independent multi-path selector experiments
- `G4 -> Y tilt` and `G5 -> X tilt` selectors independently connect those followers to `G1`
- `G6 -> turntable` connects the front follower to `G1`; subsequent `G6` rotation yaws the disk and complete mechanism 1:1
- the UI displays live X and Y tilt readouts; tilt motion is commanded through the G4 and G5 selectors rather than manual sliders
- the UI provides independent `Arm 1 length` and `Arm 2 length` number entries and sliders; changing them moves all downstream pivots, belts, plate, light, and camera geometry
- the control UI has a physical reset button that uses absolute G2-G6 encoder positions and drives one selected path at a fixed 100 deg/s
- startup view is rotated 90 degrees around the mechanism from the former G1-facing view; camera controls remain enabled

In MuJoCo, enable site labels to see `M` and `G1` in the viewer. The invisible label anchors move with the complete turntable assembly, while remaining fixed beside their labeled parts within that assembly.

Run the kinematic control launcher from the repo root:

```powershell
.\build.bat
.\runsim.bat
```

The control launcher defaults to selected-output angle mode. It requires exactly one selected path and translates `Rotation (deg)` through the configured gear ratio and, for X/Y tilt, the spool/lever ratio. Thus `+90°` with Arm 2 selected moves Arm 2 exactly `+90°`; at the default 64 mm G1 and 100 mm follower diameter, G1 rotates `-140.625°` to produce it. `Motor V` remains direct G1 speed. In this starter scene, `G1` is directly on the motor shaft, so the motor-to-`G1` ratio is 1:1. When Arm 1 is selected, `G2` is externally meshed with `G1`, `TG1` is fixed coaxially to `G2`, and the equal-size open belt stage drives `TG2` and `arm1` at the same angle as `G2`. When Arm 2 is selected, `G3` is externally meshed with `G1`, `TG3` drives timing gears `TG4`/`TG5`, and `TG6`/`arm2` is driven at the same angle. When the turntable is selected, `G6` follows the same external-mesh ratio and its change in angle is applied directly to the vertical turntable joint. Unselected drives hold their last angle. Re-selecting a drive preserves its current angle and resumes from there instead of snapping to the current `G1`-derived angle.

Uncheck `Rotation targets selected output` to make `Rotation (deg)` a direct G1
motor command and permit multiple paths at once. In direct mode, a `+90°` G1
move rotates a selected arm by `-57.6°` with the default ratio.

Before an arm move starts, the UI checks every 0.25 degrees of the requested
joint-space path. Arm 1 must stay within -80 to +80 degrees. The check uses the
current Arm 2 angle and both configured arm lengths to keep Arm 1, Arm 2, and
the tilt plate above the upper platform, which is treated as extending to
infinity. An invalid move is rejected before G1 turns and its reason appears in
the status line. This guard also applies to direct-G1 UI experiments.

When selected, follower `G4` drives Y tilt and follower `G5` drives X tilt through a virtual bidirectional cable. Cable travel is `spool rotation x spool radius`; plate rotation is that travel divided by the 25 mm plate attachment lever. The result is limited to the plate joint's +/-45 degree range. Changing spool diameter while engaged preserves the current tilt and applies the new ratio to subsequent rotation. The cable itself is intentionally not rendered. The simulation remains kinematic: it does not use gravity, torque, damping, contacts, cable tension, or actuator physics.

Physical reset assumes each selectable follower gear `G2-G6` has an absolute encoder with a calibrated zero. For cyclic Arm 1, Arm 2, and turntable motion, the controller wraps encoder error into the signed -180° through +180° range and takes the shortest direction to the nearest equivalent reset orientation. G4/G5 instead retain multi-turn position because a complete spool revolution changes cable length and is not an equivalent tilt reset. The controller engages exactly one path at a time at a fixed 100 motor-deg/s, independent of the Motor V slider. It resets `G5`/X tilt, `G4`/Y tilt, `G3`/Arm 2, `G2`/Arm 1, then `G6`/turntable. The motor and `G1` rotate normally during this sequence, and simulation time continues; neither is teleported back to zero.

For a fixed move:

1. Leave `Motor V` at its 100 motor-deg/s default or set another value above 0.
2. Keep `Rotation targets selected output` checked and select exactly one axis.
3. Set `Rotation` from `-360` to `360` output degrees with the slider or entry.
4. Press `Start`.

Changing `Rotation` alone does not move the gear. It is only applied when `Start` is pressed.

Set `G1 diameter`, shared `G2-G6 diameter`, `Spool diameter`, `Arm 1 length`, or `Arm 2 length` with either the number entry boxes or sliders. Arm lengths accept 75-300 mm and default to 150 mm. The launcher recomputes every follower center from `G1 radius + follower radius`, so all five gear circles remain tangent when either gear diameter changes. It also recenters and resizes both turntable disks under the resulting mechanism footprint. Arm length changes preserve joint angles while repositioning the downstream TG6 pivot, belt, tilt plate, flashlight, and spotlight camera.

You can also load the static geometry in MuJoCo's Python viewer after building:

```powershell
.\.venv\Scripts\python.exe -m mujoco.viewer --mjcf .\sim\simple_motor_gear.xml
```

To rebuild and compile-check the model:

```powershell
.\build.bat
```
