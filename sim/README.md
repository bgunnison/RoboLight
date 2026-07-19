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
- blue and warm-colored 8 x 8 checkerboard side walls sit about one original mechanism/base width from the mechanism, giving the room a 780 mm clear width
- a green 8 x 8 checkerboard back wall closes the room, while the 900 mm-high ceiling uses a neutral 8 x 8 checkerboard
- the checkerboard walls and ceiling provide visible, directional surfaces for the plate-mounted spotlight and camera
- the shared axle height places a 140 mm `G1` at the turntable top without intersecting the fixed base
- each follower has an inboard support post from the turntable disk to its axle
- the `G2` axle stops at the front face of `G2`; a front-mounted timing gear `TG1` is coaxial with `G2`
- `TG1` is 35 mm diameter, matching the smallest configurable `G2` size
- `TG1` drives an equal-size `TG2` directly above it through a 100 mm vertical belt run
- `TG2` has an upper support extension from the `G2` support post
- a 150 mm `arm1` is fixed to the front face of `TG2` and rotates with it
- `TG3` is coaxial with `G3` and drives an equal-size `TG4` directly above it through a second 100 mm vertical belt run
- `TG5` is coaxial with `TG4` and spins with it; `TG6` is at the end of `arm1`, with belt 3 mounted parallel to `arm1`
- a green 150 mm `arm2` is fixed to `TG6`
- a 50 mm x 50 mm tilt plate is mounted at the end of `arm2` and faces up at reset
- a small spotlight is mounted at the center of the plate and follows the plate tilt
- a camera at the spotlight lens follows the same pose and uses a 50 degree field of view matching the spotlight cone
- the control window includes a live 320 x 240 picture-in-picture feed from the spotlight camera
- the control UI has Arm 1 and Arm 2 checkboxes; Arm 1 starts engaged, Arm 2 starts disengaged
- `G4 -> Y tilt` and `G5 -> X tilt` selectors independently connect those followers to `G1`
- `G6 -> turntable` connects the front follower to `G1`; subsequent `G6` rotation yaws the disk and complete mechanism 1:1
- the X and Y sliders manually position a disengaged axis; while its gear is selected, the slider becomes a driven tilt indicator
- the control UI has a position reset button that returns the motor, gears, arms, plate tilt, and turntable yaw to zero
- startup view faces `G1`, centered, with the gear taking about 1/8 of the view height; camera controls remain enabled

In MuJoCo, enable site labels to see `M` and `G1` in the viewer. The invisible label anchors move with the complete turntable assembly, while remaining fixed beside their labeled parts within that assembly.

Run the kinematic control launcher from the repo root:

```powershell
.\build.bat
.\runsim.bat
```

The control launcher starts with a motor move. It then computes `G1`, follower `G2-G6`, timing gear `TG2` / `TG4` / `TG6`, plate motion, and turntable yaw from the mechanical relationships. In this starter scene, `G1` is directly on the motor shaft, so the motor-to-`G1` ratio is 1:1. When Arm 1 is checked, `G2` is externally meshed with `G1`, `TG1` is fixed coaxially to `G2`, and the equal-size open belt stage drives `TG2` and `arm1` at the same angle as `G2`. When Arm 2 is checked, `G3` is externally meshed with `G1`, `TG3` drives timing gears `TG4`/`TG5`, and `TG6`/`arm2` is driven at the same angle. When the turntable is selected, `G6` follows the same external-mesh ratio and its change in angle is applied directly to the vertical turntable joint. Unchecked drives hold their last angle. Re-checking a drive preserves its current angle and resumes from there instead of snapping to the current `G1`-derived angle.

When selected, follower `G4` drives Y tilt and follower `G5` drives X tilt through a virtual bidirectional cable. Cable travel is `spool rotation x spool radius`; plate rotation is that travel divided by the 25 mm plate attachment lever. The result is limited to the plate joint's +/-45 degree range. Changing spool diameter while engaged preserves the current tilt and applies the new ratio to subsequent rotation. The cable itself is intentionally not rendered. The simulation remains kinematic: it does not use gravity, torque, damping, contacts, cable tension, or actuator physics.

For a fixed move:

1. Leave `Motor V` at its 100 deg/s default or set another value above 0 deg/s.
2. Set `Rotation` from `-360` to `360` degrees, using either the slider or the number entry box.
3. Press `Start`.

Changing `Rotation` alone does not move the gear. It is only applied when `Start` is pressed.

Set `G1 diameter`, shared `G2-G6 diameter`, or `Spool diameter` with either the number entry boxes or sliders. The launcher recomputes every follower center from `G1 radius + follower radius`, so all five gear circles remain tangent when either gear diameter changes. It also recenters and resizes the turntable disk under the resulting mechanism footprint.

You can also load the static geometry in MuJoCo's Python viewer after building:

```powershell
.\.venv\Scripts\python.exe -m mujoco.viewer --mjcf .\sim\simple_motor_gear.xml
```

To rebuild and compile-check the model:

```powershell
.\build.bat
```
