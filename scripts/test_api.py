r"""Visible RoboLight API demonstration and automated smoke test.

Normal execution opens the MuJoCo simulation and a separate spotlight-camera
PIP window so the mechanism and its light-aligned view can be watched together.
The script moves every selectable path separately:

1. G1 by itself (no follower selected), then back to its start
2. Arm 1 through G2
3. Arm 2 through G3
4. Y tilt through the G4 cable spool
5. X tilt through the G5 cable spool
6. The complete turntable through G6

The five outputs remain displaced after their demonstration moves. ``reset()``
then uses the simulated absolute encoders on G2-G6 and physically returns one
gear at a fixed 100 motor-degrees/s. Reset order is X tilt, Y tilt, Arm 2,
Arm 1, and turntable. Cyclic arm/turntable encoder errors wrap to the nearest
equivalent zero, so reset does not unwind complete revolutions. Cable-spool
encoders retain multi-turn position because spool revolutions change cable
length. Nothing teleports and simulation time is not cleared. The move-and-
reset sequence repeats until the MuJoCo viewer is closed or Ctrl+C is pressed.
Closing the viewer is the normal way to end the interactive test.

Run the visible demonstration from the repository root::

    .\.venv\Scripts\python.exe .\scripts\test_api.py

The repository build uses a finite, non-visual variant so it cannot wait for a
person to close a window::

    .\.venv\Scripts\python.exe .\scripts\test_api.py --headless --cycles 1

``--cycles N`` limits the number of repeated cycles *after* the initial
all-axis demonstration and reset. Without that option, visible mode loops until
the viewer closes. ``--velocity`` and ``--degrees`` specify selected-output
degrees per second and selected-output displacement. The API translates those
values into the G1 motor motion required by each transmission path.

RoboLight is intended to become a camera-guided robotic flashlight. Pointer
recognition, physical manual-movement detection, Hijacked mode, and its blinking
LED are not implemented yet; this script tests and displays the current
single-motor kinematic transmission.

``RoboLight.move()`` returns a ``MoveError`` rather than a state snapshot.
``MoveError.OK`` means the whole command completed; any other value means it
was rejected before motion. Read ``light.state`` after a successful call. The
``move_ok()`` helper below demonstrates this normal check-and-read pattern.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import sys
import time

import mujoco


# Direct execution places ``scripts/`` rather than the repository root on
# ``sys.path``. Add the root so the documented command works without packaging.
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scripts import HWDesc, MoveError, RoboLight, RoboLightState, Selector
from sim.launch_simple_motor_gear_controls import output_degrees_per_motor_degree


ANGLE_TOLERANCE_DEGREES = 1e-7
TIME_TOLERANCE_SECONDS = 1e-7
PLATE_ATTACHMENT_LEVER_MM = 25.0
DEMO_HARDWARE = HWDesc(
    g1_diameter_mm=64.0,
    follower_diameter_mm=100.0,
    spool_diameter_mm=10.0,
    arm1_length_mm=150.0,
    arm2_length_mm=150.0,
    arm1_limit_degrees=80.0,
)
DEMO_TILT_OUTPUT_PER_MOTOR = (
    DEMO_HARDWARE.g1_diameter_mm
    / DEMO_HARDWARE.follower_diameter_mm
    * (DEMO_HARDWARE.spool_diameter_mm / 2.0)
    / PLATE_ATTACHMENT_LEVER_MM
)
MAX_DEMO_OUTPUT_VELOCITY = 720.0 * DEMO_TILT_OUTPUT_PER_MOTOR


@dataclass(frozen=True, slots=True)
class AxisDemo:
    """One visible selector path and the state field that proves it moved."""

    label: str
    selector: Selector
    output_field: str


AXIS_DEMOS = (
    AxisDemo("G1 motor input", Selector.G1, "g1_degrees"),
    AxisDemo("Arm 1 / G2", Selector.ARM1, "arm1_degrees"),
    AxisDemo("Arm 2 / G3", Selector.ARM2, "arm2_degrees"),
    AxisDemo("Y tilt / G4 spool", Selector.Y_TILT, "y_tilt_degrees"),
    AxisDemo("X tilt / G5 spool", Selector.X_TILT, "x_tilt_degrees"),
    AxisDemo("Turntable / G6", Selector.TURNTABLE, "turntable_degrees"),
)
OUTPUT_DEMOS = AXIS_DEMOS[1:]

HELD_OUTPUT_FIELDS = (
    "arm1_degrees",
    "arm2_degrees",
    "x_tilt_degrees",
    "y_tilt_degrees",
    "turntable_degrees",
)
ENCODER_FIELDS = (
    "g2_degrees",
    "g3_degrees",
    "g4_degrees",
    "g5_degrees",
    "g6_degrees",
)


def parse_args() -> argparse.Namespace:
    """Parse interactive-demo and automated-build options."""

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--headless",
        action="store_true",
        help="do not open a viewer or pace moves in wall-clock time",
    )
    parser.add_argument(
        "--cycles",
        type=int,
        default=None,
        help="repeated cycles after the initial demonstration (default: loop visibly)",
    )
    parser.add_argument(
        "--velocity",
        type=float,
        default=30.0,
        help="selected-output speed in degrees per second (default: 30)",
    )
    parser.add_argument(
        "--degrees",
        type=float,
        default=20.0,
        help="forward selected-output rotation for each axis (default: 20)",
    )
    parser.add_argument(
        "--pause",
        type=float,
        default=0.35,
        help="visible pause at each endpoint in seconds (default: 0.35)",
    )
    args = parser.parse_args()

    if args.cycles is not None and args.cycles < 0:
        parser.error("--cycles must be zero or greater")
    if (
        not math.isfinite(args.velocity)
        or not 0.0 < args.velocity <= MAX_DEMO_OUTPUT_VELOCITY
    ):
        parser.error(
            "--velocity must be above 0 and at most "
            f"{MAX_DEMO_OUTPUT_VELOCITY:g} output-deg/s so the tilt axes stay "
            "within the 720 motor-deg/s limit"
        )
    if not math.isfinite(args.degrees) or not 0.0 < args.degrees <= 45.0:
        parser.error(
            "--degrees must be above 0 and at most 45 because this test also "
            "moves the tilt axes"
        )
    if not math.isfinite(args.pause) or args.pause < 0.0:
        parser.error("--pause must be zero or greater")
    if args.headless and args.cycles is None:
        args.cycles = 1
    return args


def assert_close(label: str, actual: float, expected: float, tolerance: float) -> None:
    """Compare floating-point results and report the mechanical field name."""

    if not math.isclose(actual, expected, abs_tol=tolerance):
        raise AssertionError(f"{label}: expected {expected}, got {actual}")


def assert_angle(label: str, actual: float, expected: float) -> None:
    """Compare public API angle values, which are expressed in degrees."""

    assert_close(label, actual, expected, ANGLE_TOLERANCE_DEGREES)


def move_ok(
    light: RoboLight,
    selector: Selector | str,
    *,
    velocity: float,
    degrees: float,
) -> RoboLightState:
    """Run one high-level command and return its post-move state.

    Production callers should branch on the returned :class:`MoveError`. The
    demonstration expects every planned command to be legal, so any rejection
    is an immediate, descriptive test failure.
    """

    result = light.move(selector, velocity=velocity, degrees=degrees)
    if result is not MoveError.OK:
        raise AssertionError(
            f"{selector!s} move of {degrees:g} deg was rejected: {result.value}"
        )
    return light.state


def shortest_encoder_error_degrees(angle_degrees: float) -> float:
    """Wrap an absolute rotary encoder reading to its nearest zero."""

    return math.degrees(
        math.atan2(
            math.sin(math.radians(angle_degrees)),
            math.cos(math.radians(angle_degrees)),
        )
    )


def reset_encoder_error_degrees(field: str, angle_degrees: float) -> float:
    """Return cyclic rotary error or unique multi-turn cable-spool error."""

    if field in ("g4_degrees", "g5_degrees"):
        return angle_degrees
    return shortest_encoder_error_degrees(angle_degrees)


def arm2_world_angle_degrees(light: RoboLight) -> float:
    """Measure the rendered Arm 2 direction in the model's X/Z plane."""

    pivot_body_id = mujoco.mj_name2id(
        light.model,
        mujoco.mjtObj.mjOBJ_BODY,
        "TG6",
    )
    plate_body_id = mujoco.mj_name2id(
        light.model,
        mujoco.mjtObj.mjOBJ_BODY,
        "tilt_plate_body",
    )
    delta_x = light.data.xpos[plate_body_id, 0] - light.data.xpos[pivot_body_id, 0]
    delta_z = light.data.xpos[plate_body_id, 2] - light.data.xpos[pivot_body_id, 2]
    return math.degrees(math.atan2(delta_x, delta_z))


def verify_encoder_reset(state: RoboLightState) -> None:
    """Verify every absolute follower encoder and useful output is at zero.

    Motor/G1 position and simulation time are deliberately not reset authority;
    they are allowed to retain the result of the physical homing moves.
    """

    for field in ENCODER_FIELDS:
        assert_angle(f"{field} after reset", getattr(state, field), 0.0)
    for field in HELD_OUTPUT_FIELDS:
        assert_angle(f"{field} after reset", getattr(state, field), 0.0)


def expected_motor_delta(axis: AxisDemo, output_degrees: float, hardware: HWDesc) -> float:
    """Calculate the G1 motion required for one mechanism-level API move."""

    if axis.selector is Selector.G1:
        return output_degrees

    # G2-G6 externally mesh with G1, so the output-per-motor scale is negative.
    output_per_motor = -(
        hardware.g1_diameter_mm / hardware.follower_diameter_mm
    )
    if axis.selector in (Selector.X_TILT, Selector.Y_TILT):
        # Cable travel is spool rotation * spool radius. Plate rotation is cable
        # travel divided by the 25 mm attachment lever. The same length ratio
        # scales radians and degrees.
        spool_radius_mm = hardware.spool_diameter_mm / 2.0
        output_per_motor *= spool_radius_mm / PLATE_ATTACHMENT_LEVER_MM
    return output_degrees / output_per_motor


def show_axis_state(prefix: str, axis: AxisDemo, state: RoboLightState) -> None:
    """Print the selected output, motor position, and simulation time."""

    output = getattr(state, axis.output_field)
    print(
        f"   {prefix:<7} {axis.label:<22} "
        f"motor={state.motor_degrees:7.2f} deg, "
        f"output={output:7.2f} deg, "
        f"time={state.simulation_time_seconds:6.2f} s"
    )


def pause_if_visible(light: RoboLight, seconds: float) -> bool:
    """Pause at an endpoint, returning false if the viewer was closed."""

    if seconds <= 0.0 or not light.viewer_is_running:
        return light.viewer_is_running
    deadline = time.perf_counter() + seconds
    while light.viewer_is_running:
        light.sync_visuals()
        remaining = deadline - time.perf_counter()
        if remaining <= 0.0:
            break
        time.sleep(min(0.05, remaining))
    return light.viewer_is_running


def demonstrate_g1(
    light: RoboLight,
    *,
    velocity: float,
    degrees: float,
    pause_seconds: float,
    require_viewer: bool,
) -> bool:
    """Move the common G1 input out and back without selecting a follower."""

    axis = AXIS_DEMOS[0]
    before = light.state
    forward = move_ok(light, axis.selector, velocity=velocity, degrees=degrees)
    assert_angle("G1 forward", forward.g1_degrees, before.g1_degrees + degrees)
    assert_angle(
        "G1 get_position",
        light.get_position(axis.selector),
        forward.g1_degrees,
    )
    assert_close(
        "G1 forward duration",
        forward.simulation_time_seconds - before.simulation_time_seconds,
        degrees / velocity,
        TIME_TOLERANCE_SECONDS,
    )
    for field in HELD_OUTPUT_FIELDS:
        assert_angle(
            f"{field} held during G1-only move",
            getattr(forward, field),
            getattr(before, field),
        )
    show_axis_state("forward", axis, forward)
    if require_viewer and not pause_if_visible(light, pause_seconds):
        return False

    reverse = move_ok(light, axis.selector, velocity=velocity, degrees=-degrees)
    assert_angle("G1 returned", reverse.g1_degrees, before.g1_degrees)
    assert_close(
        "G1 reverse duration",
        reverse.simulation_time_seconds - forward.simulation_time_seconds,
        degrees / velocity,
        TIME_TOLERANCE_SECONDS,
    )
    show_axis_state("reverse", axis, reverse)
    if require_viewer and not pause_if_visible(light, pause_seconds):
        return False
    return True


def move_outputs_away_from_reset(
    light: RoboLight,
    hardware: HWDesc,
    *,
    velocity: float,
    degrees: float,
    pause_seconds: float,
    require_viewer: bool,
) -> bool:
    """Move every encoded output away from zero and leave it displaced.

    Returns false only when an interactive viewer is closed during the sequence.
    """

    for axis in OUTPUT_DEMOS:
        if require_viewer and not light.viewer_is_running:
            return False

        before = light.state
        expected_motor = expected_motor_delta(axis, degrees, hardware)
        forward = move_ok(
            light,
            axis.selector,
            velocity=velocity,
            degrees=degrees,
        )
        expected_forward = getattr(before, axis.output_field) + degrees
        assert_angle(
            f"{axis.label} forward output",
            getattr(forward, axis.output_field),
            expected_forward,
        )
        assert_angle(
            f"{axis.label} translated G1 motion",
            forward.g1_degrees - before.g1_degrees,
            expected_motor,
        )
        assert_close(
            f"{axis.label} output-speed duration",
            forward.simulation_time_seconds - before.simulation_time_seconds,
            degrees / velocity,
            TIME_TOLERANCE_SECONDS,
        )
        assert forward.last_selectors == (axis.selector.value,)
        assert_angle(
            f"{axis.label} get_position",
            light.get_position(axis.selector),
            getattr(forward, axis.output_field),
        )

        # Outputs not selected for this move must hold their previous values.
        for field in HELD_OUTPUT_FIELDS:
            if field != axis.output_field:
                assert_angle(
                    f"{field} held during {axis.label}",
                    getattr(forward, field),
                    getattr(before, field),
                )
        show_axis_state("forward", axis, forward)
        if require_viewer and not pause_if_visible(light, pause_seconds):
            return False

    return True


def physical_reset_and_verify(
    light: RoboLight,
) -> RoboLightState:
    """Run encoder homing and verify its fixed order, speed, and final state."""

    expected_order = (
        Selector.X_TILT,
        Selector.Y_TILT,
        Selector.ARM2,
        Selector.ARM1,
        Selector.TURNTABLE,
    )
    actual_order = tuple(selector for selector, _ in light.RESET_SEQUENCE)
    if actual_order != expected_order:
        raise AssertionError(f"reset order: expected {expected_order}, got {actual_order}")

    before = light.state
    gear_ratio = light.hwdesc.g1_diameter_mm / light.hwdesc.follower_diameter_mm
    expected_duration = sum(
        abs(reset_encoder_error_degrees(field, getattr(before, field)) / gear_ratio)
        for field in ENCODER_FIELDS
    ) / light.RESET_VELOCITY_DEG_S
    after = light.reset()
    verify_encoder_reset(after)

    actual_duration = after.simulation_time_seconds - before.simulation_time_seconds
    assert_close(
        "physical reset duration",
        actual_duration,
        expected_duration,
        TIME_TOLERANCE_SECONDS,
    )
    print(
        f"   physical reset complete in {actual_duration:.2f} s at "
        f"{light.RESET_VELOCITY_DEG_S:.1f} motor-deg/s"
    )
    return after


def verify_arm2_quarter_turns_and_short_reset(hardware: HWDesc) -> None:
    """Regress exact 90-degree Arm 2 moves and shortest-path encoder homing.

    Four positive 90-degree commands must report the reset orientation again.
    A fifth leaves Arm 2 one quarter-turn from zero. Reset should then move only
    that final quarter-turn rather than unwinding all 450 accumulated degrees.
    """

    light = RoboLight(hardware)
    output_velocity = 90.0
    output_degrees = 90.0
    expected_motor_step = expected_motor_delta(
        AXIS_DEMOS[2],
        output_degrees,
        hardware,
    )

    for move_number in range(1, 6):
        before = light.state
        after = move_ok(
            light,
            Selector.ARM2,
            velocity=output_velocity,
            degrees=output_degrees,
        )
        assert_angle(
            f"Arm 2 move {move_number} output delta",
            shortest_encoder_error_degrees(
                after.arm2_degrees - before.arm2_degrees
            ),
            output_degrees,
        )
        assert_angle(
            f"Arm 2 move {move_number} translated G1 delta",
            after.g1_degrees - before.g1_degrees,
            expected_motor_step,
        )
        if move_number == 4:
            assert_angle(
                "Arm 2 after four quarter-turns",
                after.arm2_degrees,
                0.0,
            )
        assert_angle(
            f"Arm 2 move {move_number} rendered world angle",
            arm2_world_angle_degrees(light),
            after.arm2_degrees,
        )

    before_reset = light.state
    assert_angle("Arm 2 after five quarter-turns", before_reset.arm2_degrees, 90.0)
    encoder_error = shortest_encoder_error_degrees(before_reset.g3_degrees)
    assert_angle("Arm 2 shortest encoder error", encoder_error, 90.0)

    after_reset = light.reset()
    verify_encoder_reset(after_reset)
    expected_duration = (
        abs(encoder_error)
        / (hardware.g1_diameter_mm / hardware.follower_diameter_mm)
        / light.RESET_VELOCITY_DEG_S
    )
    assert_close(
        "Arm 2 shortest-path reset duration",
        after_reset.simulation_time_seconds - before_reset.simulation_time_seconds,
        expected_duration,
        TIME_TOLERANCE_SECONDS,
    )
    print(
        "Shortest-reset regression: five exact 90 deg Arm 2 moves reset via "
        f"the nearest {abs(encoder_error):.0f} deg encoder path"
    )


def verify_multiturn_spool_reset(hardware: HWDesc) -> None:
    """Ensure cable spools use their unique zero rather than a cyclic shortcut."""

    light = RoboLight(hardware)
    before_reset = light.set_tilt(x_degrees=45.0)
    if abs(before_reset.g5_degrees) <= 180.0:
        raise AssertionError(
            "G5 regression setup must exceed half a spool revolution"
        )

    after_reset = light.reset()
    verify_encoder_reset(after_reset)
    gear_ratio = hardware.g1_diameter_mm / hardware.follower_diameter_mm
    expected_duration = (
        abs(before_reset.g5_degrees / gear_ratio)
        / light.RESET_VELOCITY_DEG_S
    )
    assert_close(
        "G5 unique multi-turn reset duration",
        after_reset.simulation_time_seconds - before_reset.simulation_time_seconds,
        expected_duration,
        TIME_TOLERANCE_SECONDS,
    )
    print(
        "Cable-spool regression: G5 reset used its unique multi-turn cable zero"
    )


def verify_ui_arm2_output_translation(hardware: HWDesc) -> None:
    """Verify the UI's default selected-output mode for a 90-degree Arm 2 move."""

    output_degrees = 90.0
    output_per_motor = output_degrees_per_motor_degree(
        "arm2",
        hardware.g1_diameter_mm,
        hardware.follower_diameter_mm,
        hardware.spool_diameter_mm,
    )
    motor_degrees = output_degrees / output_per_motor
    assert_angle("UI Arm 2 translated motor angle", motor_degrees, -140.625)
    assert_angle(
        "UI Arm 2 reconstructed output angle",
        motor_degrees * output_per_motor,
        output_degrees,
    )
    print(
        "UI translation regression: Arm 2 +90 deg commands G1 -140.625 deg"
    )


def verify_arm_length_hardware_settings(hardware: HWDesc) -> None:
    """Verify HWDesc arm lengths resize both links and preserve joint pose."""

    light = RoboLight(hardware)
    move_ok(light, Selector.ARM1, velocity=30.0, degrees=20.0)
    move_ok(light, Selector.ARM2, velocity=30.0, degrees=-15.0)
    before = light.state
    updated = light.set_hw(
        {
            "arm1_length_mm": 220.0,
            "arm2_length_mm": 110.0,
        }
    )
    if updated.arm1_length_mm != 220.0 or updated.arm2_length_mm != 110.0:
        raise AssertionError(f"unexpected updated arm lengths: {updated}")

    assert_angle("Arm 1 pose held during resize", light.state.arm1_degrees, before.arm1_degrees)
    assert_angle("Arm 2 pose held during resize", light.state.arm2_degrees, before.arm2_degrees)

    arm1_geom_id = mujoco.mj_name2id(
        light.model,
        mujoco.mjtObj.mjOBJ_GEOM,
        "arm1",
    )
    arm2_geom_id = mujoco.mj_name2id(
        light.model,
        mujoco.mjtObj.mjOBJ_GEOM,
        "arm2",
    )
    tg6_body_id = mujoco.mj_name2id(
        light.model,
        mujoco.mjtObj.mjOBJ_BODY,
        "TG6",
    )
    plate_body_id = mujoco.mj_name2id(
        light.model,
        mujoco.mjtObj.mjOBJ_BODY,
        "tilt_plate_body",
    )
    assert_close(
        "Arm 1 rendered length",
        light.model.geom_size[arm1_geom_id, 2] * 2000.0,
        220.0,
        ANGLE_TOLERANCE_DEGREES,
    )
    assert_close(
        "Arm 1 endpoint position",
        light.model.body_pos[tg6_body_id, 2] * 1000.0,
        220.0,
        ANGLE_TOLERANCE_DEGREES,
    )
    assert_close(
        "Arm 2 rendered length",
        light.model.geom_size[arm2_geom_id, 2] * 2000.0,
        110.0,
        ANGLE_TOLERANCE_DEGREES,
    )
    assert_close(
        "Arm 2 endpoint position",
        light.model.body_pos[plate_body_id, 2] * 1000.0,
        110.0,
        ANGLE_TOLERANCE_DEGREES,
    )

    try:
        light.set_hw({"arm1_length_mm": 74.0})
    except ValueError:
        pass
    else:
        raise AssertionError("arm1_length_mm accepted a value below 75 mm")

    print("HW regression: Arm 1/Arm 2 lengths resize links and preserve pose")


def verify_move_errors_and_platform_constraint(hardware: HWDesc) -> None:
    """Verify Arm 1 limits, Arm 2-dependent clearance, and atomic rejection."""

    light = RoboLight(hardware)

    def expect_rejection(
        selector: Selector | str,
        *,
        velocity: float,
        degrees: float,
        expected: MoveError,
    ) -> None:
        before = light.state.to_dict()
        result = light.move(selector, velocity=velocity, degrees=degrees)
        if result is not expected:
            raise AssertionError(
                f"expected {expected.value}, got {result.value} for "
                f"{selector!s} {degrees:g} deg"
            )
        after = light.state.to_dict()
        if after != before:
            raise AssertionError(
                f"rejected {result.value} command changed mechanism state"
            )

    move_ok(
        light,
        Selector.ARM1,
        velocity=30.0,
        degrees=hardware.arm1_limit_degrees,
    )
    assert_angle(
        "Arm 1 positive constraint boundary",
        light.state.arm1_degrees,
        hardware.arm1_limit_degrees,
    )
    expect_rejection(
        Selector.ARM1,
        velocity=30.0,
        degrees=0.25,
        expected=MoveError.ARM1_LIMIT,
    )

    move_ok(
        light,
        Selector.ARM1,
        velocity=30.0,
        degrees=-(2.0 * hardware.arm1_limit_degrees),
    )
    assert_angle(
        "Arm 1 negative constraint boundary",
        light.state.arm1_degrees,
        -hardware.arm1_limit_degrees,
    )
    expect_rejection(
        Selector.ARM1,
        velocity=30.0,
        degrees=-0.25,
        expected=MoveError.ARM1_LIMIT,
    )

    light.reset()
    move_ok(light, Selector.ARM2, velocity=30.0, degrees=180.0)
    expect_rejection(
        Selector.ARM1,
        velocity=30.0,
        degrees=hardware.arm1_limit_degrees,
        expected=MoveError.PLATFORM_COLLISION,
    )
    expect_rejection(
        Selector.ARM1,
        velocity=30.0,
        degrees=361.0,
        expected=MoveError.INVALID_DEGREES,
    )
    expect_rejection(
        "not_a_selector",
        velocity=30.0,
        degrees=1.0,
        expected=MoveError.INVALID_SELECTOR,
    )
    expect_rejection(
        Selector.G1,
        velocity=0.0,
        degrees=1.0,
        expected=MoveError.INVALID_VELOCITY,
    )
    expect_rejection(
        Selector.Y_TILT,
        velocity=30.0,
        degrees=46.0,
        expected=MoveError.TILT_LIMIT,
    )
    expect_rejection(
        Selector.Y_TILT,
        velocity=100.0,
        degrees=1.0,
        expected=MoveError.MOTOR_SPEED_LIMIT,
    )

    light.reset()
    updated = light.set_hw({"arm1_limit_degrees": 60.0})
    if updated.arm1_limit_degrees != 60.0:
        raise AssertionError(f"unexpected Arm 1 limit: {updated}")
    move_ok(light, Selector.ARM1, velocity=30.0, degrees=60.0)
    expect_rejection(
        Selector.ARM1,
        velocity=30.0,
        degrees=0.25,
        expected=MoveError.ARM1_LIMIT,
    )

    print(
        "Constraint regression: configured Arm 1 boundary and Arm 2-dependent "
        "infinite-platform clearance return atomic MoveError results"
    )


def verify_camera_and_feedback_api(hardware: HWDesc) -> None:
    """Verify camera image format and reserved physical-feedback results."""

    light = RoboLight(hardware)
    image = light.get_camera()
    if image.shape != (240, 320, 3):
        raise AssertionError(f"unexpected camera shape: {image.shape}")
    if image.dtype.name != "uint8":
        raise AssertionError(f"unexpected camera dtype: {image.dtype}")
    if not image.flags.owndata:
        raise AssertionError("get_camera() did not return an owned image copy")
    if int(image.max()) == int(image.min()):
        raise AssertionError("get_camera() returned a constant image")

    try:
        light.get_position(Selector.ALL)
    except ValueError:
        pass
    else:
        raise AssertionError("get_position() accepted a multi-output selector")

    if MoveError.LOST_STEPS.value != "lost_steps":
        raise AssertionError("MoveError.LOST_STEPS value changed")
    if MoveError.HIJACKED.value != "hijacked":
        raise AssertionError("MoveError.HIJACKED value changed")

    print(
        "Sensor API regression: selector positions, RGB camera capture, and "
        "stubbed lost-steps/hijacked errors are available"
    )


def main() -> None:
    """Run one complete demonstration, reset, then repeat the sequence."""

    args = parse_args()
    hardware = DEMO_HARDWARE
    light = RoboLight(hardware, realtime=not args.headless)

    try:
        if not args.headless:
            print("Opening MuJoCo viewer and spotlight-camera PIP.")
            print("Close the main viewer or press Ctrl+C to stop.")
            light.open_viewer()
            light.open_pip()
            pause_if_visible(light, 0.75)
        else:
            print("Running finite headless API smoke test.")

        verify_encoder_reset(light.state)

        print("Initial demonstration: G1 moves without a selected output")
        completed = demonstrate_g1(
            light,
            velocity=args.velocity,
            degrees=args.degrees,
            pause_seconds=0.0 if args.headless else args.pause,
            require_viewer=not args.headless,
        )
        if not completed:
            return

        print("Move all five encoded outputs away from reset")
        completed = move_outputs_away_from_reset(
            light,
            hardware,
            velocity=args.velocity,
            degrees=args.degrees,
            pause_seconds=0.0 if args.headless else args.pause,
            require_viewer=not args.headless,
        )
        if not completed:
            return

        print("Physical reset: X, Y, Arm 2, Arm 1, turntable")
        physical_reset_and_verify(light)
        if not args.headless and not pause_if_visible(light, 0.75):
            return

        cycle = 0
        while args.cycles is None or cycle < args.cycles:
            if not args.headless and not light.viewer_is_running:
                break
            cycle += 1
            print(f"Loop cycle {cycle}: move all encoded outputs away")
            completed = move_outputs_away_from_reset(
                light,
                hardware,
                velocity=args.velocity,
                degrees=args.degrees,
                pause_seconds=0.0 if args.headless else args.pause,
                require_viewer=not args.headless,
            )
            if not completed:
                break
            print(f"Loop cycle {cycle}: physical encoder reset")
            physical_reset_and_verify(light)
            if not args.headless and not pause_if_visible(light, 0.75):
                break

        if args.headless:
            verify_ui_arm2_output_translation(hardware)
            verify_arm_length_hardware_settings(hardware)
            verify_move_errors_and_platform_constraint(hardware)
            verify_camera_and_feedback_api(hardware)
            verify_arm2_quarter_turns_and_short_reset(hardware)
            verify_multiturn_spool_reset(hardware)
            print("RoboLight all-axis headless smoke test passed")
    except KeyboardInterrupt:
        print("\nRoboLight demonstration stopped by user")
    finally:
        light.close_pip()
        light.close_viewer()


if __name__ == "__main__":
    main()
