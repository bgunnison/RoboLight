r"""Visible RoboLight API demonstration and automated smoke test.

Normal execution opens the MuJoCo simulation and a separate spotlight-camera
PIP window so the mechanism and its light-aligned view can be watched together.
The script moves every selectable path separately:

1. G1 by itself (no follower selected)
2. Arm 1 through G2
3. Arm 2 through G3
4. Y tilt through the G4 cable spool
5. X tilt through the G5 cable spool
6. The complete turntable through G6

Each path moves forward and back while every unselected output holds position.
After the first complete demonstration, the mechanism resets to zero. The same
all-axis sequence then repeats until the MuJoCo viewer is closed or Ctrl+C is
pressed. Closing the viewer is the normal way to end the interactive test.

Run the visible demonstration from the repository root::

    .\.venv\Scripts\python.exe .\scripts\test_api.py

The repository build uses a finite, non-visual variant so it cannot wait for a
person to close a window::

    .\.venv\Scripts\python.exe .\scripts\test_api.py --headless --cycles 1

``--cycles N`` limits the number of repeated cycles *after* the initial
all-axis demonstration and reset. Without that option, visible mode loops until
the viewer closes. ``--velocity`` and ``--degrees`` adjust the demonstration.

RoboLight is intended to become a camera-guided robotic flashlight. Pointer
recognition, physical manual-movement detection, Hijacked mode, and its blinking
LED are not implemented yet; this script tests and displays the current
single-motor kinematic transmission.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import sys
import time


# Direct execution places ``scripts/`` rather than the repository root on
# ``sys.path``. Add the root so the documented command works without packaging.
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scripts import HWDesc, RoboLight, RoboLightState, Selector


ANGLE_TOLERANCE_DEGREES = 1e-7
TIME_TOLERANCE_SECONDS = 1e-7
PLATE_ATTACHMENT_LEVER_MM = 25.0


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

HELD_OUTPUT_FIELDS = (
    "arm1_degrees",
    "arm2_degrees",
    "x_tilt_degrees",
    "y_tilt_degrees",
    "turntable_degrees",
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
        default=90.0,
        help="motor speed in degrees per second (default: 90)",
    )
    parser.add_argument(
        "--degrees",
        type=float,
        default=60.0,
        help="forward motor rotation for each axis in degrees (default: 60)",
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
    if not math.isfinite(args.velocity) or not 0.0 < args.velocity <= 720.0:
        parser.error("--velocity must be above 0 and at most 720")
    if not math.isfinite(args.degrees) or not 0.0 < args.degrees <= 360.0:
        parser.error("--degrees must be above 0 and at most 360")
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


def verify_zero_state(state: RoboLightState) -> None:
    """Verify reset cleared the motor, all outputs, and simulation time."""

    assert_angle("motor after reset", state.motor_degrees, 0.0)
    assert_angle("G1 after reset", state.g1_degrees, 0.0)
    for field in HELD_OUTPUT_FIELDS:
        assert_angle(f"{field} after reset", getattr(state, field), 0.0)
    assert_close(
        "simulation time after reset",
        state.simulation_time_seconds,
        0.0,
        TIME_TOLERANCE_SECONDS,
    )


def expected_output_delta(axis: AxisDemo, motor_degrees: float, hardware: HWDesc) -> float:
    """Calculate the useful output change expected from one motor move."""

    if axis.selector is Selector.G1:
        return motor_degrees

    # G2-G6 are external meshes, so they turn opposite G1. The magnitude is the
    # G1-to-follower diameter ratio.
    follower_degrees = -motor_degrees * (
        hardware.g1_diameter_mm / hardware.follower_diameter_mm
    )
    if axis.selector in (Selector.X_TILT, Selector.Y_TILT):
        # Cable travel is spool rotation * spool radius. Plate rotation is cable
        # travel divided by the 25 mm attachment lever. The same length ratio
        # scales radians and degrees.
        spool_radius_mm = hardware.spool_diameter_mm / 2.0
        return follower_degrees * (spool_radius_mm / PLATE_ATTACHMENT_LEVER_MM)
    return follower_degrees


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


def run_axis_sequence(
    light: RoboLight,
    hardware: HWDesc,
    *,
    velocity: float,
    degrees: float,
    pause_seconds: float,
    require_viewer: bool,
) -> bool:
    """Move every axis out and back, asserting selection and friction hold.

    Returns false only when an interactive viewer is closed during the sequence.
    """

    for axis in AXIS_DEMOS:
        if require_viewer and not light.viewer_is_running:
            return False

        before = light.state
        expected_delta = expected_output_delta(axis, degrees, hardware)
        forward = light.move(axis.selector, velocity=velocity, degrees=degrees)
        expected_forward = getattr(before, axis.output_field) + expected_delta
        assert_angle(
            f"{axis.label} forward output",
            getattr(forward, axis.output_field),
            expected_forward,
        )
        assert forward.last_selectors == (axis.selector.value,)

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

        reverse = light.move(axis.selector, velocity=velocity, degrees=-degrees)
        assert_angle(
            f"{axis.label} returned to start",
            getattr(reverse, axis.output_field),
            getattr(before, axis.output_field),
        )
        assert_angle(
            f"motor returned after {axis.label}",
            reverse.motor_degrees,
            before.motor_degrees,
        )
        show_axis_state("reverse", axis, reverse)
        if require_viewer and not pause_if_visible(light, pause_seconds):
            return False

    return True


def main() -> None:
    """Run one complete demonstration, reset, then repeat the sequence."""

    args = parse_args()
    hardware = HWDesc(
        g1_diameter_mm=64.0,
        follower_diameter_mm=100.0,
        spool_diameter_mm=10.0,
    )
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

        initial = light.reset()
        verify_zero_state(initial)

        print("Initial demonstration: move every axis forward and back")
        completed = run_axis_sequence(
            light,
            hardware,
            velocity=args.velocity,
            degrees=args.degrees,
            pause_seconds=0.0 if args.headless else args.pause,
            require_viewer=not args.headless,
        )
        if not completed:
            return

        print("Reset after initial all-axis demonstration")
        verify_zero_state(light.reset())
        if not args.headless and not pause_if_visible(light, 0.75):
            return

        cycle = 0
        while args.cycles is None or cycle < args.cycles:
            if not args.headless and not light.viewer_is_running:
                break
            cycle += 1
            print(f"Loop cycle {cycle}: move every axis")
            completed = run_axis_sequence(
                light,
                hardware,
                velocity=args.velocity,
                degrees=args.degrees,
                pause_seconds=0.0 if args.headless else args.pause,
                require_viewer=not args.headless,
            )
            if not completed:
                break
            print(f"Reset after loop cycle {cycle}")
            verify_zero_state(light.reset())
            if not args.headless and not pause_if_visible(light, 0.75):
                break

        if args.headless:
            print("RoboLight all-axis headless smoke test passed")
    except KeyboardInterrupt:
        print("\nRoboLight demonstration stopped by user")
    finally:
        light.close_pip()
        light.close_viewer()


if __name__ == "__main__":
    main()
