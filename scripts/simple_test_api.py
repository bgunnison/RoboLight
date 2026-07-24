r"""Minimal visible RoboLight API demonstration with fixed hardware settings.

Run this file from the repository root::

    .\.venv\Scripts\python.exe .\scripts\simple_test_api.py

The MuJoCo viewer and spotlight-camera PIP open, each mechanism axis moves once,
and the physical encoder reset returns the mechanism to its reset pose. There
are no command-line options; edit ``DEMO_HARDWARE`` or the move calls below to
experiment with another configuration.
"""

from pathlib import Path
import sys


# Direct execution puts scripts/ rather than the repository root on sys.path.
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scripts import HWDesc, MoveError, RoboLight, Selector


DEMO_HARDWARE = HWDesc(
    g1_diameter_mm=64.0,
    follower_diameter_mm=100.0,
    spool_diameter_mm=10.0,
    arm1_length_mm=150.0,
    arm2_length_mm=150.0,
    arm1_limit_degrees=80.0,
    beam_angle_degrees=50.0,
    camera_fov_degrees=50.0,
)


def main() -> None:
    """Open the visual simulation, demonstrate every axis, and reset it."""

    print(f"Hardware: {DEMO_HARDWARE}")
    light = RoboLight(DEMO_HARDWARE, realtime=True)
    try:
        light.open_viewer()
        light.open_pip()
        light.set_target(0.0, -6.4, 55.0, color="red", diameter_cm=2.0)
        print("Target: (0.0, -6.4, 55.0) cm, red, 2.0 cm diameter")

        # move() angles and velocities are selected-output units.
        moves = (
            (Selector.G1, 100.0, 30.0),
            (Selector.ARM1, 30.0, 30.0),
            (Selector.ARM2, 30.0, 30.0),
            (Selector.Y_TILT, 15.0, 15.0),
            (Selector.X_TILT, 15.0, -15.0),
            (Selector.TURNTABLE, 30.0, 45.0),
        )
        for selector, velocity, degrees in moves:
            result = light.move(selector, velocity, degrees)
            print(selector.value, result.value, light.get_position(selector))
            if result is not MoveError.OK:
                break

        image = light.get_camera()
        print("Camera image:", image.shape, image.dtype)

        print("Physical reset: X tilt, Y tilt, Arm 2, Arm 1, turntable")
        reset_state = light.reset()
        print(
            f"Reset complete at simulation time "
            f"{reset_state.simulation_time_seconds:.2f} s"
        )
    finally:
        light.close_pip()
        light.close_viewer()


if __name__ == "__main__":
    main()
