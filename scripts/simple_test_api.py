r"""Minimal continuous RoboLight API demonstration.

Run this file from the repository root::

    .\.venv\Scripts\python.exe .\scripts\simple_test_api.py

The script prints its fixed ``HWDesc``, performs a physical encoder reset, and
opens the MuJoCo viewer plus the spotlight-camera PIP. It then runs forever,
randomly choosing one mechanism axis, an integer selected-output velocity, and
an integer signed displacement for each move. Every loop also acquires a camera
image and reports its latency. Close the main viewer or press Ctrl+C to stop.

There are intentionally no command-line options. Edit the constants and
``DEMO_HARDWARE`` below to experiment with another configuration.
"""

from pathlib import Path
import random
import sys
import time


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
    turntable_limit_degrees=90.0,
    beam_angle_degrees=50.0,
    camera_fov_degrees=50.0,
)

AXES = (
    Selector.G1,
    Selector.ARM1,
    Selector.ARM2,
    Selector.Y_TILT,
    Selector.X_TILT,
    Selector.TURNTABLE,
)
AXIS_NAMES = {
    Selector.G1: "Motor",
    Selector.ARM1: "Arm1",
    Selector.ARM2: "Arm2",
    Selector.Y_TILT: "Y tilt",
    Selector.X_TILT: "X tilt",
    Selector.TURNTABLE: "Turntable",
}
MIN_VELOCITY_DEG_S = 10
MAX_VELOCITY_DEG_S = 60
MIN_MOVE_DEGREES = 5
MAX_MOVE_DEGREES = 30
PAUSE_SECONDS = 0.35


def random_degrees(
    light: RoboLight,
    selector: Selector,
    generator: random.Random,
) -> int:
    """Choose a signed whole-degree move, respecting simple joint limits.

    Arm-to-platform clearance depends on both arm positions, so that geometric
    constraint remains the controller's responsibility. A rejected arm move is
    reported and followed by a physical reset.
    """

    limits = {
        Selector.ARM1: light.hwdesc.arm1_limit_degrees,
        Selector.Y_TILT: 45.0,
        Selector.X_TILT: 45.0,
        Selector.TURNTABLE: light.hwdesc.turntable_limit_degrees,
    }
    limit = limits.get(selector)
    for _ in range(20):
        magnitude = generator.randint(MIN_MOVE_DEGREES, MAX_MOVE_DEGREES)
        candidate = generator.choice((-magnitude, magnitude))
        if limit is None:
            return candidate
        target = light.get_position(selector) + candidate
        if -limit <= target <= limit:
            return candidate

    # At a constrained endpoint, guarantee that the fallback heads inward.
    current = light.get_position(selector)
    magnitude = min(MAX_MOVE_DEGREES, max(MIN_MOVE_DEGREES, round(abs(current))))
    return -magnitude if current > 0.0 else magnitude


def pause_while_visible(light: RoboLight, seconds: float) -> None:
    """Keep both visual windows responsive during the inter-move pause."""

    deadline = time.perf_counter() + seconds
    while light.viewer_is_running and time.perf_counter() < deadline:
        light.sync_visuals()
        time.sleep(min(0.05, max(0.0, deadline - time.perf_counter())))


def main() -> None:
    """Run random single-axis moves until the viewer closes or Ctrl+C."""

    print(f"HWDesc: {DEMO_HARDWARE}")
    light = RoboLight(DEMO_HARDWARE, realtime=True)
    generator = random.Random()
    try:
        light.open_viewer()
        light.open_pip()
        light.set_target(0.0, -6.4, 55.0, color="red", diameter_cm=2.0)
        print("Target: (0, -6.4, 55) cm, red, 2 cm diameter")

        print("Startup physical reset")
        before_reset = light.state.simulation_time_seconds
        reset_state = light.reset()
        reset_seconds = reset_state.simulation_time_seconds - before_reset
        print(
            f"Reset completed in {reset_seconds:.1f} sec at motor velocity "
            f"{light.RESET_VELOCITY_DEG_S:.0f} deg/sec"
        )
        print("Random motion loop started; close the viewer or press Ctrl+C.")

        while light.viewer_is_running:
            selector = generator.choice(AXES)
            velocity = generator.randint(
                MIN_VELOCITY_DEG_S,
                MAX_VELOCITY_DEG_S,
            )
            degrees = random_degrees(light, selector, generator)
            before_move = light.state.simulation_time_seconds
            result = light.move(
                selector,
                velocity=velocity,
                degrees=degrees,
            )
            elapsed = light.state.simulation_time_seconds - before_move
            name = AXIS_NAMES[selector]
            if result is MoveError.OK:
                print(
                    f"{name} moved {degrees:d} deg at {velocity:d} deg/sec "
                    f"in {elapsed:.1f} sec"
                )
            else:
                print(
                    f"{name} move {degrees:d} deg at {velocity:d} deg/sec "
                    f"rejected: {result.value}"
                )
                light.reset()
                print("Physical reset completed after rejected move")

            camera_started = time.perf_counter()
            image = light.get_camera()
            camera_seconds = time.perf_counter() - camera_started
            print(
                f"Camera image {image.shape[1]}x{image.shape[0]} acquired "
                f"in {camera_seconds:.3f} sec"
            )
            pause_while_visible(light, PAUSE_SECONDS)
    except KeyboardInterrupt:
        print("\nRoboLight random demonstration stopped by user")
    finally:
        light.close_pip()
        light.close_viewer()


if __name__ == "__main__":
    main()
