r"""Acquire a moving 3D target and keep the RoboLight spot centered on it.

The target remains stationary during initial acquisition. Once acquired, it
moves slowly toward random valid X/Y/Z waypoints inside the room. Camera
images—not the known simulated target coordinates—drive the tracking
corrections. Fast X/Y plate motion maintains lock while single-step visual
rebalancing lets the arms and turntable gradually follow.

If the red sphere leaves the camera image or centering fails, tracking motion
stops immediately. The mechanism physically resets and runs the existing
camera-only acquisition scan while the target remains stationary. If a full
scan fails, the target jumps to another random valid X/Y/Z location and the
scan repeats. Slow target motion resumes only after acquisition succeeds.

Run the visible continuous demonstration from the repository root::

    .\.venv\Scripts\python.exe .\scripts\track_target.py

Close the main viewer or press Ctrl+C to stop. A deterministic headless smoke
test is also available::

    .\.venv\Scripts\python.exe .\scripts\track_target.py --headless --steps 30 --seed 0
"""

from __future__ import annotations

import argparse
from contextlib import redirect_stderr, redirect_stdout
from dataclasses import dataclass
from datetime import datetime
import math
from pathlib import Path
import random
import sys
import time
import traceback

import numpy as np


# Direct execution puts scripts/ rather than the repository root on sys.path.
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scripts import RoboLight
from scripts.acquire_target import (
    AcquisitionResult,
    DEMO_HARDWARE,
    MAX_ACQUISITION_SECONDS,
    TARGET_DIAMETER_CM,
    TARGET_SURFACE_CLEARANCE_M,
    TeeTextStream,
    acquire_target,
    build_optimized_search_plan,
    center_visible_target,
    find_red_target,
    plot_gave_up_targets,
    random_target_position,
    rebalance_centered_target,
    SearchPlan,
)


# Edit these values to tune the visible tracking behavior. The time multiplier
# accelerates mechanism motion only; target motion uses the unscaled speed and
# update period below.
SIMULATION_TIME_MULTIPLIER = 100.0
TARGET_SPEED_CM_S = 2.0 * 10
TRACK_UPDATE_SECONDS = 0.10
TRACK_CORRECTION_TIMEOUT_SECONDS = 10.0
TRACK_REBALANCE_TRIGGER_DEGREES = 8.0
DEFAULT_HEADLESS_STEPS = 30


@dataclass(slots=True)
class TargetMotion:
    """Current target position and its next valid room waypoint."""

    position_cm: np.ndarray
    waypoint_cm: np.ndarray


def parse_args() -> argparse.Namespace:
    """Parse visible-demo and deterministic headless-test options."""

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--headless",
        action="store_true",
        help="do not open windows or pace tracking updates in wall-clock time",
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=None,
        help="number of target-motion updates (default: continuous visibly)",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="optional random seed for repeatable target motion",
    )
    args = parser.parse_args()
    if args.steps is not None and args.steps < 1:
        parser.error("--steps must be at least 1")
    if args.headless and args.steps is None:
        args.steps = DEFAULT_HEADLESS_STEPS
    return args


def target_position_is_valid(
    light: RoboLight,
    position_cm: np.ndarray,
) -> bool:
    """Return whether a sphere center obeys the room and upper-table bounds."""

    base = light.model.geom("base")
    left_wall = light.model.geom("left_wall")
    right_wall = light.model.geom("right_wall")
    back_wall = light.model.geom("back_wall")
    ceiling = light.model.geom("ceiling")
    upper_table = light.model.geom("upper_turntable_disk")
    target_origin = light.model.body("target_origin_frame")
    radius_m = TARGET_DIAMETER_CM / 200.0
    margin_m = radius_m + TARGET_SURFACE_CLEARANCE_M
    world_position = (
        light.data.xpos[target_origin.id]
        + np.asarray(position_cm, dtype=np.float64) / 100.0
    )

    inside_room = (
        world_position[0]
        >= light.data.geom_xpos[left_wall.id, 0] + left_wall.size[0] + margin_m
        and world_position[0]
        <= light.data.geom_xpos[right_wall.id, 0]
        - right_wall.size[0]
        - margin_m
        and world_position[1]
        >= light.data.geom_xpos[base.id, 1] - base.size[1] + margin_m
        and world_position[1]
        <= light.data.geom_xpos[back_wall.id, 1]
        - back_wall.size[1]
        - margin_m
        and world_position[2]
        >= light.data.geom_xpos[base.id, 2] + base.size[2] + margin_m
        and world_position[2]
        <= light.data.geom_xpos[ceiling.id, 2]
        - ceiling.size[2]
        - margin_m
    )
    if not inside_room:
        return False

    table_center = light.data.geom_xpos[upper_table.id]
    safe_table_radius = upper_table.size[0] + margin_m
    safe_table_top = table_center[2] + upper_table.size[1] + margin_m
    radial_distance = math.hypot(
        world_position[0] - table_center[0],
        world_position[1] - table_center[1],
    )
    return (
        world_position[2] >= safe_table_top
        or radial_distance >= safe_table_radius
    )


def choose_waypoint(
    light: RoboLight,
    generator: random.Random,
) -> np.ndarray:
    """Choose a random valid X/Y/Z destination in centimeters."""

    return np.asarray(
        random_target_position(light, generator),
        dtype=np.float64,
    )


def advance_target(
    light: RoboLight,
    motion: TargetMotion,
    generator: random.Random,
) -> None:
    """Advance one slow step without crossing a forbidden target region."""

    step_cm = TARGET_SPEED_CM_S * TRACK_UPDATE_SECONDS
    for _ in range(100):
        offset = motion.waypoint_cm - motion.position_cm
        distance_cm = float(np.linalg.norm(offset))
        if distance_cm <= step_cm:
            candidate = motion.waypoint_cm.copy()
        else:
            candidate = (
                motion.position_cm + offset * (step_cm / distance_cm)
            )

        if target_position_is_valid(light, candidate):
            motion.position_cm = candidate
            if distance_cm <= step_cm:
                motion.waypoint_cm = choose_waypoint(light, generator)
            return
        motion.waypoint_cm = choose_waypoint(light, generator)

    raise RuntimeError("could not find a valid direction for target motion")


def acquire_stationary_target(
    light: RoboLight,
    search_plan: SearchPlan,
    *,
    reason: str,
) -> AcquisitionResult | None:
    """Physically reset and acquire without moving the target during the scan."""

    print(f"{reason}: target frozen; physical reset and camera search")
    light.reset()
    outcome = acquire_target(
        light,
        timeout_seconds=MAX_ACQUISITION_SECONDS,
        search_plan=search_plan,
    )
    if outcome.result is None:
        print(
            f"  acquisition failed after {outcome.elapsed_seconds:.1f} sec"
        )
        return None
    result = outcome.result
    print(
        f"  acquired in {outcome.elapsed_seconds:.1f} sec: "
        f"X tilt={result.x_tilt_degrees:.1f} deg, "
        f"Y tilt={result.y_tilt_degrees:.1f} deg"
    )
    return result


def move_target_to_random_location(
    light: RoboLight,
    motion: TargetMotion,
    generator: random.Random,
) -> None:
    """Jump the target to a new valid location and choose its next waypoint."""

    motion.position_cm = choose_waypoint(light, generator)
    motion.waypoint_cm = choose_waypoint(light, generator)
    light.set_target(
        *motion.position_cm,
        color="red",
        diameter_cm=TARGET_DIAMETER_CM,
    )
    print(
        "  target moved for another acquisition attempt: "
        f"X={motion.position_cm[0]:.1f}, "
        f"Y={motion.position_cm[1]:.1f}, "
        f"Z={motion.position_cm[2]:.1f} cm"
    )


def acquire_with_random_relocation(
    light: RoboLight,
    search_plan: SearchPlan,
    motion: TargetMotion,
    generator: random.Random,
    *,
    reason: str,
    headless: bool,
) -> AcquisitionResult | None:
    """Retry acquisition, randomly relocating after each failed full scan."""

    attempt = 1
    while True:
        attempt_reason = reason if attempt == 1 else f"{reason} attempt {attempt}"
        result = acquire_stationary_target(
            light,
            search_plan,
            reason=attempt_reason,
        )
        if result is not None:
            return result
        if not headless and not light.viewer_is_running:
            return None
        move_target_to_random_location(light, motion, generator)
        attempt += 1


def track_current_target(
    light: RoboLight,
) -> AcquisitionResult | None:
    """Use one camera feedback cycle to center the currently visible target."""

    image = light.get_camera()
    detection = find_red_target(image)
    if detection is None:
        return None

    deadline = time.perf_counter() + TRACK_CORRECTION_TIMEOUT_SECONDS
    try:
        result, _ = center_visible_target(
            light,
            image,
            detection,
            1,
            deadline,
        )
        if result is None:
            return None

        tilt_norm = math.hypot(
            result.x_tilt_degrees,
            result.y_tilt_degrees,
        )
        if tilt_norm >= TRACK_REBALANCE_TRIGGER_DEGREES:
            result = rebalance_centered_target(
                light,
                result,
                deadline,
                max_iterations=1,
                announce=False,
            )
        return result
    except TimeoutError:
        return None


def mark_lost_target(
    light: RoboLight,
    viewer: object | None,
    position_cm: np.ndarray,
) -> None:
    """Leave a persistent green viewer marker at a lost-target position."""

    if viewer is None:
        return
    plot_gave_up_targets(
        light,
        viewer,
        (tuple(float(value) for value in position_cm),),
    )


def log_lost_target(
    light: RoboLight,
    *,
    loss_number: int,
    step_number: int,
    position_cm: np.ndarray,
) -> None:
    """Log the target and complete camera-pointing pose before reset."""

    state = light.state
    print(
        f"Target lost #{loss_number} after step {step_number}: "
        f"target X={position_cm[0]:.3f}, "
        f"Y={position_cm[1]:.3f}, "
        f"Z={position_cm[2]:.3f} cm; "
        f"Arm1={state.arm1_degrees:.3f}, "
        f"Arm2={state.arm2_degrees:.3f}, "
        f"Turntable={state.turntable_degrees:.3f}, "
        f"X tilt={state.x_tilt_degrees:.3f}, "
        f"Y tilt={state.y_tilt_degrees:.3f} deg"
    )


def timestamped_log_path(started_at: datetime) -> Path:
    """Create the log directory and return a unique tracking-log path."""

    log_directory = Path(__file__).resolve().parent / "logs"
    log_directory.mkdir(parents=True, exist_ok=True)
    timestamp = started_at.strftime("%Y%m%d_%H%M%S_%f")
    return log_directory / f"track_target_{timestamp}.log"


def main() -> None:
    """Acquire once, move while locked, and reacquire whenever lock is lost."""

    args = parse_args()
    generator = random.Random(args.seed)
    light = RoboLight(
        DEMO_HARDWARE,
        realtime=not args.headless,
        time_multiplier=SIMULATION_TIME_MULTIPLIER,
    )
    try:
        print(f"HWDesc: {DEMO_HARDWARE}")
        print(f"Target speed: {TARGET_SPEED_CM_S:g} cm/sec")
        print(
            "Mechanism time multiplier: "
            f"{SIMULATION_TIME_MULTIPLIER:g}x (target motion unscaled)"
        )
        search_plan = build_optimized_search_plan(light)
        print(
            f"Acquisition plan: {len(search_plan.views)} camera views at "
            f"{search_plan.coarse_pose_count} arm/turntable poses"
        )

        viewer: object | None = None
        if not args.headless:
            viewer = light.open_viewer()
            light.open_pip()
            print("Close the main viewer or press Ctrl+C to stop.")

        initial_position = choose_waypoint(light, generator)
        light.set_target(
            *initial_position,
            color="red",
            diameter_cm=TARGET_DIAMETER_CM,
        )
        motion = TargetMotion(
            position_cm=initial_position,
            waypoint_cm=choose_waypoint(light, generator),
        )
        print(
            "Initial target: "
            f"X={motion.position_cm[0]:.1f}, "
            f"Y={motion.position_cm[1]:.1f}, "
            f"Z={motion.position_cm[2]:.1f} cm"
        )

        result = acquire_with_random_relocation(
            light,
            search_plan,
            motion,
            generator,
            reason="Initial acquisition",
            headless=args.headless,
        )
        if result is None:
            return

        completed_steps = 0
        loss_count = 0
        while args.steps is None or completed_steps < args.steps:
            if not args.headless and not light.viewer_is_running:
                break
            update_started = time.perf_counter()

            advance_target(light, motion, generator)
            light.set_target(
                *motion.position_cm,
                color="red",
                diameter_cm=TARGET_DIAMETER_CM,
            )
            completed_steps += 1
            result = track_current_target(light)
            if result is None:
                loss_count += 1
                mark_lost_target(light, viewer, motion.position_cm)
                log_lost_target(
                    light,
                    loss_number=loss_count,
                    step_number=completed_steps,
                    position_cm=motion.position_cm,
                )
                result = acquire_with_random_relocation(
                    light,
                    search_plan,
                    motion,
                    generator,
                    reason="Reacquisition",
                    headless=args.headless,
                )
                if result is None:
                    return
                print("  lock restored; target motion resumed")

            if not args.headless:
                remaining = (
                    TRACK_UPDATE_SECONDS
                    - (time.perf_counter() - update_started)
                )
                if remaining > 0.0:
                    time.sleep(remaining)
                light.sync_visuals()

        print(
            f"Tracking complete: {completed_steps} target moves, "
            f"{loss_count} losses"
        )
    except KeyboardInterrupt:
        print("\nTarget tracking stopped by user")
    finally:
        light.close_pip()
        light.close_viewer()


def run_with_timestamped_log() -> int:
    """Run the demonstration while mirroring console output to a log."""

    started_at = datetime.now().astimezone()
    log_path = timestamped_log_path(started_at)
    with log_path.open("x", encoding="utf-8", buffering=1) as log_file:
        stdout_tee = TeeTextStream(sys.stdout, log_file)
        stderr_tee = TeeTextStream(sys.stderr, log_file)
        with redirect_stdout(stdout_tee), redirect_stderr(stderr_tee):
            print(f"Log file: {log_path}")
            print(f"Started: {started_at.isoformat(timespec='seconds')}")
            try:
                main()
            except Exception:
                traceback.print_exc()
                return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(run_with_timestamped_log())
