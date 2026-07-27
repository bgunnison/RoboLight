r"""Find the red camera target and center the RoboLight spotlight on it.

The acquisition loop demonstrates a first camera-guided behavior using the
public RoboLight API. Each cycle:

1. physically resets every encoded mechanism axis;
2. moves the red sphere to a random X/Y/Z position constrained by the room
   box and finite upper round table;
3. searches coarse arm/turntable poses and fine X/Y tilt views using
   ``get_camera()`` images;
4. detects the round red target and iteratively centers its image centroid.

The search code does not read the target's simulated position. Only the random
placement code knows those coordinates; acquisition uses camera pixels,
``get_position()``, and ``move()``.
Each attempt is numbered. Reported acquisition time starts after reset and
target placement, and the search gives up when it reaches 60 wall-clock
seconds. Console output and errors are also saved to a timestamped file under
``scripts/logs``.

Run the visible continuous demonstration from the repository root::

    .\.venv\Scripts\python.exe .\scripts\acquire_target.py

Close the main viewer or press Ctrl+C to stop. A deterministic finite headless
run is also available for build and regression testing::

    .\.venv\Scripts\python.exe .\scripts\acquire_target.py --headless --cycles 5 --seed 0
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
from typing import TextIO

import numpy as np


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
    beam_angle_degrees=20.0,
    camera_fov_degrees=50.0,
)

TARGET_DIAMETER_CM = 2.0
TARGET_SURFACE_CLEARANCE_M = 0.002

OUTPUT_VELOCITY_DEG_S = 45.0
SCAN_TILT_DEGREES = 20.0
MAX_CORRECTION_DEGREES = 12.0
CENTER_TOLERANCE_PIXELS = 3.0
MAX_CENTERING_IMAGES = 8
MAX_ACQUISITION_SECONDS = 60.0
MAX_DEADLINE_MOVE_CHUNK_DEGREES = 10.0
DEFAULT_VISIBLE_PAUSE_SECONDS = 2.0


class TeeTextStream:
    """Write text to the console and the current run's log file."""

    def __init__(self, console: TextIO, log_file: TextIO) -> None:
        self.console = console
        self.log_file = log_file

    @property
    def encoding(self) -> str | None:
        """Expose the console encoding expected by some print clients."""

        return self.console.encoding

    def write(self, text: str) -> int:
        """Write identical text to both destinations."""

        self.console.write(text)
        self.log_file.write(text)
        return len(text)

    def flush(self) -> None:
        """Flush both destinations so a live run's log stays current."""

        self.console.flush()
        self.log_file.flush()

    def isatty(self) -> bool:
        """Preserve console terminal detection while output is mirrored."""

        return self.console.isatty()


# At each coarse arm/turntable orientation, check the optical axis plus four
# overlapping tilt views. Coarse poses are spaced closely enough that this
# cross raster covers the gaps without an expensive full tilt grid.
TILT_SCAN_POSITIONS = (
    (0.0, 0.0),
    (-SCAN_TILT_DEGREES, 0.0),
    (0.0, SCAN_TILT_DEGREES),
    (SCAN_TILT_DEGREES, 0.0),
    (0.0, -SCAN_TILT_DEGREES),
)


@dataclass(frozen=True, slots=True)
class SearchPose:
    """One coarse camera orientation before the fine X/Y tilt raster."""

    label: str
    arm1_degrees: float
    arm2_degrees: float
    turntable_degrees: float


# Reset looks upward. Arm 2 and turntable grids cover the sides of the room;
# their spacing overlaps the 50-degree camera field of view. The final poses
# place the camera outside the upper-table edge and look down into the lower
# portion of the room without crossing the table plane.
SIDE_SEARCH_POSES = tuple(
    SearchPose(
        f"side Arm2={arm2_degrees:+.0f} Turntable={turntable_degrees:+.0f}",
        0.0,
        arm2_degrees,
        turntable_degrees,
    )
    for arm2_degrees in (30.0, -30.0, 60.0, -60.0, 90.0, -90.0)
    for turntable_degrees in (0.0, 45.0, -45.0, 90.0, -90.0)
)
DOWN_SEARCH_POSES = (
    SearchPose("down +Y", 40.0, 90.0, 90.0),
    SearchPose("down +X +Y", 40.0, 90.0, 45.0),
    SearchPose("down +X", 40.0, 90.0, 0.0),
    SearchPose("down +X -Y", 40.0, 90.0, -45.0),
    SearchPose("down -Y", 40.0, 90.0, -90.0),
    SearchPose("down +Y mirror", -40.0, -90.0, -90.0),
    SearchPose("down -X +Y", -40.0, -90.0, -45.0),
    SearchPose("down -X", -40.0, -90.0, 0.0),
    SearchPose("down -X -Y", -40.0, -90.0, 45.0),
    SearchPose("down -Y mirror", -40.0, -90.0, 90.0),
)
NEAR_TABLE_SEARCH_POSES = tuple(
    SearchPose(
        f"near table Arm1={arm1_degrees:+.0f} "
        f"Arm2={arm2_degrees:+.0f} "
        f"Turntable={turntable_degrees:+.0f}",
        arm1_degrees,
        arm2_degrees,
        turntable_degrees,
    )
    for arm1_degrees, arm2_degrees in ((-40.0, 90.0), (40.0, -90.0))
    for turntable_degrees in (0.0, 45.0, -45.0, 90.0, -90.0)
)
LOW_SEARCH_POSES = tuple(
    SearchPose(
        f"low Arm1={arm1_degrees:+.0f} "
        f"Arm2={arm2_degrees:+.0f} "
        f"Turntable={turntable_degrees:+.0f}",
        arm1_degrees,
        arm2_degrees,
        turntable_degrees,
    )
    for arm1_degrees, arm2_degrees in (
        (-60.0, 150.0),
        (-40.0, 150.0),
        (-20.0, 120.0),
        (0.0, 120.0),
        (60.0, -150.0),
        (40.0, -150.0),
        (20.0, -120.0),
        (0.0, -120.0),
    )
    for turntable_degrees in (0.0, 45.0, -45.0, 90.0, -90.0)
)
SEARCH_POSES = (
    SearchPose("up", 0.0, 0.0, 0.0),
) + (
    SIDE_SEARCH_POSES
    + DOWN_SEARCH_POSES
    + NEAR_TABLE_SEARCH_POSES
    + LOW_SEARCH_POSES
)


@dataclass(frozen=True, slots=True)
class TargetDetection:
    """Image-space location and size of one round red target candidate."""

    x_pixels: float
    y_pixels: float
    pixel_count: int


@dataclass(frozen=True, slots=True)
class AcquisitionResult:
    """Successful final acquisition state."""

    detection: TargetDetection
    images_acquired: int
    x_tilt_degrees: float
    y_tilt_degrees: float


@dataclass(frozen=True, slots=True)
class AcquisitionOutcome:
    """Result and acquisition-only wall-clock timing for one search."""

    result: AcquisitionResult | None
    elapsed_seconds: float
    timed_out: bool


def parse_args() -> argparse.Namespace:
    """Parse visible-demo and deterministic headless-test options."""

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--headless",
        action="store_true",
        help="do not open windows or pace mechanism moves in wall-clock time",
    )
    parser.add_argument(
        "--cycles",
        type=int,
        default=None,
        help="number of numbered target searches (default: loop visibly)",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="optional random seed for repeatable target positions",
    )
    parser.add_argument(
        "--pause",
        type=float,
        default=DEFAULT_VISIBLE_PAUSE_SECONDS,
        help="seconds to display each centered result (default: 2)",
    )
    args = parser.parse_args()
    if args.cycles is not None and args.cycles < 1:
        parser.error("--cycles must be at least 1")
    if not math.isfinite(args.pause) or args.pause < 0.0:
        parser.error("--pause must be zero or greater")
    if args.headless and args.cycles is None:
        args.cycles = 5
    return args


def find_red_target(image: np.ndarray) -> TargetDetection | None:
    """Return the most likely round red sphere in an RGB camera image.

    A high-saturation deep-red threshold creates candidate pixels.
    Four-connected components are then filtered for the compact, approximately
    square, well-filled silhouette of a projected sphere. This prevents the
    warm spotlight footprint and elongated red mechanism geometry from being
    mistaken for the target at some search angles.
    """

    pixels = image.astype(np.int16)
    red_mask = (
        (pixels[:, :, 0] > 90)
        & (pixels[:, :, 1] * 4 < pixels[:, :, 0])
        & (pixels[:, :, 2] * 4 < pixels[:, :, 0])
    )
    height, width = red_mask.shape
    visited = np.zeros_like(red_mask)
    candidates: list[TargetDetection] = []

    for initial_y, initial_x in np.argwhere(red_mask):
        y = int(initial_y)
        x = int(initial_x)
        if visited[y, x]:
            continue

        visited[y, x] = True
        pending = [(y, x)]
        component_x: list[int] = []
        component_y: list[int] = []
        while pending:
            pixel_y, pixel_x = pending.pop()
            component_x.append(pixel_x)
            component_y.append(pixel_y)
            for delta_y, delta_x in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                neighbor_y = pixel_y + delta_y
                neighbor_x = pixel_x + delta_x
                if not (
                    0 <= neighbor_y < height
                    and 0 <= neighbor_x < width
                    and red_mask[neighbor_y, neighbor_x]
                    and not visited[neighbor_y, neighbor_x]
                ):
                    continue
                visited[neighbor_y, neighbor_x] = True
                pending.append((neighbor_y, neighbor_x))

        pixel_count = len(component_x)
        component_width = max(component_x) - min(component_x) + 1
        component_height = max(component_y) - min(component_y) + 1
        aspect_ratio = component_width / component_height
        fill_ratio = pixel_count / (component_width * component_height)
        if (
            pixel_count >= 20
            and 0.65 <= aspect_ratio <= 1.50
            and fill_ratio >= 0.55
        ):
            candidates.append(
                TargetDetection(
                    x_pixels=sum(component_x) / pixel_count,
                    y_pixels=sum(component_y) / pixel_count,
                    pixel_count=pixel_count,
                )
            )

    if not candidates:
        return None
    return max(candidates, key=lambda candidate: candidate.pixel_count)


def require_search_time(deadline: float | None) -> None:
    """Raise when a camera search has reached its wall-clock deadline."""

    if deadline is not None and time.perf_counter() >= deadline:
        raise TimeoutError("target acquisition deadline reached")


def move_output_to(
    light: RoboLight,
    selector: Selector,
    target_degrees: float,
    deadline: float | None = None,
) -> None:
    """Move one selected output to an absolute angle through the public API."""

    require_search_time(deadline)
    current_degrees = light.get_position(selector)
    delta_degrees = target_degrees - current_degrees
    if abs(delta_degrees) < 1e-9:
        return
    while abs(delta_degrees) >= 1e-9:
        if deadline is None:
            move_degrees = delta_degrees
        else:
            move_degrees = math.copysign(
                min(
                    abs(delta_degrees),
                    MAX_DEADLINE_MOVE_CHUNK_DEGREES,
                ),
                delta_degrees,
            )
        result = light.move(
            selector,
            velocity=OUTPUT_VELOCITY_DEG_S,
            degrees=move_degrees,
        )
        if result is not MoveError.OK:
            raise RuntimeError(
                f"{selector.value} move to {target_degrees:.1f} deg rejected: "
                f"{result.value}"
            )
        require_search_time(deadline)
        delta_degrees -= move_degrees


def move_to_tilt(
    light: RoboLight,
    x_degrees: float,
    y_degrees: float,
    deadline: float | None = None,
) -> None:
    """Move X and Y tilt, one selected output at a time."""

    move_output_to(light, Selector.X_TILT, x_degrees, deadline)
    move_output_to(light, Selector.Y_TILT, y_degrees, deadline)


def move_to_search_pose(
    light: RoboLight,
    pose: SearchPose,
    deadline: float | None = None,
) -> None:
    """Move safely to one coarse arm/turntable search orientation."""

    move_to_tilt(light, 0.0, 0.0, deadline)
    current_arm1 = light.get_position(Selector.ARM1)
    if abs(current_arm1 - pose.arm1_degrees) > 1e-9:
        # Arm 1 is guaranteed its full configured travel when Arm 2 is reset.
        move_output_to(light, Selector.ARM2, 0.0, deadline)
        move_output_to(light, Selector.ARM1, pose.arm1_degrees, deadline)
    move_output_to(light, Selector.ARM2, pose.arm2_degrees, deadline)
    move_output_to(
        light,
        Selector.TURNTABLE,
        pose.turntable_degrees,
        deadline,
    )


def probe_image_axis(
    light: RoboLight,
    selector: Selector,
    baseline: TargetDetection,
    deadline: float,
) -> tuple[np.ndarray | None, int]:
    """Measure one column of the local pixel-per-degree image Jacobian."""

    baseline_angle = light.get_position(selector)
    images_acquired = 0
    for probe_degrees in (1.0, -1.0):
        require_search_time(deadline)
        probe_angle = baseline_angle + probe_degrees
        if not -45.0 <= probe_angle <= 45.0:
            continue
        move_output_to(light, selector, probe_angle, deadline)
        probe_image = light.get_camera()
        require_search_time(deadline)
        images_acquired += 1
        probe_detection = find_red_target(probe_image)
        move_output_to(light, selector, baseline_angle, deadline)
        if probe_detection is None:
            continue
        return (
            np.array(
                (
                    (probe_detection.x_pixels - baseline.x_pixels)
                    / probe_degrees,
                    (probe_detection.y_pixels - baseline.y_pixels)
                    / probe_degrees,
                )
            ),
            images_acquired,
        )
    return None, images_acquired


def measure_image_jacobian(
    light: RoboLight,
    baseline: TargetDetection,
    deadline: float,
) -> tuple[np.ndarray | None, int]:
    """Measure how local X/Y tilt changes the detected camera centroid."""

    x_column, x_images = probe_image_axis(
        light,
        Selector.X_TILT,
        baseline,
        deadline,
    )
    y_column, y_images = probe_image_axis(
        light,
        Selector.Y_TILT,
        baseline,
        deadline,
    )
    if x_column is None or y_column is None:
        return None, x_images + y_images
    return np.column_stack((x_column, y_column)), x_images + y_images


def center_visible_target(
    light: RoboLight,
    image: np.ndarray,
    detection: TargetDetection,
    images_acquired: int,
    deadline: float,
) -> tuple[AcquisitionResult | None, int]:
    """Iteratively center an already visible target."""

    for correction_number in range(MAX_CENTERING_IMAGES + 1):
        require_search_time(deadline)
        height, width = image.shape[:2]
        center_x = (width - 1) / 2.0
        center_y = (height - 1) / 2.0
        pixel_error = math.hypot(
            detection.x_pixels - center_x,
            detection.y_pixels - center_y,
        )
        if pixel_error <= CENTER_TOLERANCE_PIXELS:
            return (
                AcquisitionResult(
                    detection=detection,
                    images_acquired=images_acquired,
                    x_tilt_degrees=light.get_position(Selector.X_TILT),
                    y_tilt_degrees=light.get_position(Selector.Y_TILT),
                ),
                images_acquired,
            )
        if correction_number == MAX_CENTERING_IMAGES:
            return None, images_acquired

        image_jacobian, probe_images = measure_image_jacobian(
            light,
            detection,
            deadline,
        )
        images_acquired += probe_images
        if image_jacobian is None:
            return None, images_acquired
        pixel_error_vector = np.array(
            (
                center_x - detection.x_pixels,
                center_y - detection.y_pixels,
            )
        )
        joint_correction = np.linalg.lstsq(
            image_jacobian,
            pixel_error_vector,
            rcond=None,
        )[0]
        largest_correction = float(np.max(np.abs(joint_correction)))
        if largest_correction > MAX_CORRECTION_DEGREES:
            joint_correction *= MAX_CORRECTION_DEGREES / largest_correction

        current_x = light.get_position(Selector.X_TILT)
        current_y = light.get_position(Selector.Y_TILT)
        target_x = max(
            -45.0,
            min(45.0, current_x + float(joint_correction[0])),
        )
        target_y = max(
            -45.0,
            min(45.0, current_y + float(joint_correction[1])),
        )
        if (
            abs(target_x - current_x) < 1e-9
            and abs(target_y - current_y) < 1e-9
        ):
            return None, images_acquired
        move_to_tilt(
            light,
            target_x,
            target_y,
            deadline,
        )

        image = light.get_camera()
        require_search_time(deadline)
        images_acquired += 1
        detection = find_red_target(image)
        if detection is None:
            return None, images_acquired

    return None, images_acquired


def acquire_target(
    light: RoboLight,
    timeout_seconds: float = MAX_ACQUISITION_SECONDS,
) -> AcquisitionOutcome:
    """Search and center for at most ``timeout_seconds`` of wall-clock time."""

    if not math.isfinite(timeout_seconds) or timeout_seconds <= 0.0:
        raise ValueError("timeout_seconds must be above zero")
    started = time.perf_counter()
    deadline = started + timeout_seconds
    images_acquired = 0
    try:
        for pose in SEARCH_POSES:
            move_to_search_pose(light, pose, deadline)
            print(f"  searching {pose.label}")
            for x_degrees, y_degrees in TILT_SCAN_POSITIONS:
                move_to_tilt(light, x_degrees, y_degrees, deadline)
                image = light.get_camera()
                require_search_time(deadline)
                images_acquired += 1
                detection = find_red_target(image)
                if detection is None:
                    continue
                print(
                    f"  target detected at pixel "
                    f"({detection.x_pixels:.1f}, {detection.y_pixels:.1f})"
                )
                result, images_acquired = center_visible_target(
                    light,
                    image,
                    detection,
                    images_acquired,
                    deadline,
                )
                if result is not None:
                    return AcquisitionOutcome(
                        result=result,
                        elapsed_seconds=time.perf_counter() - started,
                        timed_out=False,
                    )
                print(
                    "  target lost while centering; resuming raster search"
                )
    except TimeoutError:
        return AcquisitionOutcome(
            result=None,
            elapsed_seconds=time.perf_counter() - started,
            timed_out=True,
        )
    return AcquisitionOutcome(
        result=None,
        elapsed_seconds=time.perf_counter() - started,
        timed_out=False,
    )


def random_target_position(
    light: RoboLight,
    generator: random.Random,
) -> tuple[float, float, float]:
    """Choose a sphere center inside the room and outside the upper table.

    Bounds come from the actual named MuJoCo geoms rather than duplicated
    numeric ranges. The target radius and a small visual clearance keep the
    sphere from intersecting a room surface or the finite round upper
    turntable.
    """

    base = light.model.geom("base")
    left_wall = light.model.geom("left_wall")
    right_wall = light.model.geom("right_wall")
    back_wall = light.model.geom("back_wall")
    ceiling = light.model.geom("ceiling")
    upper_table = light.model.geom("upper_turntable_disk")
    target_origin = light.model.body("target_origin_frame")
    radius_m = TARGET_DIAMETER_CM / 200.0
    margin_m = radius_m + TARGET_SURFACE_CLEARANCE_M

    minimum_world_x = (
        light.data.geom_xpos[left_wall.id, 0] + left_wall.size[0] + margin_m
    )
    maximum_world_x = (
        light.data.geom_xpos[right_wall.id, 0] - right_wall.size[0] - margin_m
    )
    minimum_world_y = (
        light.data.geom_xpos[base.id, 1] - base.size[1] + margin_m
    )
    maximum_world_y = (
        light.data.geom_xpos[back_wall.id, 1] - back_wall.size[1] - margin_m
    )
    minimum_world_z = (
        light.data.geom_xpos[base.id, 2] + base.size[2] + margin_m
    )
    maximum_world_z = (
        light.data.geom_xpos[ceiling.id, 2] - ceiling.size[2] - margin_m
    )

    table_center = light.data.geom_xpos[upper_table.id]
    table_radius = upper_table.size[0]
    table_half_height = upper_table.size[1]
    world_position = None
    for _ in range(10_000):
        candidate = np.array(
            (
                generator.uniform(minimum_world_x, maximum_world_x),
                generator.uniform(minimum_world_y, maximum_world_y),
                generator.uniform(minimum_world_z, maximum_world_z),
            )
        )
        radial_distance = math.hypot(
            candidate[0] - table_center[0],
            candidate[1] - table_center[1],
        )
        radial_gap = max(0.0, radial_distance - table_radius)
        vertical_gap = max(
            0.0,
            abs(candidate[2] - table_center[2]) - table_half_height,
        )
        if math.hypot(radial_gap, vertical_gap) >= margin_m:
            world_position = candidate
            break
    if world_position is None:
        raise RuntimeError("could not place a target outside the upper table")

    local_position_cm = (
        world_position - light.data.xpos[target_origin.id]
    ) * 100.0
    return tuple(float(value) for value in local_position_cm)


def pause_while_visible(light: RoboLight, seconds: float) -> None:
    """Keep the viewer and PIP responsive while displaying an acquisition."""

    deadline = time.perf_counter() + seconds
    while light.viewer_is_running and time.perf_counter() < deadline:
        light.sync_visuals()
        time.sleep(min(0.05, max(0.0, deadline - time.perf_counter())))


def timestamped_log_path(started_at: datetime) -> Path:
    """Create the log directory and return a unique timestamped log path."""

    log_directory = Path(__file__).resolve().parent / "logs"
    log_directory.mkdir(parents=True, exist_ok=True)
    timestamp = started_at.strftime("%Y%m%d_%H%M%S_%f")
    return log_directory / f"acquire_target_{timestamp}.log"


def main() -> None:
    """Reset, randomize, and reacquire the target until the run ends."""

    args = parse_args()
    generator = random.Random(args.seed)
    light = RoboLight(DEMO_HARDWARE, realtime=not args.headless)
    try:
        print(f"HWDesc: {DEMO_HARDWARE}")
        if not args.headless:
            light.open_viewer()
            light.open_pip()
            print("Close the main viewer or press Ctrl+C to stop.")

        search_number = 0
        failed_searches = 0
        while args.cycles is None or search_number < args.cycles:
            if not args.headless and not light.viewer_is_running:
                break
            search_number += 1
            print(f"Search {search_number}: physical reset")
            light.reset()

            target_x, target_y, target_z = random_target_position(
                light,
                generator,
            )
            light.set_target(
                target_x,
                target_y,
                target_z,
                color="red",
                diameter_cm=TARGET_DIAMETER_CM,
            )
            print(
                f"Search {search_number}: target moved to "
                f"X={target_x:.1f}, Y={target_y:.1f}, "
                f"Z={target_z:.1f} cm"
            )

            outcome = acquire_target(
                light,
                timeout_seconds=MAX_ACQUISITION_SECONDS,
            )
            if outcome.timed_out:
                failed_searches += 1
                print(
                    f"Search {search_number}: gave up after "
                    f"{outcome.elapsed_seconds:.1f} sec "
                    f"({MAX_ACQUISITION_SECONDS:.0f} sec limit; "
                    "acquisition only); target "
                    f"X={target_x:.1f}, Y={target_y:.1f}, "
                    f"Z={target_z:.1f} cm"
                )
                continue
            result = outcome.result
            if result is None:
                failed_searches += 1
                print(
                    f"Search {search_number}: target not found in "
                    f"{outcome.elapsed_seconds:.1f} sec "
                    "(acquisition only)"
                )
                continue

            image_height = 240
            image_width = 320
            center_x = (image_width - 1) / 2.0
            center_y = (image_height - 1) / 2.0
            final_error = math.hypot(
                result.detection.x_pixels - center_x,
                result.detection.y_pixels - center_y,
            )
            print(
                f"Search {search_number}: acquired target in "
                f"{outcome.elapsed_seconds:.1f} sec "
                "(acquisition only; reset and target move excluded)"
            )
            print(
                f"  centered in {result.images_acquired} images: "
                f"pixel error={final_error:.1f}, "
                f"X tilt={result.x_tilt_degrees:.1f} deg, "
                f"Y tilt={result.y_tilt_degrees:.1f} deg"
            )
            if not args.headless:
                pause_while_visible(light, args.pause)

        if args.headless:
            if failed_searches:
                raise RuntimeError(
                    f"{failed_searches} of {search_number} target searches "
                    "did not acquire"
                )
            print(f"Target acquisition passed for {search_number} searches")
    except KeyboardInterrupt:
        print("\nTarget acquisition stopped by user")
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
