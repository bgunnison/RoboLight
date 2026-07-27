r"""Find the red camera target and center the RoboLight spotlight on it.

The acquisition loop demonstrates a first camera-guided behavior using the
public RoboLight API. Each cycle:

1. physically resets every encoded mechanism axis;
2. moves the red sphere to a random X/Y/Z position constrained by the room
   box and the upper turntable's blocked footprint;
3. builds a line-of-sight coverage plan, then searches optimized coarse
   arm/turntable poses and fast X/Y tilt views using ``get_camera()`` images;
4. detects the round red target and iteratively centers its image centroid;
5. uses closed-loop image-space inverse kinematics to shift the pointing angle
   into the arms and turntable while returning X/Y plate tilt toward zero.

The search code does not read the target's simulated position. Only the random
placement code knows those coordinates; acquisition uses camera pixels,
``get_position()``, and ``move()``.
Each attempt is numbered. Reported acquisition time starts after reset and
target placement, and the search gives up when it reaches 60 wall-clock
seconds. Console output and errors are also saved to a timestamped file under
``scripts/logs``. If ``GAVE_UP_TARGET_LOG`` names an earlier run log, every
timed-out target location in that file is plotted as a green sphere in the
main viewer.

Run the visible continuous demonstration from the repository root::

    .\.venv\Scripts\python.exe .\scripts\acquire_target.py

Close the main viewer or press Ctrl+C to stop. A deterministic finite headless
run is also available for build and regression testing::

    .\.venv\Scripts\python.exe .\scripts\acquire_target.py --headless --cycles 5 --seed 0
"""

from __future__ import annotations

import argparse
from contextlib import redirect_stderr, redirect_stdout
from dataclasses import dataclass, replace
from datetime import datetime
import math
from pathlib import Path
import random
import re
import sys
import time
import traceback
from typing import TextIO

import mujoco
import numpy as np


# Direct execution puts scripts/ rather than the repository root on sys.path.
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scripts import HWDesc, MoveError, RoboLight, Selector


# Change this path to plot the timed-out target locations from a different log,
# or set it to None to disable the green historical-target overlay.
GAVE_UP_TARGET_LOG: Path | None = (
    Path(__file__).resolve().parent
    / "logs"
    / "acquire_target_20260727_151638_568526.log"
)
GAVE_UP_TARGET_LOG = None
GAVE_UP_TARGET_DIAMETER_CM = 2.0
GAVE_UP_TARGET_RGBA = (0.05, 0.85, 0.10, 0.80)
SIMULATION_TIME_MULTIPLIER = 10.0

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

COARSE_OUTPUT_VELOCITY_DEG_S = 45.0
TILT_OUTPUT_VELOCITY_DEG_S = 90.0
MAX_CORRECTION_DEGREES = 12.0
CENTER_TOLERANCE_PIXELS = 3.0
MAX_CENTERING_IMAGES = 8
REBALANCE_TILT_TOLERANCE_DEGREES = 1.0
REBALANCE_TILT_STEP_DEGREES = 8.0
REBALANCE_SLOW_STEP_DEGREES = 8.0
MAX_REBALANCE_ITERATIONS = 8
MAX_ACQUISITION_SECONDS = 60.0
MAX_DEADLINE_MOVE_CHUNK_DEGREES = 45.0
DEFAULT_VISIBLE_PAUSE_SECONDS = 2.0
TARGET_SPACE_SAMPLE_SPACING_M = 0.025
ESTIMATED_CAMERA_IMAGE_SECONDS = 0.08
MIN_NEW_TARGET_SAMPLE_FRACTION = 0.002
MIN_NEW_TARGET_SAMPLES_PER_ESTIMATED_SECOND = 10.0
MAX_TILT_VIEWS_PER_COARSE_POSE = 9
FIRST_MINUTE_PLANNING_SECONDS = 55.0
CAMERA_IMAGE_ASPECT_RATIO = 320.0 / 240.0


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


# Candidate tilt views form an overlapping 3x3 raster for the 50-degree camera
# FOV. The optimizer selects and orders only views that add new target-space
# coverage, so this tuple defines possibilities rather than a fixed scan.
TILT_SCAN_CANDIDATES = (
    (-20.0, -20.0),
    (0.0, -20.0),
    (20.0, -20.0),
    (20.0, 0.0),
    (0.0, 0.0),
    (-20.0, 0.0),
    (-20.0, 20.0),
    (0.0, 20.0),
    (20.0, 20.0),
)


GAVE_UP_TARGET_PATTERN = re.compile(
    r"\bgave up after\b.*?\btarget "
    r"X=(?P<x>[+-]?\d+(?:\.\d+)?), "
    r"Y=(?P<y>[+-]?\d+(?:\.\d+)?), "
    r"Z=(?P<z>[+-]?\d+(?:\.\d+)?) cm\b"
)


@dataclass(frozen=True, slots=True)
class SearchPose:
    """One coarse camera orientation before the fine X/Y tilt raster."""

    label: str
    arm1_degrees: float
    arm2_degrees: float
    turntable_degrees: float


@dataclass(frozen=True, slots=True)
class SearchView:
    """One optimized camera view at a coarse pose and fast tilt position."""

    pose: SearchPose
    x_tilt_degrees: float
    y_tilt_degrees: float
    newly_covered_samples: int
    estimated_elapsed_seconds: float


@dataclass(frozen=True, slots=True)
class SearchPlan:
    """Coverage-optimized camera views and their planning statistics."""

    views: tuple[SearchView, ...]
    sampled_target_positions: int
    covered_target_positions: int
    first_minute_covered_positions: int
    first_minute_view_count: int
    estimated_total_seconds: float

    @property
    def coverage_percent(self) -> float:
        """Percentage of sampled allowed target space covered by the plan."""

        if self.sampled_target_positions == 0:
            return 0.0
        return 100.0 * self.covered_target_positions / self.sampled_target_positions

    @property
    def first_minute_coverage_percent(self) -> float:
        """Coverage planned before reserving five seconds for centering."""

        if self.sampled_target_positions == 0:
            return 0.0
        return (
            100.0
            * self.first_minute_covered_positions
            / self.sampled_target_positions
        )

    @property
    def coarse_pose_count(self) -> int:
        """Number of distinct slow arm/turntable poses in the plan."""

        return len({view.pose for view in self.views})


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
            velocity=(
                TILT_OUTPUT_VELOCITY_DEG_S
                if selector in (Selector.X_TILT, Selector.Y_TILT)
                else COARSE_OUTPUT_VELOCITY_DEG_S
            ),
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


def probe_slow_image_axis(
    light: RoboLight,
    selector: Selector,
    baseline: TargetDetection,
    deadline: float,
) -> tuple[np.ndarray | None, int]:
    """Measure one slow joint's local pixel-per-degree image response."""

    images_acquired = 0
    for probe_degrees in (1.0, -1.0):
        require_search_time(deadline)
        move_result = light.move(
            selector,
            velocity=COARSE_OUTPUT_VELOCITY_DEG_S,
            degrees=probe_degrees,
        )
        if move_result is not MoveError.OK:
            continue
        try:
            probe_image = light.get_camera()
            images_acquired += 1
            probe_detection = find_red_target(probe_image)
        finally:
            restore_result = light.move(
                selector,
                velocity=COARSE_OUTPUT_VELOCITY_DEG_S,
                degrees=-probe_degrees,
            )
            if restore_result is not MoveError.OK:
                raise RuntimeError(
                    f"could not restore {selector.value} after visual probe: "
                    f"{restore_result.value}"
                )
        require_search_time(deadline)
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


def restore_slow_moves(
    light: RoboLight,
    completed_moves: list[tuple[Selector, float]],
) -> None:
    """Reverse completed slow-axis moves after an unsuccessful rebalance."""

    for selector, degrees in reversed(completed_moves):
        result = light.move(
            selector,
            velocity=COARSE_OUTPUT_VELOCITY_DEG_S,
            degrees=-degrees,
        )
        if result is not MoveError.OK:
            raise RuntimeError(
                f"could not restore {selector.value} after visual rebalance: "
                f"{result.value}"
            )


def rebalance_centered_target(
    light: RoboLight,
    initial_result: AcquisitionResult,
    deadline: float,
) -> AcquisitionResult:
    """Unload X/Y tilt into the arms and turntable using camera feedback.

    This is differential image-space inverse kinematics. Small one-degree
    probes measure how Arm 1, Arm 2, the turntable, and both tilt axes move the
    detected target in the current image. A least-squares step moves the slow
    axes to cancel a simultaneous step toward zero plate tilt. The target is
    then re-detected and centered before another step, so changing parallax is
    corrected from pixels rather than an assumed target depth.
    """

    result = initial_result
    slow_selectors = (
        Selector.ARM1,
        Selector.ARM2,
        Selector.TURNTABLE,
    )
    print("  rebalancing plate tilt with closed-loop camera feedback")

    for iteration in range(1, MAX_REBALANCE_ITERATIONS + 1):
        require_search_time(deadline)
        current_x = light.get_position(Selector.X_TILT)
        current_y = light.get_position(Selector.Y_TILT)
        current_tilt_norm = math.hypot(current_x, current_y)
        if current_tilt_norm <= REBALANCE_TILT_TOLERANCE_DEGREES:
            break

        slow_columns: list[np.ndarray] = []
        usable_selectors: list[Selector] = []
        images_acquired = result.images_acquired
        for selector in slow_selectors:
            column, probe_images = probe_slow_image_axis(
                light,
                selector,
                result.detection,
                deadline,
            )
            images_acquired += probe_images
            if column is not None:
                usable_selectors.append(selector)
                slow_columns.append(column)

        if len(slow_columns) < 2:
            print("  visual rebalance stopped: fewer than two movable slow axes")
            break
        slow_jacobian = np.column_stack(slow_columns)
        if np.linalg.matrix_rank(slow_jacobian) < 2:
            print("  visual rebalance stopped: slow-axis image response is singular")
            break

        tilt_jacobian, probe_images = measure_image_jacobian(
            light,
            result.detection,
            deadline,
        )
        images_acquired += probe_images
        if tilt_jacobian is None:
            print("  visual rebalance stopped: tilt response could not be measured")
            break

        tilt_scale = min(
            1.0,
            REBALANCE_TILT_STEP_DEGREES
            / max(abs(current_x), abs(current_y)),
        )
        requested_tilt_delta = (
            -np.array((current_x, current_y)) * tilt_scale
        )
        active_indices = list(range(len(usable_selectors)))
        completed_moves: list[tuple[Selector, float]] | None = None
        tilt_delta = requested_tilt_delta
        while len(active_indices) >= 2:
            active_jacobian = slow_jacobian[:, active_indices]
            if np.linalg.matrix_rank(active_jacobian) < 2:
                break
            active_slow_delta = np.linalg.lstsq(
                active_jacobian,
                -(tilt_jacobian @ requested_tilt_delta),
                rcond=None,
            )[0]
            tilt_delta = requested_tilt_delta.copy()
            largest_slow_delta = float(np.max(np.abs(active_slow_delta)))
            if largest_slow_delta > REBALANCE_SLOW_STEP_DEGREES:
                step_scale = (
                    REBALANCE_SLOW_STEP_DEGREES / largest_slow_delta
                )
                active_slow_delta *= step_scale
                tilt_delta *= step_scale

            attempted_moves: list[tuple[Selector, float]] = []
            rejected_index: int | None = None
            rejected_result: MoveError | None = None
            for index, degrees in zip(active_indices, active_slow_delta):
                selector = usable_selectors[index]
                move_degrees = float(degrees)
                move_result = light.move(
                    selector,
                    velocity=COARSE_OUTPUT_VELOCITY_DEG_S,
                    degrees=move_degrees,
                )
                if move_result is not MoveError.OK:
                    restore_slow_moves(light, attempted_moves)
                    rejected_index = index
                    rejected_result = move_result
                    break
                attempted_moves.append((selector, move_degrees))

            if rejected_index is None:
                completed_moves = attempted_moves
                break

            rejected_selector = usable_selectors[rejected_index]
            active_indices.remove(rejected_index)
            print(
                f"  {rejected_selector.value} constrained "
                f"({rejected_result.value}); retrying visual rebalance "
                "without it"
            )

        if completed_moves is None:
            print(
                "  visual rebalance stopped: the remaining slow axes cannot "
                "span the image correction"
            )
            return replace(result, images_acquired=images_acquired)

        if not completed_moves:
            print("  visual rebalance stopped: no slow-axis correction remains")
            return replace(result, images_acquired=images_acquired)

        for selector, move_degrees in completed_moves:
            if not math.isfinite(move_degrees):
                restore_slow_moves(light, completed_moves)
                print(
                    "  visual rebalance stopped: non-finite slow-axis "
                    "correction"
                )
                return replace(
                    result,
                    images_acquired=images_acquired,
                )

        move_to_tilt(
            light,
            current_x + float(tilt_delta[0]),
            current_y + float(tilt_delta[1]),
            deadline,
        )
        image = light.get_camera()
        require_search_time(deadline)
        images_acquired += 1
        detection = find_red_target(image)
        if detection is None:
            move_to_tilt(light, current_x, current_y)
            restore_slow_moves(light, completed_moves)
            print("  visual rebalance stopped before the target left view")
            return replace(result, images_acquired=images_acquired)

        centered_result, images_acquired = center_visible_target(
            light,
            image,
            detection,
            images_acquired,
            deadline,
        )
        if centered_result is None:
            move_to_tilt(light, current_x, current_y)
            restore_slow_moves(light, completed_moves)
            print("  visual rebalance stopped because centering did not converge")
            return replace(result, images_acquired=images_acquired)

        new_tilt_norm = math.hypot(
            centered_result.x_tilt_degrees,
            centered_result.y_tilt_degrees,
        )
        if new_tilt_norm >= current_tilt_norm - 0.05:
            move_to_tilt(light, current_x, current_y)
            restore_slow_moves(light, completed_moves)
            print("  visual rebalance stopped at the best measured plate tilt")
            return replace(result, images_acquired=images_acquired)

        result = replace(
            centered_result,
            images_acquired=images_acquired,
        )
        print(
            f"  visual rebalance {iteration}: "
            f"X tilt={result.x_tilt_degrees:.1f} deg, "
            f"Y tilt={result.y_tilt_degrees:.1f} deg"
        )

    return result


def target_space_samples(light: RoboLight) -> np.ndarray:
    """Sample valid target centers for camera-view coverage planning.

    The upper table blocks the entire column below its top surface. A target
    below that surface is valid only when its complete sphere, including
    clearance, is outside the table radius.
    """

    base = light.model.geom("base")
    left_wall = light.model.geom("left_wall")
    right_wall = light.model.geom("right_wall")
    back_wall = light.model.geom("back_wall")
    ceiling = light.model.geom("ceiling")
    upper_table = light.model.geom("upper_turntable_disk")
    radius_m = TARGET_DIAMETER_CM / 200.0
    margin_m = radius_m + TARGET_SURFACE_CLEARANCE_M

    bounds = (
        light.data.geom_xpos[left_wall.id, 0] + left_wall.size[0] + margin_m,
        light.data.geom_xpos[right_wall.id, 0] - right_wall.size[0] - margin_m,
        light.data.geom_xpos[base.id, 1] - base.size[1] + margin_m,
        light.data.geom_xpos[back_wall.id, 1] - back_wall.size[1] - margin_m,
        light.data.geom_xpos[base.id, 2] + base.size[2] + margin_m,
        light.data.geom_xpos[ceiling.id, 2] - ceiling.size[2] - margin_m,
    )

    def sample_axis(minimum: float, maximum: float) -> np.ndarray:
        interval_count = max(
            1,
            math.ceil(
                (maximum - minimum) / TARGET_SPACE_SAMPLE_SPACING_M
            ),
        )
        return np.linspace(minimum, maximum, interval_count + 1)

    x_values = sample_axis(bounds[0], bounds[1])
    y_values = sample_axis(bounds[2], bounds[3])
    z_values = sample_axis(bounds[4], bounds[5])
    samples = np.array(
        np.meshgrid(x_values, y_values, z_values, indexing="ij")
    ).reshape(3, -1).T

    table_center = light.data.geom_xpos[upper_table.id]
    safe_table_radius = upper_table.size[0] + margin_m
    safe_table_top = table_center[2] + upper_table.size[1] + margin_m
    radial_distance = np.hypot(
        samples[:, 0] - table_center[0],
        samples[:, 1] - table_center[1],
    )
    allowed = (
        (samples[:, 2] >= safe_table_top)
        | (radial_distance >= safe_table_radius)
    )
    return samples[allowed]


def set_planning_camera_pose(
    planner: RoboLight,
    pose: SearchPose,
    x_tilt_degrees: float,
    y_tilt_degrees: float,
) -> None:
    """Set a disposable planner model directly without simulating motion."""

    joint_degrees = {
        "turntable_yaw": pose.turntable_degrees,
        "tg2_hinge": pose.arm1_degrees,
        "tg4_hinge": pose.arm2_degrees,
        "tg6_hinge": pose.arm2_degrees,
        "plate_x_tilt": x_tilt_degrees,
        "plate_y_tilt": y_tilt_degrees,
    }
    for joint_name, degrees in joint_degrees.items():
        joint = planner.model.joint(joint_name)
        qpos_address = int(planner.model.jnt_qposadr[joint.id])
        planner.data.qpos[qpos_address] = math.radians(degrees)
    mujoco.mj_forward(planner.model, planner.data)


def candidate_view_coverage(
    planner: RoboLight,
    target_samples: np.ndarray,
) -> np.ndarray:
    """Return line-of-sight sample coverage for every candidate camera view."""

    coverage = np.zeros(
        (
            len(SEARCH_POSES),
            len(TILT_SCAN_CANDIDATES),
            len(target_samples),
        ),
        dtype=bool,
    )
    camera = planner.model.camera("spotlight_camera")
    camera_body = planner.model.body("tilt_plate_body")
    target_geom = planner.model.geom("camera_target_sphere")

    # The planner's one red target is unrelated to the sampled target centers.
    # Put it in an excluded ray group so it cannot shadow a candidate sample.
    planner.model.geom_group[target_geom.id] = 5
    ray_geom_groups = np.ones(6, dtype=np.uint8)
    ray_geom_groups[5] = 0
    vertical_tangent = math.tan(
        math.radians(float(planner.model.cam_fovy[camera.id])) / 2.0
    )
    horizontal_tangent = (
        vertical_tangent * CAMERA_IMAGE_ASPECT_RATIO
    )
    target_margin_m = (
        TARGET_DIAMETER_CM / 200.0 + TARGET_SURFACE_CLEARANCE_M
    )

    for pose_index, pose in enumerate(SEARCH_POSES):
        for tilt_index, (x_tilt, y_tilt) in enumerate(
            TILT_SCAN_CANDIDATES
        ):
            set_planning_camera_pose(
                planner,
                pose,
                x_tilt,
                y_tilt,
            )
            camera_position = planner.data.cam_xpos[camera.id].copy()
            camera_rotation = planner.data.cam_xmat[camera.id].reshape(3, 3)
            relative_world = target_samples - camera_position
            relative_camera = relative_world @ camera_rotation
            depth = -relative_camera[:, 2]
            in_frustum = (
                (depth > 0.0)
                & (
                    np.abs(relative_camera[:, 0])
                    <= depth * horizontal_tangent
                )
                & (
                    np.abs(relative_camera[:, 1])
                    <= depth * vertical_tangent
                )
            )
            sample_indices = np.flatnonzero(in_frustum)
            if len(sample_indices) == 0:
                continue

            ray_vectors = relative_world[sample_indices].copy()
            target_distances = np.linalg.norm(ray_vectors, axis=1)
            ray_vectors /= target_distances[:, None]
            hit_geom_ids = np.empty(len(sample_indices), dtype=np.int32)
            hit_distances = np.empty(len(sample_indices), dtype=np.float64)
            multi_ray_arguments = {
                "m": planner.model,
                "d": planner.data,
                "pnt": camera_position,
                "vec": ray_vectors.reshape(-1),
                "geomgroup": ray_geom_groups,
                "flg_static": 1,
                "bodyexclude": camera_body.id,
                "geomid": hit_geom_ids,
                "dist": hit_distances,
                "nray": len(sample_indices),
                "cutoff": float(np.max(target_distances)),
            }
            try:
                # MuJoCo releases before 3.10 do not expose the optional
                # surface-normal output in the Python binding.
                mujoco.mj_multiRay(**multi_ray_arguments)
            except TypeError:
                mujoco.mj_multiRay(
                    **multi_ray_arguments,
                    normal=None,
                )
            unblocked = (
                (hit_distances < 0.0)
                | (
                    hit_distances
                    >= target_distances - target_margin_m
                )
            )
            coverage[
                pose_index,
                tilt_index,
                sample_indices,
            ] = unblocked
    return coverage


def estimated_coarse_move_seconds(
    current_pose: SearchPose,
    target_pose: SearchPose,
) -> float:
    """Estimate sequential Arm 1, Arm 2, and turntable move duration."""

    output_degrees = abs(
        target_pose.turntable_degrees - current_pose.turntable_degrees
    )
    if abs(target_pose.arm1_degrees - current_pose.arm1_degrees) > 1e-9:
        # move_to_search_pose resets Arm 2 before changing Arm 1.
        output_degrees += (
            abs(current_pose.arm2_degrees)
            + abs(target_pose.arm1_degrees - current_pose.arm1_degrees)
            + abs(target_pose.arm2_degrees)
        )
    else:
        output_degrees += abs(
            target_pose.arm2_degrees - current_pose.arm2_degrees
        )
    return output_degrees / COARSE_OUTPUT_VELOCITY_DEG_S


def optimized_tilts_for_pose(
    pose_coverage: np.ndarray,
    uncovered_samples: np.ndarray,
    sample_weights: np.ndarray,
    starting_tilt: tuple[float, float],
    minimum_new_samples: int,
) -> tuple[tuple[int, ...], float, float, tuple[float, float]]:
    """Choose novel tilt views at one fixed slow-axis pose."""

    remaining = set(range(len(TILT_SCAN_CANDIDATES)))
    selected: list[int] = []
    temporary_uncovered = uncovered_samples.copy()
    current_tilt = starting_tilt
    gained_weight = 0.0
    estimated_seconds = 0.0

    while (
        remaining
        and len(selected) < MAX_TILT_VIEWS_PER_COARSE_POSE
    ):
        best: tuple[float, float, int, float, int] | None = None
        for tilt_index in remaining:
            newly_covered = (
                temporary_uncovered & pose_coverage[tilt_index]
            )
            new_sample_count = int(np.count_nonzero(newly_covered))
            new_weight = float(
                np.sum(sample_weights[newly_covered])
            )
            x_tilt, y_tilt = TILT_SCAN_CANDIDATES[tilt_index]
            travel_seconds = (
                abs(x_tilt - current_tilt[0])
                + abs(y_tilt - current_tilt[1])
            ) / TILT_OUTPUT_VELOCITY_DEG_S
            view_seconds = (
                travel_seconds + ESTIMATED_CAMERA_IMAGE_SECONDS
            )
            efficiency = new_weight / max(view_seconds, 1e-6)
            candidate = (
                efficiency,
                new_weight,
                new_sample_count,
                -view_seconds,
                -tilt_index,
            )
            if best is None or candidate > best:
                best = candidate

        assert best is not None
        best_efficiency = best[0]
        new_weight = best[1]
        new_sample_count = best[2]
        tilt_index = -best[4]
        if (
            new_sample_count < minimum_new_samples
            or best_efficiency
            < MIN_NEW_TARGET_SAMPLES_PER_ESTIMATED_SECOND
        ):
            break

        x_tilt, y_tilt = TILT_SCAN_CANDIDATES[tilt_index]
        view_seconds = (
            abs(x_tilt - current_tilt[0])
            + abs(y_tilt - current_tilt[1])
        ) / TILT_OUTPUT_VELOCITY_DEG_S + ESTIMATED_CAMERA_IMAGE_SECONDS
        selected.append(tilt_index)
        remaining.remove(tilt_index)
        estimated_seconds += view_seconds
        gained_weight += new_weight
        temporary_uncovered &= ~pose_coverage[tilt_index]
        current_tilt = (x_tilt, y_tilt)

    return (
        tuple(selected),
        gained_weight,
        estimated_seconds,
        current_tilt,
    )


def build_optimized_search_plan(light: RoboLight) -> SearchPlan:
    """Plan novel camera coverage while minimizing slow-axis repositioning."""

    planner = RoboLight(
        light.hwdesc,
        model_path=light.model_path,
        realtime=False,
    )
    target_samples = target_space_samples(planner)
    if len(target_samples) == 0:
        raise RuntimeError("search planner found no valid target-space samples")
    coverage = candidate_view_coverage(planner, target_samples)
    coverage_count = np.count_nonzero(coverage, axis=(0, 1))
    sample_weights = np.zeros(len(target_samples), dtype=np.float64)
    coverable_samples = coverage_count > 0
    sample_weights[coverable_samples] = (
        1.0 / coverage_count[coverable_samples]
    )
    # Preserve an average weight of one so efficiency thresholds retain their
    # intuitive approximate units while scarce views receive higher priority.
    if np.any(coverable_samples):
        sample_weights *= (
            np.count_nonzero(coverable_samples) / np.sum(sample_weights)
        )

    minimum_new_samples = max(
        1,
        math.ceil(
            len(target_samples) * MIN_NEW_TARGET_SAMPLE_FRACTION
        ),
    )
    uncovered = np.ones(len(target_samples), dtype=bool)
    current_pose = SearchPose("reset", 0.0, 0.0, 0.0)
    current_tilt = (0.0, 0.0)
    unvisited_pose_indices = set(range(len(SEARCH_POSES)))
    views: list[SearchView] = []
    estimated_elapsed = 0.0
    first_minute_covered = 0
    first_minute_view_count = 0

    while unvisited_pose_indices:
        best_group: (
            tuple[
                float,
                float,
                float,
                int,
                tuple[int, ...],
                float,
                tuple[float, float],
            ]
            | None
        ) = None
        for pose_index in unvisited_pose_indices:
            pose = SEARCH_POSES[pose_index]
            (
                tilt_indices,
                gained_weight,
                tilt_seconds,
                ending_tilt,
            ) = optimized_tilts_for_pose(
                coverage[pose_index],
                uncovered,
                sample_weights,
                current_tilt,
                minimum_new_samples,
            )
            if not tilt_indices:
                continue
            coarse_seconds = estimated_coarse_move_seconds(
                current_pose,
                pose,
            )
            group_seconds = coarse_seconds + tilt_seconds
            efficiency = gained_weight / max(group_seconds, 1e-6)
            candidate_group = (
                efficiency,
                gained_weight,
                -group_seconds,
                -pose_index,
                tilt_indices,
                coarse_seconds,
                ending_tilt,
            )
            if best_group is None or candidate_group > best_group:
                best_group = candidate_group

        if best_group is None:
            break

        pose_index = -best_group[3]
        tilt_indices = best_group[4]
        coarse_seconds = best_group[5]
        pose = SEARCH_POSES[pose_index]
        unvisited_pose_indices.remove(pose_index)
        estimated_elapsed += coarse_seconds

        for tilt_index in tilt_indices:
            x_tilt, y_tilt = TILT_SCAN_CANDIDATES[tilt_index]
            estimated_elapsed += (
                abs(x_tilt - current_tilt[0])
                + abs(y_tilt - current_tilt[1])
            ) / TILT_OUTPUT_VELOCITY_DEG_S
            estimated_elapsed += ESTIMATED_CAMERA_IMAGE_SECONDS
            newly_covered = int(
                np.count_nonzero(
                    uncovered & coverage[pose_index, tilt_index]
                )
            )
            uncovered &= ~coverage[pose_index, tilt_index]
            views.append(
                SearchView(
                    pose=pose,
                    x_tilt_degrees=x_tilt,
                    y_tilt_degrees=y_tilt,
                    newly_covered_samples=newly_covered,
                    estimated_elapsed_seconds=estimated_elapsed,
                )
            )
            current_tilt = (x_tilt, y_tilt)
            if estimated_elapsed <= FIRST_MINUTE_PLANNING_SECONDS:
                first_minute_covered = len(target_samples) - int(
                    np.count_nonzero(uncovered)
                )
                first_minute_view_count = len(views)

        current_pose = pose

    covered_samples = len(target_samples) - int(np.count_nonzero(uncovered))
    if not views:
        raise RuntimeError("search planner could not find a useful camera view")
    return SearchPlan(
        views=tuple(views),
        sampled_target_positions=len(target_samples),
        covered_target_positions=covered_samples,
        first_minute_covered_positions=first_minute_covered,
        first_minute_view_count=first_minute_view_count,
        estimated_total_seconds=estimated_elapsed,
    )


def acquire_target(
    light: RoboLight,
    timeout_seconds: float = MAX_ACQUISITION_SECONDS,
    search_plan: SearchPlan | None = None,
) -> AcquisitionOutcome:
    """Search and center for at most ``timeout_seconds`` of wall-clock time."""

    if not math.isfinite(timeout_seconds) or timeout_seconds <= 0.0:
        raise ValueError("timeout_seconds must be above zero")
    if search_plan is None:
        search_plan = build_optimized_search_plan(light)
    started = time.perf_counter()
    deadline = started + timeout_seconds
    images_acquired = 0
    current_pose: SearchPose | None = None
    try:
        for view in search_plan.views:
            if view.pose != current_pose:
                move_to_search_pose(light, view.pose, deadline)
                print(f"  searching {view.pose.label}")
                current_pose = view.pose
            move_to_tilt(
                light,
                view.x_tilt_degrees,
                view.y_tilt_degrees,
                deadline,
            )
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
                result = rebalance_centered_target(
                    light,
                    result,
                    deadline,
                )
                return AcquisitionOutcome(
                    result=result,
                    elapsed_seconds=time.perf_counter() - started,
                    timed_out=False,
                )
            print(
                "  target lost while centering; resuming optimized search"
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
    """Choose a sphere center inside the room and not underneath the table.

    Bounds come from the actual named MuJoCo geoms rather than duplicated
    numeric ranges. The target radius and a small visual clearance keep the
    sphere from intersecting a room surface. A target may be below the upper
    turntable only when its complete sphere is outside the table radius.
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
    safe_table_radius = upper_table.size[0] + margin_m
    safe_table_top = (
        table_center[2] + upper_table.size[1] + margin_m
    )
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
        if (
            candidate[2] >= safe_table_top
            or radial_distance >= safe_table_radius
        ):
            world_position = candidate
            break
    if world_position is None:
        raise RuntimeError(
            "could not place a target outside or above the upper table"
        )

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


def read_gave_up_targets(
    log_path: Path | None,
) -> tuple[tuple[float, float, float], ...]:
    """Read timed-out target X/Y/Z coordinates from an acquisition log."""

    if log_path is None:
        return ()

    resolved_path = log_path.expanduser().resolve()
    if not resolved_path.is_file():
        print(
            "Historical target overlay skipped; log file not found: "
            f"{resolved_path}"
        )
        return ()

    positions: list[tuple[float, float, float]] = []
    with resolved_path.open(encoding="utf-8") as log_file:
        for line in log_file:
            match = GAVE_UP_TARGET_PATTERN.search(line)
            if match is None:
                continue
            positions.append(
                (
                    float(match.group("x")),
                    float(match.group("y")),
                    float(match.group("z")),
                )
            )

    print(
        f"Read {len(positions)} gave-up target locations from "
        f"{resolved_path}"
    )
    return tuple(positions)


def plot_gave_up_targets(
    light: RoboLight,
    viewer: object,
    positions_cm: tuple[tuple[float, float, float], ...],
) -> int:
    """Plot historical target locations as green main-viewer spheres."""

    if not positions_cm:
        return 0

    scene = viewer.user_scn
    if scene is None:
        raise RuntimeError("MuJoCo viewer has no user scene for target plotting")
    available_geoms = int(scene.maxgeom - scene.ngeom)
    if len(positions_cm) > available_geoms:
        raise RuntimeError(
            f"Cannot plot {len(positions_cm)} gave-up targets; the viewer has "
            f"space for {available_geoms} user geoms"
        )

    target_origin = light.model.body("target_origin_frame")
    origin_world_m = light.data.xpos[target_origin.id].copy()
    radius_m = GAVE_UP_TARGET_DIAMETER_CM / 200.0
    size = np.array((radius_m, radius_m, radius_m), dtype=np.float64)
    orientation = np.eye(3, dtype=np.float64).reshape(9)
    color = np.asarray(GAVE_UP_TARGET_RGBA, dtype=np.float32)

    with viewer.lock():
        first_geom = int(scene.ngeom)
        for offset, position_cm in enumerate(positions_cm):
            position_world_m = (
                origin_world_m
                + np.asarray(position_cm, dtype=np.float64) / 100.0
            )
            mujoco.mjv_initGeom(
                scene.geoms[first_geom + offset],
                mujoco.mjtGeom.mjGEOM_SPHERE,
                size,
                position_world_m,
                orientation,
                color,
            )
        scene.ngeom = first_geom + len(positions_cm)

    viewer.sync()
    return len(positions_cm)


def mark_missed_target(
    light: RoboLight,
    viewer: object | None,
    position_cm: tuple[float, float, float],
) -> None:
    """Turn the active target green and leave a persistent green marker."""

    target_x, target_y, target_z = position_cm
    light.set_target(
        target_x,
        target_y,
        target_z,
        color="green",
        diameter_cm=TARGET_DIAMETER_CM,
    )
    if viewer is not None:
        plot_gave_up_targets(
            light,
            viewer,
            (position_cm,),
        )
    print(
        "  marked missed target green at "
        f"X={target_x:.1f}, Y={target_y:.1f}, Z={target_z:.1f} cm"
    )


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
    light = RoboLight(
        DEMO_HARDWARE,
        realtime=not args.headless,
        time_multiplier=SIMULATION_TIME_MULTIPLIER,
    )
    try:
        print(f"HWDesc: {DEMO_HARDWARE}")
        print(
            "Simulation time multiplier: "
            f"{SIMULATION_TIME_MULTIPLIER:g}x"
        )
        search_plan = build_optimized_search_plan(light)
        print(
            f"Optimized search plan: {len(search_plan.views)} camera views "
            f"at {search_plan.coarse_pose_count} arm/turntable poses; "
            f"{search_plan.first_minute_coverage_percent:.1f}% sampled-space "
            f"coverage in the first "
            f"{FIRST_MINUTE_PLANNING_SECONDS:.0f} estimated sec; "
            f"{search_plan.coverage_percent:.1f}% total coverage"
        )
        gave_up_targets = read_gave_up_targets(GAVE_UP_TARGET_LOG)
        viewer: object | None = None
        if not args.headless:
            viewer = light.open_viewer()
            plotted_target_count = plot_gave_up_targets(
                light,
                viewer,
                gave_up_targets,
            )
            light.open_pip()
            print(
                f"Plotted {plotted_target_count} gave-up target locations "
                "as green spheres"
            )
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
                search_plan=search_plan,
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
                mark_missed_target(
                    light,
                    viewer,
                    (target_x, target_y, target_z),
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
                mark_missed_target(
                    light,
                    viewer,
                    (target_x, target_y, target_z),
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
