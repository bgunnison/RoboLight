"""Control API for the RoboLight robotic flashlight.

RoboLight is intended to shine a flashlight where a user points while remaining
inexpensive, mechanically expressive, and entertaining. A camera is aligned
with the flashlight so a future vision loop can find a pointer, possibly a ring
worn by the user, and center the beam on it.

The mechanical design uses one motor and a selectable transmission. G1 is the
shared drive gear. Five follower paths operate Arm 1, Arm 2, Y tilt, X tilt, and
the base turntable. Physical hardware will normally engage one path at a time,
then rely on friction to hold the axis while leaving it compliant. The simulator
also permits combined selections through the low-level ``move_motor()`` method
so transmission behavior can be explored.

Each follower gear G2-G6 is assumed to have an absolute encoder with a calibrated
zero. ``reset()`` reads those positions and physically homes one gear at a fixed
100 motor-degrees/s in the order X tilt, Y tilt, Arm 2, Arm 1, and turntable.
Motor/G1 angle and simulation time advance normally during that sequence.

This module implements the current kinematic simulation, not the complete
product control loop. Camera pointer recognition, manual-displacement sensing,
the planned Hijacked mode, and its blinking status LED are not implemented yet.

The mechanism-level ``move()`` API accepts the selected output's displacement
and speed, then translates them into motor/G1 motion using gear, spool, and
cable-lever ratios. The UI uses the same output-angle translation by default;
its optional direct-G1 mode and ``move_motor()`` retain raw motor-angle units.
High-level moves return a :class:`MoveError`: ``OK`` means the move completed,
while every other result means the whole command was rejected before motion.
Arm 1 has a configurable symmetric travel limit that defaults to +/-80 degrees,
and arm moves are checked against the upper constraint plate using the current
Arm 2 pose and configured link lengths.
Only selected follower paths are coupled to G1; unselected mechanisms hold
their current simulated positions. Calls are deterministic and headless by
default, making the class useful for control development and automated tests.

Typical use::

    from scripts import HWDesc, MoveError, RoboLight, Selector

    light = RoboLight(HWDesc(spool_diameter_mm=10), realtime=True)
    light.open_viewer()
    light.open_pip()
    result = light.move(Selector.ARM1, velocity=30, degrees=20)
    if result is not MoveError.OK:
        raise RuntimeError(f"Arm 1 move rejected: {result.value}")
    result = light.move(Selector.Y_TILT, velocity=30, degrees=-15)
    state = light.state
    print(state.y_tilt_degrees)
    light.reset()
    light.close_pip()
    light.close_viewer()
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from contextlib import nullcontext
from dataclasses import asdict, dataclass, replace
from enum import Enum
import math
from pathlib import Path
from threading import RLock
import time
import tkinter as tk
from typing import Any

import mujoco
import mujoco.viewer
import numpy as np
from PIL import Image, ImageTk

from sim.launch_simple_motor_gear_controls import (
    ARM_CONSTRAINT_ARM1_LIMIT,
    ARM_CONSTRAINT_PLATFORM_COLLISION,
    ARM1_LIMIT_DEGREES as MODEL_ARM1_LIMIT_DEGREES,
    DEFAULT_G1_DIAMETER_MM,
    DEFAULT_G2_DIAMETER_MM,
    DEFAULT_ARM1_LENGTH_MM,
    DEFAULT_ARM2_LENGTH_MM,
    DEFAULT_SPEED_DEG_S,
    DEFAULT_SPOOL_DIAMETER_MM,
    G1_JOINT,
    G2_JOINT,
    G3_JOINT,
    G4_JOINT,
    G5_JOINT,
    G6_JOINT,
    MAX_DIAMETER_MM,
    MAX_ARM_LENGTH_MM,
    MAX_ARM1_LIMIT_DEGREES,
    MAX_MOVE_DEGREES,
    MAX_SPEED_DEG_S,
    MAX_SPOOL_DIAMETER_MM,
    MAX_TILT_DEGREES,
    MIN_DIAMETER_MM,
    MIN_ARM_LENGTH_MM,
    MIN_ARM1_LIMIT_DEGREES,
    MIN_MOVE_DEGREES,
    MIN_SPOOL_DIAMETER_MM,
    MIN_TILT_DEGREES,
    MODEL_PATH,
    PIP_HEIGHT,
    PIP_REFRESH_SECONDS,
    PIP_WIDTH,
    PLATE_CABLE_LEVER_ARM_M,
    PLATE_X_TILT_JOINT,
    PLATE_Y_TILT_JOINT,
    SPOTLIGHT_CAMERA,
    START_CAMERA_AZIMUTH,
    START_CAMERA_DISTANCE,
    START_CAMERA_ELEVATION,
    START_CAMERA_LOOKAT,
    TG2_JOINT,
    TG4_JOINT,
    TG6_JOINT,
    TURNTABLE_JOINT,
    arm_motion_violation,
    g1_angle_from_motor,
    g2_angle_from_g1,
    output_degrees_per_motor_degree,
    set_gear_layout,
    shortest_angular_error_to_zero,
    tilt_delta_from_spool,
)


class Selector(str, Enum):
    """Transmission paths that can be coupled to G1 during a move.

    ``G1`` rotates only the shared motor input. ``ARM1`` couples G2 and the
    first timing-belt stage. ``ARM2`` couples G3 and the second arm stages.
    ``Y_TILT`` couples the G4 cable spool, ``X_TILT`` couples the G5 cable
    spool, and ``TURNTABLE`` couples G6 to the lazy-Susan base. ``ALL`` is a
    low-level :meth:`RoboLight.move_motor` convenience that engages all five
    follower paths together.
    """

    G1 = "g1"
    ARM1 = "arm1"
    ARM2 = "arm2"
    Y_TILT = "y_tilt"
    X_TILT = "x_tilt"
    TURNTABLE = "turntable"
    ALL = "all"


class MoveError(str, Enum):
    """Outcome returned by :meth:`RoboLight.move`.

    ``OK`` means the complete command executed. Other values mean the command
    was rejected before any motion occurred, so callers may inspect
    :attr:`RoboLight.state` without needing to undo a partial move.

    Invalid selector, degree, and velocity values have separate results.
    ``MOTOR_SPEED_LIMIT`` means the requested output speed translated above
    720 motor-deg/s. ``TILT_LIMIT`` protects the +/-45-degree tilt joints.
    ``ARM1_LIMIT`` protects Arm 1's configured symmetric travel range, and
    ``PLATFORM_COLLISION`` means the arm's swept geometry would cross the
    infinite upper-turntable plane. ``LOST_STEPS`` and ``HIJACKED`` are reserved
    for physical feedback; the current kinematic simulator does not emit them.
    """

    OK = "ok"
    INVALID_SELECTOR = "invalid_selector"
    INVALID_DEGREES = "invalid_degrees"
    INVALID_VELOCITY = "invalid_velocity"
    MOTOR_SPEED_LIMIT = "motor_speed_limit"
    TILT_LIMIT = "tilt_limit"
    ARM1_LIMIT = "arm1_limit"
    PLATFORM_COLLISION = "platform_collision"
    LOST_STEPS = "lost_steps"
    HIJACKED = "hijacked"


FOLLOWER_SELECTORS = frozenset(
    {
        Selector.ARM1,
        Selector.ARM2,
        Selector.Y_TILT,
        Selector.X_TILT,
        Selector.TURNTABLE,
    }
)

SELECTOR_ALIASES = {
    "motor": Selector.G1,
    "g1": Selector.G1,
    "arm1": Selector.ARM1,
    "arm_1": Selector.ARM1,
    "g2": Selector.ARM1,
    "arm2": Selector.ARM2,
    "arm_2": Selector.ARM2,
    "g3": Selector.ARM2,
    "g4": Selector.Y_TILT,
    "y": Selector.Y_TILT,
    "y_tilt": Selector.Y_TILT,
    "g4_y_tilt": Selector.Y_TILT,
    "g5": Selector.X_TILT,
    "x": Selector.X_TILT,
    "x_tilt": Selector.X_TILT,
    "g5_x_tilt": Selector.X_TILT,
    "g6": Selector.TURNTABLE,
    "turntable": Selector.TURNTABLE,
    "g6_turntable": Selector.TURNTABLE,
    "lazy_susan": Selector.TURNTABLE,
    "all": Selector.ALL,
}


@dataclass(frozen=True, slots=True)
class HWDesc:
    """Adjustable mechanical dimensions shared with the control UI.

    Attributes:
        g1_diameter_mm: Diameter of the common motor-driven G1 gear. Valid range
            is 35-140 mm.
        follower_diameter_mm: Shared diameter of G2 through G6. Valid range is
            35-140 mm. Together with G1 diameter, this sets the follower ratio.
        spool_diameter_mm: Shared diameter of the G4 and G5 cable spools. Valid
            range is 5-50 mm. A larger spool produces more plate tilt for the
            same follower rotation.
        arm1_length_mm: Pivot-to-pivot length of Arm 1. Valid range is 75-300
            mm.
        arm2_length_mm: Pivot-to-plate length of Arm 2. Valid range is 75-300
            mm.
        arm1_limit_degrees: Symmetric physical travel limit for Arm 1. The
            default is +/-80 degrees; valid magnitudes are 1-180 degrees.
    """

    g1_diameter_mm: float = DEFAULT_G1_DIAMETER_MM
    follower_diameter_mm: float = DEFAULT_G2_DIAMETER_MM
    spool_diameter_mm: float = DEFAULT_SPOOL_DIAMETER_MM
    arm1_length_mm: float = DEFAULT_ARM1_LENGTH_MM
    arm2_length_mm: float = DEFAULT_ARM2_LENGTH_MM
    arm1_limit_degrees: float = MODEL_ARM1_LIMIT_DEGREES


@dataclass(frozen=True, slots=True)
class RoboLightState:
    """Immutable snapshot returned after each API operation.

    All angular fields are in degrees. ``motor_degrees`` and ``g1_degrees`` are
    cumulative from construction and include physical reset motion. Reset uses
    the absolute follower encoders rather than clearing the motor.
    Cyclic G2/G3/G6 encoder and Arm 1/Arm 2/turntable fields report the physical
    orientation in the signed -180 through +180 degree range, so a complete
    revolution reads as zero. G4/G5 remain multi-turn spool positions because
    their cable length is not cyclic. Tilt fields expose the useful plate
    position.
    ``simulation_time_seconds`` advances from requested move distance and
    velocity even when real-time pacing is
    disabled. ``last_selectors`` records the normalized selector names supplied
    to the most recent move.
    """

    motor_degrees: float
    g1_degrees: float
    g2_degrees: float
    g3_degrees: float
    g4_degrees: float
    g5_degrees: float
    g6_degrees: float
    arm1_degrees: float
    arm2_degrees: float
    x_tilt_degrees: float
    y_tilt_degrees: float
    turntable_degrees: float
    simulation_time_seconds: float
    last_selectors: tuple[str, ...]

    def to_dict(self) -> dict[str, Any]:
        """Return all snapshot fields as a new plain dictionary.

        The dictionary is convenient for logging, JSON preparation, and test
        output. Modifying it does not modify the simulation state.
        """

        return asdict(self)


class RoboLight:
    """Own and control one RoboLight MuJoCo simulation.

    A ``RoboLight`` instance retains its pose between calls. ``move`` commands
    one selected output in output degrees and translates that request into the
    necessary motor/G1 motion. Once the call ends, that output remains at its
    new simulated position, representing the friction hold planned for physical
    hardware. ``move_motor`` exposes direct G1 units matching the UI's optional
    direct-G1 mode and supports transmission experiments.

    Moves are blocking. They execute as quickly as the host allows by default
    while still advancing MuJoCo time according to ``velocity``. Pass
    ``realtime=True`` when the call should take the corresponding wall-clock
    duration. ``move`` uses output-side velocity; ``move_motor`` uses motor-side
    velocity.

    Args:
        hwdesc: Optional initial hardware dimensions. May be an :class:`HWDesc`
            or a mapping of hardware field names to millimeter values.
        model_path: MJCF file to load. Custom models must contain the same named
            joints as the standard RoboLight model.
        realtime: If true, sleep between simulation steps to match requested
            motion in wall-clock time.
        step_seconds: Maximum integration interval. The default is 1/120 s.

    Raises:
        FileNotFoundError: If ``model_path`` does not exist.
        RuntimeError: If a required named joint is missing from the model.
        TypeError: If hardware values are not numeric.
        ValueError: If dimensions or ``step_seconds`` are outside their limits.
    """

    _JOINTS = {
        "g1": G1_JOINT,
        "g2": G2_JOINT,
        "g3": G3_JOINT,
        "g4": G4_JOINT,
        "g5": G5_JOINT,
        "g6": G6_JOINT,
        "turntable": TURNTABLE_JOINT,
        "arm1": TG2_JOINT,
        "arm2_stage": TG4_JOINT,
        "arm2": TG6_JOINT,
        "x_tilt": PLATE_X_TILT_JOINT,
        "y_tilt": PLATE_Y_TILT_JOINT,
    }

    ARM1_LIMIT_DEGREES = MODEL_ARM1_LIMIT_DEGREES
    RESET_VELOCITY_DEG_S = DEFAULT_SPEED_DEG_S
    RESET_SEQUENCE = (
        (Selector.X_TILT, "g5"),
        (Selector.Y_TILT, "g4"),
        (Selector.ARM2, "g3"),
        (Selector.ARM1, "g2"),
        (Selector.TURNTABLE, "g6"),
    )

    def __init__(
        self,
        hwdesc: HWDesc | Mapping[str, float] | None = None,
        *,
        model_path: str | Path = MODEL_PATH,
        realtime: bool = False,
        step_seconds: float = 1.0 / 120.0,
    ) -> None:
        self._lock = RLock()
        self._model_path = Path(model_path).expanduser().resolve()
        if not self._model_path.is_file():
            raise FileNotFoundError(f"MuJoCo model not found: {self._model_path}")
        if not math.isfinite(step_seconds) or step_seconds <= 0.0:
            raise ValueError("step_seconds must be a finite value above zero")

        self.realtime = bool(realtime)
        self.step_seconds = float(step_seconds)
        self.model = mujoco.MjModel.from_xml_path(str(self._model_path))
        self.data = mujoco.MjData(self.model)
        self._qpos = self._resolve_joint_qpos()
        self._hwdesc = HWDesc()
        self._motor_angle_rad = 0.0
        self._last_selectors = (Selector.G1.value,)
        self._viewer: Any | None = None
        self._pip_root: tk.Tk | None = None
        self._pip_canvas: tk.Canvas | None = None
        self._pip_image_item: int | None = None
        self._pip_photo: ImageTk.PhotoImage | None = None
        self._pip_renderer: mujoco.Renderer | None = None
        self._last_pip_time = 0.0
        self.set_hw(hwdesc or HWDesc())
        self.reset()

    @property
    def hwdesc(self) -> HWDesc:
        """Return the currently applied, immutable hardware description."""

        return self._hwdesc

    @property
    def model_path(self) -> Path:
        """Absolute path of the MJCF file loaded by this instance."""

        return self._model_path

    @property
    def state(self) -> RoboLightState:
        """Return a new immutable snapshot without moving the mechanism."""

        with self._lock:
            return self._snapshot()

    @property
    def viewer_is_running(self) -> bool:
        """Whether this instance currently owns an open passive viewer."""

        viewer = self._viewer
        return bool(viewer is not None and viewer.is_running())

    def open_viewer(self) -> Any:
        """Open a passive MuJoCo viewer synchronized by API operations.

        The viewer uses the same initial camera pose as the Tk control launcher.
        ``move``, ``move_motor``, ``set_hw``, ``set_tilt``, and ``reset``
        synchronize it after every state update. Set ``realtime=True`` on
        :class:`RoboLight` if moves should remain visible for their requested
        physical duration.

        Calling this method while the viewer is already open returns the
        existing handle.

        Returns:
            MuJoCo passive viewer handle. The handle may also be closed directly,
            although :meth:`close_viewer` is the preferred API.
        """

        with self._lock:
            if self.viewer_is_running:
                return self._viewer

            self._viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self._viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            self._viewer.cam.lookat[:] = START_CAMERA_LOOKAT
            self._viewer.cam.distance = START_CAMERA_DISTANCE
            self._viewer.cam.azimuth = START_CAMERA_AZIMUTH
            self._viewer.cam.elevation = START_CAMERA_ELEVATION
            self._viewer.sync()
            return self._viewer

    def close_viewer(self) -> None:
        """Close the passive viewer if one is open.

        Closing the viewer does not discard the model or mechanism state, so a
        new viewer can be opened later and will display the current pose.
        """

        with self._lock:
            viewer = self._viewer
            self._viewer = None
            if viewer is not None:
                viewer.close()

    @property
    def pip_is_open(self) -> bool:
        """Whether the spotlight-camera PIP window is currently open."""

        root = self._pip_root
        if root is None:
            return False
        try:
            return bool(root.winfo_exists())
        except tk.TclError:
            return False

    def open_pip(self) -> tk.Tk:
        """Open a 320×240 window showing the spotlight-aligned camera.

        The image is rendered from the named ``spotlight_camera`` in the MuJoCo
        model. It therefore follows the plate, flashlight, arm, and turntable.
        API motion refreshes the image at the same throttled rate as the UI PIP.

        Calling this method while the PIP is already open raises and focuses the
        existing window.

        Returns:
            Tk root window that owns the PIP.
        """

        with self._lock:
            if self.pip_is_open:
                self._pip_root.lift()
                self._sync_pip(force=True)
                return self._pip_root

            self._dispose_pip()
            root = tk.Tk()
            root.title("RoboLight Spotlight Camera PIP")
            root.resizable(False, False)
            root.protocol("WM_DELETE_WINDOW", self.close_pip)

            canvas = tk.Canvas(
                root,
                width=PIP_WIDTH,
                height=PIP_HEIGHT,
                background="black",
                highlightthickness=0,
            )
            canvas.pack(padx=8, pady=(8, 4))
            image_item = canvas.create_image(0, 0, anchor="nw")
            tk.Label(root, text="50 deg view aligned with the spotlight beam").pack(
                padx=8,
                pady=(0, 8),
            )

            try:
                renderer = mujoco.Renderer(
                    self.model,
                    height=PIP_HEIGHT,
                    width=PIP_WIDTH,
                )
            except Exception:
                root.destroy()
                raise

            self._pip_root = root
            self._pip_canvas = canvas
            self._pip_image_item = image_item
            self._pip_renderer = renderer
            self._last_pip_time = 0.0
            self._sync_pip(force=True)
            return root

    def close_pip(self) -> None:
        """Close the spotlight-camera PIP and release its renderer."""

        with self._lock:
            self._dispose_pip()

    def sync_visuals(self) -> None:
        """Synchronize and pump any open viewer and PIP windows immediately.

        Motion methods call this automatically. Applications with their own
        pauses or event loops may call it to keep the standalone PIP responsive
        while the mechanism is stationary.
        """

        with self._lock:
            self._sync_visuals(force_pip=True)

    def set_hw(self, hwdesc: HWDesc | Mapping[str, float]) -> HWDesc:
        """Apply validated gear, spool, and arm dimensions, preserving pose.

        A mapping may provide any subset of the fields in :class:`HWDesc`.
        ``g2_diameter_mm`` and ``g2_g6_diameter_mm`` are accepted as aliases for
        ``follower_diameter_mm``.

        Changing gear diameter changes the ratio used by subsequent moves.
        Changing spool diameter changes the cable travel produced by subsequent
        G4/G5 rotation. Changing either arm length resizes its link and moves
        all child hardware at its endpoint. Existing angular gear, arm, tilt,
        and turntable positions do not jump when the description is applied.

        Args:
            hwdesc: Complete :class:`HWDesc` or a mapping containing one or more
                hardware fields in millimeters.

        Returns:
            The complete hardware description now applied to the model.

        Raises:
            TypeError: If ``hwdesc`` or one of its values has the wrong type.
            ValueError: If a field is unknown, non-finite, or outside its range.
        """

        with self._lock:
            candidate = self._coerce_hwdesc(hwdesc)
            self._validate_hwdesc(candidate)
            self._hwdesc = candidate
            with self._viewer_data_lock():
                self._apply_layout()
                mujoco.mj_forward(self.model, self.data)
            self._sync_visuals()
            return self._hwdesc

    def SetHW(self, hwdesc: HWDesc | Mapping[str, float]) -> HWDesc:  # noqa: N802
        """Apply hardware dimensions using the controller-style method name.

        This is an exact alias for :meth:`set_hw`. New Python code may prefer
        the lower-case spelling; integrations modeled after a hardware control
        interface may use ``SetHW``.
        """

        return self.set_hw(hwdesc)

    def get_position(self, selector: Selector | str) -> float:
        """Return the current selected-output position in degrees.

        ``G1`` returns the cumulative drive-gear angle. Arm 1, Arm 2, and the
        turntable return their cyclic signed positions from -180 through +180
        degrees. X/Y tilt return their signed plate angles. Selector aliases
        accepted by :meth:`move` are accepted here as well.

        Args:
            selector: Exactly one output selector or supported selector name.

        Returns:
            Current position of that selected output in degrees.

        Raises:
            TypeError: If ``selector`` has an unsupported type.
            ValueError: If the selector is unknown or represents several
                outputs, such as ``Selector.ALL``.
        """

        with self._lock:
            selectors = self._normalize_selectors(selector)
            if len(selectors) != 1:
                raise ValueError("get_position() requires exactly one selector")
            selected = next(iter(selectors))
            state = self._snapshot()
            fields = {
                Selector.G1: "g1_degrees",
                Selector.ARM1: "arm1_degrees",
                Selector.ARM2: "arm2_degrees",
                Selector.Y_TILT: "y_tilt_degrees",
                Selector.X_TILT: "x_tilt_degrees",
                Selector.TURNTABLE: "turntable_degrees",
            }
            return float(getattr(state, fields[selected]))

    def get_camera(self) -> np.ndarray:
        """Capture the spotlight-aligned camera as a new RGB image array.

        The result is a 240 x 320 x 3 NumPy array with ``uint8`` RGB pixels.
        It owns a copy of the rendered pixels, so later simulation updates do
        not mutate an image already returned to the caller. The array can be
        passed to Pillow, OpenCV, scikit-image, or another future image package.
        Opening the viewer or PIP is not required.

        Returns:
            Fresh ``numpy.ndarray`` containing the current camera image.
        """

        with self._lock:
            with self._viewer_data_lock():
                if self._pip_renderer is not None:
                    self._pip_renderer.update_scene(
                        self.data,
                        camera=SPOTLIGHT_CAMERA,
                    )
                    return self._pip_renderer.render().copy()

                renderer = mujoco.Renderer(
                    self.model,
                    height=PIP_HEIGHT,
                    width=PIP_WIDTH,
                )
                try:
                    renderer.update_scene(self.data, camera=SPOTLIGHT_CAMERA)
                    return renderer.render().copy()
                finally:
                    renderer.close()

    def move(
        self,
        selector: Selector | str,
        velocity: float,
        degrees: float,
    ) -> MoveError:
        """Move one selected mechanism output by a relative angle.

        This is the mechanism-level API. ``degrees`` is the requested change
        of the selected output, and ``velocity`` is that output's speed, both
        expressed in degrees. For example, selecting ``ARM1`` with
        ``degrees=20`` rotates Arm 1 by positive 20 degrees regardless of the
        configured gear diameters. Selecting ``X_TILT`` or ``Y_TILT`` applies
        the spool diameter and cable lever ratio as well.

        The controller translates the output command into the required G1
        motor rotation and motor speed. External gear meshing reverses the
        direction automatically. Arm 1 uses the symmetric travel magnitude in
        :attr:`HWDesc.arm1_limit_degrees`, which defaults to +/-80 degrees. Arm
        moves are also checked along their complete swept path against the
        upper turntable, modeled as an infinite horizontal platform. A rejected
        move performs no motion.

        Because the physical design has one motor, exactly one output may be
        selected by this method. Use :meth:`move_motor` for direct G1 commands
        or simulation experiments that deliberately engage several selectors
        together. That low-level method bypasses the arm safety checks.

        Args:
            selector: Exactly one output selector or supported selector name.
            velocity: Positive selected-output speed in degrees per second.
            degrees: Signed relative selected-output rotation from -360 through
                +360 degrees. Tilt commands must also keep the resulting plate
                angle within its -45 through +45 degree joint range.

        Returns:
            :class:`MoveError.OK` after a completed move, otherwise an enum
            identifying why the entire command was rejected. Read
            :attr:`state` or call :meth:`get_position` for the resulting
            mechanism state. Physical hardware may additionally return
            :class:`MoveError.LOST_STEPS` or :class:`MoveError.HIJACKED`; those
            two feedback paths are stubbed and never emitted by this simulator.
        """

        with self._lock:
            try:
                selectors = self._normalize_selectors(selector)
            except (TypeError, ValueError):
                return MoveError.INVALID_SELECTOR
            if len(selectors) != 1:
                return MoveError.INVALID_SELECTOR
            selected = next(iter(selectors))

            try:
                output_degrees = self._finite_number("degrees", degrees)
            except (TypeError, ValueError):
                return MoveError.INVALID_DEGREES
            if not MIN_MOVE_DEGREES <= output_degrees <= MAX_MOVE_DEGREES:
                return MoveError.INVALID_DEGREES
            if abs(output_degrees) < 1e-12:
                self._last_selectors = (selected.value,)
                return MoveError.OK

            try:
                output_speed = self._finite_number("velocity", velocity)
            except (TypeError, ValueError):
                return MoveError.INVALID_VELOCITY
            if output_speed <= 0.0:
                return MoveError.INVALID_VELOCITY

            if selected in (Selector.X_TILT, Selector.Y_TILT):
                field = "x_tilt" if selected is Selector.X_TILT else "y_tilt"
                current_degrees = math.degrees(self.data.qpos[self._qpos[field]])
                target_degrees = current_degrees + output_degrees
                if not (
                    MIN_TILT_DEGREES - 1e-9
                    <= target_degrees
                    <= MAX_TILT_DEGREES + 1e-9
                ):
                    return MoveError.TILT_LIMIT

            if selected in (Selector.ARM1, Selector.ARM2):
                with self._viewer_data_lock():
                    arm1_degrees = math.degrees(
                        self.data.qpos[self._qpos["arm1"]]
                    )
                    arm2_degrees = math.degrees(
                        self.data.qpos[self._qpos["arm2"]]
                    )
                arm_violation = arm_motion_violation(
                    arm1_degrees,
                    arm2_degrees,
                    output_degrees if selected is Selector.ARM1 else 0.0,
                    output_degrees if selected is Selector.ARM2 else 0.0,
                    self._hwdesc.arm1_length_mm,
                    self._hwdesc.arm2_length_mm,
                    self._hwdesc.arm1_limit_degrees,
                )
                if arm_violation == ARM_CONSTRAINT_ARM1_LIMIT:
                    return MoveError.ARM1_LIMIT
                if arm_violation == ARM_CONSTRAINT_PLATFORM_COLLISION:
                    return MoveError.PLATFORM_COLLISION

            output_per_motor = self._output_degrees_per_motor_degree(selected)
            motor_degrees = output_degrees / output_per_motor
            motor_speed = output_speed / abs(output_per_motor)
            if motor_speed > MAX_SPEED_DEG_S:
                return MoveError.MOTOR_SPEED_LIMIT

            # Small output ratios can require more than one legal 360-degree
            # motor move. Execute adjacent chunks at the translated speed.
            remaining = motor_degrees
            while abs(remaining) >= 1e-12:
                motor_chunk = math.copysign(
                    min(abs(remaining), MAX_MOVE_DEGREES),
                    remaining,
                )
                self.move_motor(
                    selected,
                    velocity=motor_speed,
                    degrees=motor_chunk,
                )
                remaining -= motor_chunk
            return MoveError.OK

    def move_motor(
        self,
        selector: Selector | str | Iterable[Selector | str],
        velocity: float,
        degrees: float,
    ) -> RoboLightState:
        """Perform a direct signed G1 motor move and drive selected outputs.

        ``velocity`` is a positive motor speed in degrees per second. ``degrees``
        is the signed motor rotation and must be within -360 to +360. Selectors
        may be enum values, UI-like names (``"G4"``), or an iterable of either.
        The special selector ``"all"`` couples every follower path.

        This is a relative move, not an absolute target: two successive
        45-degree calls leave the motor at 90 degrees. G1 moves on every call.
        Each selected follower starts from its current pose and follows the
        change in G1 through the configured gear ratio. An unselected output
        holds its previous pose.

        Selecting ``Y_TILT`` or ``X_TILT`` converts G4/G5 spool rotation to
        cable travel and plate tilt. Selecting ``TURNTABLE`` rotates G6 and the
        entire lazy-Susan assembly. Physical hardware is intended to select one
        output at a time, although an iterable is supported for simulation
        experiments. This method uses the same direct motor units as the UI's
        Motor V input and its optional direct-G1 rotation mode. It intentionally
        bypasses the Arm 1 and upper-platform safety checks; normal controller
        code should use :meth:`move`.

        Args:
            selector: One selector, a supported selector string, or an iterable
                of them. Examples include ``Selector.ARM1``, ``"g2"``,
                ``"G4 -> Y tilt"``, and
                ``[Selector.ARM1, Selector.ARM2]``.
            velocity: Positive motor speed in degrees per second, no greater
                than 720.
            degrees: Signed relative motor rotation from -360 through +360.

        Returns:
            State snapshot taken after the move is complete.

        Raises:
            TypeError: If an argument has an unsupported type.
            ValueError: If the selector is unknown or a numeric value is
                non-finite or outside its allowed range.
        """

        with self._lock:
            selectors = self._normalize_selectors(selector)
            move_degrees = self._finite_number("degrees", degrees)
            if not MIN_MOVE_DEGREES <= move_degrees <= MAX_MOVE_DEGREES:
                raise ValueError(
                    f"degrees must be between {MIN_MOVE_DEGREES:g} and {MAX_MOVE_DEGREES:g}"
                )
            self._last_selectors = tuple(
                item.value for item in sorted(selectors, key=lambda item: item.value)
            )
            if abs(move_degrees) < 1e-12:
                return self._snapshot()

            speed = self._finite_number("velocity", velocity)
            if not 0.0 < speed <= MAX_SPEED_DEG_S:
                raise ValueError(f"velocity must be above 0 and at most {MAX_SPEED_DEG_S:g} deg/s")

            followers = selectors & FOLLOWER_SELECTORS
            duration = abs(move_degrees) / speed
            step_count = max(1, math.ceil(duration / self.step_seconds))
            dt = duration / step_count
            motor_step = math.radians(move_degrees) / step_count
            with self._viewer_data_lock():
                references = self._capture_drive_references(followers)

            next_wall_time = time.perf_counter()
            for _ in range(step_count):
                with self._viewer_data_lock():
                    self._motor_angle_rad += motor_step
                    self._apply_motor_and_followers(followers, references)
                    self.data.time += dt
                    self._apply_layout()
                    mujoco.mj_forward(self.model, self.data)
                self._sync_visuals()

                if self.realtime:
                    next_wall_time += dt
                    delay = next_wall_time - time.perf_counter()
                    if delay > 0.0:
                        time.sleep(delay)

            return self._snapshot()

    def set_tilt(
        self,
        *,
        x_degrees: float | None = None,
        y_degrees: float | None = None,
    ) -> RoboLightState:
        """Set plate tilt directly for an API test or initial pose.

        This API-only operation bypasses G4/G5 and represents manual positioning
        or a chosen starting pose. The UI intentionally has no direct tilt
        sliders. Omitted axes keep their current positions. Each supplied angle
        must be between -45 and +45 degrees.

        Physical displacement sensing and entry into Hijacked mode are planned
        behaviors but are not yet modeled by this method.

        Args:
            x_degrees: Optional absolute X-tilt position in degrees.
            y_degrees: Optional absolute Y-tilt position in degrees.

        Returns:
            State snapshot after applying the requested tilt values.

        Raises:
            TypeError: If an angle is not numeric.
            ValueError: If an angle is non-finite or outside -45 to +45 degrees.
        """

        with self._lock:
            with self._viewer_data_lock():
                for axis, value in (("x_tilt", x_degrees), ("y_tilt", y_degrees)):
                    if value is None:
                        continue
                    angle = self._finite_number(f"{axis}_degrees", value)
                    if not MIN_TILT_DEGREES <= angle <= MAX_TILT_DEGREES:
                        raise ValueError(
                            f"{axis}_degrees must be between {MIN_TILT_DEGREES:g} "
                            f"and {MAX_TILT_DEGREES:g}"
                        )
                    tilt_rad = math.radians(angle)
                    self.data.qpos[self._qpos[axis]] = tilt_rad

                    # A hand-moved physical tilt plate reels its cable spool and
                    # changes the absolute encoder reading on G4 or G5. Keep the
                    # simulated gear encoder consistent with that assumption.
                    spool_radius_m = self._hwdesc.spool_diameter_mm / 2000.0
                    spool_angle_rad = (
                        tilt_rad * PLATE_CABLE_LEVER_ARM_M / spool_radius_m
                    )
                    encoder_key = "g5" if axis == "x_tilt" else "g4"
                    self.data.qpos[self._qpos[encoder_key]] = spool_angle_rad
                mujoco.mj_forward(self.model, self.data)
            self._sync_visuals()
            return self._snapshot()

    def reset(self) -> RoboLightState:
        """Physically drive each encoded follower to its zero reference.

        Reset does not teleport the model or clear simulation time. It engages
        one transmission path at a fixed 100 motor-degrees/s and uses the
        absolute encoder angle on each follower gear to calculate the reset
        motion. Arm and turntable angles are interpreted modulo one revolution,
        so they take the shortest path to an equivalent zero orientation. G4/G5
        retain multi-turn position because a complete spool turn changes cable
        length and is not an equivalent tilt reset. The fixed order is G5/X
        tilt, G4/Y tilt, G3/Arm 2, G2/Arm 1, and finally G6/turntable.

        G1 and the motor rotate as required by those moves and are not themselves
        the reset authority. Hardware dimensions remain unchanged. This is the
        operation future hardware will use to leave Hijacked mode after manual
        repositioning. The current simulator has no Hijacked state or LED.

        Returns:
            State snapshot after all encoded follower gears reach zero.
        """

        with self._lock:
            gear_ratio = (
                self._hwdesc.g1_diameter_mm / self._hwdesc.follower_diameter_mm
            )
            for selector, encoder_key in self.RESET_SEQUENCE:
                # Cyclic outputs wrap to their nearest equivalent zero. Issue
                # the resulting encoder error in legal motor-sized chunks.
                for _ in range(1000):
                    with self._viewer_data_lock():
                        encoder_rad = float(
                            self.data.qpos[self._qpos[encoder_key]]
                        )
                    if selector in (Selector.X_TILT, Selector.Y_TILT):
                        # A full spool turn changes cable length, so tilt has a
                        # unique multi-turn encoder zero rather than equivalent
                        # reset orientations every revolution.
                        encoder_error_rad = encoder_rad
                    else:
                        encoder_error_rad = shortest_angular_error_to_zero(
                            encoder_rad
                        )
                    if abs(encoder_error_rad) < 1e-10:
                        break
                    motor_degrees = math.degrees(encoder_error_rad) / gear_ratio
                    motor_chunk = max(
                        MIN_MOVE_DEGREES,
                        min(MAX_MOVE_DEGREES, motor_degrees),
                    )
                    self.move_motor(
                        selector,
                        velocity=self.RESET_VELOCITY_DEG_S,
                        degrees=motor_chunk,
                    )
                else:
                    raise RuntimeError(f"Reset did not converge for {selector.value}")

                self._set_encoder_reset_exact(selector)

            self._sync_visuals(force_pip=True)
            return self._snapshot()

    def _set_encoder_reset_exact(self, selector: Selector) -> None:
        """Remove numerical residue after a physical encoder-driven reset move."""

        with self._viewer_data_lock():
            if selector is Selector.X_TILT:
                self.data.qpos[self._qpos["g5"]] = 0.0
                self.data.qpos[self._qpos["x_tilt"]] = 0.0
            elif selector is Selector.Y_TILT:
                self.data.qpos[self._qpos["g4"]] = 0.0
                self.data.qpos[self._qpos["y_tilt"]] = 0.0
            elif selector is Selector.ARM2:
                self.data.qpos[self._qpos["g3"]] = 0.0
                self.data.qpos[self._qpos["arm2_stage"]] = 0.0
                self.data.qpos[self._qpos["arm2"]] = 0.0
            elif selector is Selector.ARM1:
                self.data.qpos[self._qpos["g2"]] = 0.0
                self.data.qpos[self._qpos["arm1"]] = 0.0
            elif selector is Selector.TURNTABLE:
                self.data.qpos[self._qpos["g6"]] = 0.0
                self.data.qpos[self._qpos["turntable"]] = 0.0
            else:
                raise ValueError(f"Selector has no reset encoder: {selector.value}")

            self._apply_layout()
            mujoco.mj_forward(self.model, self.data)

    def _viewer_data_lock(self) -> Any:
        """Return the viewer's model-data lock or a no-op context manager."""

        viewer = self._viewer
        if viewer is not None and viewer.is_running():
            return viewer.lock()
        return nullcontext()

    def _sync_visuals(self, *, force_pip: bool = False) -> None:
        """Publish the latest state to the viewer and spotlight PIP."""

        viewer = self._viewer
        if viewer is not None and viewer.is_running():
            viewer.sync()
        self._sync_pip(force=force_pip)

    def _sync_pip(self, *, force: bool = False) -> None:
        """Pump PIP events and render a throttled spotlight-camera frame."""

        root = self._pip_root
        renderer = self._pip_renderer
        canvas = self._pip_canvas
        image_item = self._pip_image_item
        if root is None or renderer is None or canvas is None or image_item is None:
            return

        try:
            root.update_idletasks()
            root.update()
        except tk.TclError:
            self._dispose_pip()
            return

        # The close callback can dispose the renderer while ``root.update()`` is
        # processing events, so verify that this is still the active PIP.
        if root is not self._pip_root or renderer is not self._pip_renderer:
            return

        now = time.perf_counter()
        if not force and now - self._last_pip_time < PIP_REFRESH_SECONDS:
            return

        renderer.update_scene(self.data, camera=SPOTLIGHT_CAMERA)
        pixels = renderer.render().copy()
        photo = ImageTk.PhotoImage(Image.fromarray(pixels), master=root)
        canvas.itemconfigure(image_item, image=photo)
        self._pip_photo = photo
        self._last_pip_time = now
        root.update_idletasks()

    def _dispose_pip(self) -> None:
        """Release PIP resources without acquiring the API lock."""

        renderer = self._pip_renderer
        root = self._pip_root
        self._pip_root = None
        self._pip_canvas = None
        self._pip_image_item = None
        self._pip_photo = None
        self._pip_renderer = None
        self._last_pip_time = 0.0

        if renderer is not None:
            renderer.close()
        if root is not None:
            try:
                root.destroy()
            except tk.TclError:
                pass

    def _resolve_joint_qpos(self) -> dict[str, int]:
        qpos: dict[str, int] = {}
        for key, joint_name in self._JOINTS.items():
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if joint_id < 0:
                raise RuntimeError(f"Required MuJoCo joint is missing: {joint_name}")
            qpos[key] = int(self.model.jnt_qposadr[joint_id])
        return qpos

    def _coerce_hwdesc(self, hwdesc: HWDesc | Mapping[str, float]) -> HWDesc:
        if isinstance(hwdesc, HWDesc):
            return hwdesc
        if not isinstance(hwdesc, Mapping):
            raise TypeError("hwdesc must be an HWDesc or a mapping")

        aliases = {
            "g1_diameter_mm": "g1_diameter_mm",
            "follower_diameter_mm": "follower_diameter_mm",
            "g2_diameter_mm": "follower_diameter_mm",
            "g2_g6_diameter_mm": "follower_diameter_mm",
            "spool_diameter_mm": "spool_diameter_mm",
            "arm1_length": "arm1_length_mm",
            "arm1_length_mm": "arm1_length_mm",
            "arm2_length": "arm2_length_mm",
            "arm2_length_mm": "arm2_length_mm",
            "arm1_limit": "arm1_limit_degrees",
            "arm1_limit_degrees": "arm1_limit_degrees",
        }
        values: dict[str, float] = {}
        for key, value in hwdesc.items():
            canonical = aliases.get(str(key).lower())
            if canonical is None:
                raise ValueError(f"Unknown hardware field: {key}")
            values[canonical] = value
        return replace(self._hwdesc, **values)

    def _validate_hwdesc(self, hwdesc: HWDesc) -> None:
        for field_name in ("g1_diameter_mm", "follower_diameter_mm"):
            value = self._finite_number(field_name, getattr(hwdesc, field_name))
            if not MIN_DIAMETER_MM <= value <= MAX_DIAMETER_MM:
                raise ValueError(
                    f"{field_name} must be between {MIN_DIAMETER_MM:g} and {MAX_DIAMETER_MM:g}"
                )
        spool = self._finite_number("spool_diameter_mm", hwdesc.spool_diameter_mm)
        if not MIN_SPOOL_DIAMETER_MM <= spool <= MAX_SPOOL_DIAMETER_MM:
            raise ValueError(
                f"spool_diameter_mm must be between {MIN_SPOOL_DIAMETER_MM:g} "
                f"and {MAX_SPOOL_DIAMETER_MM:g}"
            )
        for field_name in ("arm1_length_mm", "arm2_length_mm"):
            value = self._finite_number(field_name, getattr(hwdesc, field_name))
            if not MIN_ARM_LENGTH_MM <= value <= MAX_ARM_LENGTH_MM:
                raise ValueError(
                    f"{field_name} must be between {MIN_ARM_LENGTH_MM:g} "
                    f"and {MAX_ARM_LENGTH_MM:g}"
                )
        arm1_limit = self._finite_number(
            "arm1_limit_degrees",
            hwdesc.arm1_limit_degrees,
        )
        if not MIN_ARM1_LIMIT_DEGREES <= arm1_limit <= MAX_ARM1_LIMIT_DEGREES:
            raise ValueError(
                "arm1_limit_degrees must be between "
                f"{MIN_ARM1_LIMIT_DEGREES:g} and {MAX_ARM1_LIMIT_DEGREES:g}"
            )

    @staticmethod
    def _finite_number(name: str, value: float) -> float:
        if isinstance(value, bool):
            raise TypeError(f"{name} must be a real number")
        try:
            number = float(value)
        except (TypeError, ValueError) as exc:
            raise TypeError(f"{name} must be a real number") from exc
        if not math.isfinite(number):
            raise ValueError(f"{name} must be finite")
        return number

    def _normalize_selectors(
        self,
        selector: Selector | str | Iterable[Selector | str],
    ) -> frozenset[Selector]:
        if isinstance(selector, Selector):
            raw_items: list[Selector | str] = [selector]
        elif isinstance(selector, str):
            raw_items = [
                part.strip()
                for part in selector.replace("+", ",").split(",")
                if part.strip()
            ]
        elif isinstance(selector, Iterable):
            raw_items = list(selector)
        else:
            raise TypeError("selector must be a selector name or an iterable of names")
        if not raw_items:
            raise ValueError("at least one selector is required")

        normalized: set[Selector] = set()
        for item in raw_items:
            if isinstance(item, Selector):
                selected = item
            elif isinstance(item, str):
                key = item.strip().lower().replace("->", "_").replace("-", "_").replace(" ", "_")
                while "__" in key:
                    key = key.replace("__", "_")
                selected = SELECTOR_ALIASES.get(key)
                if selected is None:
                    valid = ", ".join(item.value for item in Selector)
                    raise ValueError(f"Unknown selector {item!r}; choose from: {valid}")
            else:
                raise TypeError(f"Invalid selector value: {item!r}")
            if selected is Selector.ALL:
                normalized.update(FOLLOWER_SELECTORS)
            else:
                normalized.add(selected)
        return frozenset(normalized)

    def _output_degrees_per_motor_degree(self, selector: Selector) -> float:
        """Return the signed selected-output rotation produced by one G1 degree."""

        return output_degrees_per_motor_degree(
            selector.value,
            self._hwdesc.g1_diameter_mm,
            self._hwdesc.follower_diameter_mm,
            self._hwdesc.spool_diameter_mm,
        )

    def _capture_drive_references(self, followers: frozenset[Selector]) -> dict[str, float]:
        g1_angle = g1_angle_from_motor(self._motor_angle_rad)
        g2_target = g2_angle_from_g1(
            g1_angle,
            self._hwdesc.g1_diameter_mm,
            self._hwdesc.follower_diameter_mm,
        )
        references: dict[str, float] = {}
        if Selector.ARM1 in followers:
            references["arm1_offset"] = self.data.qpos[self._qpos["arm1"]] - g2_target
        if Selector.ARM2 in followers:
            references["arm2_offset"] = self.data.qpos[self._qpos["arm2"]] - g2_target
        if Selector.Y_TILT in followers:
            references["g4_offset"] = self.data.qpos[self._qpos["g4"]] - g2_target
            references["g4_spool"] = self.data.qpos[self._qpos["g4"]]
            references["y_tilt"] = self.data.qpos[self._qpos["y_tilt"]]
        if Selector.X_TILT in followers:
            references["g5_offset"] = self.data.qpos[self._qpos["g5"]] - g2_target
            references["g5_spool"] = self.data.qpos[self._qpos["g5"]]
            references["x_tilt"] = self.data.qpos[self._qpos["x_tilt"]]
        if Selector.TURNTABLE in followers:
            references["g6_offset"] = self.data.qpos[self._qpos["g6"]] - g2_target
            references["g6"] = self.data.qpos[self._qpos["g6"]]
            references["turntable"] = self.data.qpos[self._qpos["turntable"]]
        return references

    def _apply_motor_and_followers(
        self,
        followers: frozenset[Selector],
        references: Mapping[str, float],
    ) -> None:
        g1_angle = g1_angle_from_motor(self._motor_angle_rad)
        g2_target = g2_angle_from_g1(
            g1_angle,
            self._hwdesc.g1_diameter_mm,
            self._hwdesc.follower_diameter_mm,
        )
        self.data.qpos[self._qpos["g1"]] = g1_angle

        if Selector.ARM1 in followers:
            angle = g2_target + references["arm1_offset"]
            self.data.qpos[self._qpos["g2"]] = angle
            self.data.qpos[self._qpos["arm1"]] = angle
        if Selector.ARM2 in followers:
            angle = g2_target + references["arm2_offset"]
            self.data.qpos[self._qpos["g3"]] = angle
            self.data.qpos[self._qpos["arm2_stage"]] = angle
            self.data.qpos[self._qpos["arm2"]] = angle
        if Selector.Y_TILT in followers:
            angle = g2_target + references["g4_offset"]
            self.data.qpos[self._qpos["g4"]] = angle
            tilt = references["y_tilt"] + tilt_delta_from_spool(
                angle - references["g4_spool"],
                self._hwdesc.spool_diameter_mm,
            )
            self.data.qpos[self._qpos["y_tilt"]] = self._clamp_tilt(tilt)
        if Selector.X_TILT in followers:
            angle = g2_target + references["g5_offset"]
            self.data.qpos[self._qpos["g5"]] = angle
            tilt = references["x_tilt"] + tilt_delta_from_spool(
                angle - references["g5_spool"],
                self._hwdesc.spool_diameter_mm,
            )
            self.data.qpos[self._qpos["x_tilt"]] = self._clamp_tilt(tilt)
        if Selector.TURNTABLE in followers:
            angle = g2_target + references["g6_offset"]
            self.data.qpos[self._qpos["g6"]] = angle
            self.data.qpos[self._qpos["turntable"]] = (
                references["turntable"] + angle - references["g6"]
            )

    @staticmethod
    def _clamp_tilt(angle_rad: float) -> float:
        return max(math.radians(MIN_TILT_DEGREES), min(math.radians(MAX_TILT_DEGREES), angle_rad))

    def _apply_layout(self) -> None:
        set_gear_layout(
            self.model,
            self._hwdesc.g1_diameter_mm,
            self._hwdesc.follower_diameter_mm,
            self.data.qpos[self._qpos["arm1"]],
            spool_diameter_mm=self._hwdesc.spool_diameter_mm,
            arm1_length_mm=self._hwdesc.arm1_length_mm,
            arm2_length_mm=self._hwdesc.arm2_length_mm,
        )

    def _snapshot(self) -> RoboLightState:
        degrees = math.degrees
        qpos = self.data.qpos

        def cyclic_degrees(joint_key: str) -> float:
            return degrees(
                shortest_angular_error_to_zero(qpos[self._qpos[joint_key]])
            )

        return RoboLightState(
            motor_degrees=degrees(self._motor_angle_rad),
            g1_degrees=degrees(qpos[self._qpos["g1"]]),
            g2_degrees=cyclic_degrees("g2"),
            g3_degrees=cyclic_degrees("g3"),
            g4_degrees=degrees(qpos[self._qpos["g4"]]),
            g5_degrees=degrees(qpos[self._qpos["g5"]]),
            g6_degrees=cyclic_degrees("g6"),
            arm1_degrees=cyclic_degrees("arm1"),
            arm2_degrees=cyclic_degrees("arm2"),
            x_tilt_degrees=degrees(qpos[self._qpos["x_tilt"]]),
            y_tilt_degrees=degrees(qpos[self._qpos["y_tilt"]]),
            turntable_degrees=cyclic_degrees("turntable"),
            simulation_time_seconds=float(self.data.time),
            last_selectors=self._last_selectors,
        )
