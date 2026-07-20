"""Control API for the RoboLight robotic flashlight.

RoboLight is intended to shine a flashlight where a user points while remaining
inexpensive, mechanically expressive, and entertaining. A camera is aligned
with the flashlight so a future vision loop can find a pointer, possibly a ring
worn by the user, and center the beam on it.

The mechanical design uses one motor and a selectable transmission. G1 is the
shared drive gear. Five follower paths operate Arm 1, Arm 2, Y tilt, X tilt, and
the base turntable. Physical hardware will normally engage one path at a time,
then rely on friction to hold the axis while leaving it compliant. The simulator
also permits combined selections so transmission behavior can be explored.

This module implements the current kinematic simulation, not the complete
product control loop. Camera pointer recognition, manual-displacement sensing,
the planned Hijacked mode, and its blinking status LED are not implemented yet.

G1 always follows a requested motor move. Only selected follower paths are
coupled to G1; unselected mechanisms hold their current simulated positions.
Calls are deterministic and headless by default, making the class useful for
control development and automated tests.

Typical use::

    from scripts import HWDesc, RoboLight, Selector

    light = RoboLight(HWDesc(spool_diameter_mm=10), realtime=True)
    light.open_viewer()
    light.open_pip()
    light.move(Selector.ARM1, velocity=100, degrees=90)
    state = light.move(Selector.Y_TILT, velocity=100, degrees=-30)
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
from PIL import Image, ImageTk

from sim.launch_simple_motor_gear_controls import (
    DEFAULT_G1_DIAMETER_MM,
    DEFAULT_G2_DIAMETER_MM,
    DEFAULT_SPOOL_DIAMETER_MM,
    G1_JOINT,
    G2_JOINT,
    G3_JOINT,
    G4_JOINT,
    G5_JOINT,
    G6_JOINT,
    MAX_DIAMETER_MM,
    MAX_MOVE_DEGREES,
    MAX_SPEED_DEG_S,
    MAX_SPOOL_DIAMETER_MM,
    MAX_TILT_DEGREES,
    MIN_DIAMETER_MM,
    MIN_MOVE_DEGREES,
    MIN_SPOOL_DIAMETER_MM,
    MIN_TILT_DEGREES,
    MODEL_PATH,
    PIP_HEIGHT,
    PIP_REFRESH_SECONDS,
    PIP_WIDTH,
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
    g1_angle_from_motor,
    g2_angle_from_g1,
    set_gear_layout,
    tilt_delta_from_spool,
)


class Selector(str, Enum):
    """Transmission paths that can be coupled to G1 during a move.

    ``G1`` rotates only the shared motor input. ``ARM1`` couples G2 and the
    first timing-belt stage. ``ARM2`` couples G3 and the second arm stages.
    ``Y_TILT`` couples the G4 cable spool, ``X_TILT`` couples the G5 cable
    spool, and ``TURNTABLE`` couples G6 to the lazy-Susan base. ``ALL`` is a
    simulation convenience that engages all five follower paths together.
    """

    G1 = "g1"
    ARM1 = "arm1"
    ARM2 = "arm2"
    Y_TILT = "y_tilt"
    X_TILT = "x_tilt"
    TURNTABLE = "turntable"
    ALL = "all"


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
    """

    g1_diameter_mm: float = DEFAULT_G1_DIAMETER_MM
    follower_diameter_mm: float = DEFAULT_G2_DIAMETER_MM
    spool_diameter_mm: float = DEFAULT_SPOOL_DIAMETER_MM


@dataclass(frozen=True, slots=True)
class RoboLightState:
    """Immutable snapshot returned after each API operation.

    All angular fields are in degrees. ``motor_degrees`` and ``g1_degrees`` are
    cumulative relative to the last reset. ``g2_degrees`` through
    ``g6_degrees`` expose the follower gears. Arm, tilt, and turntable fields
    expose the useful output positions. ``simulation_time_seconds`` advances
    from requested move distance and velocity even when real-time pacing is
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

    A ``RoboLight`` instance retains its pose between calls. ``move`` always
    rotates the motor and G1; the selector determines which output path also
    moves. Once the call ends, that output remains at its new simulated
    position, representing the friction hold planned for physical hardware.

    Moves are blocking. They execute as quickly as the host allows by default
    while still advancing MuJoCo time according to ``velocity``. Pass
    ``realtime=True`` when the call should take the corresponding wall-clock
    duration.

    Args:
        hwdesc: Optional initial hardware dimensions. May be an :class:`HWDesc`
            or a mapping of hardware field names to millimeter values.
        model_path: MJCF file to load. Custom models must contain the same named
            joints as the standard RoboLight model.
        realtime: If true, sleep between simulation steps to match requested
            motor velocity in wall-clock time.
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
        ``move``, ``set_hw``, ``set_tilt``, and ``reset`` synchronize it after
        every state update. Set ``realtime=True`` on :class:`RoboLight` if moves
        should remain visible for their requested physical duration.

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
        """Apply validated gear and spool dimensions, preserving current pose.

        A mapping may provide any subset of the fields in :class:`HWDesc`.
        ``g2_diameter_mm`` and ``g2_g6_diameter_mm`` are accepted as aliases for
        ``follower_diameter_mm``.

        Changing gear diameter changes the ratio used by subsequent moves.
        Changing spool diameter changes the cable travel produced by subsequent
        G4/G5 rotation. Existing gear, arm, tilt, and turntable positions do not
        jump when the description is applied.

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

    def move(
        self,
        selector: Selector | str | Iterable[Selector | str],
        velocity: float,
        degrees: float,
    ) -> RoboLightState:
        """Perform a signed relative motor move and drive selected outputs.

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
        experiments.

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
                    self.data.qpos[self._qpos[axis]] = math.radians(angle)
                mujoco.mj_forward(self.model, self.data)
            self._sync_visuals()
            return self._snapshot()

    def reset(self) -> RoboLightState:
        """Return all positions and simulation time to the zero reference.

        Hardware dimensions remain unchanged. This is the API operation future
        hardware will use to leave Hijacked mode after the user has manually
        repositioned an arm. The current simulator has no Hijacked state or LED,
        so reset presently clears only kinematic state.

        Returns:
            Zeroed state snapshot.
        """

        with self._lock:
            with self._viewer_data_lock():
                mujoco.mj_resetData(self.model, self.data)
                self._motor_angle_rad = 0.0
                self._last_selectors = (Selector.G1.value,)
                self._apply_layout()
                mujoco.mj_forward(self.model, self.data)
            self._sync_visuals(force_pip=True)
            return self._snapshot()

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
        )

    def _snapshot(self) -> RoboLightState:
        degrees = math.degrees
        qpos = self.data.qpos
        return RoboLightState(
            motor_degrees=degrees(self._motor_angle_rad),
            g1_degrees=degrees(qpos[self._qpos["g1"]]),
            g2_degrees=degrees(qpos[self._qpos["g2"]]),
            g3_degrees=degrees(qpos[self._qpos["g3"]]),
            g4_degrees=degrees(qpos[self._qpos["g4"]]),
            g5_degrees=degrees(qpos[self._qpos["g5"]]),
            g6_degrees=degrees(qpos[self._qpos["g6"]]),
            arm1_degrees=degrees(qpos[self._qpos["arm1"]]),
            arm2_degrees=degrees(qpos[self._qpos["arm2"]]),
            x_tilt_degrees=degrees(qpos[self._qpos["x_tilt"]]),
            y_tilt_degrees=degrees(qpos[self._qpos["y_tilt"]]),
            turntable_degrees=degrees(qpos[self._qpos["turntable"]]),
            simulation_time_seconds=float(self.data.time),
            last_selectors=self._last_selectors,
        )
