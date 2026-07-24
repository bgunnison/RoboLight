from pathlib import Path
import math
import time
import tkinter as tk
from tkinter import ttk

import mujoco
import mujoco.viewer
from PIL import Image, ImageTk

try:
    from sim.generate_room_meshes import ensure_room_meshes
except ModuleNotFoundError:
    from generate_room_meshes import ensure_room_meshes


ROOT = Path(__file__).resolve().parent
MODEL_PATH = ROOT / "simple_motor_gear.xml"

G1_GEOMS = {
    "disk": "gear_disk",
    "face_mark": "face_mark",
    "spoke_mark": "spoke_mark",
}
G2_GEOMS = {
    "disk": "g2_disk",
    "face_mark": "g2_face_mark",
    "spoke_mark": "g2_spoke_mark",
}
G3_GEOMS = {
    "disk": "g3_disk",
    "face_mark": "g3_face_mark",
    "spoke_mark": "g3_spoke_mark",
}
G4_GEOMS = {
    "disk": "g4_disk",
    "face_mark": "g4_face_mark",
    "spoke_mark": "g4_spoke_mark",
}
G5_GEOMS = {
    "disk": "g5_disk",
    "face_mark": "g5_face_mark",
    "spoke_mark": "g5_spoke_mark",
}
G6_GEOMS = {
    "disk": "g6_disk",
    "face_mark": "g6_face_mark",
    "spoke_mark": "g6_spoke_mark",
}
G4_SPOOL_GEOMS = {
    "disk": "g4_spool",
    "face_mark": "g4_spool_face_mark",
    "spoke_mark": "g4_spool_spoke_mark",
}
G5_SPOOL_GEOMS = {
    "disk": "g5_spool",
    "face_mark": "g5_spool_face_mark",
    "spoke_mark": "g5_spool_spoke_mark",
}
TG1_GEOMS = {
    "disk": "tg1_disk",
    "face_mark": "tg1_face_mark",
    "spoke_mark": "tg1_spoke_mark",
}
TG2_GEOMS = {
    "disk": "tg2_disk",
    "face_mark": "tg2_face_mark",
    "spoke_mark": "tg2_spoke_mark",
}
TG3_GEOMS = {
    "disk": "tg3_disk",
    "face_mark": "tg3_face_mark",
    "spoke_mark": "tg3_spoke_mark",
}
TG4_GEOMS = {
    "disk": "tg4_disk",
    "face_mark": "tg4_face_mark",
    "spoke_mark": "tg4_spoke_mark",
}
TG5_GEOMS = {
    "disk": "tg5_disk",
    "face_mark": "tg5_face_mark",
    "spoke_mark": "tg5_spoke_mark",
}
TG6_GEOMS = {
    "disk": "tg6_disk",
    "face_mark": "tg6_face_mark",
    "spoke_mark": "tg6_spoke_mark",
}
G1_JOINT = "gear_hinge"
G2_JOINT = "g2_hinge"
G3_JOINT = "g3_hinge"
G4_JOINT = "g4_hinge"
G5_JOINT = "g5_hinge"
G6_JOINT = "g6_hinge"
TURNTABLE_JOINT = "turntable_yaw"
TG2_JOINT = "tg2_hinge"
TG4_JOINT = "tg4_hinge"
TG6_JOINT = "tg6_hinge"
PLATE_X_TILT_JOINT = "plate_x_tilt"
PLATE_Y_TILT_JOINT = "plate_y_tilt"
SPOTLIGHT_CAMERA = "spotlight_camera"
SPOTLIGHT_LIGHT = "plate_spotlight"
TARGET_ORIGIN_BODY = "target_origin_frame"
TARGET_BODY = "camera_target"
TARGET_GEOM = "camera_target_sphere"
G2_BODY = "G2"
G3_BODY = "G3"
G4_BODY = "G4"
G5_BODY = "G5"
G6_BODY = "G6"
TG2_BODY = "TG2"
TG4_BODY = "TG4"
TG6_BODY = "TG6"
TILT_PLATE_BODY = "tilt_plate_body"
G2_SITE = "G2"
G3_SITE = "G3"
G4_SITE = "G4"
G5_SITE = "G5"
G6_SITE = "G6"
TG1_SITE = "TG1"
TG2_SITE = "TG2"
TG3_SITE = "TG3"
TG4_SITE = "TG4"
TG5_SITE = "TG5"
TG6_SITE = "TG6"
G2_SUPPORT_GEOM = "g2_support"
G2_AXLE_GEOM = "g2_axle"
G3_SUPPORT_GEOM = "g3_support"
G3_AXLE_GEOM = "g3_axle"
G4_SUPPORT_GEOM = "g4_support"
G4_AXLE_GEOM = "g4_axle"
G5_SUPPORT_GEOM = "g5_support"
G5_AXLE_GEOM = "g5_axle"
G6_SUPPORT_GEOM = "g6_support"
G6_AXLE_GEOM = "g6_axle"
TURNTABLE_DISK_GEOM = "turntable_disk"
TURNTABLE_MARK_GEOM = "turntable_mark"
UPPER_TURNTABLE_DISK_GEOM = "upper_turntable_disk"
UPPER_TURNTABLE_MARK_GEOM = "upper_turntable_mark"
TG2_SUPPORT_GEOM = "tg2_support"
TG2_AXLE_GEOM = "tg2_axle"
TG4_SUPPORT_GEOM = "tg4_support"
TG4_AXLE_GEOM = "tg4_axle"
BELT_LEFT_GEOM = "belt_left"
BELT_RIGHT_GEOM = "belt_right"
BELT2_LEFT_GEOM = "belt2_left"
BELT2_RIGHT_GEOM = "belt2_right"
ARM1_GEOM = "arm1"
ARM2_GEOM = "arm2"
BELT3_LEFT_GEOM = "belt3_left"
BELT3_RIGHT_GEOM = "belt3_right"
ARM1_END_AXLE_GEOM = "arm1_end_axle"
G2_CENTER_Y = -0.090
G3_CENTER_Y = -0.130
G4_CENTER_Y = -0.170
G5_CENTER_Y = -0.210
G6_CENTER_Y = -0.250
G2_SUPPORT_Y = -0.055
G3_SUPPORT_Y = -0.095
G4_SUPPORT_Y = -0.135
G5_SUPPORT_Y = -0.175
G6_SUPPORT_Y = -0.215
G2_AXLE_Y = -0.0775
G3_AXLE_Y = -0.1175
G4_AXLE_Y = -0.1575
G5_AXLE_Y = -0.1975
G6_AXLE_Y = -0.2375
G2_SUPPORT_CENTER_Z = 0.045
AXLE_CENTER_Z = 0.080
G2_CENTER_Z = AXLE_CENTER_Z
TIMING_GEAR_Y = -0.106
TIMING_GEAR2_Y = -0.146
TG5_LOCAL_Y = -0.012
TG6_LOCAL_Y = (TIMING_GEAR2_Y + TG5_LOCAL_Y) - TIMING_GEAR_Y
BELT_CENTER_DISTANCE_M = 0.100
TG2_SUPPORT_CENTER_Z = G2_CENTER_Z + (BELT_CENTER_DISTANCE_M / 2.0)
TG2_AXLE_Y = (G2_SUPPORT_Y + TIMING_GEAR_Y) / 2.0
TG4_AXLE_Y = (G3_SUPPORT_Y + TIMING_GEAR2_Y) / 2.0
START_CAMERA_LOOKAT = (0.080, -0.010, 0.450)
START_CAMERA_DISTANCE = 1.20
START_CAMERA_AZIMUTH = 90.0
START_CAMERA_ELEVATION = 0.0

DEFAULT_G1_DIAMETER_MM = 64.0
DEFAULT_G2_DIAMETER_MM = 100.0
MIN_DIAMETER_MM = 35.0
MAX_DIAMETER_MM = 140.0
TIMING_GEAR_DIAMETER_MM = MIN_DIAMETER_MM
DEFAULT_SPEED_DEG_S = 100.0
RESET_VELOCITY_DEG_S = 100.0
MIN_SPEED_DEG_S = 0.0
MAX_SPEED_DEG_S = 720.0
DEFAULT_MOVE_DEGREES = 0.0
MIN_MOVE_DEGREES = -360.0
MAX_MOVE_DEGREES = 360.0
DEFAULT_TILT_DEGREES = 0.0
MIN_TILT_DEGREES = -45.0
MAX_TILT_DEGREES = 45.0
DEFAULT_SPOOL_DIAMETER_MM = 10.0
MIN_SPOOL_DIAMETER_MM = 5.0
MAX_SPOOL_DIAMETER_MM = 50.0
DEFAULT_ARM1_LENGTH_MM = 150.0
DEFAULT_ARM2_LENGTH_MM = 150.0
MIN_ARM_LENGTH_MM = 75.0
MAX_ARM_LENGTH_MM = 300.0
PLATE_CABLE_LEVER_ARM_M = 0.025
ARM1_LIMIT_DEGREES = 80.0
MIN_ARM1_LIMIT_DEGREES = 1.0
MAX_ARM1_LIMIT_DEGREES = 180.0
DEFAULT_BEAM_ANGLE_DEGREES = 50.0
MIN_BEAM_ANGLE_DEGREES = 10.0
MAX_BEAM_ANGLE_DEGREES = 120.0
DEFAULT_CAMERA_FOV_DEGREES = 50.0
MIN_CAMERA_FOV_DEGREES = 10.0
MAX_CAMERA_FOV_DEGREES = 170.0
DEFAULT_TARGET_X_CM = 0.0
DEFAULT_TARGET_Y_CM = -6.4
DEFAULT_TARGET_Z_CM = 55.0
MIN_TARGET_X_CM = -35.0
MAX_TARGET_X_CM = 35.0
MIN_TARGET_Y_CM = -18.0
MAX_TARGET_Y_CM = 38.0
MIN_TARGET_Z_CM = -10.0
MAX_TARGET_Z_CM = 65.0
DEFAULT_TARGET_DIAMETER_CM = 2.0
MIN_TARGET_DIAMETER_CM = 0.5
MAX_TARGET_DIAMETER_CM = 3.0
TARGET_COLORS = {
    "red": (0.90, 0.04, 0.03, 1.0),
    "green": (0.05, 0.78, 0.16, 1.0),
    "blue": (0.08, 0.28, 0.92, 1.0),
    "yellow": (0.95, 0.82, 0.08, 1.0),
    "orange": (0.95, 0.38, 0.05, 1.0),
    "cyan": (0.05, 0.82, 0.88, 1.0),
    "magenta": (0.88, 0.08, 0.74, 1.0),
    "white": (0.95, 0.95, 0.95, 1.0),
}
UPPER_TURNTABLE_TOP_Z_M = 0.170
ARM_LINK_HALF_WIDTH_M = 0.006
TILT_PLATE_HALF_LENGTH_M = 0.025
TILT_PLATE_HALF_THICKNESS_M = 0.003
ARM_CONSTRAINT_ARM1_LIMIT = "arm1_limit"
ARM_CONSTRAINT_PLATFORM_COLLISION = "platform_collision"
TURNTABLE_CENTER_Y = -0.118
TURNTABLE_REAR_Y = 0.025
TURNTABLE_FRONT_Y = -0.260
TURNTABLE_RADIAL_MARGIN_M = 0.015
MOTOR_BODY_Y = -0.035
UPPER_TURNTABLE_CENTER_Y_IN_MOTOR = TURNTABLE_CENTER_Y - MOTOR_BODY_Y
PIP_WIDTH = 320
PIP_HEIGHT = 240
PIP_REFRESH_SECONDS = 0.10
MOTOR_TO_G1_RATIO = 1.0


def format_degrees(value):
    text = f"{value:.3f}".rstrip("0").rstrip(".")
    return "0" if text == "-0" else text


def format_number(value):
    text = f"{value:.3f}".rstrip("0").rstrip(".")
    return "0" if text == "-0" else text


def radius_m(diameter_mm):
    return diameter_mm / 2000.0


def normalize_target_color(color):
    """Return an RGBA tuple for a named color or RGB/RGBA iterable."""

    if isinstance(color, str):
        normalized = TARGET_COLORS.get(color.strip().lower())
        if normalized is None:
            valid = ", ".join(TARGET_COLORS)
            raise ValueError(f"Unknown target color {color!r}; choose from: {valid}")
        return normalized
    try:
        components = tuple(float(component) for component in color)
    except (TypeError, ValueError) as exc:
        raise TypeError("target color must be a name or RGB/RGBA iterable") from exc
    if len(components) == 3:
        components += (1.0,)
    if len(components) != 4 or any(
        not math.isfinite(component) or not 0.0 <= component <= 1.0
        for component in components
    ):
        raise ValueError("target color components must be finite values from 0 to 1")
    return components


def set_spotlight_beam_angle(model, beam_angle_degrees):
    """Set the full spotlight cone angle without changing the camera FOV."""

    light_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_LIGHT,
        SPOTLIGHT_LIGHT,
    )
    model.light_cutoff[light_id] = beam_angle_degrees / 2.0


def set_spotlight_camera_fov(model, camera_fov_degrees):
    """Set the fixed spotlight-camera vertical field of view."""

    camera_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_CAMERA,
        SPOTLIGHT_CAMERA,
    )
    model.cam_fovy[camera_id] = camera_fov_degrees


def set_target_layout(
    model,
    g1_diameter_mm,
    follower_diameter_mm,
    x_cm,
    y_cm,
    z_cm,
    diameter_cm=DEFAULT_TARGET_DIAMETER_CM,
    color="red",
):
    """Position the room-fixed camera target relative to the Arm 1 start."""

    target_origin_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        TARGET_ORIGIN_BODY,
    )
    target_body_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        TARGET_BODY,
    )
    target_geom_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_GEOM,
        TARGET_GEOM,
    )
    model.body_pos[target_origin_id] = (
        radius_m(g1_diameter_mm) + radius_m(follower_diameter_mm),
        TIMING_GEAR_Y,
        G2_CENTER_Z + BELT_CENTER_DISTANCE_M,
    )
    model.body_pos[target_body_id] = (
        x_cm / 100.0,
        y_cm / 100.0,
        z_cm / 100.0,
    )
    model.geom_size[target_geom_id, 0] = diameter_cm / 200.0
    model.geom_rgba[target_geom_id] = normalize_target_color(color)


def turntable_radius_m(g1_diameter_mm, follower_diameter_mm):
    g1_radius = radius_m(g1_diameter_mm)
    follower_radius = radius_m(follower_diameter_mm)
    follower_center_x = g1_radius + follower_radius
    left_x_extent = follower_center_x + max(g1_radius, 0.050)
    right_x_extent = follower_radius
    y_extent = max(
        TURNTABLE_REAR_Y - TURNTABLE_CENTER_Y,
        TURNTABLE_CENTER_Y - TURNTABLE_FRONT_Y,
    )
    footprint_radius = math.hypot(max(left_x_extent, right_x_extent), y_extent)
    return footprint_radius + TURNTABLE_RADIAL_MARGIN_M


def clamp_tilt_rad(angle_rad):
    return max(math.radians(MIN_TILT_DEGREES), min(math.radians(MAX_TILT_DEGREES), angle_rad))


def tilt_delta_from_spool(spool_angle_delta_rad, spool_diameter_mm):
    cable_travel_m = spool_angle_delta_rad * radius_m(spool_diameter_mm)
    return cable_travel_m / PLATE_CABLE_LEVER_ARM_M


def arm_configuration_violation(
    arm1_degrees,
    arm2_degrees,
    arm1_length_mm,
    arm2_length_mm,
    arm1_limit_degrees=ARM1_LIMIT_DEGREES,
):
    """Return the arm constraint violated by one two-link configuration."""

    if abs(arm1_degrees) > arm1_limit_degrees + 1e-9:
        return ARM_CONSTRAINT_ARM1_LIMIT

    arm1_angle = math.radians(arm1_degrees)
    arm2_world_angle = math.radians(arm1_degrees + arm2_degrees)
    arm1_length_m = arm1_length_mm / 1000.0
    arm2_length_m = arm2_length_mm / 1000.0
    pivot_z = G2_CENTER_Z + BELT_CENTER_DISTANCE_M
    elbow_z = pivot_z + (arm1_length_m * math.cos(arm1_angle))
    endpoint_z = elbow_z + (arm2_length_m * math.cos(arm2_world_angle))

    arm1_low_z = min(pivot_z, elbow_z) - (
        ARM_LINK_HALF_WIDTH_M * abs(math.sin(arm1_angle))
    )
    arm2_low_z = min(elbow_z, endpoint_z) - (
        ARM_LINK_HALF_WIDTH_M * abs(math.sin(arm2_world_angle))
    )
    plate_center_z = endpoint_z + (
        TILT_PLATE_HALF_THICKNESS_M * math.cos(arm2_world_angle)
    )
    plate_low_z = (
        plate_center_z
        - (TILT_PLATE_HALF_LENGTH_M * abs(math.sin(arm2_world_angle)))
        - (TILT_PLATE_HALF_THICKNESS_M * abs(math.cos(arm2_world_angle)))
    )
    if min(arm1_low_z, arm2_low_z, plate_low_z) < UPPER_TURNTABLE_TOP_Z_M - 1e-9:
        return ARM_CONSTRAINT_PLATFORM_COLLISION
    return None


def arm_motion_violation(
    arm1_start_degrees,
    arm2_start_degrees,
    arm1_delta_degrees,
    arm2_delta_degrees,
    arm1_length_mm,
    arm2_length_mm,
    arm1_limit_degrees=ARM1_LIMIT_DEGREES,
):
    """Check a linear joint-space move against the infinite upper platform."""

    largest_delta = max(abs(arm1_delta_degrees), abs(arm2_delta_degrees))
    sample_count = max(1, math.ceil(largest_delta / 0.25))
    for sample in range(sample_count + 1):
        fraction = sample / sample_count
        violation = arm_configuration_violation(
            arm1_start_degrees + (arm1_delta_degrees * fraction),
            arm2_start_degrees + (arm2_delta_degrees * fraction),
            arm1_length_mm,
            arm2_length_mm,
            arm1_limit_degrees,
        )
        if violation is not None:
            return violation
    return None


def rotate_y(v, angle_rad):
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return ((c * v[0]) + (s * v[2]), v[1], (-s * v[0]) + (c * v[2]))


def set_arm_lengths(model, arm1_length_mm, arm2_length_mm):
    """Resize both links and reposition every child attached to their ends."""

    arm1_length_m = arm1_length_mm / 1000.0
    arm2_length_m = arm2_length_mm / 1000.0

    arm1_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, ARM1_GEOM)
    model.geom_pos[arm1_geom_id, 2] = arm1_length_m / 2.0
    model.geom_size[arm1_geom_id, 2] = arm1_length_m / 2.0

    for belt_name in (BELT3_LEFT_GEOM, BELT3_RIGHT_GEOM):
        belt_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, belt_name)
        model.geom_pos[belt_id, 2] = arm1_length_m / 2.0
        model.geom_size[belt_id, 2] = arm1_length_m / 2.0

    arm1_end_axle_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_GEOM,
        ARM1_END_AXLE_GEOM,
    )
    model.geom_pos[arm1_end_axle_id, 2] = arm1_length_m

    tg6_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, TG6_BODY)
    model.body_pos[tg6_body_id, 2] = arm1_length_m

    arm2_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, ARM2_GEOM)
    model.geom_pos[arm2_geom_id, 2] = arm2_length_m / 2.0
    model.geom_size[arm2_geom_id, 2] = arm2_length_m / 2.0

    tilt_plate_body_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        TILT_PLATE_BODY,
    )
    model.body_pos[tilt_plate_body_id, 2] = arm2_length_m


def set_arm1_belt_layout(model, g2_center_x, arm_angle_rad, arm1_length_mm):
    timing_radius = radius_m(TIMING_GEAR_DIAMETER_MM)
    tg2_center_z = G2_CENTER_Z + BELT_CENTER_DISTANCE_M
    tg5_center = (g2_center_x, TIMING_GEAR2_Y + TG5_LOCAL_Y, tg2_center_z)
    arm1_length_m = arm1_length_mm / 1000.0
    tg6_offset = rotate_y((0.0, TG6_LOCAL_Y, arm1_length_m), arm_angle_rad)
    tg6_center = (
        g2_center_x + tg6_offset[0],
        TIMING_GEAR_Y + tg6_offset[1],
        tg2_center_z + tg6_offset[2],
    )

    tg5_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, TG5_SITE)
    if tg5_site_id >= 0:
        model.site_pos[tg5_site_id, 0] = tg5_center[0] + timing_radius + 0.020
        model.site_pos[tg5_site_id, 1] = tg5_center[1]
        model.site_pos[tg5_site_id, 2] = tg5_center[2] + timing_radius + 0.012

    tg6_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, TG6_SITE)
    if tg6_site_id >= 0:
        model.site_pos[tg6_site_id, 0] = tg6_center[0] + timing_radius + 0.020
        model.site_pos[tg6_site_id, 1] = tg6_center[1]
        model.site_pos[tg6_site_id, 2] = tg6_center[2] + timing_radius + 0.012


def set_gear_diameter(model, geom_names, diameter_mm):
    radius = radius_m(diameter_mm)

    gear_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_names["disk"])
    face_mark_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_names["face_mark"])
    spoke_mark_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_names["spoke_mark"])

    model.geom_size[gear_id, 0] = radius
    model.geom_size[face_mark_id, 2] = radius * 0.44
    model.geom_size[spoke_mark_id, 0] = radius * 0.80
    model.geom_pos[face_mark_id, 2] = radius * 0.40


def set_gear_layout(
    model,
    g1_diameter_mm,
    g2_diameter_mm,
    arm_angle_rad=0.0,
    spool_diameter_mm=DEFAULT_SPOOL_DIAMETER_MM,
    arm1_length_mm=DEFAULT_ARM1_LENGTH_MM,
    arm2_length_mm=DEFAULT_ARM2_LENGTH_MM,
):
    g1_radius = radius_m(g1_diameter_mm)
    g2_radius = radius_m(g2_diameter_mm)
    g2_center_x = g1_radius + g2_radius
    turntable_radius = turntable_radius_m(g1_diameter_mm, g2_diameter_mm)

    set_gear_diameter(model, G1_GEOMS, g1_diameter_mm)
    set_gear_diameter(model, G2_GEOMS, g2_diameter_mm)
    set_gear_diameter(model, G3_GEOMS, g2_diameter_mm)
    set_gear_diameter(model, G4_GEOMS, g2_diameter_mm)
    set_gear_diameter(model, G5_GEOMS, g2_diameter_mm)
    set_gear_diameter(model, G6_GEOMS, g2_diameter_mm)
    set_gear_diameter(model, G4_SPOOL_GEOMS, spool_diameter_mm)
    set_gear_diameter(model, G5_SPOOL_GEOMS, spool_diameter_mm)
    set_gear_diameter(model, TG1_GEOMS, TIMING_GEAR_DIAMETER_MM)
    set_gear_diameter(model, TG2_GEOMS, TIMING_GEAR_DIAMETER_MM)
    set_gear_diameter(model, TG3_GEOMS, TIMING_GEAR_DIAMETER_MM)
    set_gear_diameter(model, TG4_GEOMS, TIMING_GEAR_DIAMETER_MM)
    set_gear_diameter(model, TG5_GEOMS, TIMING_GEAR_DIAMETER_MM)
    set_gear_diameter(model, TG6_GEOMS, TIMING_GEAR_DIAMETER_MM)
    set_arm_lengths(model, arm1_length_mm, arm2_length_mm)

    turntable_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, TURNTABLE_JOINT)
    model.jnt_pos[turntable_joint_id, 0] = g2_center_x
    model.jnt_pos[turntable_joint_id, 1] = TURNTABLE_CENTER_Y

    turntable_disk_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_GEOM,
        TURNTABLE_DISK_GEOM,
    )
    model.geom_pos[turntable_disk_id, 0] = g2_center_x
    model.geom_pos[turntable_disk_id, 1] = TURNTABLE_CENTER_Y
    model.geom_size[turntable_disk_id, 0] = turntable_radius

    turntable_mark_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_GEOM,
        TURNTABLE_MARK_GEOM,
    )
    model.geom_pos[turntable_mark_id, 0] = g2_center_x + (turntable_radius * 0.55)
    model.geom_pos[turntable_mark_id, 1] = TURNTABLE_CENTER_Y
    model.geom_size[turntable_mark_id, 0] = turntable_radius * 0.32

    # The upper plate is parented to the motor body. Express the shared
    # turntable center in that body's local frame while matching the lower
    # disk's dynamically calculated radius and radial mark.
    upper_turntable_disk_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_GEOM,
        UPPER_TURNTABLE_DISK_GEOM,
    )
    model.geom_pos[upper_turntable_disk_id, 0] = g2_center_x
    model.geom_pos[upper_turntable_disk_id, 1] = UPPER_TURNTABLE_CENTER_Y_IN_MOTOR
    model.geom_size[upper_turntable_disk_id, 0] = turntable_radius

    upper_turntable_mark_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_GEOM,
        UPPER_TURNTABLE_MARK_GEOM,
    )
    model.geom_pos[upper_turntable_mark_id, 0] = (
        g2_center_x + (turntable_radius * 0.55)
    )
    model.geom_pos[upper_turntable_mark_id, 1] = UPPER_TURNTABLE_CENTER_Y_IN_MOTOR
    model.geom_size[upper_turntable_mark_id, 0] = turntable_radius * 0.32

    follower_layouts = (
        (G2_BODY, G2_CENTER_Y, G2_SUPPORT_GEOM, G2_SUPPORT_Y, G2_AXLE_GEOM, G2_AXLE_Y),
        (G3_BODY, G3_CENTER_Y, G3_SUPPORT_GEOM, G3_SUPPORT_Y, G3_AXLE_GEOM, G3_AXLE_Y),
        (G4_BODY, G4_CENTER_Y, G4_SUPPORT_GEOM, G4_SUPPORT_Y, G4_AXLE_GEOM, G4_AXLE_Y),
        (G5_BODY, G5_CENTER_Y, G5_SUPPORT_GEOM, G5_SUPPORT_Y, G5_AXLE_GEOM, G5_AXLE_Y),
        (G6_BODY, G6_CENTER_Y, G6_SUPPORT_GEOM, G6_SUPPORT_Y, G6_AXLE_GEOM, G6_AXLE_Y),
    )
    for body_name, center_y, support_name, support_y, axle_name, axle_y in follower_layouts:
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        model.body_pos[body_id, 0] = g2_center_x
        model.body_pos[body_id, 1] = center_y
        model.body_pos[body_id, 2] = G2_CENTER_Z

        support_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, support_name)
        if support_id >= 0:
            model.geom_pos[support_id, 0] = g2_center_x
            model.geom_pos[support_id, 1] = support_y
            model.geom_pos[support_id, 2] = G2_SUPPORT_CENTER_Z

        axle_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, axle_name)
        if axle_id >= 0:
            model.geom_pos[axle_id, 0] = g2_center_x
            model.geom_pos[axle_id, 1] = axle_y
            model.geom_pos[axle_id, 2] = G2_CENTER_Z

    timing_radius = radius_m(TIMING_GEAR_DIAMETER_MM)
    tg2_center_z = G2_CENTER_Z + BELT_CENTER_DISTANCE_M
    tg2_support_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, TG2_SUPPORT_GEOM)
    if tg2_support_id >= 0:
        model.geom_pos[tg2_support_id, 0] = g2_center_x
        model.geom_pos[tg2_support_id, 1] = G2_SUPPORT_Y
        model.geom_pos[tg2_support_id, 2] = TG2_SUPPORT_CENTER_Z

    tg2_axle_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, TG2_AXLE_GEOM)
    if tg2_axle_id >= 0:
        model.geom_pos[tg2_axle_id, 0] = g2_center_x
        model.geom_pos[tg2_axle_id, 1] = TG2_AXLE_Y
        model.geom_pos[tg2_axle_id, 2] = tg2_center_z

    tg4_support_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, TG4_SUPPORT_GEOM)
    if tg4_support_id >= 0:
        model.geom_pos[tg4_support_id, 0] = g2_center_x
        model.geom_pos[tg4_support_id, 1] = G3_SUPPORT_Y
        model.geom_pos[tg4_support_id, 2] = TG2_SUPPORT_CENTER_Z

    tg4_axle_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, TG4_AXLE_GEOM)
    if tg4_axle_id >= 0:
        model.geom_pos[tg4_axle_id, 0] = g2_center_x
        model.geom_pos[tg4_axle_id, 1] = TG4_AXLE_Y
        model.geom_pos[tg4_axle_id, 2] = tg2_center_z

    tg2_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, TG2_BODY)
    if tg2_body_id >= 0:
        model.body_pos[tg2_body_id, 0] = g2_center_x
        model.body_pos[tg2_body_id, 1] = TIMING_GEAR_Y
        model.body_pos[tg2_body_id, 2] = tg2_center_z

    tg4_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, TG4_BODY)
    if tg4_body_id >= 0:
        model.body_pos[tg4_body_id, 0] = g2_center_x
        model.body_pos[tg4_body_id, 1] = TIMING_GEAR2_Y
        model.body_pos[tg4_body_id, 2] = tg2_center_z

    belt_center_z = G2_CENTER_Z + (BELT_CENTER_DISTANCE_M / 2.0)
    for belt_geom_name, x_offset in (
        (BELT_LEFT_GEOM, -timing_radius),
        (BELT_RIGHT_GEOM, timing_radius),
    ):
        belt_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, belt_geom_name)
        if belt_id >= 0:
            model.geom_pos[belt_id, 0] = g2_center_x + x_offset
            model.geom_pos[belt_id, 1] = TIMING_GEAR_Y
            model.geom_pos[belt_id, 2] = belt_center_z
            model.geom_size[belt_id, 2] = BELT_CENTER_DISTANCE_M / 2.0

    for belt_geom_name, x_offset in (
        (BELT2_LEFT_GEOM, -timing_radius),
        (BELT2_RIGHT_GEOM, timing_radius),
    ):
        belt_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, belt_geom_name)
        if belt_id >= 0:
            model.geom_pos[belt_id, 0] = g2_center_x + x_offset
            model.geom_pos[belt_id, 1] = TIMING_GEAR2_Y
            model.geom_pos[belt_id, 2] = belt_center_z
            model.geom_size[belt_id, 2] = BELT_CENTER_DISTANCE_M / 2.0

    for site_name, center_y in (
        (G2_SITE, G2_CENTER_Y),
        (G3_SITE, G3_CENTER_Y),
        (G4_SITE, G4_CENTER_Y),
        (G5_SITE, G5_CENTER_Y),
        (G6_SITE, G6_CENTER_Y),
    ):
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        if site_id >= 0:
            model.site_pos[site_id, 0] = g2_center_x + g2_radius + 0.038
            model.site_pos[site_id, 1] = center_y - 0.015
            model.site_pos[site_id, 2] = G2_CENTER_Z + 0.045

    tg1_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, TG1_SITE)
    if tg1_site_id >= 0:
        model.site_pos[tg1_site_id, 0] = g2_center_x + timing_radius + 0.020
        model.site_pos[tg1_site_id, 1] = TIMING_GEAR_Y
        model.site_pos[tg1_site_id, 2] = G2_CENTER_Z + timing_radius + 0.012

    tg2_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, TG2_SITE)
    if tg2_site_id >= 0:
        model.site_pos[tg2_site_id, 0] = g2_center_x + timing_radius + 0.020
        model.site_pos[tg2_site_id, 1] = TIMING_GEAR_Y
        model.site_pos[tg2_site_id, 2] = tg2_center_z + timing_radius + 0.012

    tg3_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, TG3_SITE)
    if tg3_site_id >= 0:
        model.site_pos[tg3_site_id, 0] = g2_center_x + timing_radius + 0.020
        model.site_pos[tg3_site_id, 1] = TIMING_GEAR2_Y
        model.site_pos[tg3_site_id, 2] = G2_CENTER_Z + timing_radius + 0.012

    tg4_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, TG4_SITE)
    if tg4_site_id >= 0:
        model.site_pos[tg4_site_id, 0] = g2_center_x + timing_radius + 0.020
        model.site_pos[tg4_site_id, 1] = TIMING_GEAR2_Y
        model.site_pos[tg4_site_id, 2] = tg2_center_z + timing_radius + 0.012

    set_arm1_belt_layout(model, g2_center_x, arm_angle_rad, arm1_length_mm)


def g1_angle_from_motor(motor_angle_rad):
    return motor_angle_rad * MOTOR_TO_G1_RATIO


def g2_angle_from_g1(g1_angle_rad, g1_diameter_mm, g2_diameter_mm):
    return -g1_angle_rad * (g1_diameter_mm / g2_diameter_mm)


def output_degrees_per_motor_degree(
    axis_key,
    g1_diameter_mm,
    follower_diameter_mm,
    spool_diameter_mm,
):
    """Return signed selected-output degrees produced by one G1 degree."""

    if axis_key == "g1":
        return 1.0
    follower_per_motor = -(g1_diameter_mm / follower_diameter_mm)
    if axis_key in ("x_tilt", "y_tilt"):
        spool_radius_m = spool_diameter_mm / 2000.0
        return follower_per_motor * (
            spool_radius_m / PLATE_CABLE_LEVER_ARM_M
        )
    if axis_key in ("arm1", "arm2", "turntable"):
        return follower_per_motor
    raise ValueError(f"Unknown output axis: {axis_key}")


def shortest_angular_error_to_zero(angle_rad):
    """Return the signed encoder error to zero within one revolution.

    Absolute rotary encoders repeat at 360 degrees. Wrapping the current angle
    to [-pi, pi] makes a physical reset choose the shorter direction instead of
    unwinding every accumulated revolution.
    """

    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def main():
    ensure_room_meshes()
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    g1_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, G1_JOINT)
    g2_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, G2_JOINT)
    g3_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, G3_JOINT)
    g4_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, G4_JOINT)
    g5_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, G5_JOINT)
    g6_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, G6_JOINT)
    turntable_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, TURNTABLE_JOINT)
    tg2_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, TG2_JOINT)
    tg4_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, TG4_JOINT)
    tg6_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, TG6_JOINT)
    plate_x_tilt_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, PLATE_X_TILT_JOINT)
    plate_y_tilt_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, PLATE_Y_TILT_JOINT)
    g1_qpos_id = model.jnt_qposadr[g1_joint_id]
    g2_qpos_id = model.jnt_qposadr[g2_joint_id]
    g3_qpos_id = model.jnt_qposadr[g3_joint_id]
    g4_qpos_id = model.jnt_qposadr[g4_joint_id]
    g5_qpos_id = model.jnt_qposadr[g5_joint_id]
    g6_qpos_id = model.jnt_qposadr[g6_joint_id]
    turntable_qpos_id = model.jnt_qposadr[turntable_joint_id]
    tg2_qpos_id = model.jnt_qposadr[tg2_joint_id]
    tg4_qpos_id = model.jnt_qposadr[tg4_joint_id]
    tg6_qpos_id = model.jnt_qposadr[tg6_joint_id]
    plate_x_tilt_qpos_id = model.jnt_qposadr[plate_x_tilt_joint_id]
    plate_y_tilt_qpos_id = model.jnt_qposadr[plate_y_tilt_joint_id]

    root = tk.Tk()
    root.title("Simple Motor Gear Controls")
    root.resizable(False, False)

    g1_diameter_slider_var = tk.DoubleVar(value=DEFAULT_G1_DIAMETER_MM)
    g1_diameter_entry_var = tk.StringVar(value=format_number(DEFAULT_G1_DIAMETER_MM))
    g2_diameter_slider_var = tk.DoubleVar(value=DEFAULT_G2_DIAMETER_MM)
    g2_diameter_entry_var = tk.StringVar(value=format_number(DEFAULT_G2_DIAMETER_MM))
    spool_diameter_slider_var = tk.DoubleVar(value=DEFAULT_SPOOL_DIAMETER_MM)
    spool_diameter_entry_var = tk.StringVar(value=format_number(DEFAULT_SPOOL_DIAMETER_MM))
    arm1_length_slider_var = tk.DoubleVar(value=DEFAULT_ARM1_LENGTH_MM)
    arm1_length_entry_var = tk.StringVar(value=format_number(DEFAULT_ARM1_LENGTH_MM))
    arm2_length_slider_var = tk.DoubleVar(value=DEFAULT_ARM2_LENGTH_MM)
    arm2_length_entry_var = tk.StringVar(value=format_number(DEFAULT_ARM2_LENGTH_MM))
    beam_angle_slider_var = tk.DoubleVar(value=DEFAULT_BEAM_ANGLE_DEGREES)
    beam_angle_entry_var = tk.StringVar(value=format_number(DEFAULT_BEAM_ANGLE_DEGREES))
    target_x_slider_var = tk.DoubleVar(value=DEFAULT_TARGET_X_CM)
    target_x_entry_var = tk.StringVar(value=format_number(DEFAULT_TARGET_X_CM))
    target_y_slider_var = tk.DoubleVar(value=DEFAULT_TARGET_Y_CM)
    target_y_entry_var = tk.StringVar(value=format_number(DEFAULT_TARGET_Y_CM))
    target_z_slider_var = tk.DoubleVar(value=DEFAULT_TARGET_Z_CM)
    target_z_entry_var = tk.StringVar(value=format_number(DEFAULT_TARGET_Z_CM))
    target_diameter_slider_var = tk.DoubleVar(value=DEFAULT_TARGET_DIAMETER_CM)
    target_diameter_entry_var = tk.StringVar(
        value=format_number(DEFAULT_TARGET_DIAMETER_CM)
    )
    target_color_var = tk.StringVar(value="red")
    speed_var = tk.DoubleVar(value=DEFAULT_SPEED_DEG_S)
    rotation_slider_var = tk.DoubleVar(value=DEFAULT_MOVE_DEGREES)
    rotation_entry_var = tk.StringVar(value=format_degrees(DEFAULT_MOVE_DEGREES))
    output_rotation_var = tk.BooleanVar(value=True)
    tilt_x_var = tk.DoubleVar(value=DEFAULT_TILT_DEGREES)
    tilt_y_var = tk.DoubleVar(value=DEFAULT_TILT_DEGREES)
    arm1_engaged_var = tk.BooleanVar(value=True)
    arm2_engaged_var = tk.BooleanVar(value=False)
    g4_engaged_var = tk.BooleanVar(value=False)
    g5_engaged_var = tk.BooleanVar(value=False)
    g6_engaged_var = tk.BooleanVar(value=False)
    motor_angle_rad = [0.0]
    remaining_motor_move_rad = [0.0]
    arm1_drive_offset_rad = [0.0]
    arm2_drive_offset_rad = [0.0]
    g4_drive_offset_rad = [0.0]
    g5_drive_offset_rad = [0.0]
    g6_drive_offset_rad = [0.0]
    g4_spool_reference_rad = [0.0]
    g5_spool_reference_rad = [0.0]
    y_tilt_reference_rad = [0.0]
    x_tilt_reference_rad = [0.0]
    g6_reference_rad = [0.0]
    turntable_reference_rad = [0.0]
    last_spool_diameter_mm = [DEFAULT_SPOOL_DIAMETER_MM]
    arm1_was_engaged = [arm1_engaged_var.get()]
    arm2_was_engaged = [arm2_engaged_var.get()]
    g4_was_engaged = [g4_engaged_var.get()]
    g5_was_engaged = [g5_engaged_var.get()]
    g6_was_engaged = [g6_engaged_var.get()]
    reset_active = [False]
    reset_queue = []
    reset_current_axis = [None]

    frame = ttk.Frame(root, padding=12)
    frame.grid(row=0, column=0, sticky="nsew")

    pip_frame = ttk.LabelFrame(frame, text="Spotlight camera PIP", padding=6)
    pip_frame.grid(row=0, column=2, rowspan=23, sticky="n", padx=(16, 0))
    pip_canvas = tk.Canvas(
        pip_frame,
        width=PIP_WIDTH,
        height=PIP_HEIGHT,
        background="black",
        highlightthickness=0,
    )
    pip_canvas.grid(row=0, column=0)
    pip_image_item = pip_canvas.create_image(0, 0, anchor="nw")
    pip_description_var = tk.StringVar(
        value=(
            f"{format_number(DEFAULT_CAMERA_FOV_DEGREES)} deg fixed camera FOV; "
            f"{format_number(DEFAULT_BEAM_ANGLE_DEGREES)} deg spotlight beam"
        )
    )
    ttk.Label(pip_frame, textvariable=pip_description_var).grid(
        row=1,
        column=0,
        pady=(6, 0),
    )

    beam_frame = ttk.LabelFrame(pip_frame, text="Spotlight", padding=6)
    beam_frame.grid(row=2, column=0, sticky="ew", pady=(10, 0))
    beam_frame.columnconfigure(0, weight=1)
    ttk.Label(beam_frame, text="Full beam angle (deg)").grid(
        row=0,
        column=0,
        sticky="w",
    )
    beam_angle_entry = ttk.Entry(
        beam_frame,
        textvariable=beam_angle_entry_var,
        width=9,
    )
    beam_angle_entry.grid(row=0, column=1, sticky="e")
    beam_angle_slider = ttk.Scale(
        beam_frame,
        from_=MIN_BEAM_ANGLE_DEGREES,
        to=MAX_BEAM_ANGLE_DEGREES,
        variable=beam_angle_slider_var,
        orient="horizontal",
        length=295,
    )
    beam_angle_slider.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(2, 0))

    target_frame = ttk.LabelFrame(
        pip_frame,
        text="Camera target — origin at Arm 1 start",
        padding=6,
    )
    target_frame.grid(row=3, column=0, sticky="ew", pady=(10, 0))
    target_frame.columnconfigure(0, weight=1)

    def add_target_control(row, label_text, entry_var, slider_var, minimum, maximum):
        ttk.Label(target_frame, text=label_text).grid(row=row, column=0, sticky="w")
        entry = ttk.Entry(target_frame, textvariable=entry_var, width=9)
        entry.grid(row=row, column=1, sticky="e")
        slider = ttk.Scale(
            target_frame,
            from_=minimum,
            to=maximum,
            variable=slider_var,
            orient="horizontal",
            length=295,
        )
        slider.grid(
            row=row + 1,
            column=0,
            columnspan=2,
            sticky="ew",
            pady=(1, 5),
        )
        return entry, slider

    target_x_entry, target_x_slider = add_target_control(
        0,
        "Target X (cm)",
        target_x_entry_var,
        target_x_slider_var,
        MIN_TARGET_X_CM,
        MAX_TARGET_X_CM,
    )
    target_y_entry, target_y_slider = add_target_control(
        2,
        "Target Y (cm)",
        target_y_entry_var,
        target_y_slider_var,
        MIN_TARGET_Y_CM,
        MAX_TARGET_Y_CM,
    )
    target_z_entry, target_z_slider = add_target_control(
        4,
        "Target Z (cm)",
        target_z_entry_var,
        target_z_slider_var,
        MIN_TARGET_Z_CM,
        MAX_TARGET_Z_CM,
    )
    target_diameter_entry, target_diameter_slider = add_target_control(
        6,
        "Sphere diameter (cm)",
        target_diameter_entry_var,
        target_diameter_slider_var,
        MIN_TARGET_DIAMETER_CM,
        MAX_TARGET_DIAMETER_CM,
    )
    ttk.Label(target_frame, text="Target color").grid(row=8, column=0, sticky="w")
    target_color_combo = ttk.Combobox(
        target_frame,
        textvariable=target_color_var,
        values=tuple(TARGET_COLORS),
        state="readonly",
        width=10,
    )
    target_color_combo.grid(row=8, column=1, sticky="e")

    speed_label = ttk.Label(frame, width=24)
    speed_label.grid(row=0, column=0, columnspan=2, sticky="w")
    speed_slider = ttk.Scale(
        frame,
        from_=MIN_SPEED_DEG_S,
        to=MAX_SPEED_DEG_S,
        variable=speed_var,
        orient="horizontal",
        length=260,
    )
    speed_slider.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(2, 12))

    rotation_label = ttk.Label(frame, text="Selected output rotation (deg)")
    rotation_label.grid(row=2, column=0, sticky="w")
    rotation_entry = ttk.Entry(frame, textvariable=rotation_entry_var, width=12)
    rotation_entry.grid(row=2, column=1, sticky="e")
    rotation_slider = ttk.Scale(
        frame,
        from_=MIN_MOVE_DEGREES,
        to=MAX_MOVE_DEGREES,
        variable=rotation_slider_var,
        orient="horizontal",
        length=260,
    )
    rotation_slider.grid(row=3, column=0, columnspan=2, sticky="ew", pady=(2, 12))

    output_rotation_check = ttk.Checkbutton(
        frame,
        text="Rotation targets selected output",
        variable=output_rotation_var,
    )
    output_rotation_check.grid(row=4, column=0, columnspan=2, sticky="w", pady=(0, 6))

    status_label = ttk.Label(frame, width=44)
    status_label.grid(row=5, column=0, columnspan=2, sticky="w", pady=(0, 8))

    selector_vars = (
        arm1_engaged_var,
        arm2_engaged_var,
        g4_engaged_var,
        g5_engaged_var,
        g6_engaged_var,
    )

    def select_exclusively_for_output(active_var):
        if output_rotation_var.get() and active_var.get():
            for selector_var in selector_vars:
                if selector_var is not active_var:
                    selector_var.set(False)

    ttk.Checkbutton(
        frame,
        text="Arm 1",
        variable=arm1_engaged_var,
        command=lambda: select_exclusively_for_output(arm1_engaged_var),
    ).grid(
        row=8,
        column=0,
        sticky="w",
        pady=(10, 0),
    )
    ttk.Checkbutton(
        frame,
        text="Arm 2",
        variable=arm2_engaged_var,
        command=lambda: select_exclusively_for_output(arm2_engaged_var),
    ).grid(
        row=8,
        column=1,
        sticky="w",
        pady=(10, 0),
    )
    ttk.Checkbutton(
        frame,
        text="G4 -> Y tilt",
        variable=g4_engaged_var,
        command=lambda: select_exclusively_for_output(g4_engaged_var),
    ).grid(
        row=9,
        column=0,
        sticky="w",
        pady=(4, 0),
    )
    ttk.Checkbutton(
        frame,
        text="G5 -> X tilt",
        variable=g5_engaged_var,
        command=lambda: select_exclusively_for_output(g5_engaged_var),
    ).grid(
        row=9,
        column=1,
        sticky="w",
        pady=(4, 0),
    )
    ttk.Checkbutton(
        frame,
        text="G6 -> turntable",
        variable=g6_engaged_var,
        command=lambda: select_exclusively_for_output(g6_engaged_var),
    ).grid(
        row=10,
        column=0,
        columnspan=2,
        sticky="w",
        pady=(4, 0),
    )

    tilt_x_label = ttk.Label(
        frame,
        width=24,
        text=f"X tilt: {format_degrees(DEFAULT_TILT_DEGREES)} deg",
    )
    tilt_x_label.grid(row=11, column=0, columnspan=2, sticky="w", pady=(10, 0))
    tilt_y_label = ttk.Label(
        frame,
        width=24,
        text=f"Y tilt: {format_degrees(DEFAULT_TILT_DEGREES)} deg",
    )
    tilt_y_label.grid(row=12, column=0, columnspan=2, sticky="w", pady=(2, 8))

    g1_diameter_label = ttk.Label(frame, text="G1 diameter (mm)")
    g1_diameter_label.grid(row=13, column=0, sticky="w", pady=(10, 0))
    g1_diameter_entry = ttk.Entry(frame, textvariable=g1_diameter_entry_var, width=12)
    g1_diameter_entry.grid(row=13, column=1, sticky="e", pady=(10, 0))
    g1_diameter_slider = ttk.Scale(
        frame,
        from_=MIN_DIAMETER_MM,
        to=MAX_DIAMETER_MM,
        variable=g1_diameter_slider_var,
        orient="horizontal",
        length=260,
    )
    g1_diameter_slider.grid(row=14, column=0, columnspan=2, sticky="ew", pady=(2, 8))

    g2_diameter_label = ttk.Label(frame, text="G2-G6 diameter (mm)")
    g2_diameter_label.grid(row=15, column=0, sticky="w")
    g2_diameter_entry = ttk.Entry(frame, textvariable=g2_diameter_entry_var, width=12)
    g2_diameter_entry.grid(row=15, column=1, sticky="e")
    g2_diameter_slider = ttk.Scale(
        frame,
        from_=MIN_DIAMETER_MM,
        to=MAX_DIAMETER_MM,
        variable=g2_diameter_slider_var,
        orient="horizontal",
        length=260,
    )
    g2_diameter_slider.grid(row=16, column=0, columnspan=2, sticky="ew", pady=(2, 8))

    spool_diameter_label = ttk.Label(frame, text="Spool diameter (mm)")
    spool_diameter_label.grid(row=17, column=0, sticky="w")
    spool_diameter_entry = ttk.Entry(frame, textvariable=spool_diameter_entry_var, width=12)
    spool_diameter_entry.grid(row=17, column=1, sticky="e")
    spool_diameter_slider = ttk.Scale(
        frame,
        from_=MIN_SPOOL_DIAMETER_MM,
        to=MAX_SPOOL_DIAMETER_MM,
        variable=spool_diameter_slider_var,
        orient="horizontal",
        length=260,
    )
    spool_diameter_slider.grid(row=18, column=0, columnspan=2, sticky="ew", pady=(2, 0))

    arm1_length_label = ttk.Label(frame, text="Arm 1 length (mm)")
    arm1_length_label.grid(row=19, column=0, sticky="w", pady=(8, 0))
    arm1_length_entry = ttk.Entry(frame, textvariable=arm1_length_entry_var, width=12)
    arm1_length_entry.grid(row=19, column=1, sticky="e", pady=(8, 0))
    arm1_length_slider = ttk.Scale(
        frame,
        from_=MIN_ARM_LENGTH_MM,
        to=MAX_ARM_LENGTH_MM,
        variable=arm1_length_slider_var,
        orient="horizontal",
        length=260,
    )
    arm1_length_slider.grid(row=20, column=0, columnspan=2, sticky="ew", pady=(2, 0))

    arm2_length_label = ttk.Label(frame, text="Arm 2 length (mm)")
    arm2_length_label.grid(row=21, column=0, sticky="w", pady=(8, 0))
    arm2_length_entry = ttk.Entry(frame, textvariable=arm2_length_entry_var, width=12)
    arm2_length_entry.grid(row=21, column=1, sticky="e", pady=(8, 0))
    arm2_length_slider = ttk.Scale(
        frame,
        from_=MIN_ARM_LENGTH_MM,
        to=MAX_ARM_LENGTH_MM,
        variable=arm2_length_slider_var,
        orient="horizontal",
        length=260,
    )
    arm2_length_slider.grid(row=22, column=0, columnspan=2, sticky="ew", pady=(2, 0))

    def clamp_rotation(value):
        return max(MIN_MOVE_DEGREES, min(MAX_MOVE_DEGREES, value))

    def commit_rotation_entry(_event=None):
        try:
            rotation = float(rotation_entry_var.get())
        except ValueError:
            rotation = rotation_slider_var.get()
        rotation = clamp_rotation(rotation)
        rotation_slider_var.set(rotation)
        rotation_entry_var.set(format_degrees(rotation))
        return True

    def update_rotation_entry(value):
        rotation_entry_var.set(format_degrees(float(value)))

    def update_tilt_label(label, axis_name, value):
        label.configure(text=f"{axis_name} tilt: {format_degrees(float(value))} deg")

    def clamp_diameter(value):
        return max(MIN_DIAMETER_MM, min(MAX_DIAMETER_MM, value))

    def commit_diameter_entry(entry_var, slider_var, _event=None):
        try:
            diameter = float(entry_var.get())
        except ValueError:
            diameter = slider_var.get()
        diameter = clamp_diameter(diameter)
        slider_var.set(diameter)
        entry_var.set(format_number(diameter))
        return True

    def update_diameter_entry(entry_var, value):
        entry_var.set(format_number(float(value)))

    def commit_spool_diameter_entry(_event=None):
        try:
            diameter = float(spool_diameter_entry_var.get())
        except ValueError:
            diameter = spool_diameter_slider_var.get()
        diameter = max(MIN_SPOOL_DIAMETER_MM, min(MAX_SPOOL_DIAMETER_MM, diameter))
        spool_diameter_slider_var.set(diameter)
        spool_diameter_entry_var.set(format_number(diameter))
        return True

    def commit_arm_length_entry(entry_var, slider_var, _event=None):
        try:
            length = float(entry_var.get())
        except ValueError:
            length = slider_var.get()
        length = max(MIN_ARM_LENGTH_MM, min(MAX_ARM_LENGTH_MM, length))
        slider_var.set(length)
        entry_var.set(format_number(length))
        return True

    def commit_bounded_entry(
        entry_var,
        slider_var,
        minimum,
        maximum,
        _event=None,
    ):
        try:
            value = float(entry_var.get())
        except ValueError:
            value = slider_var.get()
        value = max(minimum, min(maximum, value))
        slider_var.set(value)
        entry_var.set(format_number(value))
        return True

    rotation_slider.configure(command=update_rotation_entry)
    rotation_entry.bind("<Return>", commit_rotation_entry)
    rotation_entry.bind("<FocusOut>", commit_rotation_entry)
    g1_diameter_slider.configure(command=lambda value: update_diameter_entry(g1_diameter_entry_var, value))
    g1_diameter_entry.bind(
        "<Return>",
        lambda event: commit_diameter_entry(g1_diameter_entry_var, g1_diameter_slider_var, event),
    )
    g1_diameter_entry.bind(
        "<FocusOut>",
        lambda event: commit_diameter_entry(g1_diameter_entry_var, g1_diameter_slider_var, event),
    )
    g2_diameter_slider.configure(command=lambda value: update_diameter_entry(g2_diameter_entry_var, value))
    g2_diameter_entry.bind(
        "<Return>",
        lambda event: commit_diameter_entry(g2_diameter_entry_var, g2_diameter_slider_var, event),
    )
    g2_diameter_entry.bind(
        "<FocusOut>",
        lambda event: commit_diameter_entry(g2_diameter_entry_var, g2_diameter_slider_var, event),
    )
    spool_diameter_slider.configure(
        command=lambda value: update_diameter_entry(spool_diameter_entry_var, value)
    )
    spool_diameter_entry.bind("<Return>", commit_spool_diameter_entry)
    spool_diameter_entry.bind("<FocusOut>", commit_spool_diameter_entry)
    arm1_length_slider.configure(
        command=lambda value: update_diameter_entry(arm1_length_entry_var, value)
    )
    arm1_length_entry.bind(
        "<Return>",
        lambda event: commit_arm_length_entry(
            arm1_length_entry_var,
            arm1_length_slider_var,
            event,
        ),
    )
    arm1_length_entry.bind(
        "<FocusOut>",
        lambda event: commit_arm_length_entry(
            arm1_length_entry_var,
            arm1_length_slider_var,
            event,
        ),
    )
    arm2_length_slider.configure(
        command=lambda value: update_diameter_entry(arm2_length_entry_var, value)
    )
    arm2_length_entry.bind(
        "<Return>",
        lambda event: commit_arm_length_entry(
            arm2_length_entry_var,
            arm2_length_slider_var,
            event,
        ),
    )
    arm2_length_entry.bind(
        "<FocusOut>",
        lambda event: commit_arm_length_entry(
            arm2_length_entry_var,
            arm2_length_slider_var,
            event,
        ),
    )
    bounded_controls = (
        (
            beam_angle_entry,
            beam_angle_entry_var,
            beam_angle_slider,
            beam_angle_slider_var,
            MIN_BEAM_ANGLE_DEGREES,
            MAX_BEAM_ANGLE_DEGREES,
        ),
        (
            target_x_entry,
            target_x_entry_var,
            target_x_slider,
            target_x_slider_var,
            MIN_TARGET_X_CM,
            MAX_TARGET_X_CM,
        ),
        (
            target_y_entry,
            target_y_entry_var,
            target_y_slider,
            target_y_slider_var,
            MIN_TARGET_Y_CM,
            MAX_TARGET_Y_CM,
        ),
        (
            target_z_entry,
            target_z_entry_var,
            target_z_slider,
            target_z_slider_var,
            MIN_TARGET_Z_CM,
            MAX_TARGET_Z_CM,
        ),
        (
            target_diameter_entry,
            target_diameter_entry_var,
            target_diameter_slider,
            target_diameter_slider_var,
            MIN_TARGET_DIAMETER_CM,
            MAX_TARGET_DIAMETER_CM,
        ),
    )
    for (
        entry,
        entry_var,
        slider,
        slider_var,
        minimum,
        maximum,
    ) in bounded_controls:
        slider.configure(
            command=lambda value, variable=entry_var: update_diameter_entry(
                variable,
                value,
            )
        )
        entry.bind(
            "<Return>",
            lambda event,
            variable=entry_var,
            scale_var=slider_var,
            low=minimum,
            high=maximum: commit_bounded_entry(
                variable,
                scale_var,
                low,
                high,
                event,
            ),
        )
        entry.bind(
            "<FocusOut>",
            lambda event,
            variable=entry_var,
            scale_var=slider_var,
            low=minimum,
            high=maximum: commit_bounded_entry(
                variable,
                scale_var,
                low,
                high,
                event,
            ),
        )

    def set_all_selectors(engaged=False):
        arm1_engaged_var.set(engaged)
        arm2_engaged_var.set(engaged)
        g4_engaged_var.set(engaged)
        g5_engaged_var.set(engaged)
        g6_engaged_var.set(engaged)

    def update_rotation_mode():
        if output_rotation_var.get():
            rotation_label.configure(text="Selected output rotation (deg)")
            engaged_vars = [var for var in selector_vars if var.get()]
            if len(engaged_vars) > 1:
                selected_var = engaged_vars[0]
                for selector_var in selector_vars:
                    selector_var.set(selector_var is selected_var)
        else:
            rotation_label.configure(text="G1 motor rotation (deg)")

    output_rotation_check.configure(command=update_rotation_mode)

    def engage_only_reset_axis(axis_key):
        set_all_selectors(False)
        if axis_key == "x_tilt":
            g5_engaged_var.set(True)
        elif axis_key == "y_tilt":
            g4_engaged_var.set(True)
        elif axis_key == "arm2":
            arm2_engaged_var.set(True)
        elif axis_key == "arm1":
            arm1_engaged_var.set(True)
        elif axis_key == "turntable":
            g6_engaged_var.set(True)

    def cancel_physical_reset():
        reset_active[0] = False
        reset_queue.clear()
        reset_current_axis[0] = None

    def start_move():
        cancel_physical_reset()
        commit_rotation_entry()
        commit_diameter_entry(g1_diameter_entry_var, g1_diameter_slider_var)
        commit_diameter_entry(g2_diameter_entry_var, g2_diameter_slider_var)
        commit_spool_diameter_entry()
        commit_arm_length_entry(arm1_length_entry_var, arm1_length_slider_var)
        commit_arm_length_entry(arm2_length_entry_var, arm2_length_slider_var)
        requested_degrees = rotation_slider_var.get()
        motor_move_degrees = requested_degrees
        move_description = f"G1 {format_degrees(motor_move_degrees)} deg"

        if output_rotation_var.get():
            selected_axes = [
                ("Arm 1", "arm1", arm1_engaged_var.get()),
                ("Arm 2", "arm2", arm2_engaged_var.get()),
                ("Y tilt", "y_tilt", g4_engaged_var.get()),
                ("X tilt", "x_tilt", g5_engaged_var.get()),
                ("Turntable", "turntable", g6_engaged_var.get()),
            ]
            selected_axes = [axis for axis in selected_axes if axis[2]]
            if len(selected_axes) != 1:
                remaining_motor_move_rad[0] = 0.0
                status_label.configure(
                    text="Select exactly one output, or use direct G1 mode"
                )
                return

            output_label, axis_key, _ = selected_axes[0]
            if axis_key in ("x_tilt", "y_tilt"):
                current_tilt = (
                    tilt_x_var.get() if axis_key == "x_tilt" else tilt_y_var.get()
                )
                target_tilt = current_tilt + requested_degrees
                if not MIN_TILT_DEGREES <= target_tilt <= MAX_TILT_DEGREES:
                    remaining_motor_move_rad[0] = 0.0
                    status_label.configure(
                        text=(
                            f"{output_label} target {format_degrees(target_tilt)} "
                            f"outside {MIN_TILT_DEGREES:g}..{MAX_TILT_DEGREES:g} deg"
                        )
                    )
                    return

            output_per_motor = output_degrees_per_motor_degree(
                axis_key,
                g1_diameter_slider_var.get(),
                g2_diameter_slider_var.get(),
                spool_diameter_slider_var.get(),
            )
            motor_move_degrees = requested_degrees / output_per_motor
            move_description = (
                f"{output_label} {format_degrees(requested_degrees)} deg "
                f"(G1 {format_degrees(motor_move_degrees)} deg)"
            )

        speed = speed_var.get()
        if abs(motor_move_degrees) < 0.001:
            remaining_motor_move_rad[0] = 0.0
            status_label.configure(text="No rotation requested")
            return
        if speed <= 0.0:
            status_label.configure(text="Set Motor V above 0, then Start")
            return

        # Capture every selected path against the current, unmoved G1 position.
        # Without this Start-time capture, clicking a new selector and quickly
        # pressing Start could let the first motor step occur before the normal
        # UI refresh noticed the engagement, shortening the output move.
        with viewer.lock():
            follower_move_degrees = -motor_move_degrees * (
                g1_diameter_slider_var.get() / g2_diameter_slider_var.get()
            )
            arm_violation = None
            if arm1_engaged_var.get() or arm2_engaged_var.get():
                arm_violation = arm_motion_violation(
                    math.degrees(data.qpos[tg2_qpos_id]),
                    math.degrees(data.qpos[tg6_qpos_id]),
                    follower_move_degrees if arm1_engaged_var.get() else 0.0,
                    follower_move_degrees if arm2_engaged_var.get() else 0.0,
                    arm1_length_slider_var.get(),
                    arm2_length_slider_var.get(),
                )
            if arm_violation == ARM_CONSTRAINT_ARM1_LIMIT:
                remaining_motor_move_rad[0] = 0.0
                status_label.configure(
                    text=(
                        "Move rejected: Arm 1 must remain between "
                        f"-{ARM1_LIMIT_DEGREES:g} and +{ARM1_LIMIT_DEGREES:g} deg"
                    )
                )
                return
            if arm_violation == ARM_CONSTRAINT_PLATFORM_COLLISION:
                remaining_motor_move_rad[0] = 0.0
                status_label.configure(
                    text="Move rejected: arm or tilt plate would hit upper platform"
                )
                return

            current_g1_angle_rad = g1_angle_from_motor(motor_angle_rad[0])
            current_follower_target_rad = g2_angle_from_g1(
                current_g1_angle_rad,
                g1_diameter_slider_var.get(),
                g2_diameter_slider_var.get(),
            )
            if arm1_engaged_var.get():
                arm1_drive_offset_rad[0] = (
                    data.qpos[tg2_qpos_id] - current_follower_target_rad
                )
            if arm2_engaged_var.get():
                arm2_drive_offset_rad[0] = (
                    data.qpos[tg6_qpos_id] - current_follower_target_rad
                )
            if g4_engaged_var.get():
                g4_drive_offset_rad[0] = (
                    data.qpos[g4_qpos_id] - current_follower_target_rad
                )
                g4_spool_reference_rad[0] = data.qpos[g4_qpos_id]
                y_tilt_reference_rad[0] = data.qpos[plate_y_tilt_qpos_id]
            if g5_engaged_var.get():
                g5_drive_offset_rad[0] = (
                    data.qpos[g5_qpos_id] - current_follower_target_rad
                )
                g5_spool_reference_rad[0] = data.qpos[g5_qpos_id]
                x_tilt_reference_rad[0] = data.qpos[plate_x_tilt_qpos_id]
            if g6_engaged_var.get():
                g6_drive_offset_rad[0] = (
                    data.qpos[g6_qpos_id] - current_follower_target_rad
                )
                g6_reference_rad[0] = data.qpos[g6_qpos_id]
                turntable_reference_rad[0] = data.qpos[turntable_qpos_id]

        arm1_was_engaged[0] = arm1_engaged_var.get()
        arm2_was_engaged[0] = arm2_engaged_var.get()
        g4_was_engaged[0] = g4_engaged_var.get()
        g5_was_engaged[0] = g5_engaged_var.get()
        g6_was_engaged[0] = g6_engaged_var.get()
        remaining_motor_move_rad[0] = math.radians(motor_move_degrees)
        status_label.configure(text=f"Moving {move_description}")

    def stop():
        remaining_motor_move_rad[0] = 0.0
        if reset_active[0]:
            set_all_selectors(False)
        cancel_physical_reset()
        speed_var.set(0.0)
        status_label.configure(text="Stopped")

    def reset_position():
        commit_diameter_entry(g1_diameter_entry_var, g1_diameter_slider_var)
        commit_diameter_entry(g2_diameter_entry_var, g2_diameter_slider_var)
        commit_spool_diameter_entry()
        commit_arm_length_entry(arm1_length_entry_var, arm1_length_slider_var)
        commit_arm_length_entry(arm2_length_entry_var, arm2_length_slider_var)
        remaining_motor_move_rad[0] = 0.0
        set_all_selectors(False)
        reset_queue[:] = [
            ("X tilt / G5", "x_tilt", g5_qpos_id),
            ("Y tilt / G4", "y_tilt", g4_qpos_id),
            ("Arm 2 / G3", "arm2", g3_qpos_id),
            ("Arm 1 / G2", "arm1", g2_qpos_id),
            ("Turntable / G6", "turntable", g6_qpos_id),
        ]
        reset_current_axis[0] = None
        reset_active[0] = True
        status_label.configure(
            text=f"Physical reset queued at {format_number(RESET_VELOCITY_DEG_S)} deg/s"
        )

    def finish_reset_axis(axis_key):
        if axis_key == "x_tilt":
            data.qpos[g5_qpos_id] = 0.0
            data.qpos[plate_x_tilt_qpos_id] = 0.0
            tilt_x_var.set(0.0)
            update_tilt_label(tilt_x_label, "X", 0.0)
        elif axis_key == "y_tilt":
            data.qpos[g4_qpos_id] = 0.0
            data.qpos[plate_y_tilt_qpos_id] = 0.0
            tilt_y_var.set(0.0)
            update_tilt_label(tilt_y_label, "Y", 0.0)
        elif axis_key == "arm2":
            data.qpos[g3_qpos_id] = 0.0
            data.qpos[tg4_qpos_id] = 0.0
            data.qpos[tg6_qpos_id] = 0.0
        elif axis_key == "arm1":
            data.qpos[g2_qpos_id] = 0.0
            data.qpos[tg2_qpos_id] = 0.0
        elif axis_key == "turntable":
            data.qpos[g6_qpos_id] = 0.0
            data.qpos[turntable_qpos_id] = 0.0

    def begin_next_reset_axis(g1_diameter_mm, g2_diameter_mm):
        gear_ratio = g1_diameter_mm / g2_diameter_mm
        g1_angle_rad = g1_angle_from_motor(motor_angle_rad[0])
        g2_angle_rad = g2_angle_from_g1(
            g1_angle_rad,
            g1_diameter_mm,
            g2_diameter_mm,
        )

        while reset_queue:
            label, axis_key, encoder_qpos_id = reset_queue.pop(0)
            encoder_angle_rad = float(data.qpos[encoder_qpos_id])
            # Arm and turntable orientations repeat every revolution. Cable
            # spool positions do not: another spool revolution changes cable
            # length, so G4/G5 retain their multi-turn error to the unique
            # zero-cable position.
            if axis_key in ("x_tilt", "y_tilt"):
                encoder_error_rad = encoder_angle_rad
            else:
                encoder_error_rad = shortest_angular_error_to_zero(
                    encoder_angle_rad
                )
            if abs(encoder_error_rad) < 1e-10:
                finish_reset_axis(axis_key)
                continue

            engage_only_reset_axis(axis_key)
            if axis_key == "x_tilt":
                g5_drive_offset_rad[0] = data.qpos[g5_qpos_id] - g2_angle_rad
                g5_spool_reference_rad[0] = data.qpos[g5_qpos_id]
                x_tilt_reference_rad[0] = data.qpos[plate_x_tilt_qpos_id]
            elif axis_key == "y_tilt":
                g4_drive_offset_rad[0] = data.qpos[g4_qpos_id] - g2_angle_rad
                g4_spool_reference_rad[0] = data.qpos[g4_qpos_id]
                y_tilt_reference_rad[0] = data.qpos[plate_y_tilt_qpos_id]
            elif axis_key == "arm2":
                arm2_drive_offset_rad[0] = data.qpos[tg6_qpos_id] - g2_angle_rad
            elif axis_key == "arm1":
                arm1_drive_offset_rad[0] = data.qpos[tg2_qpos_id] - g2_angle_rad
            elif axis_key == "turntable":
                g6_drive_offset_rad[0] = data.qpos[g6_qpos_id] - g2_angle_rad
                g6_reference_rad[0] = data.qpos[g6_qpos_id]
                turntable_reference_rad[0] = data.qpos[turntable_qpos_id]

            arm1_was_engaged[0] = arm1_engaged_var.get()
            arm2_was_engaged[0] = arm2_engaged_var.get()
            g4_was_engaged[0] = g4_engaged_var.get()
            g5_was_engaged[0] = g5_engaged_var.get()
            g6_was_engaged[0] = g6_engaged_var.get()
            reset_current_axis[0] = (label, axis_key)
            remaining_motor_move_rad[0] = encoder_error_rad / gear_ratio
            status_label.configure(
                text=f"Resetting {label} at {format_number(RESET_VELOCITY_DEG_S)} deg/s"
            )
            return

        set_all_selectors(False)
        reset_current_axis[0] = None
        reset_active[0] = False
        status_label.configure(text="Physical reset complete")

    ttk.Button(frame, text="Start", command=start_move).grid(row=6, column=0, sticky="ew", padx=(0, 6))
    ttk.Button(frame, text="Stop", command=stop).grid(row=6, column=1, sticky="ew")
    ttk.Button(frame, text="Physical reset", command=reset_position).grid(
        row=7,
        column=0,
        columnspan=2,
        sticky="ew",
        pady=(6, 0),
    )

    set_gear_layout(
        model,
        g1_diameter_slider_var.get(),
        g2_diameter_slider_var.get(),
        spool_diameter_mm=spool_diameter_slider_var.get(),
        arm1_length_mm=arm1_length_slider_var.get(),
        arm2_length_mm=arm2_length_slider_var.get(),
    )
    set_spotlight_beam_angle(model, beam_angle_slider_var.get())
    set_target_layout(
        model,
        g1_diameter_slider_var.get(),
        g2_diameter_slider_var.get(),
        target_x_slider_var.get(),
        target_y_slider_var.get(),
        target_z_slider_var.get(),
        target_diameter_slider_var.get(),
        target_color_var.get(),
    )
    pip_renderer = mujoco.Renderer(model, height=PIP_HEIGHT, width=PIP_WIDTH)
    pip_photo = [None]
    pip_renderer_closed = [False]
    last_pip_time = [0.0]

    def close_pip_renderer():
        if not pip_renderer_closed[0]:
            pip_renderer.close()
            pip_renderer_closed[0] = True

    def close_app():
        close_pip_renderer()
        root.destroy()

    last_wall_time = [time.perf_counter()]

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        viewer.cam.lookat[:] = START_CAMERA_LOOKAT
        viewer.cam.distance = START_CAMERA_DISTANCE
        viewer.cam.azimuth = START_CAMERA_AZIMUTH
        viewer.cam.elevation = START_CAMERA_ELEVATION

        def tick():
            if not viewer.is_running():
                close_app()
                return

            now = time.perf_counter()
            elapsed = min(now - last_wall_time[0], 0.05)
            last_wall_time[0] = now

            g1_diameter_mm = g1_diameter_slider_var.get()
            g2_diameter_mm = g2_diameter_slider_var.get()
            spool_diameter_mm = spool_diameter_slider_var.get()
            arm1_length_mm = arm1_length_slider_var.get()
            arm2_length_mm = arm2_length_slider_var.get()
            beam_angle_degrees = beam_angle_slider_var.get()
            target_x_cm = target_x_slider_var.get()
            target_y_cm = target_y_slider_var.get()
            target_z_cm = target_z_slider_var.get()
            target_diameter_cm = target_diameter_slider_var.get()
            target_color = target_color_var.get()
            speed = speed_var.get()
            motion_speed = RESET_VELOCITY_DEG_S if reset_active[0] else speed
            if reset_active[0]:
                speed_label.configure(text=f"Reset V: {RESET_VELOCITY_DEG_S:.1f} deg/s")
            else:
                speed_label.configure(text=f"Motor V: {speed:.1f} deg/s")
            pip_description_var.set(
                f"{format_number(DEFAULT_CAMERA_FOV_DEGREES)} deg fixed camera FOV; "
                f"{format_number(beam_angle_degrees)} deg spotlight beam"
            )
            plate_x_tilt_rad = math.radians(tilt_x_var.get())
            plate_y_tilt_rad = math.radians(tilt_y_var.get())
            if reset_active[0] and reset_current_axis[0] is not None:
                engage_only_reset_axis(reset_current_axis[0][1])
            g4_is_engaged = g4_engaged_var.get()
            g5_is_engaged = g5_engaged_var.get()
            g6_is_engaged = g6_engaged_var.get()
            if not g4_is_engaged:
                update_tilt_label(tilt_y_label, "Y", tilt_y_var.get())
            if not g5_is_engaged:
                update_tilt_label(tilt_x_label, "X", tilt_x_var.get())

            refresh_pip = now - last_pip_time[0] >= PIP_REFRESH_SECONDS
            pip_pixels = None

            with viewer.lock():
                if reset_active[0] and abs(remaining_motor_move_rad[0]) <= 0.0:
                    if reset_current_axis[0] is not None:
                        _, completed_axis_key = reset_current_axis[0]
                        finish_reset_axis(completed_axis_key)
                        reset_current_axis[0] = None
                        set_all_selectors(False)
                    begin_next_reset_axis(g1_diameter_mm, g2_diameter_mm)

                    # A completed reset step may have set an exact zero and
                    # updated the readout variables after these locals were
                    # sampled at the start of the tick.
                    plate_x_tilt_rad = math.radians(tilt_x_var.get())
                    plate_y_tilt_rad = math.radians(tilt_y_var.get())
                    g4_is_engaged = g4_engaged_var.get()
                    g5_is_engaged = g5_engaged_var.get()
                    g6_is_engaged = g6_engaged_var.get()

                if abs(remaining_motor_move_rad[0]) > 0.0 and motion_speed > 0.0:
                    max_step = math.radians(motion_speed) * elapsed
                    step = math.copysign(
                        min(abs(remaining_motor_move_rad[0]), max_step),
                        remaining_motor_move_rad[0],
                    )
                    motor_angle_rad[0] += step
                    remaining_motor_move_rad[0] -= step
                    if abs(remaining_motor_move_rad[0]) < 1e-6:
                        remaining_motor_move_rad[0] = 0.0
                        if reset_active[0] and reset_current_axis[0] is not None:
                            status_label.configure(
                                text=f"{reset_current_axis[0][0]} encoder at reset"
                            )
                        else:
                            status_label.configure(text="Move complete")
                g1_angle_rad = g1_angle_from_motor(motor_angle_rad[0])
                g2_angle_rad = g2_angle_from_g1(g1_angle_rad, g1_diameter_mm, g2_diameter_mm)
                data.qpos[g1_qpos_id] = g1_angle_rad
                arm1_is_engaged = arm1_engaged_var.get()
                arm2_is_engaged = arm2_engaged_var.get()
                if arm1_is_engaged and not arm1_was_engaged[0]:
                    arm1_drive_offset_rad[0] = data.qpos[tg2_qpos_id] - g2_angle_rad
                if arm2_is_engaged and not arm2_was_engaged[0]:
                    arm2_drive_offset_rad[0] = data.qpos[tg6_qpos_id] - g2_angle_rad
                if g4_is_engaged and not g4_was_engaged[0]:
                    g4_drive_offset_rad[0] = data.qpos[g4_qpos_id] - g2_angle_rad
                    g4_spool_reference_rad[0] = data.qpos[g4_qpos_id]
                    y_tilt_reference_rad[0] = data.qpos[plate_y_tilt_qpos_id]
                if g5_is_engaged and not g5_was_engaged[0]:
                    g5_drive_offset_rad[0] = data.qpos[g5_qpos_id] - g2_angle_rad
                    g5_spool_reference_rad[0] = data.qpos[g5_qpos_id]
                    x_tilt_reference_rad[0] = data.qpos[plate_x_tilt_qpos_id]
                if g6_is_engaged and not g6_was_engaged[0]:
                    g6_drive_offset_rad[0] = data.qpos[g6_qpos_id] - g2_angle_rad
                    g6_reference_rad[0] = data.qpos[g6_qpos_id]
                    turntable_reference_rad[0] = data.qpos[turntable_qpos_id]
                if abs(spool_diameter_mm - last_spool_diameter_mm[0]) > 1e-9:
                    if g4_is_engaged:
                        g4_spool_reference_rad[0] = data.qpos[g4_qpos_id]
                        y_tilt_reference_rad[0] = data.qpos[plate_y_tilt_qpos_id]
                    if g5_is_engaged:
                        g5_spool_reference_rad[0] = data.qpos[g5_qpos_id]
                        x_tilt_reference_rad[0] = data.qpos[plate_x_tilt_qpos_id]
                    last_spool_diameter_mm[0] = spool_diameter_mm
                arm1_was_engaged[0] = arm1_is_engaged
                arm2_was_engaged[0] = arm2_is_engaged
                g4_was_engaged[0] = g4_is_engaged
                g5_was_engaged[0] = g5_is_engaged
                g6_was_engaged[0] = g6_is_engaged

                if arm1_is_engaged:
                    arm1_angle_rad = g2_angle_rad + arm1_drive_offset_rad[0]
                    data.qpos[g2_qpos_id] = arm1_angle_rad
                    data.qpos[tg2_qpos_id] = arm1_angle_rad
                if arm2_is_engaged:
                    arm2_angle_rad = g2_angle_rad + arm2_drive_offset_rad[0]
                    data.qpos[g3_qpos_id] = arm2_angle_rad
                    data.qpos[tg4_qpos_id] = arm2_angle_rad
                    data.qpos[tg6_qpos_id] = arm2_angle_rad
                if g4_is_engaged:
                    g4_angle_rad = g2_angle_rad + g4_drive_offset_rad[0]
                    data.qpos[g4_qpos_id] = g4_angle_rad
                    plate_y_tilt_rad = clamp_tilt_rad(
                        y_tilt_reference_rad[0]
                        + tilt_delta_from_spool(
                            g4_angle_rad - g4_spool_reference_rad[0],
                            spool_diameter_mm,
                        )
                    )
                    tilt_y_var.set(math.degrees(plate_y_tilt_rad))
                    tilt_y_label.configure(
                        text=f"Y tilt (G4): {format_degrees(math.degrees(plate_y_tilt_rad))} deg"
                    )
                if g5_is_engaged:
                    g5_angle_rad = g2_angle_rad + g5_drive_offset_rad[0]
                    data.qpos[g5_qpos_id] = g5_angle_rad
                    plate_x_tilt_rad = clamp_tilt_rad(
                        x_tilt_reference_rad[0]
                        + tilt_delta_from_spool(
                            g5_angle_rad - g5_spool_reference_rad[0],
                            spool_diameter_mm,
                        )
                    )
                    tilt_x_var.set(math.degrees(plate_x_tilt_rad))
                    tilt_x_label.configure(
                        text=f"X tilt (G5): {format_degrees(math.degrees(plate_x_tilt_rad))} deg"
                    )
                if g6_is_engaged:
                    g6_angle_rad = g2_angle_rad + g6_drive_offset_rad[0]
                    data.qpos[g6_qpos_id] = g6_angle_rad
                    data.qpos[turntable_qpos_id] = (
                        turntable_reference_rad[0]
                        + g6_angle_rad
                        - g6_reference_rad[0]
                    )
                data.qpos[plate_x_tilt_qpos_id] = plate_x_tilt_rad
                data.qpos[plate_y_tilt_qpos_id] = plate_y_tilt_rad
                set_gear_layout(
                    model,
                    g1_diameter_mm,
                    g2_diameter_mm,
                    data.qpos[tg2_qpos_id],
                    spool_diameter_mm=spool_diameter_mm,
                    arm1_length_mm=arm1_length_mm,
                    arm2_length_mm=arm2_length_mm,
                )
                set_spotlight_beam_angle(model, beam_angle_degrees)
                set_target_layout(
                    model,
                    g1_diameter_mm,
                    g2_diameter_mm,
                    target_x_cm,
                    target_y_cm,
                    target_z_cm,
                    target_diameter_cm,
                    target_color,
                )
                data.time += elapsed
                mujoco.mj_forward(model, data)
                if refresh_pip:
                    pip_renderer.update_scene(data, camera=SPOTLIGHT_CAMERA)
                    pip_pixels = pip_renderer.render().copy()
                    last_pip_time[0] = now

            viewer.sync()
            if pip_pixels is not None:
                pip_photo[0] = ImageTk.PhotoImage(
                    image=Image.fromarray(pip_pixels),
                    master=root,
                )
                pip_canvas.itemconfigure(pip_image_item, image=pip_photo[0])
            root.after(16, tick)

        root.protocol("WM_DELETE_WINDOW", close_app)
        root.after(0, tick)
        root.mainloop()

    close_pip_renderer()


if __name__ == "__main__":
    main()
