from pathlib import Path
import math
import time
import tkinter as tk
from tkinter import ttk

import mujoco
import mujoco.viewer
from PIL import Image, ImageTk


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
G2_BODY = "G2"
G3_BODY = "G3"
G4_BODY = "G4"
G5_BODY = "G5"
G6_BODY = "G6"
TG2_BODY = "TG2"
TG4_BODY = "TG4"
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
TG2_SUPPORT_GEOM = "tg2_support"
TG2_AXLE_GEOM = "tg2_axle"
TG4_SUPPORT_GEOM = "tg4_support"
TG4_AXLE_GEOM = "tg4_axle"
BELT_LEFT_GEOM = "belt_left"
BELT_RIGHT_GEOM = "belt_right"
BELT2_LEFT_GEOM = "belt2_left"
BELT2_RIGHT_GEOM = "belt2_right"
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
ARM1_LENGTH_M = 0.150
BELT_CENTER_DISTANCE_M = 0.100
TG2_SUPPORT_CENTER_Z = G2_CENTER_Z + (BELT_CENTER_DISTANCE_M / 2.0)
TG2_AXLE_Y = (G2_SUPPORT_Y + TIMING_GEAR_Y) / 2.0
TG4_AXLE_Y = (G3_SUPPORT_Y + TIMING_GEAR2_Y) / 2.0
START_CAMERA_LOOKAT = (0.080, -0.010, 0.450)
START_CAMERA_DISTANCE = 1.20
START_CAMERA_AZIMUTH = 180.0
START_CAMERA_ELEVATION = 0.0

DEFAULT_G1_DIAMETER_MM = 64.0
DEFAULT_G2_DIAMETER_MM = 100.0
MIN_DIAMETER_MM = 35.0
MAX_DIAMETER_MM = 140.0
TIMING_GEAR_DIAMETER_MM = MIN_DIAMETER_MM
DEFAULT_SPEED_DEG_S = 100.0
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
PLATE_CABLE_LEVER_ARM_M = 0.025
TURNTABLE_CENTER_Y = -0.118
TURNTABLE_REAR_Y = 0.025
TURNTABLE_FRONT_Y = -0.260
TURNTABLE_RADIAL_MARGIN_M = 0.015
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


def rotate_y(v, angle_rad):
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return ((c * v[0]) + (s * v[2]), v[1], (-s * v[0]) + (c * v[2]))


def set_arm1_belt_layout(model, g2_center_x, arm_angle_rad):
    timing_radius = radius_m(TIMING_GEAR_DIAMETER_MM)
    tg2_center_z = G2_CENTER_Z + BELT_CENTER_DISTANCE_M
    tg5_center = (g2_center_x, TIMING_GEAR2_Y + TG5_LOCAL_Y, tg2_center_z)
    tg6_offset = rotate_y((0.0, TG6_LOCAL_Y, ARM1_LENGTH_M), arm_angle_rad)
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

    set_arm1_belt_layout(model, g2_center_x, arm_angle_rad)


def g1_angle_from_motor(motor_angle_rad):
    return motor_angle_rad * MOTOR_TO_G1_RATIO


def g2_angle_from_g1(g1_angle_rad, g1_diameter_mm, g2_diameter_mm):
    return -g1_angle_rad * (g1_diameter_mm / g2_diameter_mm)


def main():
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
    speed_var = tk.DoubleVar(value=DEFAULT_SPEED_DEG_S)
    rotation_slider_var = tk.DoubleVar(value=DEFAULT_MOVE_DEGREES)
    rotation_entry_var = tk.StringVar(value=format_degrees(DEFAULT_MOVE_DEGREES))
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

    frame = ttk.Frame(root, padding=12)
    frame.grid(row=0, column=0, sticky="nsew")

    pip_frame = ttk.LabelFrame(frame, text="Spotlight camera PIP", padding=6)
    pip_frame.grid(row=0, column=2, rowspan=20, sticky="n", padx=(16, 0))
    pip_canvas = tk.Canvas(
        pip_frame,
        width=PIP_WIDTH,
        height=PIP_HEIGHT,
        background="black",
        highlightthickness=0,
    )
    pip_canvas.grid(row=0, column=0)
    pip_image_item = pip_canvas.create_image(0, 0, anchor="nw")
    ttk.Label(pip_frame, text="50 deg view aligned with the spotlight beam").grid(
        row=1,
        column=0,
        pady=(6, 0),
    )

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

    rotation_label = ttk.Label(frame, text="Rotation (deg)")
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

    status_label = ttk.Label(frame, width=34)
    status_label.grid(row=4, column=0, columnspan=2, sticky="w", pady=(0, 8))

    ttk.Checkbutton(frame, text="Arm 1", variable=arm1_engaged_var).grid(
        row=7,
        column=0,
        sticky="w",
        pady=(10, 0),
    )
    ttk.Checkbutton(frame, text="Arm 2", variable=arm2_engaged_var).grid(
        row=7,
        column=1,
        sticky="w",
        pady=(10, 0),
    )
    ttk.Checkbutton(frame, text="G4 -> Y tilt", variable=g4_engaged_var).grid(
        row=8,
        column=0,
        sticky="w",
        pady=(4, 0),
    )
    ttk.Checkbutton(frame, text="G5 -> X tilt", variable=g5_engaged_var).grid(
        row=8,
        column=1,
        sticky="w",
        pady=(4, 0),
    )
    ttk.Checkbutton(frame, text="G6 -> turntable", variable=g6_engaged_var).grid(
        row=9,
        column=0,
        columnspan=2,
        sticky="w",
        pady=(4, 0),
    )

    tilt_x_label = ttk.Label(frame, width=24, text=f"X tilt: {format_degrees(DEFAULT_TILT_DEGREES)} deg")
    tilt_x_label.grid(row=10, column=0, columnspan=2, sticky="w", pady=(10, 0))
    tilt_x_slider = ttk.Scale(
        frame,
        from_=MIN_TILT_DEGREES,
        to=MAX_TILT_DEGREES,
        variable=tilt_x_var,
        orient="horizontal",
        length=260,
    )
    tilt_x_slider.grid(row=11, column=0, columnspan=2, sticky="ew", pady=(2, 8))

    tilt_y_label = ttk.Label(frame, width=24, text=f"Y tilt: {format_degrees(DEFAULT_TILT_DEGREES)} deg")
    tilt_y_label.grid(row=12, column=0, columnspan=2, sticky="w")
    tilt_y_slider = ttk.Scale(
        frame,
        from_=MIN_TILT_DEGREES,
        to=MAX_TILT_DEGREES,
        variable=tilt_y_var,
        orient="horizontal",
        length=260,
    )
    tilt_y_slider.grid(row=13, column=0, columnspan=2, sticky="ew", pady=(2, 8))

    g1_diameter_label = ttk.Label(frame, text="G1 diameter (mm)")
    g1_diameter_label.grid(row=14, column=0, sticky="w", pady=(10, 0))
    g1_diameter_entry = ttk.Entry(frame, textvariable=g1_diameter_entry_var, width=12)
    g1_diameter_entry.grid(row=14, column=1, sticky="e", pady=(10, 0))
    g1_diameter_slider = ttk.Scale(
        frame,
        from_=MIN_DIAMETER_MM,
        to=MAX_DIAMETER_MM,
        variable=g1_diameter_slider_var,
        orient="horizontal",
        length=260,
    )
    g1_diameter_slider.grid(row=15, column=0, columnspan=2, sticky="ew", pady=(2, 8))

    g2_diameter_label = ttk.Label(frame, text="G2-G6 diameter (mm)")
    g2_diameter_label.grid(row=16, column=0, sticky="w")
    g2_diameter_entry = ttk.Entry(frame, textvariable=g2_diameter_entry_var, width=12)
    g2_diameter_entry.grid(row=16, column=1, sticky="e")
    g2_diameter_slider = ttk.Scale(
        frame,
        from_=MIN_DIAMETER_MM,
        to=MAX_DIAMETER_MM,
        variable=g2_diameter_slider_var,
        orient="horizontal",
        length=260,
    )
    g2_diameter_slider.grid(row=17, column=0, columnspan=2, sticky="ew", pady=(2, 8))

    spool_diameter_label = ttk.Label(frame, text="Spool diameter (mm)")
    spool_diameter_label.grid(row=18, column=0, sticky="w")
    spool_diameter_entry = ttk.Entry(frame, textvariable=spool_diameter_entry_var, width=12)
    spool_diameter_entry.grid(row=18, column=1, sticky="e")
    spool_diameter_slider = ttk.Scale(
        frame,
        from_=MIN_SPOOL_DIAMETER_MM,
        to=MAX_SPOOL_DIAMETER_MM,
        variable=spool_diameter_slider_var,
        orient="horizontal",
        length=260,
    )
    spool_diameter_slider.grid(row=19, column=0, columnspan=2, sticky="ew", pady=(2, 0))

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

    rotation_slider.configure(command=update_rotation_entry)
    tilt_x_slider.configure(command=lambda value: update_tilt_label(tilt_x_label, "X", value))
    tilt_y_slider.configure(command=lambda value: update_tilt_label(tilt_y_label, "Y", value))
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

    def start_move():
        commit_rotation_entry()
        commit_diameter_entry(g1_diameter_entry_var, g1_diameter_slider_var)
        commit_diameter_entry(g2_diameter_entry_var, g2_diameter_slider_var)
        commit_spool_diameter_entry()
        motor_move_degrees = rotation_slider_var.get()
        speed = speed_var.get()
        if abs(motor_move_degrees) < 0.001:
            remaining_motor_move_rad[0] = 0.0
            status_label.configure(text="No rotation requested")
            return
        if speed <= 0.0:
            status_label.configure(text="Set Motor V above 0, then Start")
            return
        remaining_motor_move_rad[0] = math.radians(motor_move_degrees)
        status_label.configure(text=f"Moving motor {format_degrees(motor_move_degrees)} deg")

    def stop():
        remaining_motor_move_rad[0] = 0.0
        speed_var.set(0.0)
        status_label.configure(text="Stopped")

    def reset_position():
        motor_angle_rad[0] = 0.0
        remaining_motor_move_rad[0] = 0.0
        tilt_x_var.set(DEFAULT_TILT_DEGREES)
        tilt_y_var.set(DEFAULT_TILT_DEGREES)
        update_tilt_label(tilt_x_label, "X", DEFAULT_TILT_DEGREES)
        update_tilt_label(tilt_y_label, "Y", DEFAULT_TILT_DEGREES)
        arm1_drive_offset_rad[0] = 0.0
        arm2_drive_offset_rad[0] = 0.0
        g4_drive_offset_rad[0] = 0.0
        g5_drive_offset_rad[0] = 0.0
        g6_drive_offset_rad[0] = 0.0
        g4_spool_reference_rad[0] = 0.0
        g5_spool_reference_rad[0] = 0.0
        y_tilt_reference_rad[0] = 0.0
        x_tilt_reference_rad[0] = 0.0
        g6_reference_rad[0] = 0.0
        turntable_reference_rad[0] = 0.0
        last_spool_diameter_mm[0] = spool_diameter_slider_var.get()
        arm1_was_engaged[0] = arm1_engaged_var.get()
        arm2_was_engaged[0] = arm2_engaged_var.get()
        g4_was_engaged[0] = g4_engaged_var.get()
        g5_was_engaged[0] = g5_engaged_var.get()
        g6_was_engaged[0] = g6_engaged_var.get()
        with viewer.lock():
            for qpos_id in (
                g1_qpos_id,
                g2_qpos_id,
                g3_qpos_id,
                g4_qpos_id,
                g5_qpos_id,
                g6_qpos_id,
                turntable_qpos_id,
                tg2_qpos_id,
                tg4_qpos_id,
                tg6_qpos_id,
                plate_x_tilt_qpos_id,
                plate_y_tilt_qpos_id,
            ):
                data.qpos[qpos_id] = 0.0
            set_gear_layout(
                model,
                g1_diameter_slider_var.get(),
                g2_diameter_slider_var.get(),
                0.0,
                spool_diameter_mm=spool_diameter_slider_var.get(),
            )
            mujoco.mj_forward(model, data)
        status_label.configure(text="Position reset")

    ttk.Button(frame, text="Start", command=start_move).grid(row=5, column=0, sticky="ew", padx=(0, 6))
    ttk.Button(frame, text="Stop", command=stop).grid(row=5, column=1, sticky="ew")
    ttk.Button(frame, text="Reset position", command=reset_position).grid(
        row=6,
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
            speed = speed_var.get()
            speed_label.configure(text=f"Motor V: {speed:.1f} deg/s")
            plate_x_tilt_rad = math.radians(tilt_x_var.get())
            plate_y_tilt_rad = math.radians(tilt_y_var.get())
            g4_is_engaged = g4_engaged_var.get()
            g5_is_engaged = g5_engaged_var.get()
            g6_is_engaged = g6_engaged_var.get()
            tilt_y_slider.state(["disabled"] if g4_is_engaged else ["!disabled"])
            tilt_x_slider.state(["disabled"] if g5_is_engaged else ["!disabled"])
            if not g4_is_engaged:
                update_tilt_label(tilt_y_label, "Y", tilt_y_var.get())
            if not g5_is_engaged:
                update_tilt_label(tilt_x_label, "X", tilt_x_var.get())

            refresh_pip = now - last_pip_time[0] >= PIP_REFRESH_SECONDS
            pip_pixels = None

            with viewer.lock():
                if abs(remaining_motor_move_rad[0]) > 0.0 and speed > 0.0:
                    max_step = math.radians(speed) * elapsed
                    step = math.copysign(
                        min(abs(remaining_motor_move_rad[0]), max_step),
                        remaining_motor_move_rad[0],
                    )
                    motor_angle_rad[0] += step
                    remaining_motor_move_rad[0] -= step
                    if abs(remaining_motor_move_rad[0]) < 1e-6:
                        remaining_motor_move_rad[0] = 0.0
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
