from pathlib import Path

import mujoco

try:
    from sim.generate_room_meshes import ensure_room_meshes
except ModuleNotFoundError:
    from generate_room_meshes import ensure_room_meshes


SIM_DIR = Path(__file__).resolve().parent
MODEL_PATH = SIM_DIR / "simple_motor_gear.xml"
OUTPUT_PATH = SIM_DIR / "simple_motor_gear.mjb"

REQUIRED_BODIES = (
    "turntable",
    "G1",
    "G2",
    "G3",
    "G4",
    "G5",
    "G6",
    "tilt_plate_body",
    "target_origin_frame",
    "camera_target",
)
REQUIRED_JOINTS = ("gear_hinge", "plate_x_tilt", "plate_y_tilt", "turntable_yaw")
REQUIRED_CAMERAS = ("view", "spotlight_camera")
REQUIRED_LIGHTS = ("plate_spotlight",)
REQUIRED_ROOM_MESHES = (
    "left_wall_surface",
    "right_wall_surface",
    "back_wall_surface",
    "ceiling_surface",
)
REQUIRED_GEOMS = (
    "turntable_disk",
    "turntable_mark",
    "upper_turntable_disk",
    "upper_turntable_mark",
    "target_axis_x",
    "target_axis_y",
    "target_axis_z",
    "camera_target_sphere",
)


def require_named_objects(model, object_type, names):
    for name in names:
        if mujoco.mj_name2id(model, object_type, name) < 0:
            raise RuntimeError(f"Required MuJoCo object is missing: {name}")


def main():
    ensure_room_meshes()
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    require_named_objects(model, mujoco.mjtObj.mjOBJ_BODY, REQUIRED_BODIES)
    require_named_objects(model, mujoco.mjtObj.mjOBJ_JOINT, REQUIRED_JOINTS)
    require_named_objects(model, mujoco.mjtObj.mjOBJ_CAMERA, REQUIRED_CAMERAS)
    require_named_objects(model, mujoco.mjtObj.mjOBJ_LIGHT, REQUIRED_LIGHTS)
    require_named_objects(model, mujoco.mjtObj.mjOBJ_MESH, REQUIRED_ROOM_MESHES)
    require_named_objects(model, mujoco.mjtObj.mjOBJ_GEOM, REQUIRED_GEOMS)
    for mesh_name in REQUIRED_ROOM_MESHES:
        mesh_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_MESH,
            mesh_name,
        )
        if model.mesh_vertnum[mesh_id] < 1000:
            raise RuntimeError(
                f"Room mesh is too coarse for spotlight rendering: {mesh_name}"
            )

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    mujoco.mj_saveModel(model, str(OUTPUT_PATH))

    compiled_model = mujoco.MjModel.from_binary_path(str(OUTPUT_PATH))
    compiled_data = mujoco.MjData(compiled_model)
    mujoco.mj_forward(compiled_model, compiled_data)

    print(
        f"Validated {MODEL_PATH.name}: "
        f"{model.nbody} bodies, {model.njnt} joints, {model.ngeom} geoms"
    )
    print(f"Wrote {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
