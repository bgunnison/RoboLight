from pathlib import Path

import mujoco


SIM_DIR = Path(__file__).resolve().parent
MODEL_PATH = SIM_DIR / "simple_motor_gear.xml"
OUTPUT_PATH = SIM_DIR / "simple_motor_gear.mjb"

REQUIRED_BODIES = ("turntable", "G1", "G2", "G3", "G4", "G5", "G6", "tilt_plate_body")
REQUIRED_JOINTS = ("gear_hinge", "plate_x_tilt", "plate_y_tilt", "turntable_yaw")
REQUIRED_CAMERAS = ("view", "spotlight_camera")
REQUIRED_GEOMS = (
    "turntable_disk",
    "turntable_mark",
    "upper_turntable_disk",
    "upper_turntable_mark",
)


def require_named_objects(model, object_type, names):
    for name in names:
        if mujoco.mj_name2id(model, object_type, name) < 0:
            raise RuntimeError(f"Required MuJoCo object is missing: {name}")


def main():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    require_named_objects(model, mujoco.mjtObj.mjOBJ_BODY, REQUIRED_BODIES)
    require_named_objects(model, mujoco.mjtObj.mjOBJ_JOINT, REQUIRED_JOINTS)
    require_named_objects(model, mujoco.mjtObj.mjOBJ_CAMERA, REQUIRED_CAMERAS)
    require_named_objects(model, mujoco.mjtObj.mjOBJ_GEOM, REQUIRED_GEOMS)

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
