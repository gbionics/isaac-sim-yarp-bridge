"""
This script adds the realsense and the IMUs on a freshly imported ergoCub USD.
"""

import isaacsim.core.utils.physics as physics_utils
import isaacsim.core.utils.prims as prims_utils
import isaacsim.core.utils.rotations as rotations_utils
import isaacsim.core.utils.stage as stage_utils
import isaacsim.core.utils.xforms as xforms_utils
import numpy as np
import omni.kit.commands
from isaacsim.storage.native import get_assets_root_path
from omni.isaac.core.prims import XFormPrim
from pxr import Gf


def nest_robot(robot_name, subname):
    prefix = "/World/" + robot_name
    temp_name = "/World/" + subname
    final_name = prefix + "/" + subname
    prims_utils.move_prim(prefix, temp_name)
    prims_utils.create_prim(prefix)
    prims_utils.move_prim(temp_name, final_name)
    return final_name


def import_realsense(prim_path):
    assets_root = get_assets_root_path()
    if assets_root is None:
        print("Failed to get assets folder")
        return None
    print("Asset folder: " + assets_root)
    usd_path = assets_root + "/Isaac/Sensors/Intel/RealSense/rsd455.usd"
    return stage_utils.add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)


def remove_realsense_physics(prim_path):
    stage = stage_utils.get_current_stage()
    prim = stage.GetPrimAtPath(prim_path + "/RSD455")
    print("The prim " + str(prim) + " has the following APIs")

    for s in prim.GetAppliedSchemas():
        print(s)

    out = prim.RemoveAppliedSchema("PhysicsMassAPI")
    if out:
        print("PhysicsMassAPI removed")

    out = prim.RemoveAppliedSchema("PhysicsRigidBodyAPI")
    if out:
        print("PhysicsRigidBodyAPI removed")

    out = prim.RemoveAppliedSchema("PhysxRigidBodyAPI")
    if out:
        print("PhysxRigidBodyAPI removed")

    physics_utils.set_rigid_body_enabled(False, str(prim.GetPath()))
    print("Set physics:rigidBodyEnabled to False")

    print("Removed physics from prim")


def move_realsense_to_robot(realsense_path, destination_path):

    prims_utils.move_prim(realsense_path, destination_path)
    prim = XFormPrim(destination_path)
    prim.set_local_pose(
        translation=np.array([0.0, 0.0, 0.0]),
        orientation=np.array([1.0, 0.0, 0.0, 0.0]),
    )
    print("Moved RealSense in " + destination_path)


def deactivate_prim(path):
    stage = stage_utils.get_current_stage()
    robot_prim = stage.GetPrimAtPath(path)
    robot_prim.SetActive(False)
    print("Deactivated " + path)


def add_realsense_to_robot(realsense_path, robot_path):
    realsense_prim = import_realsense(realsense_path)

    if realsense_prim is None or not realsense_prim.IsValid():
        print("Failed to add the RealSense to the stage")
        return

    print("Realsense imported on the stage")
    remove_realsense_physics(realsense_path)
    move_realsense_to_robot(
        realsense_path=realsense_path,
        destination_path=robot_path + "/realsense/sensors",
    )
    # Deactivate old visuals to avoid them being visible in the camera
    deactivate_prim(robot_path + "/realsense/visuals")
    # Disable the realsense IMU since physics:rigidBodyEnabled is set to False
    # for the parent body (RSD455) and thus it would not read anything
    deactivate_prim(robot_path + "/realsense/sensors/RSD455/Imu_Sensor")

    print("Done attaching a Realsense to " + robot_path)


def find_prims_by_name(prim_names):
    stage = stage_utils.get_current_stage()
    found_prims = [x for x in stage.Traverse() if x.GetName() in prim_names]
    return found_prims


def get_parent_link(prim):
    parent_prim = prim
    while parent_prim.IsValid() and not parent_prim.HasAPI("IsaacLinkAPI"):
        parent_prim = parent_prim.GetParent()

    return parent_prim


def get_a_H_b(a_prim, b_prim):
    pos_a, quat_a = xforms_utils.get_world_pose(str(a_prim.GetPath()))
    pos_b, quat_b = xforms_utils.get_world_pose(str(b_prim.GetPath()))
    w_R_a = rotations_utils.quat_to_rot_matrix(quat_a)
    w_R_b = rotations_utils.quat_to_rot_matrix(quat_b)

    a_R_w = w_R_a.T
    a_R_b = a_R_w @ w_R_b
    a_P_b = a_R_w @ (pos_b - pos_a)

    return a_P_b, rotations_utils.rot_matrix_to_quat(a_R_b)


def add_IMU_sensor(imu_frame_prim):

    imu_name = str(imu_frame_prim.GetName())
    parent_link = get_parent_link(imu_frame_prim)

    if not parent_link.IsValid():
        print("Failed to find parent link for " + str(imu_frame_prim.GetPath()))
        return

    parent_path = str(parent_link.GetPath())
    print("The parent link for " + imu_name + " is " + parent_path)

    translation, quaternion = get_a_H_b(parent_link, imu_frame_prim)

    print("Relative position: " + str(translation))
    print("Relative quaternion: " + str(quaternion))

    success, isaac_sensor_prim = omni.kit.commands.execute(
        "IsaacSensorCreateImuSensor",
        path=imu_name + "_sensor",
        parent=parent_path,
        translation=Gf.Vec3d(translation[0], translation[1], translation[2]),
        orientation=Gf.Quatd(
            quaternion[0], quaternion[1], quaternion[2], quaternion[3]
        ),
    )
    return success


def add_IMU_sensors(list_IMUs):
    for p in find_prims_by_name(list_IMUs):
        ok = add_IMU_sensor(p)
        if ok:
            print("Added IMU: " + str(p.GetPath()))
        else:
            print("Failed to add IMU: " + str(p.GetPath()))


realsense_prim_path = "/World/rsd455"
robot_name = "ergoCubSN002"
imus = ["waist_imu_0", "head_imu_0"]

robot_prim_path = nest_robot(robot_name, "robot")
add_realsense_to_robot(realsense_prim_path, robot_prim_path)
add_IMU_sensors(imus)
