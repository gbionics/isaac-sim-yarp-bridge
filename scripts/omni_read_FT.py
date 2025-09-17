# This script is executed the first time the script node computes, or the next time
# it computes after this script is modified or the 'Reset' button is pressed.
#
# The following callback functions may be defined in this script:
#     setup(db): Called immediately after this script is executed
#     compute(db): Called every time the node computes (should always be defined)
#     cleanup(db): Called when the node is deleted or the reset button is pressed
# Available variables:
#    db: og.Database The node interface - attributes are exposed in a namespace like db.inputs.foo and db.outputs.bar.
#                    Use db.log_error, db.log_warning to report problems in the compute function.
#    og: The omni.graph.core module

# Expects four inputs:
# - FTFrame [target] The link with respect to which express the measure, e.g. /World/ergoCubSN002/l_foot_front_ft
# - FTJoint [target] The fixed joint corresponding to the FT sensor, e.g. /World/ergoCubSN002/joints/l_foot_front_ft_sensor
# - flipMeasure [bool] A boolean to flip the measurement. By default (false), the measured wrench is the one exerted by the FT.
# - timestamp [double] The simulation time to convert to a ROS compatible timestamp

# Expects four outputs:
# - force [double[3]] The measured force
# - torque [double[3]] The measured torque
# - value_sec [int] The seconds for the value timestamp
# - value_nanosec [uint] The additional number of nanoseconds for the value timestamp


import math

import isaacsim.core.utils.rotations as rotations_utils
import isaacsim.core.utils.stage as stage_utils
import isaacsim.core.utils.xforms as xforms_utils
import numpy as np
from isaacsim.core.api.robots import RobotView
from pxr.UsdPhysics import Joint


class CustomState:

    def __init__(self):
        self.robot_object = None
        self.joint_index = None
        self.sensor_transform = np.eye(4)


def get_parent_robot(prim):
    parent_prim = prim.GetParent()
    while parent_prim.IsValid() and not parent_prim.HasAPI("IsaacRobotAPI"):
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

    a_H_b = np.eye(4)
    a_H_b[:3, :3] = a_R_b
    a_H_b[:3, 3] = a_P_b

    return a_H_b


def create_robot_object_ft(db):
    FTJoint = db.inputs.FTJoint

    if not FTJoint:
        db.log_error(f"FTJoint input is empty")
        return None

    stage = stage_utils.get_current_stage()

    FTJoint_path = FTJoint[0].pathString
    joint_prim = stage.GetPrimAtPath(FTJoint_path)
    joint_name = joint_prim.GetName()

    if not joint_prim.IsValid():
        db.log_error(f"The joint prim ({joint_prim}) is not valid")
        return None

    if not joint_prim.HasAPI("IsaacJointAPI"):
        db.log_error(f"The specified prim ({joint_prim}) is not a joint")
        return None

    parent_prim = get_parent_robot(joint_prim)

    if not parent_prim.IsValid():
        db.log_error(
            f"Failed to find a parent to {joint_prim} that has the IsaacRobotAPI."
            f"Is the joint attached to a robot?"
        )
        return None

    robot_object = RobotView(
        prim_paths_expr=str(parent_prim.GetPath()), name=f"robot_ft_{FTJoint_path}"
    )

    if joint_name in robot_object.dof_names:
        db.log_error(f"The FT joint {joint_name} is not fixed.")
        return None

    # See https://docs.isaacsim.omniverse.nvidia.com/5.0.0/py/source/extensions/isaacsim.core.api/docs/index.html#isaacsim.core.api.robots.RobotView.get_measured_joint_forces

    joint_index = robot_object.get_joint_index(joint_name) + 1

    robot_object.initialize()

    joint_api = Joint(joint_prim)
    b1 = joint_api.GetBody1Rel().GetTargets()[0]
    b1_prim = stage.GetPrimAtPath(b1)

    FTFrameInput = db.inputs.FTFrame

    if not FTFrameInput:
        print(
            f"No frame provided for FT joint {joint_name}. "
            f"FT measures will be provided in the {b1_prim.GetName()} frame."
        )
        return robot_object, joint_index, np.eye(4)

    FTFrame_prim = stage.GetPrimAtPath(FTFrameInput[0].pathString)

    if not FTFrame_prim.IsValid():
        db.log_error(f"The  prim ({FTFrame_prim}) is not valid")
        return None

    FTFrame_robot_prim = get_parent_robot(FTFrame_prim)

    if FTFrame_robot_prim != parent_prim:
        db.log_error(
            f"The specified prim ({FTFrame_prim}) has parent "
            f"{FTFrame_robot_prim} that is different from {parent_prim}."
        )
        return None

    relative_transform = get_a_H_b(FTFrame_prim, b1_prim)

    print(
        f"Info: expressing {joint_name} readings in {FTFrame_prim.GetName()} "
        f"with relative transform:"
    )
    print(relative_transform)

    return robot_object, joint_index, relative_transform


def internal_state():
    return CustomState()


def setup_ft(db: og.Database) -> bool:
    output = create_robot_object_ft(db)
    if not output:
        db.log_error("Setup failed")
        return False
    robot_object, joint_index, sensor_transform = output
    db.per_instance_state.robot_object = robot_object
    db.per_instance_state.joint_index = joint_index
    db.per_instance_state.sensor_transform = sensor_transform
    return True


def setup(db: og.Database) -> bool:
    return setup_ft(db)


def cleanup(db: og.Database):
    db.per_instance_state.robot_object = None
    db.per_instance_state.joint_index = None
    db.per_instance_state.sensor_transform = np.eye(4)


def compute(db: og.Database) -> bool:
    state = db.per_instance_state
    if not hasattr(state, "robot_object") or state.robot_object is None:
        setup_ft(db)
        return False

    out = state.robot_object.get_measured_joint_forces(
        joint_indices=[state.joint_index]
    )
    if out is None:
        db.log_warning(f"Failed to get measured joint forces. Running setup again")
        setup_ft(db)
        return False
    wrench = out.squeeze()

    position = db.per_instance_state.sensor_transform[:3, 3]
    rotation = db.per_instance_state.sensor_transform[:3, :3]

    force = wrench[:3]
    torque = wrench[3:]

    db.outputs.force = rotation @ force
    db.outputs.torque = rotation @ torque + np.cross(position, db.outputs.force)

    if hasattr(db.inputs, "flipMeasure") and db.inputs.flipMeasure:
        db.outputs.force *= -1
        db.outputs.torque *= -1

    if (
        hasattr(db.inputs, "timestamp")
        and hasattr(db.outputs, "value_sec")
        and hasattr(db.outputs, "value_nanosec")
    ):
        sec = math.floor(db.inputs.timestamp)
        nanosec = (db.inputs.timestamp - sec) * 1e9
        db.outputs.value_sec = int(sec)
        db.outputs.value_nanosec = int(nanosec)

    return True
