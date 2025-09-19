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
# - robot_prim [target] The robot prim
# - joint_names [token[]] The names of the joints to set the gains for
# - desired_kps [double[]] The desired position gains (optional, if not provided zero gains will be used)
# - desired_kds [double[]] The desired velocity gains (optional, if not provided zero gains will be used)

import isaacsim.core.utils.stage as stage_utils
import omni.graph.core as og
from isaacsim.core.api.robots import Robot


def compute(db: og.Database) -> bool:
    if not hasattr(db.inputs, "robot_prim") or db.inputs.robot_prim is None:
        db.log_error("robot_prim input is not set")
        return False

    if not hasattr(db.inputs, "joint_names") or db.inputs.joint_names is None:
        db.log_error("joint_names input is not set")
        return False
    joint_names = db.inputs.joint_names

    desired_kps = db.inputs.desired_kps if hasattr(db.inputs, "desired_kps") else None
    if desired_kps is None or len(desired_kps) == 0:
        desired_kps = [0.0] * len(joint_names)
    if len(desired_kps) != len(joint_names):
        db.log_error("Length of desired_kp does not match length of joint_names")
        return False

    desired_kds = db.inputs.desired_kds if hasattr(db.inputs, "desired_kds") else None
    if desired_kds is None or len(desired_kds) == 0:
        desired_kds = [0.0] * len(joint_names)
    if len(desired_kds) != len(joint_names):
        db.log_error("Length of desired_kd does not match length of joint_names")
        return False

    stage = stage_utils.get_current_stage()
    robot_prim_path = db.inputs.robot_prim[0].pathString
    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    if not robot_prim.IsValid():
        db.log_error(f"robot_prim path {robot_prim_path} is not valid")
        return False
    if not robot_prim.HasAPI("IsaacRobotAPI"):
        db.log_error(f"The specified prim ({robot_prim}) is not a robot")
        return False
    robot = Robot(prim_path=str(robot_prim.GetPath()), name="robot_gains_set")
    robot.initialize()
    controller = robot.get_articulation_controller()

    joint_indices = []
    for j in joint_names:
        if j not in robot.dof_names:
            db.log_error(f"Joint {j} not found in the robot")
            return False
        j_robot_index = robot.dof_names.index(j)
        joint_indices.append(j_robot_index)

    kps, kds = controller.get_gains()
    for idx, joint_index in enumerate(joint_indices):
        kps[joint_index] = desired_kps[idx]
        kds[joint_index] = desired_kds[idx]

    controller.set_gains(kps, kds)

    # Print the names of the joints and their new gains
    for idx, joint_index in enumerate(joint_indices):
        print(
            f"Set gains for joint {joint_names[idx]}: "
            f"kp={desired_kps[idx]}, kd={desired_kds[idx]}"
        )

    return True
