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

# Expects four + nc inputs, where nc is the number of control boards:
# - robot_prim [target] The robot prim. Read only during initialization
# - joint_names [token[]] The names of the joints to set the gains for. Read only during initialization
# - desired_joint_damping [double[]] The desired joints damping. Read only during initialization. If not available, the node will set all joints damping to zero
# - control_board_names [token[]] The names of the control boards. Read only during initialization
# - "control_board_X"_desired_efforts [double[]] The desired efforts for control board X. All the input efforts are stacked according to the order of the control boards

import isaacsim.core.utils.stage as stage_utils
import omni.graph.core as og
from isaacsim.core.api.robots import RobotView


class SetRobotEffortsState:
    def __init__(self):
        self.initialized = False
        self.robot = None
        self.robot_joint_indices = []
        self.desired_joint_damping = []
        self.control_board_names = []
        self.exec_in_counter = 0


def internal_state():
    return SetRobotEffortsState()


def create_robot_object_robot_efforts(db: og.Database, joint_names):
    if not hasattr(db.inputs, "robot_prim") or db.inputs.robot_prim is None:
        db.log_error("robot_prim input is not set")
        return None

    stage = stage_utils.get_current_stage()
    robot_prim_path = db.inputs.robot_prim[0].pathString
    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    if not robot_prim.IsValid():
        db.log_error(f"robot_prim path {robot_prim_path} is not valid")
        return None
    if not robot_prim.HasAPI("IsaacRobotAPI"):
        db.log_error(f"The specified prim ({robot_prim}) is not a robot")
        return None
    robot = RobotView(
        prim_paths_expr=str(robot_prim.GetPath()), name="set_robot_efforts"
    )

    joint_indices = []
    for j in joint_names:
        if j not in robot.dof_names:
            db.log_error(f"Joint {j} not found in the robot")
            return None
        j_robot_index = robot.dof_names.index(j)
        joint_indices.append(j_robot_index)

    return robot, joint_indices


def initialize_and_reset_gains(db: og.Database):
    if (
        not hasattr(db.per_instance_state, "initialized")
        or not db.per_instance_state.initialized
    ):
        db.log_error("Node is not initialized")
        return

    robot = db.per_instance_state.robot
    joint_indices = db.per_instance_state.robot_joint_indices
    if not robot or not joint_indices:
        db.log_error("Robot or joint indices are not properly initialized")
        return
    damping = db.per_instance_state.desired_joint_damping
    if damping is None or len(damping) != len(joint_indices):
        db.log_error("Desired joint damping is not properly initialized")
        return

    robot.initialize()
    robot.set_effort_modes(mode="force", joint_indices=joint_indices)
    robot.set_gains(
        kps=[0.0] * len(joint_indices),
        kds=damping,
        joint_indices=joint_indices,
    )

    print("Robot efforts control initialized and gains reset for joints:")
    for j in joint_indices:
        print(f" - {robot.dof_names[j]} (index {j})")


def setup_robot_efforts(db: og.Database):

    db.per_instance_state.initialized = False

    if not hasattr(db.inputs, "joint_names"):
        db.log_error("joint_names input is not set")
        return
    joint_names = db.inputs.joint_names

    if not hasattr(db.inputs, "control_board_names"):
        db.log_error("control_board_names input is not set")
        return
    db.per_instance_state.control_board_names = db.inputs.control_board_names

    output = create_robot_object_robot_efforts(db, joint_names=joint_names)

    if not output:
        db.per_instance_state.initialized = False
        db.log_error("Failed to create robot object")
        return

    robot, robot_joint_indices = output
    if not robot or not robot_joint_indices:
        db.per_instance_state.initialized = False
        db.log_error("Either the robot or the joint indices are invalid")
        return

    if hasattr(db.inputs, "desired_joint_damping"):
        db.per_instance_state.desired_joint_damping = db.inputs.desired_joint_damping
        if len(db.per_instance_state.desired_joint_damping) != len(robot_joint_indices):
            db.log_error(
                "Length of desired_joint_damping does not match number of joints"
            )
            db.per_instance_state.initialized = False
            return
    else:
        db.per_instance_state.desired_joint_damping = [0.0] * len(robot_joint_indices)

    db.per_instance_state.robot = robot
    db.per_instance_state.robot_joint_indices = robot_joint_indices

    db.per_instance_state.exec_in_counter = 0
    db.per_instance_state.initialized = True

    initialize_and_reset_gains(db)


def setup(db: og.Database):
    setup_robot_efforts(db)


def cleanup(db: og.Database):
    if (
        not hasattr(db.per_instance_state, "initialized")
        or not db.per_instance_state.initialized
    ):
        return

    db.per_instance_state.robot = None
    db.per_instance_state.exec_in_counter = 0
    db.per_instance_state.initialized = False


def compute(db: og.Database) -> bool:

    state = db.per_instance_state
    if not hasattr(state, "initialized") or not state.initialized:
        setup_robot_efforts(db)
        return False

    robot = state.robot
    joint_indices = state.robot_joint_indices
    control_board_names = state.control_board_names

    state.exec_in_counter += 1
    if state.exec_in_counter < len(control_board_names):
        # Every control board connects to the execIn of this node, so we need to wait
        # until we have received at least one execIn from each control board before
        # setting the efforts.
        return True
    state.exec_in_counter = 0

    kps, kds = robot.get_gains(joint_indices=joint_indices)
    desired_kds = db.per_instance_state.desired_joint_damping
    if (
        kps is None
        or kds is None
        or any(
            kp != 0.0 or kd != desired_kd
            for kp, kd, desired_kd in zip(kps.squeeze(), kds.squeeze(), desired_kds)
        )
    ):
        db.log_warning("Gains are not as expected, reinitializing")
        initialize_and_reset_gains(db)
        return False

    measured_efforts = robot.get_measured_joint_efforts(joint_indices=joint_indices)
    if measured_efforts is None:
        db.log_warning("Trying to reinitialize the robot efforts control")
        initialize_and_reset_gains(db)
        return False

    all_desired_efforts = []
    for board_name in control_board_names:
        input_name = f"{board_name}_desired_efforts"
        if not hasattr(db.inputs, input_name):
            db.log_error(f"Input {input_name} is not set")
            return False
        desired_efforts = getattr(db.inputs, input_name)
        all_desired_efforts.extend(desired_efforts)

    if len(all_desired_efforts) != len(joint_indices):
        db.log_error(
            f"Number of desired efforts ({len(all_desired_efforts)}) "
            f"does not match number of joints ({len(joint_indices)})"
        )
        return False

    robot.set_joint_efforts(efforts=all_desired_efforts, joint_indices=joint_indices)

    return True
