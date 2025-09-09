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

# Expects seven inputs:
# - deltaTime [float]: Time passed since last compute (in seconds)
# - refence_joint_names [list[str]]: The list of joint names (optional)
# - reference_position_commands [list[float]]: The list of joint position commands.
# - reference_velocity_commands [list[float]]: The list of joint velocity commands.
# - reference_effort_commands [list[float]]: The list of joint effort commands.
# - robot_prim [target]: The robot prim
# - domain_id [int]: The ROS2 domain ID (optional)
# - useDomainIDEnvVar [bool]: Define whether to get the domain ID from an env var or not (optional)

import dataclasses
import enum
import os

import isaacsim.core.utils.stage as stage_utils
import omni.graph.core as og
import rclpy
from isaacsim.core.api.robots import Robot
from rcl_interfaces.msg import ParameterType, ParameterValue, SetParametersResult
from rcl_interfaces.srv import GetParameters as GetParametersSrv
from rcl_interfaces.srv import SetParameters as SetParametersSrv
from rclpy.context import Context as ROS2Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node as ROS2Node


# TODO: these might be inputs, eventually added automatically if they are not existing
@dataclasses.dataclass
class ControlBoardSettings:
    node_name: str
    node_set_parameters_service_name: str
    node_get_parameters_service_name: str
    node_timeout: float
    joint_names: list[str]
    position_p_gains: list[float]
    position_i_gains: list[float]
    position_d_gains: list[float]
    position_max_integral: list[float]
    position_max_output: list[float]


# TODO: change this
settings = ControlBoardSettings(
    node_name="isaac_sim_control_board_state",
    node_set_parameters_service_name="ergocub/controlboard/set_parameters",
    node_get_parameters_service_name="ergocub/controlboard/get_parameters",
    node_timeout=0.1,
    joint_names=[
        "camera_tilt",
        "neck_pitch",
        "neck_roll",
        "neck_yaw",
        "torso_pitch",
        "torso_roll",
        "torso_yaw",
        "l_shoulder_pitch",
        "l_shoulder_roll",
        "l_shoulder_yaw",
        "l_elbow",
        "l_thumb_add",
        "l_thumb_prox",
        "l_thumb_dist",
        "l_index_add",
        "l_index_prox",
        "l_index_dist",
        "l_middle_prox",
        "l_middle_dist",
        "l_ring_prox",
        "l_ring_dist",
        "l_pinkie_prox",
        "l_pinkie_dist",
        "r_shoulder_pitch",
        "r_shoulder_roll",
        "r_shoulder_yaw",
        "r_elbow",
        "r_thumb_add",
        "r_thumb_prox",
        "r_thumb_dist",
        "r_index_add",
        "r_index_prox",
        "r_index_dist",
        "r_middle_prox",
        "r_middle_dist",
        "r_ring_prox",
        "r_ring_dist",
        "r_pinkie_prox",
        "r_pinkie_dist",
        "l_hip_pitch",
        "l_hip_roll",
        "l_hip_yaw",
        "l_knee",
        "l_ankle_pitch",
        "l_ankle_roll",
        "r_hip_pitch",
        "r_hip_roll",
        "r_hip_yaw",
        "r_knee",
        "r_ankle_pitch",
        "r_ankle_roll",
    ],
    position_p_gains=[100.0] * 51,
    position_i_gains=[1.0] * 51,
    position_d_gains=[20.0] * 51,
    position_max_integral=[10.0] * 51,
    position_max_output=[100.0] * 51,
)


class ControlMode(enum.IntEnum):
    IDLE = 0
    POSITION = 1
    POSITION_DIRECT = 2
    VELOCITY = 3
    TORQUE = 4


class ControlBoardState:
    joint_names: list[str]
    control_modes: list[int]
    previous_control_modes: list[int]
    position_p_gains: list[float]
    position_i_gains: list[float]
    position_d_gains: list[float]
    position_max_integral: list[float]
    position_max_output: list[float]

    def __init__(self, s: ControlBoardSettings):
        self.joint_names = s.joint_names
        n_joints = len(self.joint_names)
        self.control_modes = [ControlMode.POSITION] * n_joints
        self.previous_control_modes = [ControlMode.POSITION] * n_joints
        self.position_p_gains = s.position_p_gains
        self.position_i_gains = s.position_i_gains
        self.position_d_gains = s.position_d_gains
        self.position_max_integral = s.position_max_integral
        self.position_max_output = s.position_max_output


class ControlBoardNode(ROS2Node):
    def __init__(
        self,
        node_name: str,
        set_service_name: str,
        get_service_name: str,
        context,
        state,
    ):
        super().__init__(node_name, context=context)
        self.set_srv = self.create_service(
            SetParametersSrv, set_service_name, self.callback_set_parameters
        )
        self.get_srv = self.create_service(
            GetParametersSrv, get_service_name, self.callback_get_parameters
        )
        self.state = state

    def set_vector_parameter(self, name: str, value: list):
        if hasattr(self.state, name):
            if len(value) == len(getattr(self.state, name)):
                if len(getattr(self.state, name)) > 0 and not all(
                    isinstance(v, type(getattr(self.state, name)[0])) for v in value
                ):
                    return False, (
                        f"Invalid type for parameter {name}: "
                        f"expected {type(getattr(self.state, name)[0])}"
                    )
                setattr(self.state, name, value)
                return True, "accepted"
            else:
                return False, f"Invalid length for parameter {name}"
        else:
            return False, f"Unknown parameter {name}"

    def set_scalar_parameter(self, name: str, index, value):
        if hasattr(self.state, name):
            vec = getattr(self.state, name)
            if 0 <= index < len(vec):
                if not isinstance(vec[index], type(value)):
                    return False, (
                        f"Invalid type for parameter {name}[{index}]: "
                        f"expected {type(vec[index])}, got {type(value)}"
                    )
                vec[index] = value
                return True, "accepted"
            else:
                return False, (
                    f"Index {index} out of range for "
                    f"parameter {name} with size {len(vec)}."
                )
        else:
            return False, f"Unknown parameter {name}"

    def get_vector_parameter(self, name: str):
        output = ParameterValue()
        output.type = ParameterType.PARAMETER_NOT_SET

        if hasattr(self.state, name) and len(getattr(self.state, name)) > 0:
            vec = getattr(self.state, name)
            if all(isinstance(v, int) for v in vec):
                output.type = ParameterType.PARAMETER_INTEGER_ARRAY
                output.integer_array_value = getattr(self.state, name)
            elif all(isinstance(v, float) for v in vec):
                output.type = ParameterType.PARAMETER_DOUBLE_ARRAY
                output.double_array_value = getattr(self.state, name)
            elif all(isinstance(v, str) for v in vec):
                output.type = ParameterType.PARAMETER_STRING_ARRAY
                output.string_array_value = getattr(self.state, name)
            else:
                output.type = ParameterType.PARAMETER_NOT_SET
                print(
                    f"Unsupported type for parameter {name}: {[type(v) for v in vec]}. "
                    f"This should not have happened!"
                )
        return output

    def get_scalar_parameter(self, name: str, index: int):
        output = ParameterValue()
        output.type = ParameterType.PARAMETER_NOT_SET
        if hasattr(self.state, name):
            vec = getattr(self.state, name)
            if 0 <= index < len(vec):
                v = vec[index]
                if isinstance(v, int):
                    output.type = ParameterType.PARAMETER_INTEGER
                    output.integer_value = v
                elif isinstance(v, float):
                    output.type = ParameterType.PARAMETER_DOUBLE
                    output.double_value = v
                elif isinstance(v, str):
                    output.type = ParameterType.PARAMETER_STRING
                    output.string_value = v
                else:
                    print(
                        f"Unsupported type for parameter {name}[{index}]: {type(v)}. "
                        f"This should not have happened!"
                    )

        return output

    @staticmethod
    def extract_indexed_name(name: str):
        if "[" in name and name.endswith("]"):
            base_name = name[: name.index("[")]
            try:
                index = int(name[name.index("[") + 1 : -1])
                return base_name, index
            except ValueError:
                pass
        return name, None

    def callback_set_parameters(self, request, response):
        results = []
        for p in request.parameters:
            name, index = self.extract_indexed_name(p.name)
            value = rclpy.parameter.parameter_value_to_python(p.value)
            r = SetParametersResult()
            if index is not None:
                r.successful, r.reason = self.set_scalar_parameter(name, index, value)
            else:
                if isinstance(value, list):
                    r.successful, r.reason = self.set_vector_parameter(name, value)
                else:
                    r.successful = False
                    r.reason = f"Parameter {p.name} expects a list value"

            results.append(r)
        response.results = results
        return response

    def callback_get_parameters(self, request, response):
        results = []
        for n in request.names:
            name, index = self.extract_indexed_name(n)
            if index is not None:
                param_value = self.get_scalar_parameter(name, index)
            else:
                param_value = self.get_vector_parameter(name)
            results.append(param_value)
        response.values = results
        return response


class ControlBoardPID:
    kp: float
    ki: float
    kd: float
    max_integral: float
    max_output: float
    reference: float | None
    measurement: float
    integral_state: float
    previous_error: float
    output: float

    def __init__(self, p=0.0, i=0.0, d=0.0, max_integral=0.0, max_output=0.0):
        self.kp = p
        self.ki = i
        self.kd = d
        self.max_integral = max_integral
        self.max_output = max_output

        self.integral_state = 0.0
        self.previous_error = 0.0
        self.reference = None

    def set_refence(self, reference):
        self.reference = reference

    def compute(self, delta_time, measurement):

        if not self.reference:
            self.reference = measurement

        error = self.reference - measurement
        # Proportional term
        p_term = self.kp * error

        # Integral term
        self.integral_state += error * delta_time
        self.integral_state = max(
            min(self.integral_state, self.max_integral), -self.max_integral
        )
        i_term = self.ki * self.integral_state

        # Derivative term
        derivative = (
            (error - self.previous_error) / delta_time if delta_time > 0 else 0.0
        )
        self.previous_error = error
        d_term = self.kd * derivative

        # Total output
        self.output = p_term + i_term + d_term
        self.output = max(min(self.output, self.max_output), -self.max_output)

        return self.output

    def reset(self):
        self.integral_state = 0.0
        self.previous_error = 0.0

    def get_output(self):
        return self.output

    def get_error(self):
        return self.previous_error


class ControlBoardData:
    def __init__(self):
        self.state = None
        self.pids = None
        self.context = None
        self.node = None
        self.executor = None
        self.robot = None
        self.robot_joint_indices = None
        self.initialized = False


def choose_domain_id(db) -> int:
    """
    Replicates ROS2Context selection logic:
      - if use_env_var and ROS_DOMAIN_ID exists: use it
      - otherwise use domain_id_input
    """
    domain_id_input = (
        db.inputs.domain_id
        if hasattr(db.inputs, "domain_id") and db.inputs.domain_id is not None
        else 0
    )
    use_env_var = (
        db.inputs.useDomainIDEnvVar
        if hasattr(db.inputs, "useDomainIDEnvVar")
        and db.inputs.useDomainIDEnvVar is not None
        else True
    )

    if use_env_var:
        s = os.environ.get("ROS_DOMAIN_ID")
        if s is not None:
            try:
                return int(s)
            except Exception:
                pass
    return int(domain_id_input)


def create_robot_object(db: og.Database, name, joint_names):
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
    robot = Robot(prim_path=robot_prim, name=name)
    robot.initialize()

    joint_indices = []
    for j in joint_names:
        if j not in robot.dof_names:
            db.log_error(f"Joint {j} not found in the robot")
            return None
        j_robot_index = robot.dof_names.index(j)
        joint_indices.append(j_robot_index)

    return robot, joint_indices


def setup(db: og.Database):
    domain_id = choose_domain_id(db=db)
    db.per_instance_state.state = ControlBoardState(s=settings)
    db.per_instance_state.pids = {}
    db.per_instance_state.context = ROS2Context()
    db.per_instance_state.context.init(domain_id=domain_id)
    db.per_instance_state.node = ControlBoardNode(
        node_name=settings.node_name,
        set_service_name=settings.node_set_parameters_service_name,
        get_service_name=settings.node_get_parameters_service_name,
        context=db.per_instance_state.context,
        state=db.per_instance_state.state,
    )
    db.per_instance_state.executor = SingleThreadedExecutor(
        context=db.per_instance_state.context
    )
    db.per_instance_state.executor.add_node(db.per_instance_state.node)
    robot, robot_joint_indices = create_robot_object(
        db, name=settings.node_name, joint_names=settings.joint_names
    )
    if not robot:
        db.per_instance_state.initialized = False
        db.log_error("Failed to create robot object")
        return

    db.per_instance_state.robot = robot
    db.per_instance_state.robot_joint_indices = robot_joint_indices

    db.per_instance_state.initialized = True


def cleanup(db: og.Database):
    if (
        not hasattr(db.per_instance_state, "initialized")
        or not db.per_instance_state.initialized
    ):
        return

    db.per_instance_state.executor.shutdown()
    db.per_instance_state.node.destroy_node()
    db.per_instance_state.context.destroy()
    db.per_instance_state.robot = None
    db.per_instance_state.initialized = False


def get_pid_output(
    db: og.Database,
    joint_index: int,
    measured_position: float,
    measured_velocity: float,
    measured_effort: float,
    lower_limit: float,
    upper_limit: float,
    max_velocity: float,
) -> float:
    script_state = db.per_instance_state
    name = script_state.state.joint_names[joint_index]
    delta_time = (
        db.inputs.deltaTime
        if hasattr(db.inputs, "deltaTime") and db.inputs.deltaTime is not None
        else 0.0
    )

    if joint_index not in script_state.pids:
        script_state.pids[joint_index] = {}

    control_mode = script_state.state.control_modes[joint_index]
    if control_mode not in script_state.pids[joint_index]:
        if control_mode == ControlMode.POSITION:  # position control
            script_state.pids[joint_index][control_mode] = ControlBoardPID(
                p=script_state.state.position_p_gains[joint_index],
                i=script_state.state.position_i_gains[joint_index],
                d=script_state.state.position_d_gains[joint_index],
                max_integral=script_state.state.position_max_integral[joint_index],
                max_output=script_state.state.position_max_output[joint_index],
            )
        if control_mode == ControlMode.IDLE:
            script_state.pids[joint_index][control_mode] = None
        else:
            script_state.pids[joint_index][control_mode] = None
            print(
                f"Unsupported control mode {control_mode} "
                f"for joint {script_state.state.joint_names[joint_index]}"
            )

    pid = script_state.pids[joint_index][control_mode]
    if pid is None:
        return 0.0

    if control_mode == ControlMode.POSITION:
        # TODO: add smoother

        if script_state.state.previous_control_modes[joint_index] != control_mode:
            pid.reset()
        script_state.state.previous_control_modes[joint_index] = control_mode

        if (
            hasattr(db.inputs, "reference_joint_names")
            and db.inputs.reference_joint_names is not None
            and name in db.inputs.reference_joint_names
        ):
            cmd_index = db.inputs.reference_joint_names.index(name)
            if (
                hasattr(db.inputs, "reference_position_commands")
                and db.inputs.reference_position_commands is not None
                and cmd_index < len(db.inputs.reference_position_commands)
                and db.inputs.reference_position_commands[cmd_index] is not None
            ):
                reference = db.inputs.reference_position_commands[cmd_index]
                reference = max(min(reference, upper_limit), lower_limit)
                pid.set_refence(reference)

        return pid.compute(delta_time, measured_position)
    else:
        return 0.0


def compute(db: og.Database):
    if (
        not hasattr(db.per_instance_state, "initialized")
        or not db.per_instance_state.initialized
    ):
        setup(db)

    if not db.per_instance_state.initialized:
        return False

    script_state = db.per_instance_state

    joint_state = script_state.robot.get_joints_state()
    if joint_state is None:
        db.log_warning(
            f"Failed to get joint states in {script_state.robot.name}. "
            f"Initializing again."
        )
        script_state.robot.initialize()
        return False

    if rclpy.ok(context=script_state.context):
        script_state.executor.spin_once(timeout_sec=settings.node_timeout)

    output_effort = []
    for i in range(len(script_state.state.joint_names)):
        robot_index = script_state.robot_joint_indices[i]
        measured_position = joint_state.positions[robot_index]
        measured_velocity = joint_state.velocities[robot_index]
        measured_effort = joint_state.efforts[robot_index]
        has_limits = script_state.robot.dof_properties["hasLimits"][robot_index]

        if has_limits:
            lower_limit = script_state.robot.dof_properties["lower"][robot_index]
            upper_limit = script_state.robot.dof_properties["upper"][robot_index]
            max_velocity = script_state.robot.dof_properties["maxVelocity"][robot_index]
            max_effort = script_state.robot.dof_properties["maxEffort"][robot_index]
        else:
            lower_limit = -float("inf")
            upper_limit = float("inf")
            max_velocity = float("inf")
            max_effort = float("inf")

        effort = get_pid_output(
            db=db,
            joint_index=i,
            measured_position=measured_position,
            measured_velocity=measured_velocity,
            measured_effort=measured_effort,
            lower_limit=lower_limit,
            upper_limit=upper_limit,
            max_velocity=max_velocity,
        )
        effort = max(min(effort, max_effort), -max_effort)
        output_effort.append(effort)

    script_state.robot.set_joint_efforts(
        efforts=output_effort, joint_indices=script_state.robot_joint_indices
    )

    return True


def internal_state():
    return ControlBoardData()
