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

# Expects 36 inputs:
# - timestamp [double]: The current simulation time (in seconds)
# - deltaTime [double]: Time passed since last compute (in seconds)
# - reference_joint_names [token[]]: The list of joint names for the input reference commands.
# - reference_position_commands [double[]]: The list of joint position commands.
# - reference_velocity_commands [double[]]: The list of joint velocity commands.
# - reference_effort_commands [double[]]: The list of joint effort commands.
# - robot_prim [target]: The robot prim. Read only during setup.
# - domain_id [uchar]: The ROS2 domain ID (optional). Read only during setup.
# - useDomainIDEnvVar [bool]: Define whether to get the domain ID from an env var or not (optional). Read only during setup.
# - node_name [str]: The ROS2 node name. Read only during setup.
# - node_set_parameters_service_name [str]: The ROS2 set parameters service name. Read only during setup.
# - node_get_parameters_service_name [str]: The ROS2 get parameters service name. Read only during setup.
# - node_state_topic_name [str]: The ROS2 joint state topic name. Read only during setup.
# - node_motor_state_topic_name [str]: The ROS2 motor state topic name. Read only during setup.
# - node_timeout [double]: The ROS2 node timeout (in seconds). Read only during setup.
# - joint_names [token[]]: The list of joint names. Read only during setup.
# - position_p_gains [double[]]: The list of position PID proportional gains. Read only during setup.
# - position_i_gains [double[]]: The list of position PID integral gains. Read only during setup.
# - position_d_gains [double[]]: The list of position PID derivative gains. Read only during setup.
# - position_max_integral [double[]]: The list of position PID maximum integral terms. Read only during setup.
# - position_max_output [double[]]: The list of position PID maximum output terms. Read only during setup.
# - position_max_error [double[]]: The list of position PID maximum error terms. Read only during setup.
# - position_default_velocity [double]: The default velocity used to smooth position commands. Read only during setup.
# - compliant_stiffness [double[]]: The list of compliant mode stiffness. Read only during setup.
# - compliant_damping [double[]]: The list of compliant mode damping. Read only during setup.
# - velocity_p_gains [double[]]: The list of velocity PID proportional gains. Read only during setup.
# - velocity_i_gains [double[]]: The list of velocity PID integral gains. Read only during setup.
# - velocity_d_gains [double[]]: The list of velocity PID derivative gains. Read only during setup.
# - velocity_max_integral [double[]]: The list of velocity PID maximum integral terms. Read only during setup.
# - velocity_max_output [double[]]: The list of velocity PID maximum output terms. Read only during setup.
# - velocity_max_error [double[]]: The list of velocity PID maximum error terms. Read only during setup.
# - gearbox_ratios [double[]]: The list of gearbox ratios (motor to joint). Read only during setup.
# - motor_torque_constants [double[]]: The list of motor torque constants. Read only during setup.
# - motor_current_noise_variance [double[]]: The list of motor current noise variances. Read only during setup.
# - motor_spring_stiffness [double[]]: The list of motor springs stiffness. Read only during setup.
# - motor_max_currents [double[]]: The list of motor maximum currents. Read only during setup.

import dataclasses
import enum
import math
import os

import isaacsim.core.utils.stage as stage_utils
import numpy as np
import omni.graph.core as og
import rclpy
from isaacsim.core.api.robots import Robot
from rcl_interfaces.msg import ParameterType, ParameterValue, SetParametersResult
from rcl_interfaces.srv import GetParameters as GetParametersSrv
from rcl_interfaces.srv import SetParameters as SetParametersSrv
from rclpy.context import Context as ROS2Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node as ROS2Node
from sensor_msgs.msg import JointState


@dataclasses.dataclass
class ControlBoardSettings:
    # Node settings
    node_name: str
    node_set_parameters_service_name: str
    node_get_parameters_service_name: str
    node_state_topic_name: str
    node_motor_state_topic_name: str
    node_timeout: float
    # Joint settings
    joint_names: list[str]
    # Position PID settings
    position_p_gains: list[float]
    position_i_gains: list[float]
    position_d_gains: list[float]
    position_max_integral: list[float]
    position_max_output: list[float]
    position_max_error: list[float]
    position_default_velocity: float
    # Compliant mode settings
    compliant_stiffness: list[float]
    compliant_damping: list[float]
    # Velocity PID settings
    velocity_p_gains: list[float]
    velocity_i_gains: list[float]
    velocity_d_gains: list[float]
    velocity_max_integral: list[float]
    velocity_max_output: list[float]
    velocity_max_error: list[float]
    # Motor settings
    gearbox_ratios: list[float]  # Motor to joint
    motor_torque_constants: list[float]
    motor_current_noise_variance: list[float]
    motor_spring_stiffness: list[float]
    motor_max_currents: list[float]


class ControlMode(enum.IntEnum):
    IDLE = 0
    POSITION = 1
    POSITION_DIRECT = 2
    VELOCITY = 3
    TORQUE = 4
    CURRENT = 5
    HARDWARE_FAULT = 6


class ControlBoardState:
    joint_names: list[str]

    # Control modes
    control_modes: list[int]
    previous_control_modes: list[int]
    compliant_modes: list[bool]
    hf_messages: list[str]

    # Position PID settings
    position_p_gains: list[float]
    position_i_gains: list[float]
    position_d_gains: list[float]
    position_max_integral: list[float]
    position_max_output: list[float]
    position_max_error: list[float]

    # Compliant mode settings
    compliant_stiffness: list[float]
    compliant_damping: list[float]

    # Velocity PID settings
    velocity_p_gains: list[float]
    velocity_i_gains: list[float]
    velocity_d_gains: list[float]
    velocity_max_integral: list[float]
    velocity_max_output: list[float]
    velocity_max_error: list[float]

    # Position PID state
    position_pid_references: list[float]
    position_pid_errors: list[float]
    position_pid_outputs: list[float]
    is_motion_done: list[bool]
    position_pid_enabled: list[bool]
    position_pid_to_reset: list[bool]
    position_pid_to_stop: list[bool]

    # Position Direct PID state
    position_direct_pid_references: list[float]
    position_direct_pid_errors: list[float]
    position_direct_pid_outputs: list[float]
    position_direct_pid_enabled: list[bool]
    position_direct_pid_to_reset: list[bool]

    # Velocity PID state
    velocity_pid_references: list[float]
    velocity_pid_errors: list[float]
    velocity_pid_outputs: list[float]
    velocity_pid_enabled: list[bool]
    velocity_pid_to_reset: list[bool]

    # Torque PID state
    torque_pid_references: list[float]
    torque_pid_errors: list[float]
    torque_pid_outputs: list[float]
    torque_pid_enabled: list[bool]

    # Current PID state
    current_pid_references: list[float]
    current_pid_errors: list[float]
    current_pid_outputs: list[float]
    current_pid_enabled: list[bool]

    # Motor settings
    gearbox_ratios: list[float]  # Motor to joint
    motor_torque_constants: list[float]
    motor_current_noise_variance: list[float]
    motor_spring_stiffness: list[float]
    motor_max_currents: list[float]

    def __init__(self, settings):
        self.joint_names = settings.joint_names
        n_joints = len(self.joint_names)
        self.control_modes = [ControlMode.POSITION] * n_joints
        self.previous_control_modes = [ControlMode.POSITION] * n_joints
        self.compliant_modes = [False] * n_joints
        self.hf_messages = [""] * n_joints
        self.position_p_gains = settings.position_p_gains
        self.position_i_gains = settings.position_i_gains
        self.position_d_gains = settings.position_d_gains
        self.position_max_integral = settings.position_max_integral
        self.position_max_output = settings.position_max_output
        self.position_max_error = settings.position_max_error
        self.compliant_stiffness = settings.compliant_stiffness
        self.compliant_damping = settings.compliant_damping
        self.velocity_p_gains = settings.velocity_p_gains
        self.velocity_i_gains = settings.velocity_i_gains
        self.velocity_d_gains = settings.velocity_d_gains
        self.velocity_max_integral = settings.velocity_max_integral
        self.velocity_max_output = settings.velocity_max_output
        self.velocity_max_error = settings.velocity_max_error
        self.position_pid_references = [float("nan")] * n_joints
        self.position_pid_errors = [float("nan")] * n_joints
        self.position_pid_outputs = [float("nan")] * n_joints
        self.is_motion_done = [False] * n_joints
        self.position_pid_enabled = [True] * n_joints
        self.position_pid_to_reset = [False] * n_joints
        self.position_pid_to_stop = [False] * n_joints
        self.position_direct_pid_references = [float("nan")] * n_joints
        self.position_direct_pid_errors = [float("nan")] * n_joints
        self.position_direct_pid_outputs = [float("nan")] * n_joints
        self.position_direct_pid_enabled = [True] * n_joints
        self.position_direct_pid_to_reset = [False] * n_joints
        self.velocity_pid_references = [float("nan")] * n_joints
        self.velocity_pid_errors = [float("nan")] * n_joints
        self.velocity_pid_outputs = [float("nan")] * n_joints
        self.velocity_pid_enabled = [True] * n_joints
        self.velocity_pid_to_reset = [False] * n_joints
        self.torque_pid_references = [float("nan")] * n_joints
        # Since we output directly the effort in torque mode, the error is always 0
        self.torque_pid_errors = [0.0] * n_joints
        self.torque_pid_outputs = [float("nan")] * n_joints
        self.torque_pid_enabled = [True] * n_joints
        # Similarly, for the current mode, we convert the current reference to torque
        self.current_pid_references = [float("nan")] * n_joints
        self.current_pid_errors = [0.0] * n_joints
        self.current_pid_outputs = [float("nan")] * n_joints
        self.current_pid_enabled = [True] * n_joints
        self.gearbox_ratios = settings.gearbox_ratios
        self.motor_torque_constants = settings.motor_torque_constants
        self.motor_current_noise_variance = settings.motor_current_noise_variance
        self.motor_spring_stiffness = settings.motor_spring_stiffness
        self.motor_max_currents = settings.motor_max_currents
        self.settings = settings


class ControlBoardNode(ROS2Node):
    def __init__(
        self,
        node_name: str,
        set_service_name: str,
        get_service_name: str,
        state_topic_name: str,
        motor_state_topic_name: str,
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
        self.state_pub = self.create_publisher(JointState, state_topic_name, 10)
        self.motor_state_pub = self.create_publisher(
            JointState, motor_state_topic_name, 10
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
            if len(vec) == 0:
                output.type = ParameterType.PARAMETER_NOT_SET
            elif all(isinstance(v, int) for v in vec):
                output.type = ParameterType.PARAMETER_INTEGER_ARRAY
                output.integer_array_value = getattr(self.state, name)
            elif all(isinstance(v, float) for v in vec):
                output.type = ParameterType.PARAMETER_DOUBLE_ARRAY
                output.double_array_value = getattr(self.state, name)
            elif all(isinstance(v, str) for v in vec):
                output.type = ParameterType.PARAMETER_STRING_ARRAY
                output.string_array_value = getattr(self.state, name)
            elif all(isinstance(v, bool) for v in vec):
                output.type = ParameterType.PARAMETER_BOOL_ARRAY
                output.bool_array_value = getattr(self.state, name)
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
                elif isinstance(v, bool):
                    output.type = ParameterType.PARAMETER_BOOL
                    output.bool_value = v
                else:
                    print(
                        f"Unsupported type for parameter {name}[{index}]: {type(v)}. "
                        f"This should not have happened!"
                    )
            else:
                output.type = ParameterType.PARAMETER_NOT_SET
                print(
                    f"Index {index} out of range for parameter {name} "
                    f"with size {len(vec)}. This should not have happened!"
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

    def publish_joint_state(self, timestamp, positions, velocities, efforts):
        msg = JointState()
        msg.header.stamp.sec = int(timestamp)
        msg.header.stamp.nanosec = int((timestamp - int(timestamp)) * 1e9)
        msg.name = self.state.joint_names
        msg.position = positions
        msg.velocity = velocities
        msg.effort = efforts
        self.state_pub.publish(msg)

    def publish_motor_state(self, timestamp, positions, velocities, currents):
        msg = JointState()
        msg.header.stamp.sec = int(timestamp)
        msg.header.stamp.nanosec = int((timestamp - int(timestamp)) * 1e9)
        msg.name = self.state.joint_names
        msg.position = positions
        msg.velocity = velocities
        msg.effort = currents
        self.motor_state_pub.publish(msg)


# This class has been inspired from
# https://github.com/robotology/gz-sim-yarp-plugins/blob/eb489facf1e5ca2eef59e8b5875d7c784780bd3a/plugins/controlboard/src/ControlBoardTrajectory.cpp#L130-L366
class MinJerkTrajectoryGenerator:
    def __init__(self):
        # trajectory parameters
        self.initial_position = 0.0
        self.final_position = 0.0
        self.initial_scaled_velocity = 0.0

        self.speed = 0.0
        self.num_steps = 0.0
        self.step = 0.0
        self.cur_t = 0.0

        # trajectory coefficients
        self.coefficient_1 = 0.0
        self.coefficient_2 = 0.0
        self.coefficient_3 = 0.0

        self.trajectory_completed = True
        self.initialized = False

    def _compute_fifth_order_poly(self, t: float) -> float:
        output = self.initial_scaled_velocity * t
        tmp = t * t * t
        output += tmp * self.coefficient_1
        tmp *= t
        output -= tmp * self.coefficient_2
        tmp *= t
        output += tmp * self.coefficient_3
        return output

    def _compute_fifth_order_poly_velocity(self, t: float) -> float:
        output = -2 * self.initial_scaled_velocity * t - self.initial_scaled_velocity
        output += (
            30 * self.initial_position
            + 15 * self.initial_scaled_velocity
            - 30 * self.final_position
        ) * (t * t)
        output = -output / self.num_steps * (t - 1) * (t - 1)
        return output

    def compute_current_velocity(self) -> float:
        if self.trajectory_completed or not self.initialized:
            return 0.0
        if self.cur_t == 0:
            return 0.0
        elif self.cur_t < 1.0 - self.step:
            return self._compute_fifth_order_poly_velocity(self.cur_t)
        return 0.0

    def abort_trajectory(self, limit: float) -> bool:
        self.trajectory_completed = True
        self.cur_t = 0.0
        self.final_position = limit
        self.initialized = False
        return True

    def compute_trajectory(self) -> float | None:
        if not self.initialized:
            return None

        if self.trajectory_completed:
            target = self.final_position
            return target

        if self.cur_t == 0:
            self.cur_t += self.step

            target = self.initial_position
            return target
        elif self.cur_t < 1.0 - self.step:
            target = self._compute_fifth_order_poly(self.cur_t) + self.initial_position

            self.cur_t += self.step

            return target

        self.trajectory_completed = True
        return self.final_position

    def init_trajectory(
        self,
        current_position: float,
        final_position: float,
        speed: float,
        delta_t: int,
    ) -> bool:

        if speed <= 0:
            return False

        initial_velocity = self.compute_current_velocity()
        self.initial_position = current_position
        self.final_position = final_position

        self.speed = speed

        # Number of steps to complete the trajectory
        self.num_steps = (
            abs(self.final_position - self.initial_position) / abs(speed)
        ) / delta_t

        # Scale the initial velocity to the new trajectory time
        self.initial_scaled_velocity = initial_velocity * self.num_steps

        dx0 = self.initial_scaled_velocity
        self.coefficient_1 = (
            10 * (self.final_position - self.initial_position) - 6 * dx0
        )
        self.coefficient_2 = (
            15 * (self.final_position - self.initial_position) - 8 * dx0
        )
        self.coefficient_3 = 6 * (self.final_position - self.initial_position) - 3 * dx0

        if self.num_steps < 1:
            self.abort_trajectory(final_position)
            self.step = 0.0
            return False
        else:
            self.step = 1.0 / self.num_steps

        self.cur_t = 0.0
        self.trajectory_completed = False
        self.initialized = True

        return True


class ControlBoardPID:
    kp: float
    ki: float
    kd: float
    max_integral: float
    max_output: float
    max_error: float
    default_velocity: float
    reference: float | None
    input_reference: float | None
    input_reference_velocity: float | None
    measurement: float
    integral_state: float
    previous_error: float
    output: float
    smoother: None

    def __init__(
        self,
        p,
        i,
        d,
        compliant_stiffness,
        compliant_damping,
        max_integral,
        max_output,
        max_error,
        default_velocity,
    ):
        self.kp = p
        self.ki = i
        self.kd = d

        self.compliant_mode = False
        self.compliant_stiffness = compliant_stiffness
        self.compliant_damping = compliant_damping

        self.max_integral = max_integral
        self.max_output = max_output
        self.max_error = max_error
        self.default_velocity = default_velocity

        self.measurement = 0.0
        self.output = 0.0

        self.integral_state = 0.0
        self.previous_error = 0.0
        self.reference = None
        self.smoother = None
        self.input_reference = None
        self.input_reference_velocity = None
        self.compliant_offset = None

    def set_reference(self, reference, velocity=None):
        self.input_reference = reference
        self.input_reference_velocity = velocity

    def set_smoother(self, smoother):
        self.smoother = smoother

    def set_compliant_mode(self, compliant):
        if compliant != self.compliant_mode:
            self.reset()

        self.compliant_mode = compliant

    def set_compliant_mode_offset(self, offset):
        self.compliant_offset = offset

    def compute(self, delta_time, measurement, measurement_velocity=None):
        if self.smoother:
            # Reinitialize the smoother if not initialized or if the reference changed
            if not self.smoother.initialized or self.input_reference:
                # If no input reference is set, use the current measurement as reference
                ref = self.input_reference if self.input_reference else measurement
                speed = (
                    self.input_reference_velocity
                    if self.input_reference_velocity
                    else self.default_velocity
                )
                self.smoother.init_trajectory(
                    current_position=measurement,
                    final_position=ref,
                    speed=speed,
                    delta_t=delta_time,
                )

            self.reference = (
                self.smoother.compute_trajectory()
                if self.smoother.initialized
                else measurement
            )
        elif self.input_reference:
            self.reference = self.input_reference

        # If no reference is set, use the current measurement as reference
        if self.reference is None:
            self.reference = measurement

        self.input_reference = None  # reset the input reference
        self.input_reference_velocity = None  # reset the input reference velocity

        error = self.reference - measurement
        error = max(min(error, self.max_error), -self.max_error)
        self.output = (
            self._stiff_output(delta_time, error)
            if not self.compliant_mode
            else self._compliant_output(error, measurement_velocity)
        )

        self.previous_error = error
        self.output = max(min(self.output, self.max_output), -self.max_output)

        return self.output

    def _stiff_output(self, delta_time, error):
        # Proportional term
        p_term = self.kp * error

        # Integral term
        self.integral_state += error * delta_time
        self.integral_state = max(
            min(self.integral_state, self.max_integral), -self.max_integral
        )
        i_term = self.ki * self.integral_state

        # Derivative term. The velocity is estimated and not using the measurement
        derivative = (
            (error - self.previous_error) / delta_time if delta_time > 0 else 0.0
        )
        d_term = self.kd * derivative

        # Total output
        return p_term + i_term + d_term

    def _compliant_output(self, error, measurement_velocity):
        offset = self.compliant_offset if self.compliant_offset else 0.0
        velocity = measurement_velocity if measurement_velocity else 0.0
        return (
            self.compliant_stiffness * error
            - self.compliant_damping * velocity
            + offset
        )

    def update_gains(
        self,
        p,
        i,
        d,
        compliant_stiffness,
        compliant_damping,
        max_integral,
        max_output,
        max_error,
    ):
        self.kp = p
        self.ki = i
        self.kd = d
        self.compliant_stiffness = compliant_stiffness
        self.compliant_damping = compliant_damping
        self.max_integral = max_integral
        self.max_output = max_output
        self.max_error = max_error

    def reset(self):
        self.integral_state = 0.0
        self.previous_error = 0.0
        if self.smoother:
            self.smoother.initialized = False

        self.compliant_offset = None
        self.compliant_mode = False

    def get_output(self):
        return self.output

    def get_error(self):
        return self.previous_error

    def get_reference(self):
        return self.reference


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
    # Replicates ROS2Context selection logic:
    #   - if use_env_var and ROS_DOMAIN_ID exists: use it
    #   - otherwise use domain_id_input

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


def fill_settings_from_db(db: og.Database) -> ControlBoardSettings:
    s = ControlBoardSettings(
        node_name=db.inputs.node_name,
        node_set_parameters_service_name=db.inputs.node_set_parameters_service_name,
        node_get_parameters_service_name=db.inputs.node_get_parameters_service_name,
        node_state_topic_name=db.inputs.node_state_topic_name,
        node_motor_state_topic_name=db.inputs.node_motor_state_topic_name,
        node_timeout=db.inputs.node_timeout,
        joint_names=db.inputs.joint_names,
        position_p_gains=db.inputs.position_p_gains,
        position_i_gains=db.inputs.position_i_gains,
        position_d_gains=db.inputs.position_d_gains,
        position_max_integral=db.inputs.position_max_integral,
        position_max_output=db.inputs.position_max_output,
        position_max_error=db.inputs.position_max_error,
        position_default_velocity=db.inputs.position_default_velocity,
        compliant_stiffness=db.inputs.compliant_stiffness,
        compliant_damping=db.inputs.compliant_damping,
        velocity_p_gains=db.inputs.velocity_p_gains,
        velocity_i_gains=db.inputs.velocity_i_gains,
        velocity_d_gains=db.inputs.velocity_d_gains,
        velocity_max_integral=db.inputs.velocity_max_integral,
        velocity_max_output=db.inputs.velocity_max_output,
        velocity_max_error=db.inputs.velocity_max_error,
        gearbox_ratios=db.inputs.gearbox_ratios,
        motor_torque_constants=db.inputs.motor_torque_constants,
        motor_current_noise_variance=db.inputs.motor_current_noise_variance,
        motor_spring_stiffness=db.inputs.motor_spring_stiffness,
        motor_max_currents=db.inputs.motor_max_currents,
    )
    n_joints = len(s.joint_names)
    if not (
        len(s.position_p_gains)
        == len(s.position_i_gains)
        == len(s.position_d_gains)
        == len(s.position_max_integral)
        == len(s.position_max_output)
        == len(s.position_max_error)
        == len(s.compliant_stiffness)
        == len(s.compliant_damping)
        == len(s.velocity_p_gains)
        == len(s.velocity_i_gains)
        == len(s.velocity_d_gains)
        == len(s.velocity_max_integral)
        == len(s.velocity_max_output)
        == len(s.velocity_max_error)
        == len(s.gearbox_ratios)
        == len(s.motor_torque_constants)
        == len(s.motor_current_noise_variance)
        == len(s.motor_spring_stiffness)
        == len(s.motor_max_currents)
        == n_joints
    ):
        db.log_error("Input joint parameter lists must have the same length")
    return s


def create_robot_object_cb(db: og.Database, name, joint_names):
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


def setup_cb(db: og.Database):
    domain_id = choose_domain_id(db=db)
    settings = fill_settings_from_db(db=db)
    db.per_instance_state.state = ControlBoardState(settings=settings)
    db.per_instance_state.pids = {}
    db.per_instance_state.context = ROS2Context()
    db.per_instance_state.context.init(domain_id=domain_id)
    db.per_instance_state.node = ControlBoardNode(
        node_name=settings.node_name,
        set_service_name=settings.node_set_parameters_service_name,
        get_service_name=settings.node_get_parameters_service_name,
        state_topic_name=settings.node_state_topic_name,
        motor_state_topic_name=settings.node_motor_state_topic_name,
        context=db.per_instance_state.context,
        state=db.per_instance_state.state,
    )
    db.per_instance_state.executor = SingleThreadedExecutor(
        context=db.per_instance_state.context
    )
    db.per_instance_state.executor.add_node(db.per_instance_state.node)

    output = create_robot_object_cb(
        db, name=settings.node_name, joint_names=settings.joint_names
    )

    if not output:
        db.per_instance_state.initialized = False
        db.log_error("Failed to create robot object")
        return

    robot, robot_joint_indices = output
    if not robot or not robot_joint_indices:
        db.per_instance_state.initialized = False
        db.log_error("Either the robot or the joint indices are invalid")
        return

    db.per_instance_state.robot = robot
    db.per_instance_state.robot_joint_indices = robot_joint_indices

    db.per_instance_state.initialized = True


def setup(db: og.Database):
    setup_cb(db)


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
    measured_motor_current: float,
    lower_limit: float,
    upper_limit: float,
    max_velocity: float,
    max_effort: float,
    max_current: float,
) -> float:
    script_state = db.per_instance_state
    cb_state = script_state.state
    name = cb_state.joint_names[joint_index]
    delta_time = (
        db.inputs.deltaTime
        if hasattr(db.inputs, "deltaTime") and db.inputs.deltaTime is not None
        else 0.0
    )

    if joint_index not in script_state.pids:
        script_state.pids[joint_index] = {}

    control_mode = cb_state.control_modes[joint_index]

    reference_names = (
        db.inputs.reference_joint_names
        if hasattr(db.inputs, "reference_joint_names")
        and db.inputs.reference_joint_names is not None
        else []
    )

    has_reference_name = reference_names and name in reference_names
    reference_index = reference_names.index(name) if has_reference_name else None

    def get_reference(field):
        if (
            has_reference_name
            and hasattr(db.inputs, field)
            and getattr(db.inputs, field) is not None
            and reference_index < len(getattr(db.inputs, field))
            and not math.isnan(getattr(db.inputs, field)[reference_index])
        ):
            return getattr(db.inputs, field)[reference_index]
        return None

    reference_position = get_reference("reference_position_commands")
    if reference_position:
        reference_position = max(min(reference_position, upper_limit), lower_limit)

    reference_velocity = get_reference("reference_velocity_commands")
    if reference_velocity:
        reference_velocity = max(min(reference_velocity, max_velocity), -max_velocity)

    reference_effort = get_reference("reference_effort_commands")
    if reference_effort:
        reference_effort = max(min(reference_effort, max_effort), -max_effort)

    # The current control considers the effort reference as a current reference
    reference_current = get_reference("reference_effort_commands")
    if reference_current:
        reference_current = max(min(reference_current, max_current), -max_current)

    if control_mode == ControlMode.POSITION:
        if not cb_state.position_pid_enabled:
            cb_state.control_modes[joint_index] = ControlMode.HARDWARE_FAULT
            cb_state.hf_messages[joint_index] = "Position PID not enabled."
            return 0.0

        pid = script_state.pids[joint_index].get(control_mode, None)
        if pid is None:
            script_state.pids[joint_index][control_mode] = ControlBoardPID(
                p=cb_state.position_p_gains[joint_index],
                i=cb_state.position_i_gains[joint_index],
                d=cb_state.position_d_gains[joint_index],
                compliant_stiffness=cb_state.compliant_stiffness[joint_index],
                compliant_damping=cb_state.compliant_damping[joint_index],
                max_integral=cb_state.position_max_integral[joint_index],
                max_output=cb_state.position_max_output[joint_index],
                max_error=cb_state.position_max_error[joint_index],
                default_velocity=cb_state.settings.position_default_velocity,
            )
            pid = script_state.pids[joint_index][control_mode]
            pid.set_smoother(MinJerkTrajectoryGenerator())
        else:
            pid.update_gains(
                p=cb_state.position_p_gains[joint_index],
                i=cb_state.position_i_gains[joint_index],
                d=cb_state.position_d_gains[joint_index],
                compliant_stiffness=cb_state.compliant_stiffness[joint_index],
                compliant_damping=cb_state.compliant_damping[joint_index],
                max_integral=cb_state.position_max_integral[joint_index],
                max_output=cb_state.position_max_output[joint_index],
                max_error=cb_state.position_max_error[joint_index],
            )

        if cb_state.previous_control_modes[joint_index] != control_mode:
            pid.reset()
        cb_state.previous_control_modes[joint_index] = control_mode

        if cb_state.position_pid_to_stop[joint_index]:
            pid.smoother.abort_trajectory(measured_position)
            cb_state.position_pid_to_stop[joint_index] = False

        elif reference_position:
            ref_vel = (
                reference_velocity
                if reference_velocity and reference_velocity > 0.0
                else None
            )
            pid.set_reference(reference_position, ref_vel)

        pid.set_compliant_mode(cb_state.compliant_modes[joint_index])
        if reference_effort:
            pid.set_compliant_mode_offset(reference_effort)

        return pid.compute(delta_time, measured_position, measured_velocity)

    elif control_mode == ControlMode.POSITION_DIRECT:
        if not cb_state.position_direct_pid_enabled:
            cb_state.control_modes[joint_index] = ControlMode.HARDWARE_FAULT
            cb_state.hf_messages[joint_index] = "Position Direct PID not enabled."
            return 0.0

        pid = script_state.pids[joint_index].get(control_mode, None)
        if pid is None:
            script_state.pids[joint_index][control_mode] = ControlBoardPID(
                p=cb_state.position_p_gains[joint_index],
                i=cb_state.position_i_gains[joint_index],
                d=cb_state.position_d_gains[joint_index],
                compliant_stiffness=cb_state.compliant_stiffness[joint_index],
                compliant_damping=cb_state.compliant_damping[joint_index],
                max_integral=cb_state.position_max_integral[joint_index],
                max_output=cb_state.position_max_output[joint_index],
                max_error=cb_state.position_max_error[joint_index],
                default_velocity=cb_state.settings.position_default_velocity,
            )
            pid = script_state.pids[joint_index][control_mode]
        else:
            pid.update_gains(
                p=cb_state.position_p_gains[joint_index],
                i=cb_state.position_i_gains[joint_index],
                d=cb_state.position_d_gains[joint_index],
                compliant_stiffness=cb_state.compliant_stiffness[joint_index],
                compliant_damping=cb_state.compliant_damping[joint_index],
                max_integral=cb_state.position_max_integral[joint_index],
                max_output=cb_state.position_max_output[joint_index],
                max_error=cb_state.position_max_error[joint_index],
            )

        if control_mode == ControlMode.POSITION and pid.smoother is None:
            pid.set_smoother(MinJerkTrajectoryGenerator())

        if cb_state.previous_control_modes[joint_index] != control_mode:
            pid.reset()
        cb_state.previous_control_modes[joint_index] = control_mode

        if reference_position:
            pid.set_reference(reference_position)

        pid.set_compliant_mode(cb_state.compliant_modes[joint_index])
        if reference_effort:
            pid.set_compliant_mode_offset(reference_effort)

        return pid.compute(delta_time, measured_position, measured_velocity)

    elif control_mode == ControlMode.VELOCITY:
        if not cb_state.velocity_pid_enabled:
            cb_state.control_modes[joint_index] = ControlMode.HARDWARE_FAULT
            cb_state.hf_messages[joint_index] = "Velocity PID not enabled."
            return 0.0

        pid = script_state.pids[joint_index].get(control_mode, None)
        if pid is None:
            script_state.pids[joint_index][control_mode] = ControlBoardPID(
                p=cb_state.velocity_p_gains[joint_index],
                i=cb_state.velocity_i_gains[joint_index],
                d=cb_state.velocity_d_gains[joint_index],
                compliant_stiffness=cb_state.compliant_stiffness[joint_index],  # Unused
                compliant_damping=cb_state.compliant_damping[joint_index],  # Unused
                max_integral=cb_state.velocity_max_integral[joint_index],
                max_output=cb_state.velocity_max_output[joint_index],
                max_error=cb_state.velocity_max_error[joint_index],
                default_velocity=cb_state.settings.position_default_velocity,  # Unused
            )
            pid = script_state.pids[joint_index][control_mode]
        else:
            pid.update_gains(
                p=cb_state.velocity_p_gains[joint_index],
                i=cb_state.velocity_i_gains[joint_index],
                d=cb_state.velocity_d_gains[joint_index],
                compliant_stiffness=cb_state.compliant_stiffness[joint_index],  # Unused
                compliant_damping=cb_state.compliant_damping[joint_index],  # Unused
                max_integral=cb_state.velocity_max_integral[joint_index],
                max_output=cb_state.velocity_max_output[joint_index],
                max_error=cb_state.velocity_max_error[joint_index],
            )

        if cb_state.previous_control_modes[joint_index] != control_mode:
            pid.reset()
        cb_state.previous_control_modes[joint_index] = control_mode

        if reference_velocity:
            pid.set_reference(reference_velocity)

        return pid.compute(delta_time, measured_velocity)

    elif control_mode == ControlMode.TORQUE:
        if not cb_state.torque_pid_enabled:
            cb_state.control_modes[joint_index] = ControlMode.HARDWARE_FAULT
            cb_state.hf_messages[joint_index] = "Torque PID not enabled."
            return 0.0

        if (
            cb_state.previous_control_modes[joint_index] != control_mode
            or control_mode not in script_state.pids[joint_index]
        ):
            # For the torque mode, we don't set a PID, but we just store the reference
            script_state.pids[joint_index][control_mode] = measured_effort
        cb_state.previous_control_modes[joint_index] = control_mode

        if reference_effort:
            script_state.pids[joint_index][control_mode] = reference_effort

        return script_state.pids[joint_index][control_mode]

    elif control_mode == ControlMode.CURRENT:
        if not cb_state.current_pid_enabled:
            cb_state.control_modes[joint_index] = ControlMode.HARDWARE_FAULT
            cb_state.hf_messages[joint_index] = "Current PID not enabled."
            return 0.0

        if (
            cb_state.previous_control_modes[joint_index] != control_mode
            or control_mode not in script_state.pids[joint_index]
        ):
            # For the current mode, we don't set a PID, but we set a dict
            # with the reference current and the corresponding torque
            script_state.pids[joint_index][control_mode] = {}
            script_state.pids[joint_index][control_mode][
                "reference"
            ] = measured_motor_current
        cb_state.previous_control_modes[joint_index] = control_mode

        current_dict = script_state.pids[joint_index][control_mode]
        gearbox_ratio = cb_state.gearbox_ratios[joint_index]
        motor_torque_constant = cb_state.motor_torque_constants[joint_index]
        motor_stiffness = cb_state.motor_spring_stiffness[joint_index]

        if reference_current:
            current_dict["reference"] = reference_current

        output_current = current_dict["reference"]
        sping_torque = motor_stiffness * measured_position
        k_total = motor_torque_constant * gearbox_ratio
        output_torque = output_current * k_total - sping_torque
        current_dict["torque"] = output_torque
        return output_torque

    elif control_mode == ControlMode.IDLE or control_mode == ControlMode.HARDWARE_FAULT:
        cb_state.previous_control_modes[joint_index] = control_mode
        return 0.0
    else:
        cb_state.control_modes[joint_index] = ControlMode.HARDWARE_FAULT
        cb_state.previous_control_modes[joint_index] = ControlMode.HARDWARE_FAULT
        cb_state.hf_messages[joint_index] = f"Unsupported control mode {control_mode}"
        db.log_error(
            f"Unsupported control mode {control_mode} "
            f"for joint {cb_state.joint_names[joint_index]}"
        )
        return 0.0


def update_state(db: og.Database):
    script_state = db.per_instance_state
    if not script_state.initialized:
        return
    cb_state = script_state.state

    # Note: the values are actually updated only if the joint is in the correct control
    # mode and the PID is enabled

    for i in range(len(cb_state.joint_names)):
        position_pid = None
        position_direct_pid = None
        velocity_pid = None
        torque_reference = None
        current_dict = None
        if i in script_state.pids:
            position_pid = script_state.pids[i].get(ControlMode.POSITION, None)
            position_direct_pid = script_state.pids[i].get(
                ControlMode.POSITION_DIRECT, None
            )
            velocity_pid = script_state.pids[i].get(ControlMode.VELOCITY, None)
            torque_reference = script_state.pids[i].get(ControlMode.TORQUE, None)
            current_dict = script_state.pids[i].get(ControlMode.CURRENT, None)

        if position_pid:
            reference = position_pid.get_reference()
            if reference:
                cb_state.position_pid_references[i] = reference

            cb_state.position_pid_errors[i] = position_pid.get_error()
            cb_state.position_pid_outputs[i] = position_pid.get_output()
            if position_pid.smoother:
                cb_state.is_motion_done[i] = position_pid.smoother.trajectory_completed

        if position_direct_pid:
            reference = position_direct_pid.get_reference()
            if reference:
                cb_state.position_direct_pid_references[i] = reference

            cb_state.position_direct_pid_errors[i] = position_direct_pid.get_error()
            cb_state.position_direct_pid_outputs[i] = position_direct_pid.get_output()

        if velocity_pid:
            reference = velocity_pid.get_reference()
            if reference:
                cb_state.velocity_pid_references[i] = reference

            cb_state.velocity_pid_errors[i] = velocity_pid.get_error()
            cb_state.velocity_pid_outputs[i] = velocity_pid.get_output()

        if torque_reference is not None:
            cb_state.torque_pid_references[i] = torque_reference
            cb_state.torque_pid_outputs[i] = torque_reference

        if current_dict is not None:
            cb_state.current_pid_references[i] = current_dict["reference"]
            cb_state.current_pid_outputs[i] = current_dict["torque"]


def reset_requested_pids(db: og.Database):
    script_state = db.per_instance_state
    if not script_state.initialized:
        return

    cb_state = script_state.state
    original = cb_state.settings

    for i in range(len(cb_state.joint_names)):
        if cb_state.position_pid_to_reset[i]:
            cb_state.position_p_gains[i] = original.position_p_gains[i]
            cb_state.position_i_gains[i] = original.position_i_gains[i]
            cb_state.position_d_gains[i] = original.position_d_gains[i]
            cb_state.position_max_integral[i] = original.position_max_integral[i]
            cb_state.position_max_output[i] = original.position_max_output[i]
            cb_state.position_max_error[i] = original.position_max_error[i]
            cb_state.compliant_stiffness[i] = original.compliant_stiffness[i]
            cb_state.compliant_damping[i] = original.compliant_damping[i]
            if i in script_state.pids and ControlMode.POSITION in script_state.pids[i]:
                script_state.pids[i][ControlMode.POSITION].reset()
            cb_state.position_pid_to_reset[i] = False

        if cb_state.position_direct_pid_to_reset[i]:
            cb_state.position_p_gains[i] = original.position_p_gains[i]
            cb_state.position_i_gains[i] = original.position_i_gains[i]
            cb_state.position_d_gains[i] = original.position_d_gains[i]
            cb_state.position_max_integral[i] = original.position_max_integral[i]
            cb_state.position_max_output[i] = original.position_max_output[i]
            cb_state.position_max_error[i] = original.position_max_error[i]
            cb_state.compliant_stiffness[i] = original.compliant_stiffness[i]
            cb_state.compliant_damping[i] = original.compliant_damping[i]
            if (
                i in script_state.pids
                and ControlMode.POSITION_DIRECT in script_state.pids[i]
            ):
                script_state.pids[i][ControlMode.POSITION_DIRECT].reset()
            cb_state.position_direct_pid_to_reset[i] = False

        if cb_state.velocity_pid_to_reset[i]:
            cb_state.velocity_p_gains[i] = original.velocity_p_gains[i]
            cb_state.velocity_i_gains[i] = original.velocity_i_gains[i]
            cb_state.velocity_d_gains[i] = original.velocity_d_gains[i]
            cb_state.velocity_max_integral[i] = original.velocity_max_integral[i]
            cb_state.velocity_max_output[i] = original.velocity_max_output[i]
            cb_state.velocity_max_error[i] = original.velocity_max_error[i]
            if i in script_state.pids and ControlMode.VELOCITY in script_state.pids[i]:
                script_state.pids[i][ControlMode.VELOCITY].reset()
            cb_state.velocity_pid_to_reset[i] = False


def compute_motor_state(
    db: og.Database, joint_index, joint_position, joint_velocity, joint_effort
):
    script_state = db.per_instance_state
    cb_state = script_state.state
    gearbox_ratio = cb_state.gearbox_ratios[joint_index]

    def compute_motor_current() -> float:

        motor_torque_constant = cb_state.motor_torque_constants[joint_index]
        motor_stiffness = cb_state.motor_spring_stiffness[joint_index]

        # Add a Gaussian noise to the motor current
        noise_variance = cb_state.motor_current_noise_variance[joint_index]
        if noise_variance <= 0:
            additive_noise = 0.0
        else:
            additive_noise = np.random.normal(0.0, math.sqrt(noise_variance))

        simulated_effort = joint_effort + motor_stiffness * joint_position

        k_total = gearbox_ratio * motor_torque_constant

        if k_total == 0:
            return 0.0

        motor_current = simulated_effort / k_total + additive_noise
        return motor_current

    motor_position = joint_position * gearbox_ratio
    motor_velocity = joint_velocity * gearbox_ratio
    return motor_position, motor_velocity, compute_motor_current()


def compute(db: og.Database):
    if (
        not hasattr(db.per_instance_state, "initialized")
        or not db.per_instance_state.initialized
    ):
        setup_cb(db)

    if not db.per_instance_state.initialized:
        return False

    script_state = db.per_instance_state
    cb_state = script_state.state

    joint_state = script_state.robot.get_joints_state()
    if joint_state is None:
        db.log_warning(
            f"Failed to get joint states in {script_state.robot.name}. "
            f"Initializing again."
        )
        script_state.robot.initialize()
        return False

    if rclpy.ok(context=script_state.context):
        script_state.executor.spin_once(timeout_sec=cb_state.settings.node_timeout)

    reset_requested_pids(db)

    output_effort = []
    positions = [0.0] * len(cb_state.joint_names)
    velocities = [0.0] * len(cb_state.joint_names)
    efforts = [0.0] * len(cb_state.joint_names)
    motor_positions = [0.0] * len(cb_state.joint_names)
    motor_velocities = [0.0] * len(cb_state.joint_names)
    motor_currents = [0.0] * len(cb_state.joint_names)
    for i in range(len(cb_state.joint_names)):
        robot_index = script_state.robot_joint_indices[i]

        measured_position = joint_state.positions[robot_index]
        positions[i] = measured_position

        measured_velocity = joint_state.velocities[robot_index]
        velocities[i] = measured_velocity

        measured_effort = joint_state.efforts[robot_index]
        efforts[i] = joint_state.efforts[robot_index]

        motor_position, motor_velocity, motor_current = compute_motor_state(
            db, i, measured_position, measured_velocity, measured_effort
        )
        motor_positions[i] = motor_position
        motor_velocities[i] = motor_velocity
        motor_currents[i] = motor_current

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

        max_current = (
            cb_state.motor_max_currents[i]
            if cb_state.motor_max_currents[i] > 0.0
            else float("inf")
        )

        if abs(measured_effort) > max_effort:
            cb_state.control_modes[i] = ControlMode.HARDWARE_FAULT
            cb_state.hf_messages[i] = (
                f"Measured effort {measured_effort} exceeds max effort {max_effort}"
            )

        if abs(motor_current) > max_current:
            cb_state.control_modes[i] = ControlMode.HARDWARE_FAULT
            cb_state.hf_messages[i] = (
                f"Measured motor current {motor_current} exceeds max current {max_current}"
            )

        effort = get_pid_output(
            db=db,
            joint_index=i,
            measured_position=measured_position,
            measured_velocity=measured_velocity,
            measured_effort=measured_effort,
            measured_motor_current=motor_current,
            lower_limit=lower_limit,
            upper_limit=upper_limit,
            max_velocity=max_velocity,
            max_effort=max_effort,
            max_current=max_current,
        )
        effort = max(min(effort, max_effort), -max_effort)
        output_effort.append(effort)

    script_state.robot.set_joint_efforts(
        efforts=output_effort, joint_indices=script_state.robot_joint_indices
    )

    update_state(db)

    timestamp = (
        db.inputs.timestamp
        if hasattr(db.inputs, "timestamp") and db.inputs.timestamp is not None
        else 0.0
    )
    script_state.node.publish_joint_state(
        timestamp=timestamp,
        positions=positions,
        velocities=velocities,
        efforts=efforts,
    )
    script_state.node.publish_motor_state(
        timestamp=timestamp,
        positions=motor_positions,
        velocities=motor_velocities,
        currents=motor_currents,
    )

    return True


def internal_state():
    return ControlBoardData()
