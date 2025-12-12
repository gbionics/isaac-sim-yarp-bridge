"""
This script generates a OmniGraph action graph publishing joint states, IMUs, FTs,
and camera readings on ROS2. The settings for creating the graph are at the end of the
file.
"""

#######################################################################################
################### EDIT ONLY THE SETTINGS AT THE END OF THE FILE #####################
#######################################################################################

import dataclasses
import math
import os

import isaacsim.core.utils.stage as stage_utils
import omni.graph.core as og
from pxr import PhysxSchema, Sdf, UsdPhysics


@dataclasses.dataclass
class ControlBoard:
    name: str
    joint_names: list[str]
    joint_damping: list[float]
    position_p_gains: list[float]
    position_i_gains: list[float]
    position_d_gains: list[float]
    position_max_integral: list[float]
    position_max_output: list[float]
    position_max_error: list[float]
    position_default_velocity: float
    compliant_stiffness: list[float]
    compliant_damping: list[float]
    velocity_p_gains: list[float]
    velocity_i_gains: list[float]
    velocity_d_gains: list[float]
    velocity_max_integral: list[float]
    velocity_max_output: list[float]
    velocity_max_error: list[float]
    gearbox_ratios: list[float]
    motor_torque_constants: list[float]
    motor_current_noise_variance: list[float]
    motor_spring_stiffness: list[float]
    motor_max_currents: list[float]


@dataclasses.dataclass
class Imu:
    name: str
    target: str


@dataclasses.dataclass
class Camera:
    prefix: str
    suffix: str
    type: str
    target: str


@dataclasses.dataclass
class FT:
    name: str
    joint: str
    frame: str
    flip: bool


@dataclasses.dataclass
class Settings:
    sensors_graph_path: str
    camera_graph_path: str
    robot_path: str
    topic_prefix: str
    domain_id: int
    useDomainIDEnvVar: bool
    node_timeout: float
    control_boards: list[ControlBoard]
    imus: list[Imu]
    cameras: list[Camera]
    FTs: list[FT]
    physics_frequency: float
    render_frequency: float


# When creating nodes, first you specify the name of the node, and then the type of
# node. If instead of the type, you pass a dict with a series of other actions, you
# are creating a compound node instead (basically a subgraph). The inputs and
# outputs of a compound node are defined by "promoting" inputs and outputs of the
# internal nodes. If multiple internal nodes need to be connected to the same
# input/outputs, this connection is done explicitly after creating the compound,
# using the compound name.


def merge_actions(action_list):
    output = {}
    for d in action_list:
        if d is None:
            continue
        for key, value in d.items():
            if key in output:
                for val in value:
                    output[key].append(val)
            else:
                output[key] = value
    return output


def create_basic_nodes_sensors(graph_keys, settings):
    return {
        graph_keys.CREATE_NODES: [
            ("phys_tick", "isaacsim.core.nodes.OnPhysicsStep"),
            ("sim_time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("ros2_context", "isaacsim.ros2.bridge.ROS2Context"),
        ],
        graph_keys.SET_VALUES: [
            ("sim_time.inputs:resetOnStop", True),
            ("ros2_context.inputs:domain_id", settings.domain_id),
            ("ros2_context.inputs:useDomainIDEnvVar", settings.useDomainIDEnvVar),
        ],
    }


def create_basic_nodes_camera(graph_keys, settings):
    return {
        graph_keys.CREATE_NODES: [
            ("tick", "omni.graph.action.OnPlaybackTick"),
            ("ros2_context_camera", "isaacsim.ros2.bridge.ROS2Context"),
        ],
        graph_keys.SET_VALUES: [
            ("ros2_context_camera.inputs:domain_id", settings.domain_id),
            (
                "ros2_context_camera.inputs:useDomainIDEnvVar",
                settings.useDomainIDEnvVar,
            ),
        ],
    }


def create_ros2_clock_publisher(graph_keys):
    return {
        graph_keys.CREATE_NODES: [
            ("ros2_clock_publisher", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ],
        graph_keys.CONNECT: [
            ("phys_tick.outputs:step", "ros2_clock_publisher.inputs:execIn"),
            ("ros2_context.outputs:context", "ros2_clock_publisher.inputs:context"),
            (
                "sim_time.outputs:simulationTime",
                "ros2_clock_publisher.inputs:timeStamp",
            ),
        ],
    }


def create_control_board_subcompound(
    graph_keys, board_settings, full_settings, promoted
):
    compound_name = board_settings.name + "_compound"
    subscriber_name = board_settings.name + "_subscriber"
    cb_script_name = board_settings.name + "_script"
    cb_topic_prefix = full_settings.topic_prefix + "/" + board_settings.name
    script_file = os.path.join(
        os.environ["ISAAC_SIM_YARP_BRIDGE_PATH"], "scripts", "omni_control_board.py"
    )

    if not os.path.exists(script_file):
        raise FileNotFoundError(
            f"Control board script file not found at {script_file}. "
            "Please ensure ISAAC_SIM_YARP_BRIDGE_PATH is set correctly."
        )

    compound_actions = {
        graph_keys.CREATE_NODES: [
            (subscriber_name, "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
            (cb_script_name, "omni.graph.scriptnode.ScriptNode"),
        ],
        graph_keys.CREATE_ATTRIBUTES: [
            (cb_script_name + ".inputs:timestamp", "double"),
            (cb_script_name + ".inputs:deltaTime", "double"),
            (cb_script_name + ".inputs:reference_joint_names", "token[]"),
            (cb_script_name + ".inputs:reference_position_commands", "double[]"),
            (cb_script_name + ".inputs:reference_velocity_commands", "double[]"),
            (cb_script_name + ".inputs:reference_effort_commands", "double[]"),
            (cb_script_name + ".inputs:reference_timestamp", "double"),
            (cb_script_name + ".inputs:robot_prim", "target"),
            (cb_script_name + ".inputs:domain_id", "uchar"),
            (cb_script_name + ".inputs:useDomainIDEnvVar", "bool"),
            (cb_script_name + ".inputs:node_name", "string"),
            (cb_script_name + ".inputs:node_set_parameters_service_name", "string"),
            (cb_script_name + ".inputs:node_parameters_event_topic_name", "string"),
            (cb_script_name + ".inputs:node_state_topic_name", "string"),
            (cb_script_name + ".inputs:node_motor_state_topic_name", "string"),
            (cb_script_name + ".inputs:node_timeout", "double"),
            (cb_script_name + ".inputs:joint_names", "token[]"),
            (cb_script_name + ".inputs:position_p_gains", "double[]"),
            (cb_script_name + ".inputs:position_i_gains", "double[]"),
            (cb_script_name + ".inputs:position_d_gains", "double[]"),
            (cb_script_name + ".inputs:position_max_integral", "double[]"),
            (cb_script_name + ".inputs:position_max_output", "double[]"),
            (cb_script_name + ".inputs:position_max_error", "double[]"),
            (cb_script_name + ".inputs:position_default_velocity", "double"),
            (cb_script_name + ".inputs:compliant_stiffness", "double[]"),
            (cb_script_name + ".inputs:compliant_damping", "double[]"),
            (cb_script_name + ".inputs:velocity_p_gains", "double[]"),
            (cb_script_name + ".inputs:velocity_i_gains", "double[]"),
            (cb_script_name + ".inputs:velocity_d_gains", "double[]"),
            (cb_script_name + ".inputs:velocity_max_integral", "double[]"),
            (cb_script_name + ".inputs:velocity_max_output", "double[]"),
            (cb_script_name + ".inputs:velocity_max_error", "double[]"),
            (cb_script_name + ".inputs:gearbox_ratios", "double[]"),
            (cb_script_name + ".inputs:motor_torque_constants", "double[]"),
            (cb_script_name + ".inputs:motor_current_noise_variance", "double[]"),
            (cb_script_name + ".inputs:motor_spring_stiffness", "double[]"),
            (cb_script_name + ".inputs:motor_max_currents", "double[]"),
            (cb_script_name + ".outputs:desired_efforts", "double[]"),
        ],
        graph_keys.SET_VALUES: [
            (
                subscriber_name + ".inputs:topicName",
                cb_topic_prefix + "/joint_state/input",
            ),
            (cb_script_name + ".inputs:usePath", True),
            (cb_script_name + ".inputs:scriptPath", script_file),
            (cb_script_name + ".inputs:robot_prim", full_settings.robot_path),
            (cb_script_name + ".inputs:domain_id", full_settings.domain_id),
            (
                cb_script_name + ".inputs:useDomainIDEnvVar",
                full_settings.useDomainIDEnvVar,
            ),
            (cb_script_name + ".inputs:node_name", board_settings.name + "_node"),
            (
                cb_script_name + ".inputs:node_set_parameters_service_name",
                cb_topic_prefix + "/set_parameters",
            ),
            (
                cb_script_name + ".inputs:node_parameters_event_topic_name",
                cb_topic_prefix + "/parameter_events",
            ),
            (
                cb_script_name + ".inputs:node_state_topic_name",
                cb_topic_prefix + "/joint_state",
            ),
            (
                cb_script_name + ".inputs:node_motor_state_topic_name",
                cb_topic_prefix + "/motor_state",
            ),
            (cb_script_name + ".inputs:node_timeout", full_settings.node_timeout),
            (cb_script_name + ".inputs:joint_names", board_settings.joint_names),
            (
                cb_script_name + ".inputs:position_p_gains",
                board_settings.position_p_gains,
            ),
            (
                cb_script_name + ".inputs:position_i_gains",
                board_settings.position_i_gains,
            ),
            (
                cb_script_name + ".inputs:position_d_gains",
                board_settings.position_d_gains,
            ),
            (
                cb_script_name + ".inputs:position_max_integral",
                board_settings.position_max_integral,
            ),
            (
                cb_script_name + ".inputs:position_max_output",
                board_settings.position_max_output,
            ),
            (
                cb_script_name + ".inputs:position_max_error",
                board_settings.position_max_error,
            ),
            (
                cb_script_name + ".inputs:position_default_velocity",
                board_settings.position_default_velocity,
            ),
            (
                cb_script_name + ".inputs:compliant_stiffness",
                board_settings.compliant_stiffness,
            ),
            (
                cb_script_name + ".inputs:compliant_damping",
                board_settings.compliant_damping,
            ),
            (
                cb_script_name + ".inputs:velocity_p_gains",
                board_settings.velocity_p_gains,
            ),
            (
                cb_script_name + ".inputs:velocity_i_gains",
                board_settings.velocity_i_gains,
            ),
            (
                cb_script_name + ".inputs:velocity_d_gains",
                board_settings.velocity_d_gains,
            ),
            (
                cb_script_name + ".inputs:velocity_max_integral",
                board_settings.velocity_max_integral,
            ),
            (
                cb_script_name + ".inputs:velocity_max_output",
                board_settings.velocity_max_output,
            ),
            (
                cb_script_name + ".inputs:velocity_max_error",
                board_settings.velocity_max_error,
            ),
            (cb_script_name + ".inputs:gearbox_ratios", board_settings.gearbox_ratios),
            (
                cb_script_name + ".inputs:motor_torque_constants",
                board_settings.motor_torque_constants,
            ),
            (
                cb_script_name + ".inputs:motor_current_noise_variance",
                board_settings.motor_current_noise_variance,
            ),
            (
                cb_script_name + ".inputs:motor_spring_stiffness",
                board_settings.motor_spring_stiffness,
            ),
            (
                cb_script_name + ".inputs:motor_max_currents",
                board_settings.motor_max_currents,
            ),
        ],
        graph_keys.PROMOTE_ATTRIBUTES: [
            (subscriber_name + ".inputs:execIn", "inputs:execIn"),
            (subscriber_name + ".inputs:context", "inputs:context"),
            (cb_script_name + ".inputs:timestamp", "inputs:timestamp"),
            (cb_script_name + ".inputs:deltaTime", "inputs:deltaTime"),
            (cb_script_name + ".outputs:execOut", "outputs:execOut"),
            (cb_script_name + ".outputs:desired_efforts", "outputs:desired_efforts"),
        ],
        graph_keys.CONNECT: [
            (
                subscriber_name + ".outputs:jointNames",
                cb_script_name + ".inputs:reference_joint_names",
            ),
            (
                subscriber_name + ".outputs:positionCommand",
                cb_script_name + ".inputs:reference_position_commands",
            ),
            (
                subscriber_name + ".outputs:velocityCommand",
                cb_script_name + ".inputs:reference_velocity_commands",
            ),
            (
                subscriber_name + ".outputs:effortCommand",
                cb_script_name + ".inputs:reference_effort_commands",
            ),
            (
                subscriber_name + ".outputs:timeStamp",
                cb_script_name + ".inputs:reference_timestamp",
            ),
        ],
    }

    output = {
        graph_keys.CREATE_NODES: [(compound_name, compound_actions)],
        # Since we can promote only one execIn, we do the connections here explicitly
        graph_keys.CONNECT: [
            (compound_name + ".inputs:execIn", cb_script_name + ".inputs:execIn"),
        ],
    }

    if promoted:
        output[graph_keys.PROMOTE_ATTRIBUTES] = [
            (compound_name + ".inputs:execIn", "inputs:execIn"),
            (compound_name + ".inputs:context", "inputs:context"),
            (compound_name + ".inputs:timestamp", "inputs:timestamp"),
            (compound_name + ".inputs:deltaTime", "inputs:deltaTime"),
        ]

    return output, compound_name


def create_control_board_compounds(graph_keys, settings):
    if len(settings.control_boards) == 0:
        return None

    compound_name = "ros2_control_boards_compound"
    connections = []
    subcompound_actions = []

    script_file = os.path.join(
        os.environ["ISAAC_SIM_YARP_BRIDGE_PATH"], "scripts", "omni_set_robot_efforts.py"
    )
    if not os.path.exists(script_file):
        raise FileNotFoundError(
            f"Set robot efforts script file not found at {script_file}. "
            "Please ensure ISAAC_SIM_YARP_BRIDGE_PATH is set correctly."
        )

    set_efforts_name = "set_robot_efforts"
    all_joints = []
    damping = []
    control_boards = []
    for cb in settings.control_boards:
        all_joints.extend(cb.joint_names)
        damping.extend(cb.joint_damping)
        control_boards.append(cb.name)

    set_efforts_actions = {
        graph_keys.CREATE_NODES: [
            (set_efforts_name, "omni.graph.scriptnode.ScriptNode"),
        ],
        graph_keys.CREATE_ATTRIBUTES: [
            (set_efforts_name + ".inputs:robot_prim", "target"),
            (set_efforts_name + ".inputs:joint_names", "token[]"),
            (set_efforts_name + ".inputs:desired_joint_damping", "double[]"),
            (set_efforts_name + ".inputs:control_board_names", "token[]"),
        ],
        graph_keys.SET_VALUES: [
            (set_efforts_name + ".inputs:usePath", True),
            (set_efforts_name + ".inputs:scriptPath", script_file),
            (set_efforts_name + ".inputs:robot_prim", settings.robot_path),
            (set_efforts_name + ".inputs:joint_names", all_joints),
            (set_efforts_name + ".inputs:desired_joint_damping", damping),
            (set_efforts_name + ".inputs:control_board_names", control_boards),
        ],
        graph_keys.CONNECT: [],
    }

    promote = True
    for board in settings.control_boards:
        compound_actions, subcompound_name = create_control_board_subcompound(
            graph_keys, board, settings, promoted=promote
        )
        subcompound_actions.append(compound_actions)
        if not promote:
            # Add the connections to the inner compound inputs for the internal inputs
            # that have not been promoted
            connections.append(
                (compound_name + ".inputs:execIn", subcompound_name + ".inputs:execIn")
            )
            connections.append(
                (
                    compound_name + ".inputs:context",
                    subcompound_name + ".inputs:context",
                )
            )
            connections.append(
                (
                    compound_name + ".inputs:timestamp",
                    subcompound_name + ".inputs:timestamp",
                )
            )
            connections.append(
                (
                    compound_name + ".inputs:deltaTime",
                    subcompound_name + ".inputs:deltaTime",
                )
            )

        promote = False  # Only the first compound promotes the attributes

        set_efforts_actions[graph_keys.CREATE_ATTRIBUTES].append(
            (f"{set_efforts_name}.inputs:{board.name}_desired_efforts", "double[]")
        )
        set_efforts_actions[graph_keys.CONNECT].append(
            (
                f"{subcompound_name}.outputs:desired_efforts",
                f"{set_efforts_name}.inputs:{board.name}_desired_efforts",
            ),
        )
        # We connect the execOut of each control board to the execIn of the set_efforts
        # node, so that we set the efforts only after all control boards have been
        # updated. The script inside the set_efforts node takes care of waiting
        # until it has received at least one execIn from each control board before
        # setting the efforts.
        connections.append(
            (subcompound_name + ".outputs:execOut", set_efforts_name + ".inputs:execIn")
        )

    subcompound_actions.append(set_efforts_actions)

    connections.append(("phys_tick.outputs:step", compound_name + ".inputs:execIn"))
    connections.append(
        ("ros2_context.outputs:context", compound_name + ".inputs:context")
    )
    connections.append(
        ("sim_time.outputs:simulationTime", compound_name + ".inputs:timestamp")
    )
    connections.append(
        ("phys_tick.outputs:deltaSimulationTime", compound_name + ".inputs:deltaTime")
    )

    return {
        graph_keys.CREATE_NODES: [(compound_name, merge_actions(subcompound_actions))],
        graph_keys.CONNECT: connections,
    }


def create_imu_subcompound(graph_keys, settings, name, target):
    read_node_name = "read_" + name
    publish_node_name = "publish_" + name
    return {
        graph_keys.CREATE_NODES: [
            (read_node_name, "isaacsim.sensors.physics.IsaacReadIMU"),
            (publish_node_name, "isaacsim.ros2.bridge.ROS2PublishImu"),
        ],
        graph_keys.SET_VALUES: [
            (read_node_name + ".inputs:imuPrim", target),
            (
                publish_node_name + ".inputs:topicName",
                settings.topic_prefix + "/IMU/" + name,
            ),
            (
                publish_node_name + ".inputs:frameId",
                name,
            ),
        ],
        graph_keys.PROMOTE_ATTRIBUTES: [
            (read_node_name + ".inputs:execIn", "inputs:execIn"),
            (publish_node_name + ".inputs:context", "inputs:context"),
        ],
        graph_keys.CONNECT: [
            (
                read_node_name + ".outputs:execOut",
                publish_node_name + ".inputs:execIn",
            ),
            (
                read_node_name + ".outputs:angVel",
                publish_node_name + ".inputs:angularVelocity",
            ),
            (
                read_node_name + ".outputs:linAcc",
                publish_node_name + ".inputs:linearAcceleration",
            ),
            (
                read_node_name + ".outputs:orientation",
                publish_node_name + ".inputs:orientation",
            ),
            (
                read_node_name + ".outputs:sensorTime",
                publish_node_name + ".inputs:timeStamp",
            ),
        ],
    }


def create_imu_compounds(graph_keys, settings):
    if len(settings.imus) == 0:
        return None

    compound_name = "ros2_imus_compound"
    imu_compounds = []
    first_compound = None
    connections = []
    for imu in settings.imus:
        subcompound_name = imu.name + "_compound"
        imu_compounds.append(
            (
                subcompound_name,
                create_imu_subcompound(graph_keys, settings, imu.name, imu.target),
            )
        )
        if not first_compound:
            # Only the first compound gets the attributes promoted
            first_compound = subcompound_name
        else:
            # Add the connections to the inner compound inputs for the internal inputs
            # that have not been promoted
            connections.append(
                (compound_name + ".inputs:execIn", subcompound_name + ".inputs:execIn")
            )
            connections.append(
                (
                    compound_name + ".inputs:context",
                    subcompound_name + ".inputs:context",
                )
            )

    connections.append(("phys_tick.outputs:step", compound_name + ".inputs:execIn"))
    connections.append(
        ("ros2_context.outputs:context", compound_name + ".inputs:context")
    )

    return {
        graph_keys.CREATE_NODES: [
            (
                compound_name,
                {
                    graph_keys.CREATE_NODES: imu_compounds,
                    graph_keys.PROMOTE_ATTRIBUTES: [
                        (first_compound + ".inputs:execIn", "inputs:execIn"),
                        (first_compound + ".inputs:context", "inputs:context"),
                    ],
                },
            )
        ],
        graph_keys.CONNECT: connections,
    }


# For the IMU we have a function that creates the nodes for a single IMU in
# a compound, and another function that creates all the IMU subcompounds
# and a compound to include them all.
# Fot the cameras instead, we have some additional connections to do after
# the creation of the subcompound (since we have two nodes that needs both
# execIn and context). Hence, the first function creates the nodes AND the
# subcompound, while the second creates the full compound merging all actions.


def create_camera_subcompound(
    graph_keys,
    topic_prefix,
    camera_prefix,
    camera_suffix,
    camera_target,
    camera_type,
    promoted,
):
    camera_prefix_name = camera_prefix.replace("/", "_")
    render_node_name = "render_" + camera_prefix_name + "_" + camera_suffix
    publish_node_name = "publish_" + camera_prefix_name + "_" + camera_suffix
    publish_info_node_name = "publish_info_" + camera_prefix_name + "_" + camera_suffix
    compound_name = camera_prefix_name + "_" + camera_suffix + "_compound"
    topic_name = topic_prefix + "/" + camera_prefix + "/" + camera_suffix
    topic_info_name = topic_name + "/info"

    compound_actions = {
        graph_keys.CREATE_NODES: [
            (
                render_node_name,
                "isaacsim.core.nodes.IsaacCreateRenderProduct",
            ),
            (publish_node_name, "isaacsim.ros2.bridge.ROS2CameraHelper"),
            (
                publish_info_node_name,
                "isaacsim.ros2.bridge.ROS2CameraInfoHelper",
            ),
        ],
        graph_keys.SET_VALUES: [
            (
                render_node_name + ".inputs:cameraPrim",
                camera_target,
            ),
            (
                publish_node_name + ".inputs:topicName",
                topic_name,
            ),
            (publish_node_name + ".inputs:type", camera_type),
            (
                publish_info_node_name + ".inputs:topicName",
                topic_info_name,
            ),
        ],
        graph_keys.PROMOTE_ATTRIBUTES: [
            (render_node_name + ".inputs:execIn", "inputs:execIn"),
            (publish_node_name + ".inputs:context", "inputs:context"),
        ],
        graph_keys.CONNECT: [
            (
                render_node_name + ".outputs:execOut",
                publish_node_name + ".inputs:execIn",
            ),
            (
                render_node_name + ".outputs:execOut",
                publish_info_node_name + ".inputs:execIn",
            ),
            (
                render_node_name + ".outputs:renderProductPath",
                publish_node_name + ".inputs:renderProductPath",
            ),
            (
                render_node_name + ".outputs:renderProductPath",
                publish_info_node_name + ".inputs:renderProductPath",
            ),
        ],
    }

    output = {
        graph_keys.CREATE_NODES: [(compound_name, compound_actions)],
        # Since we can promote only one "context" input, we do the connection
        # here explicitly
        graph_keys.CONNECT: [
            (
                compound_name + ".inputs:context",
                publish_info_node_name + ".inputs:context",
            ),
        ],
    }

    if promoted:
        # If the sucompound is promoted, promote the inputs
        output[graph_keys.PROMOTE_ATTRIBUTES] = [
            (compound_name + ".inputs:execIn", "inputs:execIn"),
            (compound_name + ".inputs:context", "inputs:context"),
        ]

    return output, compound_name


def create_camera_compounds(graph_keys, settings):
    if len(settings.cameras) == 0:
        return None

    compound_name = "ros2_cameras_compound"
    subcompound_actions = []
    promote = True
    connections = []
    for camera in settings.cameras:
        actions, subcompound_name = create_camera_subcompound(
            graph_keys,
            settings.topic_prefix,
            camera.prefix,
            camera.suffix,
            camera.target,
            camera.type,
            promote,
        )
        subcompound_actions.append(actions)

        if not promote:
            connections.append(
                (compound_name + ".inputs:execIn", subcompound_name + ".inputs:execIn")
            )
            connections.append(
                (
                    compound_name + ".inputs:context",
                    subcompound_name + ".inputs:context",
                )
            )

        promote = False  # We promote only the first

    connections.append(("tick.outputs:tick", compound_name + ".inputs:execIn"))
    connections.append(
        ("ros2_context_camera.outputs:context", compound_name + ".inputs:context")
    )

    return {
        graph_keys.CREATE_NODES: [(compound_name, merge_actions(subcompound_actions))],
        graph_keys.CONNECT: connections,
    }


def create_ft_subcompound(graph_keys, name, joint, frame, flip, topic_prefix):
    script_node_name = "script_" + name
    break_force_node_name = "break_" + name + "_force"
    break_torque_node_name = "break_" + name + "_torque"
    publish_node_name = "publisher_" + name
    topic_name = topic_prefix + "/" + name
    script_path = os.path.join(
        os.environ["ISAAC_SIM_YARP_BRIDGE_PATH"], "scripts", "omni_read_FT.py"
    )

    if not os.path.exists(script_path):
        raise FileNotFoundError(
            f"FT script file not found at {script_path}. "
            "Please ensure ISAAC_SIM_YARP_BRIDGE_PATH is set correctly."
        )

    return {
        graph_keys.CREATE_NODES: [
            (script_node_name, "omni.graph.scriptnode.ScriptNode"),
            (break_force_node_name, "omni.graph.nodes.BreakVector3"),
            (break_torque_node_name, "omni.graph.nodes.BreakVector3"),
            (publish_node_name, "isaacsim.ros2.bridge.ROS2Publisher"),
        ],
        graph_keys.CREATE_ATTRIBUTES: [
            (script_node_name + ".inputs:FTJoint", "target"),
            (script_node_name + ".inputs:FTFrame", "target"),
            (script_node_name + ".inputs:flipMeasure", "bool"),
            (script_node_name + ".inputs:timestamp", "double"),
            (script_node_name + ".outputs:force", "double[3]"),
            (script_node_name + ".outputs:torque", "double[3]"),
            (script_node_name + ".outputs:value_sec", "int"),
            (script_node_name + ".outputs:value_nanosec", "uint"),
            # The following would be created automatically after creation
            # but in order to connect to them, we create them manually
            (publish_node_name + ".inputs:header:frame_id", "token"),
            (publish_node_name + ".inputs:header:stamp:nanosec", "uint"),
            (publish_node_name + ".inputs:header:stamp:sec", "int"),
            (publish_node_name + ".inputs:wrench:force:x", "double"),
            (publish_node_name + ".inputs:wrench:force:y", "double"),
            (publish_node_name + ".inputs:wrench:force:z", "double"),
            (publish_node_name + ".inputs:wrench:torque:x", "double"),
            (publish_node_name + ".inputs:wrench:torque:y", "double"),
            (publish_node_name + ".inputs:wrench:torque:z", "double"),
        ],
        graph_keys.SET_VALUES: [
            (script_node_name + ".inputs:FTJoint", joint),
            (script_node_name + ".inputs:FTFrame", frame),
            (script_node_name + ".inputs:flipMeasure", flip),
            (script_node_name + ".inputs:usePath", True),
            (script_node_name + ".inputs:scriptPath", script_path),
            (publish_node_name + ".inputs:header:frame_id", name),
            (publish_node_name + ".inputs:messageName", "WrenchStamped"),
            (publish_node_name + ".inputs:messagePackage", "geometry_msgs"),
            (
                publish_node_name + ".inputs:topicName",
                topic_name,
            ),
        ],
        graph_keys.PROMOTE_ATTRIBUTES: [
            (script_node_name + ".inputs:timestamp", "inputs:timestamp"),
            (script_node_name + ".inputs:execIn", "inputs:execIn"),
            (publish_node_name + ".inputs:context", "inputs:context"),
        ],
        graph_keys.CONNECT: [
            (
                script_node_name + ".outputs:execOut",
                publish_node_name + ".inputs:execIn",
            ),
            (
                script_node_name + ".outputs:force",
                break_force_node_name + ".inputs:tuple",
            ),
            (
                break_force_node_name + ".outputs:x",
                publish_node_name + ".inputs:wrench:force:x",
            ),
            (
                break_force_node_name + ".outputs:y",
                publish_node_name + ".inputs:wrench:force:y",
            ),
            (
                break_force_node_name + ".outputs:z",
                publish_node_name + ".inputs:wrench:force:z",
            ),
            (
                script_node_name + ".outputs:torque",
                break_torque_node_name + ".inputs:tuple",
            ),
            (
                break_torque_node_name + ".outputs:x",
                publish_node_name + ".inputs:wrench:torque:x",
            ),
            (
                break_torque_node_name + ".outputs:y",
                publish_node_name + ".inputs:wrench:torque:y",
            ),
            (
                break_torque_node_name + ".outputs:z",
                publish_node_name + ".inputs:wrench:torque:z",
            ),
            (
                script_node_name + ".outputs:value_sec",
                publish_node_name + ".inputs:header:stamp:sec",
            ),
            (
                script_node_name + ".outputs:value_nanosec",
                publish_node_name + ".inputs:header:stamp:nanosec",
            ),
        ],
    }


def create_ft_compounds(graph_keys, settings):
    if len(settings.FTs) == 0:
        return None

    ft_compounds = []
    first_compound = None
    connections = []

    compound_name = "ros2_FTs_compound"
    for ft in settings.FTs:
        subcompound_name = ft.name + "_compound"
        ft_compounds.append(
            (
                subcompound_name,
                create_ft_subcompound(
                    graph_keys,
                    ft.name,
                    ft.joint,
                    ft.frame,
                    ft.flip,
                    settings.topic_prefix + "/FT",
                ),
            )
        )
        if not first_compound:
            # Only the first compound gets the attributes promoted
            first_compound = subcompound_name
        else:
            # Add the connections to the inner compound inputs for the internal inputs
            # that have not been promoted
            connections.append(
                (
                    compound_name + ".inputs:timestamp",
                    subcompound_name + ".inputs:timestamp",
                )
            )
            connections.append(
                (compound_name + ".inputs:execIn", subcompound_name + ".inputs:execIn")
            )
            connections.append(
                (
                    compound_name + ".inputs:context",
                    subcompound_name + ".inputs:context",
                )
            )

    connections.append(
        ("sim_time.outputs:simulationTime", compound_name + ".inputs:timestamp")
    )
    connections.append(("phys_tick.outputs:step", compound_name + ".inputs:execIn"))
    connections.append(
        ("ros2_context.outputs:context", compound_name + ".inputs:context")
    )

    return {
        graph_keys.CREATE_NODES: [
            (
                compound_name,
                {
                    graph_keys.CREATE_NODES: ft_compounds,
                    graph_keys.PROMOTE_ATTRIBUTES: [
                        (first_compound + ".inputs:timestamp", "inputs:timestamp"),
                        (first_compound + ".inputs:execIn", "inputs:execIn"),
                        (first_compound + ".inputs:context", "inputs:context"),
                    ],
                },
            )
        ],
        graph_keys.CONNECT: connections,
    }


def create_graph(path, pipeline_stage, actions_list):
    (graph, nodes, prims, name_to_object_map) = og.Controller.edit(
        {
            "graph_path": path,
            "evaluator_name": "execution",
            "pipeline_stage": pipeline_stage,
        },
        merge_actions(actions_list),
    )
    if graph.is_valid():
        print(f"Graph {path} created successfully!")
    else:
        print("FAILED TO CREATE GRAPH!")


def set_sim_frequencies(phys_frequency, render_frequency):
    if phys_frequency is not None:

        stage = stage_utils.get_current_stage()

        physics_scene_name = "/World/physicsScene"
        # Add a physics scene prim to stage
        UsdPhysics.Scene.Define(stage, Sdf.Path(physics_scene_name))

        # Add PhysxSceneAPI
        PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath(physics_scene_name))

        physx_scene_api = PhysxSchema.PhysxSceneAPI.Get(stage, physics_scene_name)
        # Number of physics steps per second
        physx_scene_api.CreateTimeStepsPerSecondAttr(phys_frequency)
        print(f"Physics frequency changed to {phys_frequency}Hz.")

    if render_frequency is not None:
        layer = stage.GetRootLayer()
        # Number of rendering updates per second
        # This frequency also affects the "Playback Tick" of the OmniGraph
        layer.timeCodesPerSecond = render_frequency
        print(f"Render frequency changed to {render_frequency}Hz.")


#######################################################################################
########################## EDIT ONLY THE SETTINGS HERE BELOW ##########################
#######################################################################################
robot_name = "ergoCubSN002"
robot_path = f"/World/{robot_name}/robot"
realsense_prefix = robot_path + "/realsense/sensors/RSD455"
s = Settings(
    sensors_graph_path=f"/World/{robot_name}/ros2_action_graph_sensors",
    camera_graph_path=f"/World/{robot_name}/ros2_action_graph_camera",
    robot_path=robot_path,
    topic_prefix="/ergocub",
    domain_id=0,
    useDomainIDEnvVar=True,
    node_timeout=0,
    control_boards=[
        ControlBoard(
            name="head",
            joint_names=["neck_pitch", "neck_roll", "neck_yaw", "camera_tilt"],
            joint_damping=[1.0, 1.0, 1.0, 1.0],
            position_p_gains=[17.0, 17.0, 17.0, 0.5],
            position_i_gains=[0.003, 0.003, 0.003, 0.003],
            position_d_gains=[0.5, 0.5, 0.5, 0.01],
            position_max_integral=[9999.0, 9999.0, 9999.0, 9999.0],
            position_max_output=[9999.0, 9999.0, 9999.0, 9999.0],
            position_max_error=[9999.0, 9999.0, 9999.0, 9999.0],
            velocity_p_gains=[8.726, 8.726, 8.726, 8.726],
            velocity_i_gains=[0.003, 0.003, 0.003, 0.003],
            velocity_d_gains=[0.035, 0.035, 0.035, 0.035],
            velocity_max_integral=[9999.0, 9999.0, 9999.0, 9999.0],
            velocity_max_output=[9999.0, 9999.0, 9999.0, 9999.0],
            velocity_max_error=[9999.0, 9999.0, 9999.0, 9999.0],
            position_default_velocity=10.0 * math.pi / 180.0,
            compliant_stiffness=[0.0, 0.0, 0.0, 0.0],
            compliant_damping=[0.0, 0.0, 0.0, 0.0],
            gearbox_ratios=[100.0, 100.0, 100.0, -141.0],
            motor_torque_constants=[1.0, 1.0, 1.0, 1.0],
            motor_current_noise_variance=[0.01, 0.01, 0.01, 0.01],
            motor_spring_stiffness=[0.0, 0.0, 0.0, 0.0],
            motor_max_currents=[9999.0, 9999.0, 9999.0, 9999.0],
        ),
        ControlBoard(
            name="torso",
            joint_names=["torso_roll", "torso_pitch", "torso_yaw"],
            joint_damping=[1.0, 1.0, 1.0],
            position_p_gains=[x * 10 for x in [70.0, 70.0, 70.0]],
            position_i_gains=[x * 10 for x in [0.17, 0.17, 0.17]],
            position_d_gains=[x * 20 for x in [0.15, 0.15, 0.15]],
            position_max_integral=[9999.0, 9999.0, 9999.0],
            position_max_output=[9999.0, 9999.0, 9999.0],
            position_max_error=[9999.0, 9999.0, 9999.0],
            velocity_p_gains=[8.726, 8.726, 8.726],
            velocity_i_gains=[0.002, 0.002, 0.002],
            velocity_d_gains=[0.035, 0.035, 0.035],
            velocity_max_integral=[9999.0, 9999.0, 9999.0],
            velocity_max_output=[9999.0, 9999.0, 9999.0],
            velocity_max_error=[9999.0, 9999.0, 9999.0],
            position_default_velocity=10.0 * math.pi / 180.0,
            compliant_stiffness=[0.0, 0.0, 0.0],
            compliant_damping=[0.0, 0.0, 0.0],
            gearbox_ratios=[100.0, 160.0, -100.0],
            motor_torque_constants=[1.0, 1.0, 1.0],
            motor_current_noise_variance=[0.01, 0.01, 0.01],
            motor_spring_stiffness=[0.0, 0.0, 0.0],
            motor_max_currents=[9999.0, 9999.0, 9999.0],
        ),
        ControlBoard(
            name="left_leg",
            joint_names=[
                "l_hip_pitch",
                "l_hip_roll",
                "l_hip_yaw",
                "l_knee",
                "l_ankle_pitch",
                "l_ankle_roll",
            ],
            joint_damping=[x * 5.0 for x in [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]],
            position_p_gains=[
                x * 3.0 for x in [1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1200.0]
            ],
            position_i_gains=[x * 0.0 for x in [0.17, 0.17, 0.35, 0.35, 0.35, 0.35]],
            position_d_gains=[x * 20.0 for x in [0.15, 0.15, 0.35, 0.15, 0.15, 0.15]],
            position_max_integral=[9999.0] * 6,
            position_max_output=[9999.0] * 6,
            position_max_error=[9999.0] * 6,
            velocity_p_gains=[8.726, 8.726, 8.726, 8.726, 8.726, 8.726],
            velocity_i_gains=[0.176, 0.176, 0.176, 0.176, 0.176, 0.176],
            velocity_d_gains=[0.349, 0.349, 0.349, 0.349, 0.349, 0.349],
            velocity_max_integral=[9999.0] * 6,
            velocity_max_output=[9999.0] * 6,
            velocity_max_error=[9999.0] * 6,
            position_default_velocity=10.0 * math.pi / 180.0,
            compliant_stiffness=[0.1, 0.1, 0.1, 0.1, 30.0, 30.0],
            compliant_damping=[0.05, 0.05, 0.05, 0.05, 0.05, 0.05],
            gearbox_ratios=[-100.0, 160.0, -100.0, -100.0, -100.00, -160.00],
            motor_torque_constants=[1.0] * 6,
            motor_current_noise_variance=[0.01] * 6,
            motor_spring_stiffness=[0.0] * 6,
            motor_max_currents=[9999.0] * 6,
        ),
        ControlBoard(
            name="right_leg",
            joint_names=[
                "r_hip_pitch",
                "r_hip_roll",
                "r_hip_yaw",
                "r_knee",
                "r_ankle_pitch",
                "r_ankle_roll",
            ],
            joint_damping=[x * 5.0 for x in [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]],
            position_p_gains=[
                x * 3.0 for x in [1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1200.0]
            ],
            position_i_gains=[x * 0.0 for x in [0.17, 0.17, 0.35, 0.35, 0.35, 0.35]],
            position_d_gains=[x * 20.0 for x in [0.15, 0.15, 0.35, 0.15, 0.15, 0.15]],
            position_max_integral=[9999.0] * 6,
            position_max_output=[9999.0] * 6,
            position_max_error=[9999.0] * 6,
            velocity_p_gains=[8.726, 8.726, 8.726, 8.726, 8.726, 8.726],
            velocity_i_gains=[0.176, 0.176, 0.176, 0.176, 0.176, 0.176],
            velocity_d_gains=[0.349, 0.349, 0.349, 0.349, 0.349, 0.349],
            velocity_max_integral=[9999.0] * 6,
            velocity_max_output=[9999.0] * 6,
            velocity_max_error=[9999.0] * 6,
            position_default_velocity=10.0 * math.pi / 180.0,
            compliant_stiffness=[0.1, 0.1, 0.1, 0.1, 30.0, 30.0],
            compliant_damping=[0.05, 0.05, 0.05, 0.05, 0.05, 0.05],
            gearbox_ratios=[100.00, -160.00, 100.0, 100.0, 100.0, 160.0],
            motor_torque_constants=[1.0] * 6,
            motor_current_noise_variance=[0.01] * 6,
            motor_spring_stiffness=[0.0] * 6,
            motor_max_currents=[9999.0] * 6,
        ),
        ControlBoard(
            name="left_arm",
            joint_names=[
                "l_shoulder_pitch",
                "l_shoulder_roll",
                "l_shoulder_yaw",
                "l_elbow",
                "l_wrist_yaw",
                "l_wrist_roll",
                "l_wrist_pitch",
            ],
            joint_damping=[x * 1.0 for x in [10.0, 10.0, 10.0, 10.0, 1.0, 1.0, 1.0]],
            position_p_gains=[
                x * 5.0 for x in [100.0, 100.0, 100.0, 100.0, 1.745, 1.745, 1.745]
            ],
            position_i_gains=[0.174, 0.174, 0.174, 0.174, 0.174, 0.174, 0.0],
            position_d_gains=[0.174, 0.174, 0.174, 0.174, 0.174, 0.174, 0.0],
            position_max_integral=[9999.0] * 7,
            position_max_output=[9999.0] * 7,
            position_max_error=[9999.0] * 7,
            velocity_p_gains=[8.726, 8.726, 8.726, 5.236, 5.236, 5.236, 5.236],
            velocity_i_gains=[0.002, 0.002, 0.002, 0.0, 0.0, 0.0, 0.0],
            velocity_d_gains=[0.035, 0.035, 0.035, 0.002, 0.002, 0.002, 0.0],
            velocity_max_integral=[9999.0] * 7,
            velocity_max_output=[9999.0] * 7,
            velocity_max_error=[9999.0] * 7,
            position_default_velocity=10.0 * math.pi / 180.0,
            compliant_stiffness=[0.3, 0.3, 0.2, 0.1, 0.0, 0.0, 0.0],
            compliant_damping=[0.05, 0.05, 0.05, 0.05, 0.0, 0.0, 0.0],
            gearbox_ratios=[100.0, 100.0, 100.0, -100.0, -392.0, -392.0, -392.0],
            motor_torque_constants=[1.0] * 7,
            motor_current_noise_variance=[0.01] * 7,
            motor_spring_stiffness=[0.0] * 7,
            motor_max_currents=[9999.0] * 7,
        ),
        ControlBoard(
            name="right_arm",
            joint_names=[
                "r_shoulder_pitch",
                "r_shoulder_roll",
                "r_shoulder_yaw",
                "r_elbow",
                "r_wrist_yaw",
                "r_wrist_roll",
                "r_wrist_pitch",
            ],
            joint_damping=[x * 1.0 for x in [10.0, 10.0, 10.0, 10.0, 1.0, 1.0, 1.0]],
            position_p_gains=[
                x * 5.0 for x in [100.0, 100.0, 100.0, 100.0, 1.745, 1.745, 1.745]
            ],
            position_i_gains=[0.174, 0.174, 0.174, 0.174, 0.174, 0.174, 0.0],
            position_d_gains=[0.174, 0.174, 0.174, 0.174, 0.174, 0.174, 0.0],
            position_max_integral=[9999.0] * 7,
            position_max_output=[9999.0] * 7,
            position_max_error=[9999.0] * 7,
            velocity_p_gains=[8.726, 8.726, 8.726, 5.236, 5.236, 5.236, 5.236],
            velocity_i_gains=[0.002, 0.002, 0.002, 0.0, 0.0, 0.0, 0.0],
            velocity_d_gains=[0.035, 0.035, 0.035, 0.002, 0.002, 0.002, 0.0],
            velocity_max_integral=[9999.0] * 7,
            velocity_max_output=[9999.0] * 7,
            velocity_max_error=[9999.0] * 7,
            position_default_velocity=10.0 * math.pi / 180.0,
            compliant_stiffness=[0.3, 0.3, 0.2, 0.1, 0.0, 0.0, 0.0],
            compliant_damping=[0.05, 0.05, 0.05, 0.05, 0.0, 0.0, 0.0],
            gearbox_ratios=[-100.0, -100.0, -100.0, 100.0, -392.0, -392.0, -392.0],
            motor_torque_constants=[1.0] * 7,
            motor_current_noise_variance=[0.01] * 7,
            motor_spring_stiffness=[0.0] * 7,
            motor_max_currents=[9999.0] * 7,
        ),
        ControlBoard(
            name="left_hand",
            joint_names=[
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
            ],
            joint_damping=[1.0] * 12,
            position_p_gains=[5.0] * 12,
            position_i_gains=[0.0] * 12,
            position_d_gains=[0.0] * 12,
            position_max_integral=[9999.0] * 12,
            position_max_output=[9999.0] * 12,
            position_max_error=[9999.0] * 12,
            velocity_p_gains=[0.01] * 12,
            velocity_i_gains=[0.0] * 12,
            velocity_d_gains=[0.0] * 12,
            velocity_max_integral=[9999.0] * 12,
            velocity_max_output=[9999.0] * 12,
            velocity_max_error=[9999.0] * 12,
            position_default_velocity=10.0 * math.pi / 180.0,
            compliant_stiffness=[0.0] * 12,
            compliant_damping=[0.0] * 12,
            gearbox_ratios=[159.0] * 12,
            motor_torque_constants=[1.0] * 12,
            motor_current_noise_variance=[0.01] * 12,
            motor_spring_stiffness=[0.1] * 12,
            motor_max_currents=[9999.0] * 12,
        ),
        ControlBoard(
            name="right_hand",
            joint_names=[
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
            ],
            joint_damping=[1.0] * 12,
            position_p_gains=[5.0] * 12,
            position_i_gains=[0.0] * 12,
            position_d_gains=[0.0] * 12,
            position_max_integral=[9999.0] * 12,
            position_max_output=[9999.0] * 12,
            position_max_error=[9999.0] * 12,
            velocity_p_gains=[0.01] * 12,
            velocity_i_gains=[0.0] * 12,
            velocity_d_gains=[0.0] * 12,
            velocity_max_integral=[9999.0] * 12,
            velocity_max_output=[9999.0] * 12,
            velocity_max_error=[9999.0] * 12,
            position_default_velocity=10.0 * math.pi / 180.0,
            compliant_stiffness=[0.0] * 12,
            compliant_damping=[0.0] * 12,
            gearbox_ratios=[159.0] * 12,
            motor_torque_constants=[1.0] * 12,
            motor_current_noise_variance=[0.01] * 12,
            motor_spring_stiffness=[0.1] * 12,
            motor_max_currents=[9999.0] * 12,
        ),
    ],
    imus=[
        # The name of the imu is also the frameId
        Imu(name="waist_imu_0", target=robot_path + "/torso_1/waist_imu_0_sensor"),
        Imu(name="head_imu_0", target=robot_path + "/head/head_imu_0_sensor"),
    ],
    cameras=[
        Camera(
            prefix="realsense",
            suffix="rgb",
            type="rgb",
            target=realsense_prefix + "/Camera_OmniVision_OV9782_Color",
        ),
        Camera(
            prefix="realsense",
            suffix="depth",
            type="depth",
            target=realsense_prefix + "/Camera_Pseudo_Depth",
        ),
    ],
    # The name of the FT is also the frameId
    FTs=[
        FT(
            name="l_leg_ft",
            joint=robot_path + "/joints/l_leg_ft_sensor",
            frame=robot_path + "/l_hip_2/l_leg_ft",
            flip=False,
        ),
        FT(
            name="l_foot_front_ft",
            joint=robot_path + "/joints/l_foot_front_ft_sensor",
            frame=robot_path + "/l_ankle_2/l_foot_front_ft",
            flip=False,
        ),
        FT(
            name="l_foot_rear_ft",
            joint=robot_path + "/joints/l_foot_rear_ft_sensor",
            frame=robot_path + "/l_ankle_2/l_foot_rear_ft",
            flip=False,
        ),
        FT(
            name="r_leg_ft",
            joint=robot_path + "/joints/r_leg_ft_sensor",
            frame=robot_path + "/r_hip_2/r_leg_ft",
            flip=False,
        ),
        FT(
            name="r_foot_front_ft",
            joint=robot_path + "/joints/r_foot_front_ft_sensor",
            frame=robot_path + "/r_ankle_2/r_foot_front_ft",
            flip=False,
        ),
        FT(
            name="r_foot_rear_ft",
            joint=robot_path + "/joints/r_foot_rear_ft_sensor",
            frame=robot_path + "/r_ankle_2/r_foot_rear_ft",
            flip=False,
        ),
        FT(
            name="l_arm_ft",
            joint=robot_path + "/joints/l_arm_ft_sensor",
            frame=robot_path + "/l_shoulder_2/l_arm_ft",
            flip=False,
        ),
        FT(
            name="r_arm_ft",
            joint=robot_path + "/joints/r_arm_ft_sensor",
            frame=robot_path + "/r_shoulder_2/r_arm_ft",
            flip=False,
        ),
    ],
    physics_frequency=100.0,  # Physics frequency in Hz (set to None to keep default)
    render_frequency=30.0,  # Render frequency in Hz (set to None to keep default)
)

#######################################################################################
#######################################################################################
#######################################################################################

set_sim_frequencies(s.physics_frequency, s.render_frequency)
keys = og.Controller.Keys
create_graph(
    path=s.sensors_graph_path,
    pipeline_stage=og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
    actions_list=[
        create_basic_nodes_sensors(keys, s),
        create_ros2_clock_publisher(keys),
        create_control_board_compounds(keys, s),
        create_imu_compounds(keys, s),
        create_ft_compounds(keys, s),
    ],
)

create_graph(
    path=s.camera_graph_path,
    pipeline_stage=og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
    actions_list=[
        create_basic_nodes_camera(keys, s),
        create_camera_compounds(keys, s),
    ],
)
