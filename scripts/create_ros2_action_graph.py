#######################################################################################
################### EDIT ONLY THE SETTINGS AT THE END OF THE FILE #####################
#######################################################################################


import dataclasses

import omni.graph.core as og


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
    graph_path: str
    articulation_root: str
    topic_prefix: str
    imus: list[Imu]
    cameras: list[Camera]
    FTs: list[FT]


def merge_actions(action_list):
    output = {}
    for d in action_list:
        for key, value in d.items():
            if key in output:
                for val in value:
                    output[key].append(val)
            else:
                output[key] = value
    return output


def add_basic_nodes(graph_keys):
    return {
        graph_keys.CREATE_NODES: [
            ("tick", "omni.graph.action.OnPlaybackTick"),
            ("sim_time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("ros2_context", "isaacsim.ros2.bridge.ROS2Context"),
        ],
    }


def add_ros2_clock_publisher(graph_keys):
    return {
        graph_keys.CREATE_NODES: [
            ("ros2_clock_publisher", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ],
        graph_keys.CONNECT: [
            ("tick.outputs:tick", "ros2_clock_publisher.inputs:execIn"),
            ("ros2_context.outputs:context", "ros2_clock_publisher.inputs:context"),
            (
                "sim_time.outputs:simulationTime",
                "ros2_clock_publisher.inputs:timeStamp",
            ),
        ],
    }


# When creating nodes, first you specify the name of the node, and then the type of
# node. If instead of the type, you pass a dict with a series of other actions, you
# are creating a compound node instead (basically a subgraph). The inputs and
# outputs of a compound node are defined by "promoting" inputs and outputs of the
# internal nodes. If multiple internal nodes need to be connected to the same
# input/outputs, this connection is done explicitly after creating the compound,
# using the compound name.


def add_ros2_joint_compound(graph_keys, settings):
    compound_name = "ros2_joint_compound"
    return {
        graph_keys.CREATE_NODES: [
            (
                compound_name,
                {
                    graph_keys.CREATE_NODES: [
                        (
                            "ros2_joint_publisher",
                            "isaacsim.ros2.bridge.ROS2PublishJointState",
                        ),
                        (
                            "ros2_joint_subscriber",
                            "isaacsim.ros2.bridge.ROS2SubscribeJointState",
                        ),
                        (
                            "articulation_controller",
                            "isaacsim.core.nodes.IsaacArticulationController",
                        ),
                    ],
                    graph_keys.SET_VALUES: [
                        (
                            "ros2_joint_publisher.inputs:targetPrim",
                            settings.articulation_root,
                        ),
                        (
                            "ros2_joint_publisher.inputs:topicName",
                            settings.topic_prefix + "/joint_state",
                        ),
                        (
                            "ros2_joint_subscriber.inputs:topicName",
                            settings.topic_prefix + "/joint_state/input",
                        ),
                        (
                            "articulation_controller.inputs:targetPrim",
                            settings.articulation_root,
                        ),
                    ],
                    graph_keys.PROMOTE_ATTRIBUTES: [
                        ("ros2_joint_publisher.inputs:execIn", "inputs:execIn"),
                        ("ros2_joint_publisher.inputs:context", "inputs:context"),
                        ("ros2_joint_publisher.inputs:timeStamp", "inputs:timeStamp"),
                    ],
                    graph_keys.CONNECT: [
                        (
                            "ros2_joint_subscriber.outputs:effortCommand",
                            "articulation_controller.inputs:effortCommand",
                        ),
                        (
                            "ros2_joint_subscriber.outputs:jointNames",
                            "articulation_controller.inputs:jointNames",
                        ),
                        (
                            "ros2_joint_subscriber.outputs:positionCommand",
                            "articulation_controller.inputs:positionCommand",
                        ),
                        (
                            "ros2_joint_subscriber.outputs:velocityCommand",
                            "articulation_controller.inputs:velocityCommand",
                        ),
                    ],
                },
            )
        ],
        graph_keys.CONNECT: [
            ("tick.outputs:tick", compound_name + ".inputs:execIn"),
            (
                "ros2_context.outputs:context",
                compound_name + ".inputs:context",
            ),
            (
                "sim_time.outputs:simulationTime",
                compound_name + ".inputs:timeStamp",
            ),
            (
                compound_name + ".inputs:execIn",
                "ros2_joint_subscriber.inputs:execIn",
            ),
            (
                compound_name + ".inputs:context",
                "ros2_joint_subscriber.inputs:context",
            ),
            (
                compound_name + ".inputs:execIn",
                "articulation_controller.inputs:execIn",
            ),
        ],
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
        return

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

    connections.append(("tick.outputs:tick", compound_name + ".inputs:execIn"))
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
        return

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
        ("ros2_context.outputs:context", compound_name + ".inputs:context")
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

    script_code = """
# Expects three inputs:
# - FTJoint [target] The fixed joint corresponding to the FT sensor, e.g. /World/ergoCubSN002/joints/l_foot_front_ft_sensor
# - FTFrame [target] The link with respect to which express the measure, e.g. /World/ergoCubSN002/l_foot_front_ft
# - flipMeasure [bool] A boolean to flip the measurement. By default (false), the measured wrench is the one exerted by the FT.

# Expects two outputs:
# - force [double[3]] The measured force
# - torque [double[3]] The measured torque

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


def create_robot_object(db):
    FTJoint = db.inputs.FTJoint

    if not FTJoint:
        db.log_error(f"FTJoint input is empty")
        return

    stage = stage_utils.get_current_stage()

    FTJoint_path = FTJoint[0].pathString
    joint_prim = stage.GetPrimAtPath(FTJoint_path)
    joint_name = joint_prim.GetName()

    if not joint_prim.IsValid():
        db.log_error(f"The joint prim ({joint_prim}) is not valid")
        return

    if not joint_prim.HasAPI("IsaacJointAPI"):
        db.log_error(f"The specified prim ({joint_prim}) is not a joint")
        return

    parent_prim = get_parent_robot(joint_prim)

    if not parent_prim.IsValid():
        db.log_error(
            f"Failed to find a parent to {joint_prim} that has the IsaacRobotAPI."
            f"Is the joint attached to a robot?"
        )
        return

    robot_object = RobotView(
        prim_paths_expr=str(parent_prim.GetPath()), name=f"robot_ft_{FTJoint_path}"
    )

    if joint_name in robot_object.dof_names:
        db.log_error(f"The FT joint {joint_name} is not fixed.")
        return

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
        return

    FTFrame_robot_prim = get_parent_robot(FTFrame_prim)

    if FTFrame_robot_prim != parent_prim:
        db.log_error(
            f"The specified prim ({FTFrame_prim}) has parent "
            f"{FTFrame_robot_prim} that is different from {parent_prim}."
        )
        return

    relative_transform = get_a_H_b(FTFrame_prim, b1_prim)

    print(
        f"Info: expressing {joint_name} readings in {FTFrame_prim.GetName()} "
        f"with relative transform:"
    )
    print(relative_transform)

    return robot_object, joint_index, relative_transform


def internal_state():
    return CustomState()


def setup(db: og.Database) -> bool:
    output = create_robot_object(db)
    if not output:
        db.log_error("Setup failed")
        return False
    robot_object, joint_index, sensor_transform = output
    db.per_instance_state.robot_object = robot_object
    db.per_instance_state.joint_index = joint_index
    db.per_instance_state.sensor_transform = sensor_transform
    return True


def cleanup(db: og.Database):
    db.per_instance_state.robot_object = None
    db.per_instance_state.joint_index = None
    db.per_instance_state.sensor_transform = np.eye(4)


def compute(db: og.Database) -> bool:
    state = db.per_instance_state
    if not hasattr(state, "robot_object") or state.robot_object is None:
        setup(db)
        return False

    out = state.robot_object.get_measured_joint_forces(
        joint_indices=[state.joint_index]
    )
    if out is None:
        db.log_warning(f"Failed to get measured joint forces. Running setup again")
        setup(db)
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

    return True

    """

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
            (script_node_name + ".outputs:force", "double[3]"),
            (script_node_name + ".outputs:torque", "double[3]"),
            # The following would be created automatically after creation
            # but in order to connect to them, we create them manually
            (publish_node_name + ".inputs:force:x", "double"),
            (publish_node_name + ".inputs:force:y", "double"),
            (publish_node_name + ".inputs:force:z", "double"),
            (publish_node_name + ".inputs:torque:x", "double"),
            (publish_node_name + ".inputs:torque:y", "double"),
            (publish_node_name + ".inputs:torque:z", "double"),
        ],
        graph_keys.SET_VALUES: [
            (script_node_name + ".inputs:FTJoint", joint),
            (script_node_name + ".inputs:FTFrame", frame),
            (script_node_name + ".inputs:flipMeasure", flip),
            (script_node_name + ".inputs:script", script_code),
            (publish_node_name + ".inputs:messageName", "Wrench"),
            (publish_node_name + ".inputs:messagePackage", "geometry_msgs"),
            (
                publish_node_name + ".inputs:topicName",
                topic_name,
            ),
        ],
        graph_keys.PROMOTE_ATTRIBUTES: [
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
                publish_node_name + ".inputs:force:x",
            ),
            (
                break_force_node_name + ".outputs:y",
                publish_node_name + ".inputs:force:y",
            ),
            (
                break_force_node_name + ".outputs:z",
                publish_node_name + ".inputs:force:z",
            ),
            (
                script_node_name + ".outputs:torque",
                break_torque_node_name + ".inputs:tuple",
            ),
            (
                break_torque_node_name + ".outputs:x",
                publish_node_name + ".inputs:torque:x",
            ),
            (
                break_torque_node_name + ".outputs:y",
                publish_node_name + ".inputs:torque:y",
            ),
            (
                break_torque_node_name + ".outputs:z",
                publish_node_name + ".inputs:torque:z",
            ),
        ],
    }


def create_ft_compounds(graph_keys, settings):
    if len(settings.FTs) == 0:
        return

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
                (compound_name + ".inputs:execIn", subcompound_name + ".inputs:execIn")
            )
            connections.append(
                (
                    compound_name + ".inputs:context",
                    subcompound_name + ".inputs:context",
                )
            )

    connections.append(("tick.outputs:tick", compound_name + ".inputs:execIn"))
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
                        (first_compound + ".inputs:execIn", "inputs:execIn"),
                        (first_compound + ".inputs:context", "inputs:context"),
                    ],
                },
            )
        ],
        graph_keys.CONNECT: connections,
    }


def create_graph(actions_list):
    (graph, nodes, prims, name_to_object_map) = og.Controller.edit(
        {"graph_path": s.graph_path, "evaluator_name": "execution"},
        merge_actions(actions_list),
    )
    if graph.is_valid():
        print("Graph created successfully!")
    else:
        print("FAILED TO CREATE GRAPH!")


#######################################################################################
########################## EDIT ONLY THE SETTINGS HERE BELOW ##########################
#######################################################################################

robot_path = "/World/ergoCubSN002/robot"
realsense_prefix = robot_path + "/realsense/sensors/RSD455"
s = Settings(
    graph_path="/World/ergoCubSN002/ros2_action_graph",
    articulation_root=robot_path + "/root_link",
    topic_prefix="/ergocub",
    imus=[
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
)

#######################################################################################
#######################################################################################
#######################################################################################

keys = og.Controller.Keys
create_graph(
    [
        add_basic_nodes(keys),
        add_ros2_clock_publisher(keys),
        add_ros2_joint_compound(keys, s),
        create_imu_compounds(keys, s),
        create_camera_compounds(keys, s),
        create_ft_compounds(keys, s),
    ]
)
