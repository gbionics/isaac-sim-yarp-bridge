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

# Expects one output:
# - outputs:bundle [bundle] The output bundle. Note: the name is this weird to be coherent with the programmatic creation of the node


import omni.graph.core as og


class ControlBoardState:
    control_modes: list[int]
    position_pid_gains: list[float]


def set_values_to_bundle(db):
    # For some reason, in script nodes, a bundle output
    # is not a classical output and needs to be retrieved
    # differently, passing via the node API.
    # Moreover, the name is edited with a outputs_ instead of outputs:

    bundle_attr = db.node.get_attribute("outputs_outputs:bundle")
    if not bundle_attr.is_valid():
        db.log_error("Failed to add get the output bundle")
        return False
    bundle = bundle_attr.get()

    state = db.per_instance_state
    # The control modes are of type int[]
    attr = bundle.create_attribute(
        name="control_modes",
        element_count=len(state.control_modes),
        type=og.Type(
            base_type=og.BaseDataType.INT,
            tuple_count=1,
            array_depth=1,
            role=og.AttributeRole.VECTOR,
        ),
    )
    if not attr.is_valid():
        db.log_error("Failed to add control_modes attribute to bundle")
        return False

    if not attr.set(state.control_modes):
        db.log_error("Failed to set control_modes attribute to bundle")
        return False

    # The gains are double[3] and in this case, it is considered as a single element
    # but with tuple count 3. Note that also array_depth changed from 1 to 0
    attr = bundle.create_attribute(
        name="position_pid_gains",
        element_count=1,
        type=og.Type(
            base_type=og.BaseDataType.DOUBLE,
            tuple_count=3,
            array_depth=0,
            role=og.AttributeRole.VECTOR,
        ),
    )
    if not attr.is_valid():
        db.log_error("Failed to add control_modes attribute to bundle")
        return False

    if not attr.set(state.position_pid_gains):
        db.log_error("Failed to set control_modes attribute to bundle")
        return False

    return True


def setup(db: og.Database):
    db.per_instance_state.control_modes = [1] * 23
    db.per_instance_state.position_pid_gains = [1.0, 2.0, 3.0]


def cleanup(db: og.Database):
    pass


def compute(db: og.Database):
    # TODO: understand how to do the check on whether setup has been called

    setup(db)
    set_values_to_bundle(db)

    return True


def internal_state():
    return ControlBoardState()
