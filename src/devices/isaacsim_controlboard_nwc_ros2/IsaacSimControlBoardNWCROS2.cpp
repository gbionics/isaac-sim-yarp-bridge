// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-2-Clause
#include "IsaacSimControlBoardNWCROS2.h"
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>

#include <limits>

YARP_DECLARE_LOG_COMPONENT(CB)
YARP_LOG_COMPONENT(CB, "yarp.device.IsaacSimControlBoardNWCROS2")

constexpr double rad2deg = 180.0 / M_PI;
constexpr double deg2rad = M_PI / 180.0;

//TODO: may of the things we get from services need to be converted from rad to deg and viceversa

static const std::string joint_names_tag = "joint_names";
static const std::string joint_types_tag = "joint_types";
static const std::string max_positions_tag = "max_positions";
static const std::string min_positions_tag = "min_positions";
static const std::string max_velocities_tag = "max_velocities";
static const std::string max_efforts_tag = "max_efforts";
static const std::string control_modes_tag = "control_modes";
static const std::string previous_control_modes_tag = "previous_control_modes";
static const std::string compliant_modes_tag = "compliant_modes";
static const std::string hf_messages_tag = "hf_messages";
static const std::string position_p_gains_tag = "position_p_gains";
static const std::string position_i_gains_tag = "position_i_gains";
static const std::string position_d_gains_tag = "position_d_gains";
static const std::string position_max_integral_tag = "position_max_integral";
static const std::string position_max_output_tag = "position_max_output";
static const std::string position_max_error_tag = "position_max_error";
static const std::string home_positions_tag = "home_positions";
static const std::string compliant_stiffness_tag = "compliant_stiffness";
static const std::string compliant_damping_tag = "compliant_damping";
static const std::string velocity_p_gains_tag = "velocity_p_gains";
static const std::string velocity_i_gains_tag = "velocity_i_gains";
static const std::string velocity_d_gains_tag = "velocity_d_gains";
static const std::string velocity_max_integral_tag = "velocity_max_integral";
static const std::string velocity_max_output_tag = "velocity_max_output";
static const std::string velocity_max_error_tag = "velocity_max_error";
static const std::string position_pid_references_tag = "position_pid_references";
static const std::string position_pid_errors_tag = "position_pid_errors";
static const std::string position_pid_outputs_tag = "position_pid_outputs";
static const std::string is_motion_done_tag = "is_motion_done";
static const std::string position_pid_enabled_tag = "position_pid_enabled";
static const std::string position_pid_to_reset_tag = "position_pid_to_reset";
static const std::string position_pid_to_stop_tag = "position_pid_to_stop";
static const std::string velocity_pid_references_tag = "velocity_pid_references";
static const std::string velocity_pid_errors_tag = "velocity_pid_errors";
static const std::string velocity_pid_outputs_tag = "velocity_pid_outputs";
static const std::string velocity_pid_enabled_tag = "velocity_pid_enabled";
static const std::string velocity_pid_to_reset_tag = "velocity_pid_to_reset";
static const std::string torque_pid_references_tag = "torque_pid_references";
static const std::string torque_pid_errors_tag = "torque_pid_errors";
static const std::string torque_pid_outputs_tag = "torque_pid_outputs";
static const std::string torque_pid_enabled_tag = "torque_pid_enabled";
static const std::string current_pid_references_tag = "current_pid_references";
static const std::string current_pid_errors_tag = "current_pid_errors";
static const std::string current_pid_outputs_tag = "current_pid_outputs";
static const std::string current_pid_enabled_tag = "current_pid_enabled";
static const std::string gearbox_ratios_tag = "gearbox_ratios";
static const std::string motor_torque_constants_tag = "motor_torque_constants";
static const std::string motor_current_noise_variance_tag = "motor_current_noise_variance";
static const std::string motor_spring_stiffness_tag = "motor_spring_stiffness";
static const std::string motor_max_currents_tag = "motor_max_currents";

using Type = rcl_interfaces::msg::ParameterType;

yarp::dev::IsaacSimControlBoardNWCROS2::~IsaacSimControlBoardNWCROS2()
{
    close();
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::open(yarp::os::Searchable& config)
{
    {
        std::lock_guard<std::mutex> lock(m_mutex);

        std::string errorPrefix = "[open] ";
        if (!m_paramsParser.parseParams(config))
        {
            yCError(CB) << errorPrefix << "Error while parsing configuration parameters.";
            return false;
        }

        if (!rclcpp::ok())
        {
            rclcpp::init(0, nullptr);
        }

        m_node = std::make_shared<CBNode>(m_paramsParser.m_node_name,
                                          m_paramsParser.m_joint_state_topic_name,
                                          m_paramsParser.m_motor_state_topic_name,
                                          m_paramsParser.m_joint_references_topic_name,
                                          m_paramsParser.m_get_parameters_service_name,
                                          m_paramsParser.m_set_parameters_service_name,
                                          m_paramsParser.m_service_request_timeout,
                                          this);
        m_executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
        m_executor->add_node(m_node);
        m_executorThread = std::thread([this]() { m_executor->spin(); });
    }
    return setup();
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::close()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_node)
    {
        m_executor->cancel();
        m_executorThread.join();
        m_node.reset();
    }
    m_ready = false;
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setPid(const yarp::dev::PidControlTypeEnum& pidtype, int j, const yarp::dev::Pid& p)
{
    std::string errorPrefix = "[setPid] ";

    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Services are not ready.";
        return false;
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    std::vector<rcl_interfaces::msg::Parameter> params;
    if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION)
    {
        rcl_interfaces::msg::Parameter p_param;
        std::string suffix_tag = "[" + std::to_string(j) + "]";
        p_param.name = position_p_gains_tag + suffix_tag;
        p_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        p_param.value.double_value = p.kp;
        params.push_back(p_param);
        rcl_interfaces::msg::Parameter i_param;
        i_param.name = position_i_gains_tag + suffix_tag;
        i_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        i_param.value.double_value = p.ki;
        params.push_back(i_param);
        rcl_interfaces::msg::Parameter d_param;
        d_param.name = position_d_gains_tag + suffix_tag;
        d_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        d_param.value.double_value = p.kd;
        params.push_back(d_param);
        rcl_interfaces::msg::Parameter max_integral_param;
        max_integral_param.name = position_max_integral_tag + suffix_tag;
        max_integral_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        max_integral_param.value.double_value = p.max_int;
        params.push_back(max_integral_param);
        rcl_interfaces::msg::Parameter max_output_param;
        max_output_param.name = position_max_output_tag + suffix_tag;
        max_output_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        max_output_param.value.double_value = p.max_output;
        params.push_back(max_output_param);
        auto results = m_node->setParameters(params);
        if (results.size() != params.size())
        {
            yCError(CB) << errorPrefix << "Error while setting position pid (" << j << ") parameters";
            return false;
        }
        bool success = true;
        for (size_t i = 0; i < results.size(); ++i)
        {
            if (!results[i].successful)
            {
                yCError(CB) << errorPrefix << "Error while setting position pid parameter"
                                           << params[i].name + ":" << results[i].reason;
                success = false;
            }
        }
        return success;
    }
    if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY)
    {
        rcl_interfaces::msg::Parameter p_param;
        std::string suffix_tag = "[" + std::to_string(j) + "]";
        p_param.name = velocity_p_gains_tag + suffix_tag;
        p_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        p_param.value.double_value = p.kp;
        params.push_back(p_param);
        rcl_interfaces::msg::Parameter i_param;
        i_param.name = velocity_i_gains_tag + suffix_tag;
        i_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        i_param.value.double_value = p.ki;
        params.push_back(i_param);
        rcl_interfaces::msg::Parameter d_param;
        d_param.name = velocity_d_gains_tag + suffix_tag;
        d_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        d_param.value.double_value = p.kd;
        params.push_back(d_param);
        rcl_interfaces::msg::Parameter max_integral_param;
        max_integral_param.name = velocity_max_integral_tag + suffix_tag;
        max_integral_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        max_integral_param.value.double_value = p.max_int;
        params.push_back(max_integral_param);
        rcl_interfaces::msg::Parameter max_output_param;
        max_output_param.name = velocity_max_output_tag + suffix_tag;
        max_output_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        max_output_param.value.double_value = p.max_output;
        params.push_back(max_output_param);
        auto results = m_node->setParameters(params);
        if (results.size() != params.size())
        {
            yCError(CB) << errorPrefix << "Error while setting velocity pid (" << j << ")  parameters.";
            return false;
        }
        bool success = true;
        for (size_t i = 0; i < results.size(); ++i)
        {
            if (!results[i].successful)
            {
                yCError(CB) << errorPrefix << "Error while setting velocity pid parameter"
                    << params[i].name + ":" << results[i].reason;
                success = false;
            }
        }
        return success;
    }
    if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_TORQUE ||
             pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_CURRENT)
    {
        yCWarning(CB) << errorPrefix << "Setting torque/current pid does not have any effect on Isaac Sim.";
        return true;
    }

    yCError(CB) << errorPrefix << "Unknown pid type.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setPids(const yarp::dev::PidControlTypeEnum& pidtype, const yarp::dev::Pid* ps)
{
    std::string errorPrefix = "[setPids] ";

    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Services are not ready.";
        return false;
    }

    std::lock_guard<std::mutex> lock(m_mutex);

    size_t numberOfJoints = m_jointNames.size();


    std::vector<rcl_interfaces::msg::Parameter> params;

    if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION)
    {
        rcl_interfaces::msg::Parameter p_param;
        p_param.name = position_p_gains_tag;
        p_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
        p_param.value.double_array_value.resize(numberOfJoints);

        rcl_interfaces::msg::Parameter i_param;
        i_param.name = position_i_gains_tag;
        i_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
        i_param.value.double_array_value.resize(numberOfJoints);

        rcl_interfaces::msg::Parameter d_param;
        d_param.name = position_d_gains_tag;
        d_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
        d_param.value.double_array_value.resize(numberOfJoints);

        rcl_interfaces::msg::Parameter max_integral_param;
        max_integral_param.name = position_max_integral_tag;
        max_integral_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
        max_integral_param.value.double_array_value.resize(numberOfJoints);

        rcl_interfaces::msg::Parameter max_output_param;
        max_output_param.name = position_max_output_tag;
        max_output_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
        max_output_param.value.double_array_value.resize(numberOfJoints);


        for (size_t j = 0; j < numberOfJoints; j++)
        {
            p_param.value.double_array_value[j] = ps[j].kp;
            i_param.value.double_array_value[j] = ps[j].ki;
            d_param.value.double_array_value[j] = ps[j].kd;
            max_integral_param.value.double_array_value[j] = ps[j].max_int;
            max_output_param.value.double_array_value[j] = ps[j].max_output;
        }

        params.push_back(p_param);
        params.push_back(i_param);
        params.push_back(d_param);
        params.push_back(max_integral_param);
        params.push_back(max_output_param);

        auto results = m_node->setParameters(params);
        if (results.size() != params.size())
        {
            yCError(CB) << errorPrefix << "Error while setting position pid parameters.";
            return false;
        }
        bool success = true;
        for (size_t i = 0; i < results.size(); ++i)
        {
            if (!results[i].successful)
            {
                yCError(CB) << errorPrefix << "Error while setting position pid parameter"
                    << params[i].name + ":" << results[i].reason;
                success = false;
            }
        }
        return success;
    }
    if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY)
    {
        rcl_interfaces::msg::Parameter p_param;
        p_param.name = velocity_p_gains_tag;
        p_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
        p_param.value.double_array_value.resize(numberOfJoints);

        rcl_interfaces::msg::Parameter i_param;
        i_param.name = velocity_i_gains_tag;
        i_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
        i_param.value.double_array_value.resize(numberOfJoints);

        rcl_interfaces::msg::Parameter d_param;
        d_param.name = velocity_d_gains_tag;
        d_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
        d_param.value.double_array_value.resize(numberOfJoints);

        rcl_interfaces::msg::Parameter max_integral_param;
        max_integral_param.name = velocity_max_integral_tag;
        max_integral_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
        max_integral_param.value.double_array_value.resize(numberOfJoints);

        rcl_interfaces::msg::Parameter max_output_param;
        max_output_param.name = velocity_max_output_tag;
        max_output_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
        max_output_param.value.double_array_value.resize(numberOfJoints);

        for (size_t j = 0; j < numberOfJoints; j++)
        {
            p_param.value.double_array_value[j] = ps[j].kp;
            i_param.value.double_array_value[j] = ps[j].ki;
            d_param.value.double_array_value[j] = ps[j].kd;
            max_integral_param.value.double_array_value[j] = ps[j].max_int;
            max_output_param.value.double_array_value[j] = ps[j].max_output;
        }

        params.push_back(p_param);
        params.push_back(i_param);
        params.push_back(d_param);
        params.push_back(max_integral_param);
        params.push_back(max_output_param);

        auto results = m_node->setParameters(params);
        if (results.size() != params.size())
        {
            yCError(CB) << errorPrefix << "Error while setting velocity pid parameters.";
            return false;
        }
        bool failed = false;
        for (size_t i = 0; i < results.size(); ++i)
        {
            if (!results[i].successful)
            {
                yCError(CB) << errorPrefix << "Error while setting velocity pid parameter"
                    << params[i].name + ":" << results[i].reason;
                failed = true;
            }
        }
        return failed;
    }
    if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_TORQUE ||
        pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_CURRENT)
    {
        yCWarning(CB) << errorPrefix << "Setting torque/current pid does not have any effect on Isaac Sim.";
        return true;
    }

    yCError(CB) << errorPrefix << "Unknown pid type.";

    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setPidReference(const yarp::dev::PidControlTypeEnum& pidtype, int j, double ref)
{
    std::string errorPrefix = "[setPidReference] ";
    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Not ready to send references.";
        return false;
    }
    std::lock_guard<std::mutex> lock(m_mutex);

    {
        std::lock_guard<std::mutex> lock_reference(m_jointReferences.mutex);

        if (j < 0 || static_cast<size_t>(j) >= m_jointReferences.name.size())
        {
            yCError(CB) << errorPrefix << "Joint index out of range. Got " << j << ", expected [0," << m_jointReferences.name.size() - 1 << "]";
            return false;
        }

        if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION)
        {
            m_jointReferences.position[j] = ref;
            if (m_compliant[j])
            {
                m_jointReferences.effort[j] = m_compliantOffset[j];
            }
        }
        else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY)
        {
            m_jointReferences.velocity[j] = ref;
        }
        else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_TORQUE ||
            pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_CURRENT)
        {
            m_jointReferences.effort[j] = ref;
        }
        m_jointReferences.valid = true;
    }

    m_node->publishReferences(m_jointReferences);
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setPidReferences(const yarp::dev::PidControlTypeEnum& pidtype, const double* refs)
{
    std::string errorPrefix = "[setPidReferences] ";
    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Not ready to send references.";
        return false;
    }
    std::lock_guard<std::mutex> lock(m_mutex);
    {
        std::lock_guard<std::mutex> lock_reference(m_jointReferences.mutex);
        size_t numberOfJoints = m_jointReferences.name.size();
        if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION)
        {
            for (size_t j = 0; j < numberOfJoints; j++)
            {
                m_jointReferences.position[j] = refs[j];
                if (m_compliant[j])
                {
                    m_jointReferences.effort[j] = m_compliantOffset[j];
                }
            }
        }
        else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY)
        {
            for (size_t j = 0; j < numberOfJoints; j++)
            {
                m_jointReferences.velocity[j] = refs[j];
            }
        }
        else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_TORQUE ||
            pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_CURRENT)
        {
            for (size_t j = 0; j < numberOfJoints; j++)
            {
                m_jointReferences.effort[j] = refs[j];
            }
        }
        m_jointReferences.valid = true;
    }
    m_node->publishReferences(m_jointReferences);
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setPidErrorLimit(const yarp::dev::PidControlTypeEnum& pidtype, int j, double limit)
{
    std::string errorPrefix = "[setPidErrorLimit] ";

    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Services are not ready.";
        return false;
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    std::vector<rcl_interfaces::msg::Parameter> params;
    if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION)
    {
        rcl_interfaces::msg::Parameter max_error_param;
        std::string suffix_tag = "[" + std::to_string(j) + "]";
        max_error_param.name = position_max_error_tag + suffix_tag;
        max_error_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        max_error_param.value.double_value = limit;
        params.push_back(max_error_param);
        auto results = m_node->setParameters(params);
        if (results.size() != params.size())
        {
            yCError(CB) << errorPrefix << "Error while setting position pid (" << j << ") error limit.";
            return false;
        }
        bool success = true;
        for (const auto& res : results)
        {
            if (!res.successful)
            {
                yCError(CB) << errorPrefix << "Error while setting position pid (" << j << ") error limit parameter:" << res.reason;
                success = false;
            }
        }
        return success;
    }
    if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY)
    {
        rcl_interfaces::msg::Parameter max_error_param;
        std::string suffix_tag = "[" + std::to_string(j) + "]";
        max_error_param.name = velocity_max_error_tag + suffix_tag;
        max_error_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        max_error_param.value.double_value = limit;
        params.push_back(max_error_param);
        auto results = m_node->setParameters(params);
        if (results.size() != params.size())
        {
            yCError(CB) << errorPrefix << "Error while setting velocity pid (" << j << ") error limit.";
            return false;
        }
        bool success = true;
        for (const auto& res : results)
        {
            if (!res.successful)
            {
                yCError(CB) << errorPrefix << "Error while setting velocity pid (" << j << ") error limit parameter:" << res.reason;
                success = false;
            }
        }
        return success;
    }
    if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_TORQUE ||
        pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_CURRENT)
    {
        yCWarning(CB) << errorPrefix << "Setting torque/current pid does not have any effect on Isaac Sim.";
        return true;
    }
    yCError(CB) << errorPrefix << "Unknown pid type.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setPidErrorLimits(const yarp::dev::PidControlTypeEnum& pidtype, const double* limits)
{
    std::string errorPrefix = "[setPidErrorLimits] ";
    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Services are not ready.";
        return false;
    }

    std::lock_guard<std::mutex> lock(m_mutex);

    size_t numberOfJoints = m_jointNames.size();
    std::vector<rcl_interfaces::msg::Parameter> params;
    if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION)
    {
        rcl_interfaces::msg::Parameter max_error_param;
        max_error_param.name = position_max_error_tag;
        max_error_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
        max_error_param.value.double_array_value.resize(numberOfJoints);
        for (size_t j = 0; j < numberOfJoints; j++)
        {
            max_error_param.value.double_array_value[j] = limits[j];
        }
        params.push_back(max_error_param);
        auto results = m_node->setParameters(params);
        if (results.size() != params.size())
        {
            yCError(CB) << errorPrefix << "Error while setting position pid error limits.";
            return false;
        }
        bool success = true;
        for (const auto& res : results)
        {
            if (!res.successful)
            {
                yCError(CB) << errorPrefix << "Error while setting position pid error limit parameter:" << res.reason;
                success = false;
            }
        }
        return success;
    }
    if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY)
    {
        rcl_interfaces::msg::Parameter max_error_param;
        max_error_param.name = velocity_max_error_tag;
        max_error_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
        max_error_param.value.double_array_value.resize(numberOfJoints);
        for (size_t j = 0; j < numberOfJoints; j++)
        {
            max_error_param.value.double_array_value[j] = limits[j];
        }
        params.push_back(max_error_param);
        auto results = m_node->setParameters(params);
        if (results.size() != params.size())
        {
            yCError(CB) << errorPrefix << "Error while setting velocity pid error limits.";
            return false;
        }
        bool success = true;
        for (const auto& res : results)
        {
            if (!res.successful)
            {
                yCError(CB) << errorPrefix << "Error while setting velocity pid error limit parameter:" << res.reason;
                success = false;
            }
        }
        return success;
    }
    if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_TORQUE ||
        pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_CURRENT)
    {
        yCWarning(CB) << errorPrefix << "Setting torque/current pid does not have any effect on Isaac Sim.";
        return true;
    }
    yCError(CB) << errorPrefix << "Unknown pid type.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getPidError(const yarp::dev::PidControlTypeEnum& pidtype, int j, double* err)
{
    std::string errorPrefix = "[getPidError] ";

    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Services are not ready.";
        return false;
    }

    std::lock_guard<std::mutex> lock(m_mutex);

    std::string suffix_tag = "[" + std::to_string(j) + "]";
    std::string parameter_name;
    std::string pid_type_str;

    if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION)
    {
        parameter_name = position_pid_errors_tag + suffix_tag;
        pid_type_str = "position";
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY)
    {
        parameter_name = velocity_pid_errors_tag + suffix_tag;
        pid_type_str = "velocity";
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_TORQUE)
    {
        parameter_name = torque_pid_errors_tag + suffix_tag;
        pid_type_str = "torque";
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_CURRENT)
    {
        parameter_name = current_pid_errors_tag + suffix_tag;
        pid_type_str = "current";
    }
    else
    {
        yCError(CB) << errorPrefix << "Unknown pid type.";
        return false;
    }

    auto result = m_node->getParameters({ {parameter_name, Type::PARAMETER_DOUBLE} });
    if (result.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting pid error for type " << pid_type_str << "joint" << j;
        return false;
    }

    *err = result[0].double_value;
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getPidErrors(const yarp::dev::PidControlTypeEnum& pidtype, double* errs)
{
    std::string errorPrefix = "[getPidErrors] ";
    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Services are not ready.";
        return false;
    }

    std::lock_guard<std::mutex> lock(m_mutex);

    std::string parameter_name;
    std::string pid_type_str;

    if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION)
    {
        parameter_name = position_pid_errors_tag;
        pid_type_str = "position";
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY)
    {
        parameter_name = velocity_pid_errors_tag;
        pid_type_str = "velocity";
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_TORQUE)
    {
        parameter_name = torque_pid_errors_tag;
        pid_type_str = "torque";
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_CURRENT)
    {
        parameter_name = current_pid_errors_tag;
        pid_type_str = "current";
    }
    else
    {
        yCError(CB) << errorPrefix << "Unknown pid type.";
        return false;
    }

    auto result = m_node->getParameters({ {parameter_name, Type::PARAMETER_DOUBLE_ARRAY} });
    if (result.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting pid errors for type " << pid_type_str << ".";
        return false;
    }

    std::copy(result[0].double_array_value.begin(), result[0].double_array_value.end(), errs);
    return true;
}
bool yarp::dev::IsaacSimControlBoardNWCROS2::getPidOutput(const yarp::dev::PidControlTypeEnum& pidtype, int j, double* out)
{
    std::string errorPrefix = "[getPidOutput] ";

    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Services are not ready.";
        return false;
    }
    std::lock_guard<std::mutex> lock(m_mutex);

    std::string suffix_tag = "[" + std::to_string(j) + "]";
    std::string parameter_name;
    std::string pid_type_str;

    if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION)
    {
        parameter_name = position_pid_outputs_tag + suffix_tag;
        pid_type_str = "position";
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY)
    {
        parameter_name = velocity_pid_outputs_tag + suffix_tag;
        pid_type_str = "velocity";
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_TORQUE)
    {
        parameter_name = torque_pid_outputs_tag + suffix_tag;
        pid_type_str = "torque";
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_CURRENT)
    {
        parameter_name = current_pid_outputs_tag + suffix_tag;
        pid_type_str = "current";
    }
    else
    {
        yCError(CB) << errorPrefix << "Unknown pid type.";
        return false;
    }

    auto result = m_node->getParameters({ {parameter_name, Type::PARAMETER_DOUBLE} });
    if (result.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting pid output for type " << pid_type_str << "joint" << j;
        return false;
    }

    *out = result[0].double_value;
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getPidOutputs(const yarp::dev::PidControlTypeEnum& pidtype, double* outs)
{
    std::string errorPrefix = "[getPidOutputs] ";
    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Services are not ready.";
        return false;
    }

    std::lock_guard<std::mutex> lock(m_mutex);

    std::string parameter_name;
    std::string pid_type_str;

    if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION)
    {
        parameter_name = position_pid_outputs_tag;
        pid_type_str = "position";
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY)
    {
        parameter_name = velocity_pid_outputs_tag;
        pid_type_str = "velocity";
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_TORQUE)
    {
        parameter_name = torque_pid_outputs_tag;
        pid_type_str = "torque";
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_CURRENT)
    {
        parameter_name = current_pid_outputs_tag;
        pid_type_str = "current";
    }
    else
    {
        yCError(CB) << errorPrefix << "Unknown pid type.";
        return false;
    }

    auto result = m_node->getParameters({ {parameter_name, Type::PARAMETER_DOUBLE_ARRAY} });
    if (result.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting pid outputs for type " << pid_type_str << ".";
        return false;
    }

    std::copy(result[0].double_array_value.begin(), result[0].double_array_value.end(), outs);
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setPidOffset(const yarp::dev::PidControlTypeEnum& pidtype, int j, double v)
{
    yCError(CB) << "[setPidOffset] Setting PID offsets is not implemented.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getPid(const yarp::dev::PidControlTypeEnum& pidtype, int j, yarp::dev::Pid* p)
{
    std::string errorPrefix = "[getPid] ";

    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Services are not ready.";
        return false;
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    std::string suffix_tag = "[" + std::to_string(j) + "]";
    CBNode::Parameters parameters;
    std::string pid_type_str;

    if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION)
    {
        parameters = { {position_p_gains_tag + suffix_tag, Type::PARAMETER_DOUBLE},
                       {position_i_gains_tag + suffix_tag, Type::PARAMETER_DOUBLE},
                       {position_d_gains_tag + suffix_tag, Type::PARAMETER_DOUBLE},
                       {position_max_integral_tag + suffix_tag, Type::PARAMETER_DOUBLE},
                       {position_max_output_tag + suffix_tag, Type::PARAMETER_DOUBLE} };
        pid_type_str = "position";
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY)
    {
        parameters = { {velocity_p_gains_tag + suffix_tag, Type::PARAMETER_DOUBLE},
                       {velocity_i_gains_tag + suffix_tag, Type::PARAMETER_DOUBLE},
                       {velocity_d_gains_tag + suffix_tag, Type::PARAMETER_DOUBLE},
                       {velocity_max_integral_tag + suffix_tag, Type::PARAMETER_DOUBLE},
                       {velocity_max_output_tag + suffix_tag, Type::PARAMETER_DOUBLE} };
        pid_type_str = "velocity";
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_TORQUE)
    {
        yCWarning(CB) << errorPrefix << "There is no torque PID in IsaacSim";
        return true;
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_CURRENT)
    {
        yCWarning(CB) << errorPrefix << "There is no current PID in IsaacSim.";
        return true;
    }
    else
    {
        yCError(CB) << errorPrefix << "Unknown pid type.";
        return false;
    }

    auto results = m_node->getParameters(parameters);
    if (results.size() != parameters.size())
    {
        yCError(CB) << errorPrefix << "Error while getting pid parameters for type " << pid_type_str  << "joint" << j;
        return false;
    }

    p->kp = results[0].double_value;
    p->ki = results[1].double_value;
    p->kd = results[2].double_value;
    p->max_int = results[3].double_value;
    p->max_output = results[4].double_value;
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getPids(const yarp::dev::PidControlTypeEnum& pidtype, yarp::dev::Pid* pids)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getPids] ";

    CBNode::Parameters parameters;
    std::string pid_type_str;

    if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION)
    {
        parameters = { {position_p_gains_tag, Type::PARAMETER_DOUBLE_ARRAY},
                       {position_i_gains_tag, Type::PARAMETER_DOUBLE_ARRAY},
                       {position_d_gains_tag, Type::PARAMETER_DOUBLE_ARRAY},
                       {position_max_integral_tag, Type::PARAMETER_DOUBLE_ARRAY},
                       {position_max_output_tag,Type::PARAMETER_DOUBLE_ARRAY} };
        pid_type_str = "position";
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY)
    {
        parameters = { {velocity_p_gains_tag, Type::PARAMETER_DOUBLE_ARRAY},
                       {velocity_i_gains_tag,Type::PARAMETER_DOUBLE_ARRAY},
                       {velocity_d_gains_tag, Type::PARAMETER_DOUBLE_ARRAY},
                       {velocity_max_integral_tag,Type::PARAMETER_DOUBLE_ARRAY},
                       {velocity_max_output_tag, Type::PARAMETER_DOUBLE_ARRAY} };
        pid_type_str = "velocity";
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_TORQUE)
    {
        yCWarning(CB) << errorPrefix << "There is no torque PID in IsaacSim.";
        return true;
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_CURRENT)
    {
        yCWarning(CB) << errorPrefix << "There is no current PID in IsaacSim.";
        return true;
    }
    else
    {
        yCError(CB) << errorPrefix << "Unknown pid type.";
        return false;
    }

    auto results = m_node->getParameters(parameters);

    if (results.size() != parameters.size())
    {
        yCError(CB) << errorPrefix << "Error while getting pid parameters for type " << pid_type_str << ".";
        return false;
    }

    size_t numberOfJoints = results[0].double_array_value.size();
    for (size_t j = 0; j < numberOfJoints; j++)
    {
        pids[j].kp = results[0].double_array_value[j];
        pids[j].ki = results[1].double_array_value[j];
        pids[j].kd = results[2].double_array_value[j];
        pids[j].max_int = results[3].double_array_value[j];
        pids[j].max_output = results[4].double_array_value[j];
    }
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getPidReference(const yarp::dev::PidControlTypeEnum& pidtype, int j, double* ref)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getPidReference] ";
    std::string suffix_tag = "[" + std::to_string(j) + "]";
    std::string parameter_name;
    std::string pid_type_str;

    if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION)
    {
        parameter_name = position_pid_references_tag + suffix_tag;
        pid_type_str = "position";
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY)
    {
        parameter_name = velocity_pid_references_tag + suffix_tag;
        pid_type_str = "velocity";
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_TORQUE)
    {
        parameter_name = torque_pid_references_tag + suffix_tag;
        pid_type_str = "torque";
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_CURRENT)
    {
        parameter_name = current_pid_references_tag + suffix_tag;
        pid_type_str = "current";
    }
    else
    {
        yCError(CB) << errorPrefix << "Unknown pid type.";
        return false;
    }

    auto result = m_node->getParameters({ {parameter_name, Type::PARAMETER_DOUBLE} });

    if (result.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting pid reference for type " << pid_type_str << "joint" << j;
        return false;
    }

    *ref = result[0].double_value;

    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getPidReferences(const yarp::dev::PidControlTypeEnum& pidtype, double* refs)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getPidReferences] ";
    std::string parameter_name;
    std::string pid_type_str;

    if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION)
    {
        parameter_name = position_pid_references_tag;
        pid_type_str = "position";
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY)
    {
        parameter_name = velocity_pid_references_tag;
        pid_type_str = "velocity";
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_TORQUE)
    {
        parameter_name = torque_pid_references_tag;
        pid_type_str = "torque";
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_CURRENT)
    {
        parameter_name = current_pid_references_tag;
        pid_type_str = "current";
    }
    else
    {
        yCError(CB) << errorPrefix << "Unknown pid type.";
        return false;
    }

    auto result = m_node->getParameters({ {parameter_name, Type::PARAMETER_DOUBLE_ARRAY} });

    if (result.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting pid references for type " << pid_type_str << ".";
        return false;
    }

    std::copy(result[0].double_array_value.begin(), result[0].double_array_value.end(), refs);
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getPidErrorLimit(const yarp::dev::PidControlTypeEnum& pidtype, int j, double* limit)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getPidErrorLimit] ";
    std::string suffix_tag = "[" + std::to_string(j) + "]";
    std::string parameter_name;
    std::string pid_type_str;

    if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION)
    {
        parameter_name = position_max_error_tag + suffix_tag;
        pid_type_str = "position";
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY)
    {
        parameter_name = velocity_max_error_tag + suffix_tag;
        pid_type_str = "velocity";
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_TORQUE)
    {
        yCWarning(CB) << errorPrefix << "There is no torque PID in IsaacSim.";
        return true;
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_CURRENT)
    {
        yCWarning(CB) << errorPrefix << "There is no current PID in IsaacSim.";
        return true;
    }
    else
    {
        yCError(CB) << errorPrefix << "Unknown pid type.";
        return false;
    }

    auto result = m_node->getParameters({ {parameter_name, Type::PARAMETER_DOUBLE} });
    if (result.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting pid error limit for type " << pid_type_str << "joint" << j;
        return false;
    }

    *limit = result[0].double_value;
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getPidErrorLimits(const yarp::dev::PidControlTypeEnum& pidtype, double* limits)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getPidErrorLimits] ";
    std::string parameter_name;
    std::string pid_type_str;

    if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION)
    {
        parameter_name = position_max_error_tag;
        pid_type_str = "position";
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY)
    {
        parameter_name = velocity_max_error_tag;
        pid_type_str = "velocity";
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_TORQUE)
    {
        yCWarning(CB) << errorPrefix << "There is no torque PID in IsaacSim.";
        return true;
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_CURRENT)
    {
        yCWarning(CB) << errorPrefix << "There is no current PID in IsaacSim.";
        return true;
    }
    else
    {
        yCError(CB) << errorPrefix << "Unknown pid type.";
        return false;
    }

    auto result = m_node->getParameters({ {parameter_name, Type::PARAMETER_DOUBLE_ARRAY} });
    if (result.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting pid error limits for type " << pid_type_str << ".";
        return false;
    }

    std::copy(result[0].double_array_value.begin(), result[0].double_array_value.end(), limits);
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::resetPid(const yarp::dev::PidControlTypeEnum& pidtype, int j)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[resetPid] ";
    std::string suffix_tag = "[" + std::to_string(j) + "]";
    std::string pid_type_str;
    std::vector<rcl_interfaces::msg::Parameter> params;
    rcl_interfaces::msg::Parameter reset_param;

    if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION)
    {
        reset_param.name = position_pid_to_reset_tag + suffix_tag;
        pid_type_str = "position";
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY)
    {
        reset_param.name = velocity_pid_to_reset_tag + suffix_tag;
        pid_type_str = "velocity";
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_TORQUE)
    {
        yCWarning(CB) << errorPrefix << "There is no torque PID in IsaacSim.";
        return true;
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_CURRENT)
    {
        yCWarning(CB) << errorPrefix << "There is no current PID in IsaacSim.";
        return true;
    }
    else
    {
        yCError(CB) << errorPrefix << "Unknown pid type.";
        return false;
    }

    reset_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    reset_param.value.bool_value = true;
    params.push_back(reset_param);
    auto results = m_node->setParameters(params);

    if (results.size() != params.size())
    {
        yCError(CB) << errorPrefix << "Error while resetting pid for type" << pid_type_str << "joint" << j;
        return false;
    }

    bool success = true;
    for (const auto& res : results)
    {
        if (!res.successful)
        {
            yCError(CB) << errorPrefix << "Error while resetting pid of type"
                                       << pid_type_str << "joint" << j << ":" << res.reason;
            success = false;
        }
    }

    return success;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::disablePid(const yarp::dev::PidControlTypeEnum& pidtype, int j)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[disablePid] ";
    std::string suffix_tag = "[" + std::to_string(j) + "]";
    std::string pid_type_str;
    std::vector<rcl_interfaces::msg::Parameter> params;
    rcl_interfaces::msg::Parameter disable_param;

    if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION)
    {
        disable_param.name = position_pid_enabled_tag + suffix_tag;
        pid_type_str = "position";
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY)
    {
        rcl_interfaces::msg::Parameter disable_param;
        disable_param.name = velocity_pid_enabled_tag + suffix_tag;
        pid_type_str = "velocity";
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_TORQUE)
    {
        rcl_interfaces::msg::Parameter disable_param;
        disable_param.name = torque_pid_enabled_tag + suffix_tag;
        pid_type_str = "torque";
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_CURRENT)
    {
        rcl_interfaces::msg::Parameter disable_param;
        disable_param.name = current_pid_enabled_tag + suffix_tag;
        pid_type_str = "current";
    }
    else
    {
        yCError(CB) << errorPrefix << "Unknown pid type.";
        return false;
    }

    disable_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    disable_param.value.bool_value = false;
    params.push_back(disable_param);


    auto results = m_node->setParameters(params);
    if (results.size() != params.size())
    {
        yCError(CB) << errorPrefix << "Error while disabling pid for type" << pid_type_str << "joint" << j;
        return false;
    }
    bool success = true;
    for (const auto& res : results)
    {
        if (!res.successful)
        {
            yCError(CB) << errorPrefix << "Error while disabling pid of type"
                << pid_type_str << "joint" << j << ":" << res.reason;
            success = false;
        }
    }
    return success;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::enablePid(const yarp::dev::PidControlTypeEnum& pidtype, int j)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[enablePid] ";
    std::string suffix_tag = "[" + std::to_string(j) + "]";
    std::string pid_type_str;
    std::vector<rcl_interfaces::msg::Parameter> params;
    rcl_interfaces::msg::Parameter enable_param;

    if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION)
    {
        enable_param.name = position_pid_enabled_tag + suffix_tag;
        pid_type_str = "position";
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY)
    {
        enable_param.name = velocity_pid_enabled_tag + suffix_tag;
        pid_type_str = "velocity";
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_TORQUE)
    {
        enable_param.name = torque_pid_enabled_tag + suffix_tag;
        pid_type_str = "torque";
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_CURRENT)
    {
        enable_param.name = current_pid_enabled_tag + suffix_tag;
        pid_type_str = "current";
    }
    else
    {
        yCError(CB) << errorPrefix << "Unknown pid type.";
        return false;
    }

    enable_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    enable_param.value.bool_value = true;
    params.push_back(enable_param);
    auto results = m_node->setParameters(params);
    if (results.size() != params.size())
    {
        yCError(CB) << errorPrefix << "Error while enabling pid for type" << pid_type_str << "joint" << j;
        return false;
    }
    bool success = true;
    for (const auto& res : results)
    {
        if (!res.successful)
        {
            yCError(CB) << errorPrefix << "Error while enabling pid of type"
                << pid_type_str << "joint" << j << ":" << res.reason;
            success = false;
        }
    }
    return success;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::isPidEnabled(const yarp::dev::PidControlTypeEnum& pidtype, int j, bool* enabled)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[isPidEnabled] ";
    std::string suffix_tag = "[" + std::to_string(j) + "]";
    std::string parameter_name;
    std::string pid_type_str;

    if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION)
    {
        parameter_name = position_pid_enabled_tag + suffix_tag;
        pid_type_str = "position";
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY)
    {
        parameter_name = velocity_pid_enabled_tag + suffix_tag;
        pid_type_str = "velocity";
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_TORQUE)
    {
        parameter_name = torque_pid_enabled_tag + suffix_tag;
        pid_type_str = "torque";
    }
    else if (pidtype == yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_CURRENT)
    {
        parameter_name = current_pid_enabled_tag + suffix_tag;
        pid_type_str = "current";
    }
    else
    {
        yCError(CB) << errorPrefix << "Unknown pid type.";
        return false;
    }

    auto result = m_node->getParameters({ {parameter_name, Type::PARAMETER_BOOL} });
    if (result.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting if pid is enabled for type " << pid_type_str << "joint" << j;
        return false;
    }

    *enabled = result[0].bool_value;
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getAxes(int* ax)
{
    std::string errorPrefix = "[getAxes] ";
    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Services are not ready.";
        return false;
    }

    std::lock_guard<std::mutex> lock(m_mutex);

    *ax = static_cast<int>(m_jointNames.size());
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::positionMove(int j, double ref)
{
    std::string errorPrefix = "[positionMove] ";
    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Not ready to send references";
        return false;
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    {
        std::lock_guard<std::mutex> lock_reference(m_jointReferences.mutex);

        if (j < 0 || static_cast<size_t>(j) >= m_jointReferences.name.size())
        {
            yCError(CB) << errorPrefix << "Joint index out of range. Got " << j << ", expected [0," << m_jointReferences.name.size() - 1 << "]";
            return false;
        }

        m_jointReferences.position[j] = ref;

        if (m_compliant[j])
        {
            m_jointReferences.effort[j] = m_compliantOffset[j];
        }

        m_jointReferences.valid = true;
    }

    m_node->publishReferences(m_jointReferences);
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::positionMove(const double* refs)
{
    std::string errorPrefix = "[positionMove] ";
    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Not ready to send references.";
        return false;
    }
    std::lock_guard<std::mutex> lock(m_mutex);
    {
        std::lock_guard<std::mutex> lock_reference(m_jointReferences.mutex);
        size_t numberOfJoints = m_jointReferences.name.size();
        for (size_t j = 0; j < numberOfJoints; j++)
        {
            m_jointReferences.position[j] = refs[j];
            if (m_compliant[j])
            {
                m_jointReferences.effort[j] = m_compliantOffset[j];
            }
        }
        m_jointReferences.valid = true;
    }
    m_node->publishReferences(m_jointReferences);
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::positionMove(const int n_joints, const int* joints, const double* refs)
{
    std::string errorPrefix = "[positionMove] ";
    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Not ready to send references.";
        return false;
    }
    std::lock_guard<std::mutex> lock(m_mutex);
    {
        std::lock_guard<std::mutex> lock_reference(m_jointReferences.mutex);
        size_t numberOfJoints = m_jointReferences.name.size();
        for (int i = 0; i < n_joints; i++)
        {
            int j = joints[i];
            if (j < 0 || static_cast<size_t>(j) >= numberOfJoints)
            {
                yCError(CB) << errorPrefix << "Joint index out of range. Got" << j << ", expected [0," << numberOfJoints - 1 << "]";
                return false;
            }
            m_jointReferences.position[j] = refs[i];
            if (m_compliant[j])
            {
                m_jointReferences.effort[j] = m_compliantOffset[j];
            }
        }
        m_jointReferences.valid = true;
    }
    m_node->publishReferences(m_jointReferences);
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getTargetPosition(const int joint, double* ref)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getTargetPosition] ";
    std::string suffix_tag = "[" + std::to_string(joint) + "]";
    std::string parameter_name = position_pid_references_tag + suffix_tag;
    auto result = m_node->getParameters({ {parameter_name, Type::PARAMETER_DOUBLE} });
    if (result.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting target position for joint" << joint;
        return false;
    }
    *ref = result[0].double_value;
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getTargetPositions(double* refs)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getTargetPositions] ";
    auto result = m_node->getParameters({ {position_pid_references_tag, Type::PARAMETER_DOUBLE_ARRAY} });
    if (result.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting target positions.";
        return false;
    }
    std::copy(result[0].double_array_value.begin(), result[0].double_array_value.end(), refs);
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getTargetPositions(const int n_joint, const int* joints, double* refs)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getTargetPositions] ";
    auto result = m_node->getParameters({ {position_pid_references_tag, Type::PARAMETER_DOUBLE_ARRAY} });
    if (result.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting target positions.";
        return false;
    }
    const auto& position_array = result[0].double_array_value;
    for (int i = 0; i < n_joint; i++)
    {
        int j = joints[i];
        if (j < 0 || static_cast<size_t>(j) >= position_array.size())
        {
            yCError(CB) << errorPrefix << "Index" << j << "out of range. Valid range is [0," << position_array.size() - 1 << "]";
            return false;
        }
        refs[i] = position_array[j];
    }
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::relativeMove(int j, double delta)
{
    std::string errorPrefix = "[relativeMove] ";
    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Not ready to send references.";
        return false;
    }
    std::lock_guard<std::mutex> lock(m_mutex);
    {
        std::lock_guard<std::mutex> lock_reference(m_jointReferences.mutex);
        if (j < 0 || static_cast<size_t>(j) >= m_jointReferences.name.size())
        {
            yCError(CB) << errorPrefix << "Joint index out of range. Got" << j << ", expected [0," << m_jointReferences.name.size() - 1 << "]";
            return false;
        }

        if (!m_jointState.valid)
        {
            yCError(CB) << errorPrefix << "Cannot perform relative move. Current joint states are not valid.";
            return false;
        }

        std::lock_guard<std::mutex> lock_measurement(m_jointState.mutex);
        m_jointReferences.position[j] = m_jointState.position[j] + delta;
        if (m_compliant[j])
        {
            m_jointReferences.effort[j] = m_compliantOffset[j];
        }
        m_jointReferences.valid = true;
    }
    m_node->publishReferences(m_jointReferences);
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::relativeMove(const double* deltas)
{
    std::string errorPrefix = "[relativeMove] ";
    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Not ready to send references.";
        return false;
    }
    std::lock_guard<std::mutex> lock(m_mutex);
    {
        std::lock_guard<std::mutex> lock_reference(m_jointReferences.mutex);
        size_t numberOfJoints = m_jointReferences.name.size();
        if (!m_jointState.valid)
        {
            yCError(CB) << errorPrefix << "Cannot perform relative move. Current joint states are not valid.";
            return false;
        }
        std::lock_guard<std::mutex> lock_measurement(m_jointState.mutex);
        for (size_t j = 0; j < numberOfJoints; j++)
        {
            m_jointReferences.position[j] = m_jointState.position[j] + deltas[j];
            if (m_compliant[j])
            {
                m_jointReferences.effort[j] = m_compliantOffset[j];
            }
        }
        m_jointReferences.valid = true;
    }
    m_node->publishReferences(m_jointReferences);
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::relativeMove(const int n_joints, const int* joints, const double* deltas)
{
    std::string errorPrefix = "[relativeMove] ";
    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Not ready to send references.";
        return false;
    }
    std::lock_guard<std::mutex> lock(m_mutex);
    {
        std::lock_guard<std::mutex> lock_reference(m_jointReferences.mutex);
        size_t numberOfJoints = m_jointReferences.name.size();
        if (!m_jointState.valid)
        {
            yCError(CB) << errorPrefix << "Cannot perform relative move. Current joint states are not valid.";
            return false;
        }
        std::lock_guard<std::mutex> lock_measurement(m_jointState.mutex);
        for (int i = 0; i < n_joints; i++)
        {
            int j = joints[i];
            if (j < 0 || static_cast<size_t>(j) >= numberOfJoints)
            {
                yCError(CB) << errorPrefix << "Joint index out of range. Got" << j << ", expected [0," << numberOfJoints - 1 << "]";
                return false;
            }
            m_jointReferences.position[j] = m_jointState.position[j] + deltas[i];
            if (m_compliant[j])
            {
                m_jointReferences.effort[j] = m_compliantOffset[j];
            }
        }
        m_jointReferences.valid = true;
    }
    m_node->publishReferences(m_jointReferences);
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::checkMotionDone(int j, bool* flag)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[checkMotionDone] ";

    std::string suffix_tag = "[" + std::to_string(j) + "]";

    auto results = m_node->getParameters({ {is_motion_done_tag + suffix_tag, Type::PARAMETER_BOOL} });

    if (results.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting motion done state.";
        return false;
    }

    *flag = results[0].bool_value;

    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::checkMotionDone(bool* flag)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[checkMotionDone] ";
    auto results = m_node->getParameters({ {is_motion_done_tag, Type::PARAMETER_BOOL_ARRAY} });

    if (results.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting motion done state.";
        return false;
    }

    // Check if any of the joints is still moving
    *flag = true;
    for (const auto& val : results[0].bool_array_value)
    {
        if (!val)
        {
            *flag = false;
            break;
        }
    }

    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::checkMotionDone(const int n_joints, const int* joints, bool* flags)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[checkMotionDone] ";

    auto results = m_node->getParameters({ {is_motion_done_tag, Type::PARAMETER_BOOL_ARRAY} });

    if (results.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting motion done state.";
        return false;
    }

    const auto& motion_done_array = results[0].bool_array_value;

    for (int i = 0; i < n_joints; i++)
    {
        int j = joints[i];
        if (j < 0 || j >= static_cast<int>(motion_done_array.size()))
        {
            yCError(CB) << errorPrefix << "Index" << j << "out of range. Valid range is [0," << motion_done_array.size() - 1 << "]";
            return false;
        }
        flags[i] = motion_done_array[j];
    }

    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRefSpeed(int j, double sp)
{
    std::string errorPrefix = "[setRefSpeed] ";

    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Not ready to send references";
        return false;
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    std::lock_guard<std::mutex> lock_reference(m_jointReferences.mutex);

    if (j < 0 || static_cast<size_t>(j) >= m_jointReferences.name.size())
    {
        yCError(CB) << errorPrefix << "Joint index out of range. Got " << j << ", expected [0," << m_jointReferences.name.size() - 1 << "]";
        return false;
    }

    // Set the reference, but don't send it
    // It will be sent at the next positionMove command
    m_jointReferences.velocity[j] = sp;

    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRefSpeeds(const double* spds)
{
    std::string errorPrefix = "[setRefSpeeds] ";
    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Not ready to send references";
        return false;
    }
    std::lock_guard<std::mutex> lock(m_mutex);
    std::lock_guard<std::mutex> lock_reference(m_jointReferences.mutex);
    size_t numberOfJoints = m_jointReferences.name.size();
    for (size_t j = 0; j < numberOfJoints; j++)
    {
        // Set the reference, but don't send it
        // It will be sent at the next positionMove command
        m_jointReferences.velocity[j] = spds[j];
    }
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRefSpeeds(const int n_joints, const int* joints, const double* spds)
{
    std::string errorPrefix = "[setRefSpeeds] ";
    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Not ready to send references";
        return false;
    }
    std::lock_guard<std::mutex> lock(m_mutex);
    std::lock_guard<std::mutex> lock_reference(m_jointReferences.mutex);
    size_t numberOfJoints = m_jointReferences.name.size();
    for (int i = 0; i < n_joints; i++)
    {
        int j = joints[i];
        if (j < 0 || static_cast<size_t>(j) >= numberOfJoints)
        {
            yCError(CB) << errorPrefix << "Joint index out of range. Got" << j << ", expected [0," << numberOfJoints - 1 << "]";
            return false;
        }
        // Set the reference, but don't send it
        // It will be sent at the next positionMove command
        m_jointReferences.velocity[j] = spds[i];
    }
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRefAcceleration(int j, double acc)
{
    yCError(CB) << "[setRefAcceleration] It is not possible to set reference accelerations in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRefAccelerations(const double* accs)
{
    yCError(CB) << "[setRefAccelerations] It is not possible to set reference accelerations in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRefAccelerations(const int n_joints, const int* joints, const double* accs)
{
    yCError(CB) << "[setRefAccelerations] It is not possible to set reference accelerations in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefSpeed(int j, double* ref)
{
    // TODO
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefSpeeds(double* spds)
{
    // TODO
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefSpeeds(const int n_joints, const int* joints, double* spds)
{
    // TODO
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefAcceleration(int j, double* acc)
{
    yCError(CB) << "[getRefAcceleration] It is not possible to get reference accelerations in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefAccelerations(double* accs)
{
    yCError(CB) << "[getRefAccelerations] It is not possible to get reference accelerations in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefAccelerations(const int n_joints, const int* joints, double* accs)
{
    yCError(CB) << "[getRefAccelerations] It is not possible to get reference accelerations in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::stop(int j)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[stop] ";
    std::string suffix_tag = "[" + std::to_string(j) + "]";

    rcl_interfaces::msg::Parameter stop_param;
    stop_param.name = position_pid_to_stop_tag + suffix_tag;
    stop_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    stop_param.value.bool_value = true;
    auto results = m_node->setParameters({ stop_param });

    if (results.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while stopping the joint" << j << ".";
        return false;
    }
    if (!results[0].successful)
    {
        yCError(CB) << errorPrefix << "Error while stopping the joint" << j << ":" << results[0].reason;
        return false;
    }
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::stop()
{
    std::string errorPrefix = "[stop] ";

    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Services are not ready.";
        return false;
    }

    std::lock_guard<std::mutex> lock(m_mutex);

    size_t numberOfJoints = m_jointNames.size();

    rcl_interfaces::msg::Parameter stop_param;
    stop_param.name = position_pid_to_stop_tag;
    stop_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY;
    stop_param.value.bool_array_value = std::vector<bool>(numberOfJoints, true);
    auto results = m_node->setParameters({ stop_param });
    if (results.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while stopping all the joints.";
        return false;
    }
    if (!results[0].successful)
    {
        yCError(CB) << errorPrefix << "Error while stopping all the joints:" << results[0].reason;
        return false;
    }
    return true;

}

bool yarp::dev::IsaacSimControlBoardNWCROS2::stop(const int n_joints, const int* joints)
{
    std::string errorPrefix = "[stop] ";
    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Services are not ready.";
        return false;
    }

    std::lock_guard<std::mutex> lock(m_mutex);

    size_t numberOfJoints = m_jointNames.size();
    rcl_interfaces::msg::Parameter stop_param;
    stop_param.name = position_pid_to_stop_tag;
    stop_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY;
    stop_param.value.bool_array_value = std::vector<bool>(numberOfJoints, false);
    for (int i = 0; i < n_joints; i++)
    {
        int j = joints[i];
        if (j < 0 || j >= static_cast<int>(numberOfJoints))
        {
            yCError(CB) << errorPrefix << "Index" << j << "out of range. Valid range is [0," << numberOfJoints - 1 << "]";
            return false;
        }
        stop_param.value.bool_array_value[j] = true;
    }
    auto results = m_node->setParameters({ stop_param });
    if (results.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while stopping the joints.";
        return false;
    }
    if (!results[0].successful)
    {
        yCError(CB) << errorPrefix << "Error while stopping the joints:" << results[0].reason;
        return false;
    }
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getLastJointFault(int j, int& fault, std::string& message)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getLastJointFault] ";
    std::string suffix_tag = "[" + std::to_string(j) + "]";
    auto results = m_node->getParameters({ {hf_messages_tag + suffix_tag, Type::PARAMETER_STRING} });
    if (results.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting last joint fault.";
        return false;
    }

    fault = -1;
    message = results[0].string_value;
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::velocityMove(int j, double v)
{
    std::string errorPrefix = "[velocityMove] ";
    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Not ready to send references";
        return false;
    }
    std::lock_guard<std::mutex> lock(m_mutex);
    {
        std::lock_guard<std::mutex> lock_reference(m_jointReferences.mutex);
        if (j < 0 || static_cast<size_t>(j) >= m_jointReferences.name.size())
        {
            yCError(CB) << errorPrefix << "Joint index out of range. Got " << j << ", expected [0," << m_jointReferences.name.size() - 1 << "]";
            return false;
        }
        m_jointReferences.velocity[j] = v;
        m_jointReferences.valid = true;
    }
    m_node->publishReferences(m_jointReferences);
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::velocityMove(const double* v)
{
    std::string errorPrefix = "[velocityMove] ";
    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Not ready to send references";
        return false;
    }
    std::lock_guard<std::mutex> lock(m_mutex);
    {
        std::lock_guard<std::mutex> lock_reference(m_jointReferences.mutex);
        size_t numberOfJoints = m_jointReferences.name.size();
        for (size_t j = 0; j < numberOfJoints; j++)
        {
            m_jointReferences.velocity[j] = v[j];
        }
        m_jointReferences.valid = true;
    }
    m_node->publishReferences(m_jointReferences);
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::resetEncoder(int j)
{
    yCError(CB) << "[resetEncoder] It is not possible to reset an encoder in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::resetEncoders()
{
    yCError(CB) << "[resetEncoders] It is not possible to reset encoders in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setEncoder(int j, double val)
{
    yCError(CB) << "[setEncoder] It is not possible to set an encoder in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setEncoders(const double* vals)
{
    yCError(CB) << "[setEncoders] It is not possible to set encoders in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getEncoder(int j, double* v)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getEncoder] ";
    if (!m_jointState.valid.load())
    {
        yCError(CB) << errorPrefix << "No valid data received yet.";
        return false;
    }

    std::lock_guard<std::mutex> lock_measurements(m_jointState.mutex);
    if (j < 0 || j >= static_cast<int>(m_jointState.position.size()))
    {
        yCError(CB) << errorPrefix << "Index" << j << "out of range. Valid range is [0," << m_jointState.position.size() - 1 << "]";
        return false;
    }
    *v = m_jointState.position[j];
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getEncoders(double* encs)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getEncoders] ";
    if (!m_jointState.valid.load())
    {
        yCError(CB) << errorPrefix << "No valid data received yet.";
        return false;
    }

    std::lock_guard<std::mutex> lock_measurements(m_jointState.mutex);
    std::copy(m_jointState.position.begin(), m_jointState.position.end(), encs);
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getEncodersTimed(double* encs, double* t)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getEncodersTimed] ";
    if (!m_jointState.valid.load())
    {
        yCError(CB) << errorPrefix << "No valid data received yet.";
        return false;
    }

    std::lock_guard<std::mutex> lock_measurements(m_jointState.mutex);

    std::copy(m_jointState.position.begin(), m_jointState.position.end(), encs);

    // Copy in timestamp a vector of size equal to the number of joints and equal to the timestamp of the measurement
    std::fill(t, t + m_jointState.position.size(), m_jointState.timestamp.getTime());

    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getEncoderTimed(int j, double* v, double* t)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getEncoderTimed] ";
    if (!m_jointState.valid.load())
    {
        yCError(CB) << errorPrefix << "No valid data received yet.";
        return false;
    }

    std::lock_guard<std::mutex> lock_measurements(m_jointState.mutex);

    if (j < 0 || j >= static_cast<int>(m_jointState.position.size()))
    {
        yCError(CB) << errorPrefix << "Index" << j << "out of range. Valid range is [0," << m_jointState.position.size() - 1 << "]";
        return false;
    }
    *v = m_jointState.position[j];
    *t = m_jointState.timestamp.getTime();
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getEncoderSpeed(int j, double* sp)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getEncoderSpeed] ";
    if (!m_jointState.valid.load())
    {
        yCError(CB) << errorPrefix << "No valid data received yet.";
        return false;
    }

    std::lock_guard<std::mutex> lock_measurements(m_jointState.mutex);

    if (j < 0 || j >= static_cast<int>(m_jointState.velocity.size()))
    {
        yCError(CB) << errorPrefix << "Index" << j << "out of range. Valid range is [0," << m_jointState.velocity.size() - 1 << "]";
        return false;
    }
    *sp = m_jointState.velocity[j];
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getEncoderSpeeds(double* spds)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getEncoderSpeeds] ";
    if (!m_jointState.valid.load())
    {
        yCError(CB) << errorPrefix << "No valid data received yet.";
        return false;
    }

    std::lock_guard<std::mutex> lock_measurements(m_jointState.mutex);

    std::copy(m_jointState.velocity.begin(), m_jointState.velocity.end(), spds);
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getEncoderAcceleration(int j, double* acc)
{
    yCError(CB) << "[getEncoderAcceleration] It is not possible to get encoder acceleration in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getEncoderAccelerations(double* accs)
{
    yCError(CB) << "[getEncoderAccelerations] It is not possible to get encoder accelerations in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getNumberOfMotorEncoders(int* num)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getNumberOfMotorEncoders] ";
    if (!m_motorState.valid.load())
    {
        yCError(CB) << errorPrefix << "No valid data received yet.";
        return false;
    }
    std::lock_guard<std::mutex> lock_measurements(m_motorState.mutex);
    *num = static_cast<int>(m_motorState.name.size());
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::resetMotorEncoder(int m)
{
    yCError(CB) << "[resetMotorEncoder] It is not possible to reset a motor encoder in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::resetMotorEncoders()
{
    yCError(CB) << "[resetMotorEncoders] It is not possible to reset motor encoders in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setMotorEncoderCountsPerRevolution(int m, const double cpr)
{
    yCError(CB) << "[setMotorEncoderCountsPerRevolution] It is not possible to set motor encoder counts per revolution in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getMotorEncoderCountsPerRevolution(int m, double* cpr)
{

    yCError(CB) << "[getMotorEncoderCountsPerRevolution] It is not possible to get motor encoder counts per revolution in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setMotorEncoder(int m, const double val)
{
    yCError(CB) << "[setMotorEncoder] It is not possible to set a motor encoder in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setMotorEncoders(const double* vals)
{
    yCError(CB) << "[setMotorEncoders] It is not possible to set motor encoders in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getMotorEncoder(int m, double* v)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getMotorEncoder] ";
    if (!m_motorState.valid.load())
    {
        yCError(CB) << errorPrefix << "No valid data received yet.";
        return false;
    }
    std::lock_guard<std::mutex> lock_measurements(m_motorState.mutex);
    if (m < 0 || m >= static_cast<int>(m_motorState.position.size()))
    {
        yCError(CB) << errorPrefix << "Index" << m << "out of range. Valid range is [0," << m_motorState.position.size() - 1 << "]";
        return false;
    }
    *v = m_motorState.position[m];
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getMotorEncoders(double* encs)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getMotorEncoders] ";
    if (!m_motorState.valid.load())
    {
        yCError(CB) << errorPrefix << "No valid data received yet.";
        return false;
    }
    std::lock_guard<std::mutex> lock_measurements(m_motorState.mutex);
    std::copy(m_motorState.position.begin(), m_motorState.position.end(), encs);
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getMotorEncodersTimed(double* encs, double* t)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getMotorEncodersTimed] ";
    if (!m_motorState.valid.load())
    {
        yCError(CB) << errorPrefix << "No valid data received yet.";
        return false;
    }
    std::lock_guard<std::mutex> lock_measurements(m_motorState.mutex);
    std::copy(m_motorState.position.begin(), m_motorState.position.end(), encs);
    // Copy in timestamp a vector of size equal to the number of motors and equal to the timestamp of the measurement
    std::fill(t, t + m_motorState.position.size(), m_motorState.timestamp.getTime());
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getMotorEncoderTimed(int m, double* v, double* t)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getMotorEncoderTimed] ";
    if (!m_motorState.valid.load())
    {
        yCError(CB) << errorPrefix << "No valid data received yet.";
        return false;
    }
    std::lock_guard<std::mutex> lock_measurements(m_motorState.mutex);
    if (m < 0 || m >= static_cast<int>(m_motorState.position.size()))
    {
        yCError(CB) << errorPrefix << "Index" << m << "out of range. Valid range is [0," << m_motorState.position.size() - 1 << "]";
        return false;
    }
    *v = m_motorState.position[m];
    *t = m_motorState.timestamp.getTime();
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getMotorEncoderSpeed(int m, double* sp)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getMotorEncoderSpeed] ";
    if (!m_motorState.valid.load())
    {
        yCError(CB) << errorPrefix << "No valid data received yet.";
        return false;
    }
    std::lock_guard<std::mutex> lock_measurements(m_motorState.mutex);
    if (m < 0 || m >= static_cast<int>(m_motorState.velocity.size()))
    {
        yCError(CB) << errorPrefix << "Index" << m << "out of range. Valid range is [0," << m_motorState.velocity.size() - 1 << "]";
        return false;
    }
    *sp = m_motorState.velocity[m];
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getMotorEncoderSpeeds(double* spds)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getMotorEncoderSpeeds] ";
    if (!m_motorState.valid.load())
    {
        yCError(CB) << errorPrefix << "No valid data received yet.";
        return false;
    }
    std::lock_guard<std::mutex> lock_measurements(m_motorState.mutex);
    std::copy(m_motorState.velocity.begin(), m_motorState.velocity.end(), spds);
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getMotorEncoderAcceleration(int m, double* acc)
{
    yCError(CB) << "[getMotorEncoderAcceleration] It is not possible to get motor encoder acceleration in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getMotorEncoderAccelerations(double* accs)
{
    yCError(CB) << "[getMotorEncoderAccelerations] It is not possible to get motor encoder accelerations in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::enableAmp(int j)
{
    yCError(CB) << "[enableAmp] It is not possible to enable an amplifier in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::disableAmp(int j)
{
    yCError(CB) << "[disableAmp] It is not possible to disable an amplifier in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getAmpStatus(int* st)
{
    yCError(CB) << "[getAmpStatus] It is not possible to get amplifier status in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getAmpStatus(int j, int* v)
{
    yCError(CB) << "[getAmpStatus] It is not possible to get amplifier status in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setMaxCurrent(int j, double v)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[setMaxCurrent] ";
    std::string suffix_tag = "[" + std::to_string(j) + "]";
    rcl_interfaces::msg::Parameter max_current_param;
    max_current_param.name = motor_max_currents_tag + suffix_tag;
    max_current_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    max_current_param.value.double_value = v;
    auto results = m_node->setParameters({ max_current_param });
    if (results.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while setting max current for joint" << j << ".";
        return false;
    }
    if (!results[0].successful)
    {
        yCError(CB) << errorPrefix << "Error while setting max current for joint" << j << ":" << results[0].reason;
        return false;
    }
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getMaxCurrent(int j, double* v)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getMaxCurrent] ";
    std::string suffix_tag = "[" + std::to_string(j) + "]";
    auto results = m_node->getParameters({ {motor_max_currents_tag + suffix_tag, Type::PARAMETER_DOUBLE} });
    if (results.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting max current for joint" << j << ".";
        return false;
    }

    *v = results[0].double_value;
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getNominalCurrent(int m, double* val)
{
    yCError(CB) << "[getNominalCurrent] It is not possible to get nominal current in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setNominalCurrent(int m, const double val)
{
    yCError(CB) << "[setNominalCurrent] It is not possible to set nominal current in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getPeakCurrent(int m, double* val)
{
    yCError(CB) << "[getPeakCurrent] It is not possible to get peak current in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setPeakCurrent(int m, const double val)
{
    yCError(CB) << "[setPeakCurrent] It is not possible to set peak current in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getPWM(int m, double* val)
{
    yCError(CB) << "[getPWM] It is not possible to get PWM in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getPWMLimit(int m, double* val)
{
    yCError(CB) << "[getPWMLimit] It is not possible to get PWM limit in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setPWMLimit(int m, const double val)
{
    yCError(CB) << "[setPWMLimit] It is not possible to set PWM limit in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getPowerSupplyVoltage(int m, double* val)
{
    yCError(CB) << "[getPowerSupplyVoltage] It is not possible to get power supply voltage in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setLimits(int j, double min, double max)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[setLimits] ";
    std::string suffix_tag = "[" + std::to_string(j) + "]";
    rcl_interfaces::msg::Parameter min_param;
    min_param.name = min_positions_tag + suffix_tag;
    min_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    min_param.value.double_value = min;
    rcl_interfaces::msg::Parameter max_param;
    max_param.name = max_positions_tag + suffix_tag;
    max_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    max_param.value.double_value = max;
    auto results = m_node->setParameters({ min_param, max_param });
    if (results.size() != 2)
    {
        yCError(CB) << errorPrefix << "Error while setting limits for joint" << j << ".";
        return false;
    }
    if (!results[0].successful)
    {
        yCError(CB) << errorPrefix << "Error while setting min limit for joint" << j << ":" << results[0].reason;
        return false;
    }
    if (!results[1].successful)
    {
        yCError(CB) << errorPrefix << "Error while setting max limit for joint" << j << ":" << results[1].reason;
        return false;
    }
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getLimits(int j, double* min, double* max)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getLimits] ";
    std::string suffix_tag = "[" + std::to_string(j) + "]";
    auto results = m_node->getParameters({ {min_positions_tag + suffix_tag, Type::PARAMETER_DOUBLE},
                                           {max_positions_tag + suffix_tag, Type::PARAMETER_DOUBLE} });
    if (results.size() != 2)
    {
        yCError(CB) << errorPrefix << "Error while getting limits for joint" << j << ".";
        return false;
    }

    *min = results[0].double_value;
    *max = results[1].double_value;
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setVelLimits(int j, double min, double max)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[setVelLimits] ";
    std::string suffix_tag = "[" + std::to_string(j) + "]";
    rcl_interfaces::msg::Parameter max_param;
    max_param.name = max_velocities_tag + suffix_tag;
    max_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;

    if (min != max)
    {
        yCWarning(CB) << errorPrefix << "Isaac sim supports only symmetric velocity limits. "
                                        "The minimum absolute value of the two will be considered.";
    }

    max_param.value.double_value = std::min(std::abs(min), std::abs(max));

    auto results = m_node->setParameters({ max_param });
    if (results.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while setting velocity limits for joint" << j << ".";
        return false;
    }
    if (!results[0].successful)
    {
        yCError(CB) << errorPrefix << "Error while setting velocity limits for joint" << j << ":" << results[0].reason;
        return false;
    }
    return true;

}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getVelLimits(int j, double* min, double* max)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getVelLimits] ";
    std::string suffix_tag = "[" + std::to_string(j) + "]";
    auto results = m_node->getParameters({ {max_velocities_tag + suffix_tag, Type::PARAMETER_DOUBLE} });
    if (results.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting velocity limits for joint" << j << ".";
        return false;
    }

    *max = results[0].double_value;
    *min = -(*max);
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRemoteVariable(std::string key, yarp::os::Bottle& val)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getRemoteVariable] ";
    auto results = m_node->getParameters({ {key, Type::PARAMETER_NOT_SET} });
    if (results.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting remote variable" << key << ".";
        return false;
    }

    switch (results[0].type)
    {
    case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL:
        val.addInt8(results[0].bool_value);
        break;
    case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER:
        val.addInt32(static_cast<int32_t>(results[0].integer_value));
        break;
    case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE:
        val.addFloat64(results[0].double_value);
        break;
    case rcl_interfaces::msg::ParameterType::PARAMETER_STRING:
        val.addString(results[0].string_value);
        break;
    case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY:
        for (auto v : results[0].bool_array_value)
            val.addInt8(v);
        break;
    case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY:
        for (auto v : results[0].integer_array_value)
            val.addInt32(static_cast<int32_t>(v));
        break;
    case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY:
        for (auto v : results[0].double_array_value)
            val.addFloat64(v);
        break;
    case rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY:
        for (auto v : results[0].string_array_value)
            val.addString(v);
        break;
    default:
        yCError(CB) << errorPrefix << "Error while getting remote variable" << key << ". Unsupported parameter type.";
        return false;
    }

    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRemoteVariable(std::string key, const yarp::os::Bottle& val)
{
    yCError(CB) << "[setRemoteVariable] It is not possible to set remote variables in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRemoteVariablesList(yarp::os::Bottle* listOfKeys)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getRemoteVariablesList] ";

    if (listOfKeys == nullptr)
    {
        yCError(CB) << errorPrefix << "listOfKeys is a null pointer.";
        return false;
    }

    listOfKeys->addString(joint_names_tag);
    listOfKeys->addString(max_positions_tag);
    listOfKeys->addString(min_positions_tag);
    listOfKeys->addString(max_velocities_tag);
    listOfKeys->addString(max_efforts_tag);
    listOfKeys->addString(control_modes_tag);
    listOfKeys->addString(previous_control_modes_tag);
    listOfKeys->addString(compliant_modes_tag);
    listOfKeys->addString(hf_messages_tag);
    listOfKeys->addString(position_p_gains_tag);
    listOfKeys->addString(position_i_gains_tag);
    listOfKeys->addString(position_d_gains_tag);
    listOfKeys->addString(position_max_integral_tag);
    listOfKeys->addString(position_max_output_tag);
    listOfKeys->addString(position_max_error_tag);
    listOfKeys->addString(home_positions_tag);
    listOfKeys->addString(compliant_stiffness_tag);
    listOfKeys->addString(compliant_damping_tag);
    listOfKeys->addString(velocity_p_gains_tag);
    listOfKeys->addString(velocity_i_gains_tag);
    listOfKeys->addString(velocity_d_gains_tag);
    listOfKeys->addString(velocity_max_integral_tag);
    listOfKeys->addString(velocity_max_output_tag);
    listOfKeys->addString(velocity_max_error_tag);
    listOfKeys->addString(position_pid_references_tag);
    listOfKeys->addString(position_pid_errors_tag);
    listOfKeys->addString(position_pid_outputs_tag);
    listOfKeys->addString(is_motion_done_tag);
    listOfKeys->addString(position_pid_enabled_tag);
    listOfKeys->addString(position_pid_to_reset_tag);
    listOfKeys->addString(position_pid_to_stop_tag);
    listOfKeys->addString(velocity_pid_references_tag);
    listOfKeys->addString(velocity_pid_errors_tag);
    listOfKeys->addString(velocity_pid_outputs_tag);
    listOfKeys->addString(velocity_pid_enabled_tag);
    listOfKeys->addString(velocity_pid_to_reset_tag);
    listOfKeys->addString(torque_pid_references_tag);
    listOfKeys->addString(torque_pid_errors_tag);
    listOfKeys->addString(torque_pid_outputs_tag);
    listOfKeys->addString(torque_pid_enabled_tag);
    listOfKeys->addString(current_pid_references_tag);
    listOfKeys->addString(current_pid_errors_tag);
    listOfKeys->addString(current_pid_outputs_tag);
    listOfKeys->addString(current_pid_enabled_tag);
    listOfKeys->addString(gearbox_ratios_tag);
    listOfKeys->addString(motor_torque_constants_tag);
    listOfKeys->addString(motor_current_noise_variance_tag);
    listOfKeys->addString(motor_spring_stiffness_tag);
    listOfKeys->addString(motor_max_currents_tag);

    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::isCalibratorDevicePresent(bool* isCalib)
{
    *isCalib = false;
    return false;
}

yarp::dev::IRemoteCalibrator* yarp::dev::IsaacSimControlBoardNWCROS2::getCalibratorDevice()
{
    return nullptr;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::calibrateSingleJoint(int j)
{
    yCError(CB) << "[calibrateSingleJoint] It is not possible to calibrate a joint in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::calibrateWholePart()
{
    yCError(CB) << "[calibrateWholePart] It is not possible to calibrate joints in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::homingSingleJoint(int j)
{
    std::string errorPrefix = "[homingSingleJoint] ";

    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Not ready to send references";
        return false;
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    std::string suffix_tag = "[" + std::to_string(j) + "]";
    auto results = m_node->getParameters({ {home_positions_tag + suffix_tag, Type::PARAMETER_DOUBLE} });
    if (results.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting home position for joint" << j << ".";
        return false;
    }
    double homePosition = results[0].double_value;

    {
        std::lock_guard<std::mutex> lock_reference(m_jointReferences.mutex);

        // We need to convert since when sending the message, it will be converted back

        if (!m_jointReferences.convert_to_deg_if_revolute(j, homePosition, m_jointReferences.position[j]))
        {
            yCError(CB) << errorPrefix << "Joint index out of range. Got " << j << ", expected [0," << m_jointReferences.name.size() - 1 << "]";
            return false;
        }

        if (m_compliant[j])
        {
            m_jointReferences.effort[j] = m_compliantOffset[j];
        }

        m_jointReferences.valid = true;
    }
    m_node->publishReferences(m_jointReferences);
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::homingWholePart()
{
    std::string errorPrefix = "[homingWholePart] ";
    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Not ready to send references";
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        auto results = m_node->getParameters({ {home_positions_tag, Type::PARAMETER_DOUBLE_ARRAY} });
        if (results.size() != 1)
        {
            yCError(CB) << errorPrefix << "Error while getting home positions.";
            return false;
        }

        std::lock_guard<std::mutex> lock_reference(m_jointReferences.mutex);
        size_t numberOfJoints = m_jointReferences.name.size();
        for (size_t j = 0; j < numberOfJoints; j++)
        {
            m_jointReferences.convert_to_deg_if_revolute(j, results[0].double_array_value[j],
                                                 m_jointReferences.position[j]);

            if (m_compliant[j])
            {
                m_jointReferences.effort[j] = m_compliantOffset[j];
            }
        }
        m_jointReferences.valid = true;
    }
    m_node->publishReferences(m_jointReferences);

    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::parkSingleJoint(int j, bool _wait)
{
    yCError(CB) << "[parkSingleJoint] It is not possible to park a joint in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::parkWholePart()
{
    yCError(CB) << "[parkWholePart] It is not possible to park joints in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::quitCalibrate()
{
    yCError(CB) << "[quitCalibrate] It is not possible to quit calibration in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::quitPark()
{
    yCError(CB) << "[quitPark] It is not possible to quit park in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::calibrateAxisWithParams(int j, unsigned int ui, double v1, double v2, double v3)
{
    yCError(CB) << "[calibrateAxisWithParams] It is not possible to calibrate a joint in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setCalibrationParameters(int j, const yarp::dev::CalibrationParameters& params)
{
    yCError(CB) << "[setCalibrationParameters] It is not possible to set calibration parameters in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::calibrationDone(int j)
{
    yCError(CB) << "[calibrationDone] It is not possible to check calibration status in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::abortPark()
{
    yCError(CB) << "[abortPark] It is not possible to abort park in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::abortCalibration()
{
    yCError(CB) << "[abortCalibration] It is not possible to abort calibration in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getNumberOfMotors(int* num)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getNumberOfMotors] ";
    if (!m_motorState.valid.load())
    {
        yCError(CB) << errorPrefix << "No valid data received yet.";
        return false;
    }
    std::lock_guard<std::mutex> lock_measurements(m_motorState.mutex);
    *num = static_cast<int>(m_motorState.name.size());
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getTemperature(int m, double* val)
{
    yCErrorOnce(CB) << "[getTemperature] It is not possible to get temperature in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getTemperatures(double* vals)
{
    yCErrorOnce(CB) << "[getTemperatures] It is not possible to get temperatures in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getTemperatureLimit(int m, double* val)
{
    yCError(CB) << "[getTemperatureLimit] It is not possible to get temperature limit in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setTemperatureLimit(int m, const double val)
{
    yCError(CB) << "[setTemperatureLimit] It is not possible to set temperature limit in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getGearboxRatio(int m, double* val)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getGearboxRatio] ";
    std::string suffix_tag = "[" + std::to_string(m) + "]";
    auto results = m_node->getParameters({ {gearbox_ratios_tag + suffix_tag, Type::PARAMETER_DOUBLE} });
    if (results.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting gearbox ratio for motor" << m << ".";
        return false;
    }

    *val = results[0].double_value;
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setGearboxRatio(int m, const double val)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[setGearboxRatio] ";
    std::string suffix_tag = "[" + std::to_string(m) + "]";
    rcl_interfaces::msg::Parameter gearbox_ratio_param;
    gearbox_ratio_param.name = gearbox_ratios_tag + suffix_tag;
    gearbox_ratio_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    gearbox_ratio_param.value.double_value = val;
    auto results = m_node->setParameters({ gearbox_ratio_param });
    if (results.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while setting gearbox ratio for motor" << m << ".";
        return false;
    }
    if (!results[0].successful)
    {
        yCError(CB) << errorPrefix << "Error while setting gearbox ratio for motor" << m << ":" << results[0].reason;
        return false;
    }
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getAxisName(int j, std::string& name)
{
    std::string errorPrefix = "[getAxisName] ";
    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Services are not ready.";
        return false;
    }

    std::lock_guard<std::mutex> lock(m_mutex);

    size_t numberOfJoints = m_jointNames.size();
    if (j < 0 || j >= static_cast<int>(numberOfJoints))
    {
        yCError(CB) << errorPrefix << "Index" << j << "out of range. Valid range is [0," << numberOfJoints - 1 << "]";
        return false;
    }
    name = m_jointNames[j];
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getJointType(int j, yarp::dev::JointTypeEnum& type)
{
    std::string errorPrefix = "[getJointType] ";
    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Services are not ready.";
        return false;
    }
    std::lock_guard<std::mutex> lock(m_jointState.mutex);
    size_t numberOfJoints = m_jointState.jointTypes.size();
    if (j < 0 || j >= static_cast<int>(numberOfJoints))
    {
        yCError(CB) << errorPrefix << "Index" << j << "out of range. Valid range is [0," << numberOfJoints - 1 << "]";
        return false;
    }

    if (m_jointState.jointTypes[j] == 0)
    {
        type = yarp::dev::JointTypeEnum::VOCAB_JOINTTYPE_REVOLUTE;
    }
    else if (m_jointState.jointTypes[j] == 1)
    {
        type = yarp::dev::JointTypeEnum::VOCAB_JOINTTYPE_PRISMATIC;
    }
    else
    {
        yCError(CB) << errorPrefix << "Joint type for joint" << j << "is unknown.";
        return false;
    }
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefTorques(double* refs)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getRefTorques] ";

    auto results = m_node->getParameters({ {torque_pid_references_tag, Type::PARAMETER_DOUBLE_ARRAY} });
    if (results.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting torque references.";
        return false;
    }
    if (results[0].double_array_value.size() != m_jointNames.size())
    {
        yCError(CB) << errorPrefix << "Size of torque references does not match number of joints.";
        return false;
    }
    std::copy(results[0].double_array_value.begin(), results[0].double_array_value.end(), refs);
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefTorque(int j, double* t)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getRefTorque] ";
    std::string suffix_tag = "[" + std::to_string(j) + "]";
    auto results = m_node->getParameters({ {torque_pid_references_tag + suffix_tag, Type::PARAMETER_DOUBLE} });
    if (results.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting torque reference for joint" << j << ".";
        return false;
    }
    *t = results[0].double_value;
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRefTorques(const double* t)
{
    std::string errorPrefix = "[setRefTorques] ";
    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Not ready to send references";
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        std::lock_guard<std::mutex> lock_reference(m_jointReferences.mutex);
        size_t numberOfJoints = m_jointReferences.name.size();
        std::copy(t, t + numberOfJoints, m_jointReferences.effort.begin());
        m_jointReferences.valid = true;
    }
    m_node->publishReferences(m_jointReferences);
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRefTorque(int j, double t)
{
    std::string errorPrefix = "[setRefTorque] ";
    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Not ready to send references";
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        std::lock_guard<std::mutex> lock_reference(m_jointReferences.mutex);
        if (j < 0 || j >= static_cast<int>(m_jointReferences.name.size()))
        {
            yCError(CB) << errorPrefix << "Joint index out of range. Got " << j << ", expected [0," << m_jointReferences.name.size() - 1 << "]";
            return false;
        }
        m_jointReferences.effort[j] = t;
        m_jointReferences.valid = true;
    }
    m_node->publishReferences(m_jointReferences);
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRefTorques(const int n_joint, const int* joints, const double* t)
{
    std::string errorPrefix = "[setRefTorques] ";
    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Not ready to send references";
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        std::lock_guard<std::mutex> lock_reference(m_jointReferences.mutex);
        size_t numberOfJoints = m_jointReferences.name.size();
        for (int i = 0; i < n_joint; i++)
        {
            int j = joints[i];
            if (j < 0 || j >= static_cast<int>(numberOfJoints))
            {
                yCError(CB) << errorPrefix << "Joint index out of range. Got " << j << ", expected [0," << numberOfJoints - 1 << "]";
                return false;
            }
            m_jointReferences.effort[j] = t[i];
        }
        m_jointReferences.valid = true;
    }
    m_node->publishReferences(m_jointReferences);
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getMotorTorqueParams(int j, yarp::dev::MotorTorqueParameters* params)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getMotorTorqueParams] ";
    std::string suffix_tag = "[" + std::to_string(j) + "]";
    auto results = m_node->getParameters({ {motor_torque_constants_tag + suffix_tag, Type::PARAMETER_DOUBLE} });
    if (results.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting motor torque constant for motor" << j << ".";
        return false;
    }

    params->ktau = results[0].double_value;

    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setMotorTorqueParams(int j, const yarp::dev::MotorTorqueParameters params)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[setMotorTorqueParams] ";
    std::string suffix_tag = "[" + std::to_string(j) + "]";
    rcl_interfaces::msg::Parameter motor_torque_constant_param;
    motor_torque_constant_param.name = motor_torque_constants_tag + suffix_tag;
    motor_torque_constant_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    motor_torque_constant_param.value.double_value = params.ktau;
    auto results = m_node->setParameters({ motor_torque_constant_param });
    if (results.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while setting motor torque constant for motor" << j << ".";
        return false;
    }
    if (!results[0].successful)
    {
        yCError(CB) << errorPrefix << "Error while setting motor torque constant for motor" << j << ":" << results[0].reason;
        return false;
    }
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setImpedance(int j, double stiff, double damp)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[setImpedance] ";
    std::string suffix_tag = "[" + std::to_string(j) + "]";
    rcl_interfaces::msg::Parameter stiff_param;
    stiff_param.name = compliant_stiffness_tag + suffix_tag;
    stiff_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    stiff_param.value.double_value = stiff;
    rcl_interfaces::msg::Parameter damp_param;
    damp_param.name = compliant_damping_tag + suffix_tag;
    damp_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    damp_param.value.double_value = damp;
    auto results = m_node->setParameters({ stiff_param, damp_param });
    if (results.size() != 2)
    {
        yCError(CB) << errorPrefix << "Error while setting impedance for joint" << j << ".";
        return false;
    }
    if (!results[0].successful)
    {
        yCError(CB) << errorPrefix << "Error while setting stiffness for joint" << j << ":" << results[0].reason;
        return false;
    }
    if (!results[1].successful)
    {
        yCError(CB) << errorPrefix << "Error while setting damping for joint" << j << ":" << results[1].reason;
        return false;
    }
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setImpedanceOffset(int j, double offset)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[setImpedanceOffset] ";

    if (j < 0 || j >= static_cast<int>(m_compliantOffset.size()))
    {
        yCError(CB) << errorPrefix << "Joint index out of range. Got " << j << ", expected [0," << m_compliantOffset.size() - 1 << "]";
        return false;
    }

    // We store the offset, but it will be sent only with the next position command
    m_compliantOffset[j] = offset;
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getTorque(int j, double* t)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getTorque] ";
    if (!m_jointState.valid.load())
    {
        yCError(CB) << errorPrefix << "No valid data received yet.";
        return false;
    }
    std::lock_guard<std::mutex> lock_measurements(m_jointState.mutex);
    if (j < 0 || j >= static_cast<int>(m_jointState.effort.size()))
    {
        yCError(CB) << errorPrefix << "Index" << j << "out of range. Valid range is [0," << m_jointState.effort.size() - 1 << "]";
        return false;
    }
    *t = m_jointState.effort[j];
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getTorques(double* t)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getTorques] ";
    if (!m_jointState.valid.load())
    {
        yCError(CB) << errorPrefix << "No valid data received yet.";
        return false;
    }
    std::lock_guard<std::mutex> lock_measurements(m_jointState.mutex);
    std::copy(m_jointState.effort.begin(), m_jointState.effort.end(), t);
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getTorqueRange(int j, double* min, double* max)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getTorqueRange] ";
    std::string suffix_tag = "[" + std::to_string(j) + "]";
    auto results = m_node->getParameters({ {max_efforts_tag + suffix_tag, Type::PARAMETER_DOUBLE} });
    if (results.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting max effort for joint" << j << ".";
        return false;
    }

    *max = results[0].double_value;
    *min = -(*max);
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getTorqueRanges(double* min, double* max)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getTorqueRanges] ";
    auto results = m_node->getParameters({ {max_efforts_tag, Type::PARAMETER_DOUBLE_ARRAY} });
    if (results.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting max efforts.";
        return false;
    }

    std::copy(results[0].double_array_value.begin(), results[0].double_array_value.end(), max);
    for (size_t i = 0; i < results[0].double_array_value.size(); i++)
    {
        min[i] = -max[i];
    }
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getImpedance(int j, double* stiff, double* damp)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getImpedance] ";
    std::string suffix_tag = "[" + std::to_string(j) + "]";
    auto results = m_node->getParameters({ {compliant_stiffness_tag + suffix_tag, Type::PARAMETER_DOUBLE},
                                                                  {compliant_damping_tag + suffix_tag, Type::PARAMETER_DOUBLE} });
    if (results.size() != 2)
    {
        yCError(CB) << errorPrefix << "Error while getting impedance for joint" << j << ".";
        return false;
    }

    *stiff = results[0].double_value;
    *damp = results[1].double_value;
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getImpedanceOffset(int j, double* offset)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getImpedanceOffset] ";
    if (j < 0 || j >= static_cast<int>(m_compliantOffset.size()))
    {
        yCError(CB) << errorPrefix << "Joint index out of range. Got " << j << ", expected [0," << m_compliantOffset.size() - 1 << "]";
        return false;
    }
    *offset = m_compliantOffset[j];
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getCurrentImpedanceLimit(int j, double* min_stiff, double* max_stiff, double* min_damp, double* max_damp)
{
    yCError(CB) << "[getCurrentImpedanceLimit] It is not possible to get impedance limits in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getControlMode(int j, int* mode)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getControlMode] ";
    std::string suffix_tag = "[" + std::to_string(j) + "]";
    auto results = m_node->getParameters({ {control_modes_tag + suffix_tag, Type::PARAMETER_INTEGER} });
    if (results.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting control mode for joint" << j << ".";
        return false;
    }

    *mode = results[0].integer_value;

    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getControlModes(int* modes)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getControlModes] ";
    auto results = m_node->getParameters({ {control_modes_tag, Type::PARAMETER_INTEGER_ARRAY} });
    if (results.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting control modes.";
        return false;
    }
    for (size_t i = 0; i < results[0].integer_array_value.size(); i++)
    {
        modes[i] = results[0].integer_array_value[i];
    }
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getControlModes(const int n_joint, const int* joints, int* modes)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getControlModes] ";
    auto results = m_node->getParameters({ {control_modes_tag, Type::PARAMETER_INTEGER_ARRAY} });
    if (results.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting control modes.";
        return false;
    }

    for (int i = 0; i < n_joint; i++)
    {
        int j = joints[i];
        if (j < 0 || j >= static_cast<int>(results[0].integer_array_value.size()))
        {
            yCError(CB) << errorPrefix << "Index" << j << "out of range. Valid range is [0," << results[0].integer_array_value.size() - 1 << "]";
            return false;
        }
        modes[i] = results[0].integer_array_value[j];
    }

    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setControlMode(const int j, const int mode)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[setControlMode] ";
    std::string suffix_tag = "[" + std::to_string(j) + "]";
    rcl_interfaces::msg::Parameter control_mode_param;
    control_mode_param.name = control_modes_tag + suffix_tag;
    control_mode_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    control_mode_param.value.integer_value = mode;
    auto results = m_node->setParameters({ control_mode_param });
    if (results.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while setting control mode for joint" << j << ".";
        return false;
    }
    if (!results[0].successful)
    {
        yCError(CB) << errorPrefix << "Error while setting control mode for joint" << j << ":" << results[0].reason;
        return false;
    }
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setControlModes(const int n_joints, const int* joints, int* modes)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[setControlModes] ";
    std::vector<rcl_interfaces::msg::Parameter> control_mode_params;
    for (int i = 0; i < n_joints; i++)
    {
        int j = joints[i];
        int mode = modes[i];
        std::string suffix_tag = "[" + std::to_string(j) + "]";
        rcl_interfaces::msg::Parameter control_mode_param;
        control_mode_param.name = control_modes_tag + suffix_tag;
        control_mode_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
        control_mode_param.value.integer_value = mode;
        control_mode_params.push_back(control_mode_param);
    }
    auto results = m_node->setParameters(control_mode_params);
    if (results.size() != static_cast<size_t>(n_joints))
    {
        yCError(CB) << errorPrefix << "Error while setting control modes.";
        return false;
    }
    for (int i = 0; i < n_joints; i++)
    {
        if (!results[i].successful)
        {
            yCError(CB) << errorPrefix << "Error while setting control mode for joint" << joints[i] << ":" << results[i].reason;
            return false;
        }
    }
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setControlModes(int* modes)
{
    std::string errorPrefix = "[setControlModes] ";

    if (modes == nullptr)
    {
        yCError(CB) << errorPrefix << "modes is a null pointer.";
        return false;
    }

    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Services are not ready.";
        return false;
    }

    std::lock_guard<std::mutex> lock(m_mutex);

    size_t numberOfJoints = m_jointNames.size();

    rcl_interfaces::msg::Parameter control_mode_param;
    control_mode_param.name = control_modes_tag;
    control_mode_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY;
    control_mode_param.value.integer_array_value.resize(numberOfJoints);
    std::copy(modes, modes + numberOfJoints, control_mode_param.value.integer_array_value.begin());
    auto results = m_node->setParameters({ control_mode_param });
    if (results.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while setting control modes.";
        return false;
    }
    if (!results[0].successful)
    {
        yCError(CB) << errorPrefix << "Error while setting control modes:" << results[0].reason;
        return false;
    }
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setPosition(int j, double ref)
{
    std::string errorPrefix = "[setPosition] ";
    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Not ready to send references";
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        std::lock_guard<std::mutex> lock_reference(m_jointReferences.mutex);
        if (j < 0 || j >= static_cast<int>(m_jointReferences.name.size()))
        {
            yCError(CB) << errorPrefix << "Joint index out of range. Got " << j << ", expected [0," << m_jointReferences.name.size() - 1 << "]";
            return false;
        }
        m_jointReferences.position[j] = ref;
        if (m_compliant[j])
        {
            m_jointReferences.effort[j] = m_compliantOffset[j];
        }
        m_jointReferences.valid = true;
    }
    m_node->publishReferences(m_jointReferences);
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setPositions(const int n_joints, const int* joints, const double* dpos)
{
    std::string errorPrefix = "[setPositions] ";
    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Not ready to send references";
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        std::lock_guard<std::mutex> lock_reference(m_jointReferences.mutex);
        size_t numberOfJoints = m_jointReferences.name.size();
        for (int i = 0; i < n_joints; i++)
        {
            int j = joints[i];
            if (j < 0 || j >= static_cast<int>(numberOfJoints))
            {
                yCError(CB) << errorPrefix << "Joint index out of range. Got " << j << ", expected [0," << numberOfJoints - 1 << "]";
                return false;
            }
            m_jointReferences.position[j] = dpos[i];
            if (m_compliant[j])
            {
                m_jointReferences.effort[j] = m_compliantOffset[j];
            }
        }
        m_jointReferences.valid = true;
    }
    m_node->publishReferences(m_jointReferences);
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setPositions(const double* refs)
{
    std::string errorPrefix = "[setPositions] ";
    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Not ready to send references";
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        std::lock_guard<std::mutex> lock_reference(m_jointReferences.mutex);
        size_t numberOfJoints = m_jointReferences.name.size();
        for (size_t j = 0; j < numberOfJoints; j++)
        {
            m_jointReferences.position[j] = refs[j];
            if (m_compliant[j])
            {
                m_jointReferences.effort[j] = m_compliantOffset[j];
            }
        }
        m_jointReferences.valid = true;
    }
    m_node->publishReferences(m_jointReferences);
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefPosition(const int joint, double* ref)
{
    std::string errorPrefix = "[getRefPosition] ";

    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Services are not ready.";
        return false;
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    std::string suffix_tag = "[" + std::to_string(joint) + "]";
    auto results = m_node->getParameters({ {position_pid_references_tag + suffix_tag, Type::PARAMETER_DOUBLE} });
    if (results.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting position reference for joint" << joint << ".";
        return false;
    }
    *ref = results[0].double_value;
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefPositions(double* refs)
{
    std::string errorPrefix = "[getRefPositions] ";
    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Services are not ready.";
        return false;
    }
    std::lock_guard<std::mutex> lock(m_mutex);
    auto results = m_node->getParameters({ {position_pid_references_tag, Type::PARAMETER_DOUBLE_ARRAY} });
    if (results.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting position references.";
        return false;
    }
    if (results[0].double_array_value.size() != m_jointNames.size())
    {
        yCError(CB) << errorPrefix << "Size of position references does not match number of joints.";
        return false;
    }
    std::copy(results[0].double_array_value.begin(), results[0].double_array_value.end(), refs);
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefPositions(const int n_joint, const int* joints, double* refs)
{
    std::string errorPrefix = "[getRefPositions] ";
    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Services are not ready.";
        return false;
    }
    std::lock_guard<std::mutex> lock(m_mutex);
    auto results = m_node->getParameters({ {position_pid_references_tag, Type::PARAMETER_DOUBLE_ARRAY} });
    if (results.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting position references.";
        return false;
    }
    if (results[0].double_array_value.size() != m_jointNames.size())
    {
        yCError(CB) << errorPrefix << "Size of position references does not match number of joints.";
        return false;
    }
    for (int i = 0; i < n_joint; i++)
    {
        int j = joints[i];
        if (j < 0 || j >= static_cast<int>(results[0].double_array_value.size()))
        {
            yCError(CB) << errorPrefix << "Index" << j << "out of range. Valid range is [0," << results[0].double_array_value.size() - 1 << "]";
            return false;
        }
        refs[i] = results[0].double_array_value[j];
    }
    return true;
}

yarp::os::Stamp yarp::dev::IsaacSimControlBoardNWCROS2::getLastInputStamp()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getLastInputStamp] ";
    if (!m_jointState.valid.load())
    {
        yCError(CB) << errorPrefix << "No valid data received yet.";
        return yarp::os::Stamp();
    }
    std::lock_guard<std::mutex> lock_measurements(m_jointState.mutex);
    return m_jointState.timestamp;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::velocityMove(const int n_joints, const int* joints, const double* spds)
{
    std::string errorPrefix = "[velocityMove] ";
    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Not ready to send references";
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        std::lock_guard<std::mutex> lock_reference(m_jointReferences.mutex);
        size_t numberOfJoints = m_jointReferences.name.size();
        for (int i = 0; i < n_joints; i++)
        {
            int j = joints[i];
            if (j < 0 || j >= static_cast<int>(numberOfJoints))
            {
                yCError(CB) << errorPrefix << "Joint index out of range. Got " << j << ", expected [0," << numberOfJoints - 1 << "]";
                return false;
            }
            m_jointReferences.velocity[j] = spds[i];
        }
        m_jointReferences.valid = true;
    }
    m_node->publishReferences(m_jointReferences);
    return true;

}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefVelocity(const int joint, double* vel)
{
    std::string errorPrefix = "[getRefVelocity] ";
    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Services are not ready.";
        return false;
    }
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string suffix_tag = "[" + std::to_string(joint) + "]";
    auto results = m_node->getParameters({ {velocity_pid_references_tag + suffix_tag, Type::PARAMETER_DOUBLE} });
    if (results.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting velocity reference for joint" << joint << ".";
        return false;
    }
    *vel = results[0].double_value;
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefVelocities(double* vels)
{
    std::string errorPrefix = "[getRefVelocities] ";
    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Services are not ready.";
        return false;
    }
    std::lock_guard<std::mutex> lock(m_mutex);
    auto results = m_node->getParameters({ {velocity_pid_references_tag, Type::PARAMETER_DOUBLE_ARRAY} });
    if (results.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting velocity references.";
        return false;
    }
    if (results[0].double_array_value.size() != m_jointNames.size())
    {
        yCError(CB) << errorPrefix << "Size of velocity references does not match number of joints.";
        return false;
    }
    std::copy(results[0].double_array_value.begin(), results[0].double_array_value.end(), vels);
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefVelocities(const int n_joint, const int* joints, double* vels)
{
    std::string errorPrefix = "[getRefVelocities] ";
    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Services are not ready.";
        return false;
    }
    std::lock_guard<std::mutex> lock(m_mutex);
    auto results = m_node->getParameters({ {velocity_pid_references_tag, Type::PARAMETER_DOUBLE_ARRAY} });
    if (results.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting velocity references.";
        return false;
    }
    if (results[0].double_array_value.size() != m_jointNames.size())
    {
        yCError(CB) << errorPrefix << "Size of velocity references does not match number of joints.";
        return false;
    }
    for (int i = 0; i < n_joint; i++)
    {
        int j = joints[i];
        if (j < 0 || j >= static_cast<int>(results[0].double_array_value.size()))
        {
            yCError(CB) << errorPrefix << "Index" << j << "out of range. Valid range is [0," << results[0].double_array_value.size() - 1 << "]";
            return false;
        }
        vels[i] = results[0].double_array_value[j];
    }
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getInteractionMode(int j, yarp::dev::InteractionModeEnum* mode)
{
    std::string errorPrefix = "[getInteractionMode] ";
    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Services are not ready.";
        return false;
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    std::string suffix_tag = "[" + std::to_string(j) + "]";

    if (j < 0 || j >= static_cast<int>(m_compliant.size()))
    {
        yCError(CB) << errorPrefix << "Index" << j << "out of range. Valid range is [0," << m_compliant.size() - 1 << "]";
        return false;
    }

    auto results = m_node->getParameters({ {compliant_modes_tag + suffix_tag, Type::PARAMETER_BOOL} });
    if (results.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting interaction mode for joint" << j << ".";
        return false;
    }
    bool compliant = results[0].bool_value;

    *mode = compliant ? yarp::dev::InteractionModeEnum::VOCAB_IM_COMPLIANT : yarp::dev::InteractionModeEnum::VOCAB_IM_STIFF;
    m_compliant[j] = compliant;

    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getInteractionModes(int n_joints, int* joints, yarp::dev::InteractionModeEnum* modes)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getInteractionModes] ";
    auto results = m_node->getParameters({ {compliant_modes_tag, Type::PARAMETER_BOOL_ARRAY} });
    if (results.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting interaction modes.";
        return false;
    }

    m_compliant = results[0].bool_array_value; //Update the cached value
    m_compliantOffset.resize(m_compliant.size(), 0.0);

    for (int i = 0; i < n_joints; i++)
    {
        int j = joints[i];
        if (j < 0 || j >= static_cast<int>(m_compliant.size()))
        {
            yCError(CB) << errorPrefix << "Index" << j << "out of range. Valid range is [0," << m_compliant.size() - 1 << "]";
            return false;
        }
        modes[i] = m_compliant[j] ? yarp::dev::InteractionModeEnum::VOCAB_IM_COMPLIANT : yarp::dev::InteractionModeEnum::VOCAB_IM_STIFF;
    }
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getInteractionModes(yarp::dev::InteractionModeEnum* modes)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getInteractionModes] ";
    auto results = m_node->getParameters({ {compliant_modes_tag, Type::PARAMETER_BOOL_ARRAY} });
    if (results.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting interaction modes.";
        return false;
    }

    m_compliant = results[0].bool_array_value; //Update the cached value
    m_compliantOffset.resize(m_compliant.size(), 0.0);

    for (size_t i = 0; i < m_compliant.size(); i++)
    {
        modes[i] = m_compliant[i] ? yarp::dev::InteractionModeEnum::VOCAB_IM_COMPLIANT : yarp::dev::InteractionModeEnum::VOCAB_IM_STIFF;
    }
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setInteractionMode(int j, yarp::dev::InteractionModeEnum mode)
{
    std::string errorPrefix = "[setInteractionMode] ";

    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Services are not ready.";
        return false;
    }

    bool compliant = (mode == yarp::dev::InteractionModeEnum::VOCAB_IM_COMPLIANT);
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string suffix_tag = "[" + std::to_string(j) + "]";

    if (j < 0 || j >= static_cast<int>(m_compliant.size()))
    {
        yCError(CB) << errorPrefix << "Index" << j << "out of range. Valid range is [0," << m_compliant.size() - 1 << "]";
        return false;
    }

    rcl_interfaces::msg::Parameter compliant_mode_param;
    compliant_mode_param.name = compliant_modes_tag + suffix_tag;
    compliant_mode_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    compliant_mode_param.value.bool_value = compliant;
    auto results = m_node->setParameters({ compliant_mode_param });
    if (results.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while setting interaction mode for joint" << j << ".";
        return false;
    }
    if (!results[0].successful)
    {
        yCError(CB) << errorPrefix << "Error while setting interaction mode for joint" << j << ":" << results[0].reason;
        return false;
    }

    m_compliant[j] = compliant;
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setInteractionModes(int n_joints, int* joints, yarp::dev::InteractionModeEnum* modes)
{
    std::string errorPrefix = "[setInteractionModes] ";

    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Services are not ready.";
        return false;
    }

    std::vector<rcl_interfaces::msg::SetParametersResult> results;
    std::lock_guard<std::mutex> lock(m_mutex);
    std::vector<rcl_interfaces::msg::Parameter> compliant_mode_params;
    for (int i = 0; i < n_joints; i++)
    {
        int j = joints[i];

        if (j < 0 || j >= static_cast<int>(m_compliant.size()))
        {
            yCError(CB) << errorPrefix << "Index" << j << "out of range. Valid range is [0," << m_compliant.size() - 1 << "]";
            return false;
        }

        yarp::dev::InteractionModeEnum mode = modes[i];
        std::string suffix_tag = "[" + std::to_string(j) + "]";
        rcl_interfaces::msg::Parameter compliant_mode_param;
        compliant_mode_param.name = compliant_modes_tag + suffix_tag;
        compliant_mode_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
        compliant_mode_param.value.bool_value = (mode == yarp::dev::InteractionModeEnum::VOCAB_IM_COMPLIANT);
        compliant_mode_params.push_back(compliant_mode_param);
    }
    results = m_node->setParameters(compliant_mode_params);
    if (results.size() != static_cast<size_t>(n_joints))
    {
        yCError(CB) << errorPrefix << "Error while setting interaction modes.";
        return false;
    }
    bool success = true;
    for (int i = 0; i < n_joints; i++)
    {
        if (results[i].successful)
        {
            int j = joints[i];
            yarp::dev::InteractionModeEnum mode = modes[i];
            m_compliant[j] = (mode == yarp::dev::InteractionModeEnum::VOCAB_IM_COMPLIANT);
            m_compliantOffset[j] = 0.0;
        }
        else
        {
            // How to update cached values
            yCError(CB) << errorPrefix << "Error while setting interaction mode for joint" << joints[i] << ":" << results[i].reason;
            success = false;
        }
    }

    return success;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setInteractionModes(yarp::dev::InteractionModeEnum* modes)
{
    std::string errorPrefix = "[setInteractionModes] ";
    if (modes == nullptr)
    {
        yCError(CB) << errorPrefix << "modes is a null pointer.";
        return false;
    }
    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Services are not ready.";
        return false;
    }

    std::lock_guard<std::mutex> lock(m_mutex);

    size_t numberOfJoints = m_compliant.size();
    rcl_interfaces::msg::Parameter compliant_mode_param;
    compliant_mode_param.name = compliant_modes_tag;
    compliant_mode_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY;
    compliant_mode_param.value.bool_array_value.resize(numberOfJoints);
    for (size_t i = 0; i < numberOfJoints; i++)
    {
        compliant_mode_param.value.bool_array_value[i] = (modes[i] == yarp::dev::InteractionModeEnum::VOCAB_IM_COMPLIANT);
    }
    auto results = m_node->setParameters({ compliant_mode_param });
    if (results.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while setting interaction modes.";
        return false;
    }
    if (!results[0].successful)
    {
        yCError(CB) << errorPrefix << "Error while setting interaction modes:" << results[0].reason;
        return false;
    }

    m_compliant = compliant_mode_param.value.bool_array_value; //Update the cached value
    std::fill(m_compliantOffset.begin(), m_compliantOffset.end(), 0.0);
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRefDutyCycle(int m, double ref)
{
    yCError(CB) << "[setRefDutyCycle] It is not possible to set a reference duty cycle in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRefDutyCycles(const double* refs)
{
    yCError(CB) << "[setRefDutyCycles] It is not possible to set reference duty cycles in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefDutyCycle(int m, double* ref)
{
    yCErrorOnce(CB) << "[getRefDutyCycle] It is not possible to get a reference duty cycle in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefDutyCycles(double* refs)
{
    yCErrorOnce(CB) << "[getRefDutyCycles] It is not possible to get reference duty cycles in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getDutyCycle(int m, double* val)
{
    yCErrorOnce(CB) << "[getDutyCycle] It is not possible to get a duty cycle in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getDutyCycles(double* vals)
{
    yCErrorOnce(CB) << "[getDutyCycles] It is not possible to get duty cycles in Isaac sim.";
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getCurrent(int m, double* curr)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getCurrent] ";
    if (!m_motorState.valid.load())
    {
        yCError(CB) << errorPrefix << "No valid data received yet.";
        return false;
    }
    std::lock_guard<std::mutex> lock_measurements(m_motorState.mutex);
    if (m < 0 || m >= static_cast<int>(m_motorState.effort.size()))
    {
        yCError(CB) << errorPrefix << "Index" << m << "out of range. Valid range is [0," << m_motorState.effort.size() - 1 << "]";
        return false;
    }
    *curr = m_motorState.effort[m];
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getCurrents(double* currs)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getCurrents] ";
    if (!m_motorState.valid.load())
    {
        yCError(CB) << errorPrefix << "No valid data received yet.";
        return false;
    }
    std::lock_guard<std::mutex> lock_measurements(m_motorState.mutex);
    std::copy(m_motorState.effort.begin(), m_motorState.effort.end(), currs);
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getCurrentRange(int m, double* min, double* max)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getCurrentRange] ";
    std::string suffix_tag = "[" + std::to_string(m) + "]";
    auto results = m_node->getParameters({ {motor_max_currents_tag + suffix_tag, Type::PARAMETER_DOUBLE} });
    if (results.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting max current for motor" << m << ".";
        return false;
    }

    *max = results[0].double_value;
    *min = -(*max);
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getCurrentRanges(double* min, double* max)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getCurrentRanges] ";
    auto results = m_node->getParameters({ {motor_max_currents_tag, Type::PARAMETER_DOUBLE_ARRAY} });
    if (results.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting max currents.";
        return false;
    }

    if (min == nullptr || max == nullptr)
    {
        yCError(CB) << errorPrefix << "min or max is a null pointer.";
        return false;
    }

    std::copy(results[0].double_array_value.begin(), results[0].double_array_value.end(), max);
    for (size_t i = 0; i < results[0].double_array_value.size(); i++)
    {
        min[i] = -max[i];
    }
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRefCurrents(const double* currs)
{
    std::string errorPrefix = "[setRefCurrents] ";
    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Not ready to send references";
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        std::lock_guard<std::mutex> lock_reference(m_jointReferences.mutex);
        size_t numberOfJoints = m_jointReferences.name.size();
        for (size_t j = 0; j < numberOfJoints; j++)
        {
            m_jointReferences.effort[j] = currs[j];
        }
        m_jointReferences.valid = true;
    }
    m_node->publishReferences(m_jointReferences);
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRefCurrent(int m, double curr)
{
    std::string errorPrefix = "[setRefCurrent] ";
    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Not ready to send references";
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        std::lock_guard<std::mutex> lock_reference(m_jointReferences.mutex);
        if (m < 0 || m >= static_cast<int>(m_jointReferences.name.size()))
        {
            yCError(CB) << errorPrefix << "Joint index out of range. Got " << m << ", expected [0," << m_jointReferences.name.size() - 1 << "]";
            return false;
        }
        m_jointReferences.effort[m] = curr;
        m_jointReferences.valid = true;
    }
    m_node->publishReferences(m_jointReferences);
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRefCurrents(const int n_motor, const int* motors, const double* currs)
{
    std::string errorPrefix = "[setRefCurrents] ";
    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Not ready to send references";
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        std::lock_guard<std::mutex> lock_reference(m_jointReferences.mutex);
        size_t numberOfJoints = m_jointReferences.name.size();
        for (int i = 0; i < n_motor; i++)
        {
            int j = motors[i];
            if (j < 0 || j >= static_cast<int>(numberOfJoints))
            {
                yCError(CB) << errorPrefix << "Joint index out of range. Got " << j << ", expected [0," << numberOfJoints - 1 << "]";
                return false;
            }
            m_jointReferences.effort[j] = currs[i];
        }
        m_jointReferences.valid = true;
    }
    m_node->publishReferences(m_jointReferences);
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefCurrents(double* currs)
{
    std::string errorPrefix = "[getRefCurrents] ";
    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Services are not ready.";
        return false;
    }
    std::lock_guard<std::mutex> lock(m_mutex);
    auto results = m_node->getParameters({ {current_pid_references_tag, Type::PARAMETER_DOUBLE_ARRAY} });
    if (results.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting effort references.";
        return false;
    }
    if (results[0].double_array_value.size() != m_jointNames.size())
    {
        yCError(CB) << errorPrefix << "Size of effort references does not match number of joints.";
        return false;
    }
    std::copy(results[0].double_array_value.begin(), results[0].double_array_value.end(), currs);
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefCurrent(int m, double* curr)
{
    std::string errorPrefix = "[getRefCurrent] ";
    if (!m_ready)
    {
        yCError(CB) << errorPrefix << "Services are not ready.";
        return false;
    }
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string suffix_tag = "[" + std::to_string(m) + "]";
    auto results = m_node->getParameters({ {current_pid_references_tag + suffix_tag, Type::PARAMETER_DOUBLE} });
    if (results.size() != 1)
    {
        yCError(CB) << errorPrefix << "Error while getting effort reference for joint" << m << ".";
        return false;
    }
    *curr = results[0].double_value;
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setup()
{
    if (m_ready)
    {
        return m_ready;
    }

    std::lock_guard<std::mutex> lock(m_mutex);

    std::string errorPrefix = "[setup] ";

    if (!m_node->waitServicesAvailable())
    {
        yCError(CB) << errorPrefix << "Not all services are available.";
        m_ready = false;
        return m_ready;
    }

    // The impedance offset is set continuosly from outside,
    // but this is using the effort part in the refences message.
    // So we need to keep track of which joints are actually in
    // compliant mode to avoid overwriting a real effort reference.

    auto result = m_node->getParameters({ {joint_names_tag, Type::PARAMETER_STRING_ARRAY},
                                          {joint_types_tag, Type::PARAMETER_INTEGER_ARRAY},
                                          {compliant_modes_tag, Type::PARAMETER_BOOL_ARRAY} });
    if (result.size() != 2)
    {
        yCError(CB) << errorPrefix << "Error while getting joint names and types.";
        m_ready = false;
        return m_ready;
    }

    if (result[0].string_array_value.size() != result[1].integer_array_value.size())
    {
        yCError(CB) << errorPrefix << "Joint names and types have different size.";
        m_ready = false;
        return m_ready;
    }

    if (result[0].string_array_value.size() != result[2].bool_array_value.size())
    {
        yCError(CB) << errorPrefix << "Joint names and compliant modes have different size.";
        m_ready = false;
        return m_ready;
    }

    m_jointNames = result[0].string_array_value;
    m_compliant = result[2].bool_array_value;
    m_compliantOffset.resize(m_compliant.size(), 0.0);

    {
        std::lock_guard<std::mutex> lock_measurements(m_jointState.mutex);
        m_jointState.jointTypes = result[1].integer_array_value;
    }

    {
        std::lock_guard<std::mutex> lock_references(m_jointReferences.mutex);
        m_jointReferences.name = m_jointNames;
        m_jointReferences.jointTypes = result[1].integer_array_value;
        m_jointReferences.resize();
    }

    m_ready = true;
    return m_ready;
}

void yarp::dev::IsaacSimControlBoardNWCROS2::updateJointMeasurements(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> lock(m_jointState.mutex);

    m_jointState.convert_to_vectors(msg);

    if (m_jointState.valid && m_ready && m_jointState.name.size() != m_jointNames.size())
    {
        yCWarning(CB) << "[updateJointMeasurements] Number of joints in the received message (" << m_jointState.name.size()
            << ") is different from the expected one (" << m_jointNames.size() << "). Stopping..";
        m_ready = false;
    }
}

void yarp::dev::IsaacSimControlBoardNWCROS2::updateMotorMeasurements(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> lock(m_motorState.mutex);
    // Assuming all the motors are revolute
    m_motorState.convert_to_vectors(msg);
}

void yarp::dev::IsaacSimControlBoardNWCROS2::JointsState::convert_to_vectors(const sensor_msgs::msg::JointState::ConstSharedPtr& js)
{
    bool useConversions = jointTypes.size() == js->name.size();
    name = js->name;
    position = js->position;
    velocity = js->velocity;
    if (useConversions) {
        for (size_t i = 0; i < position.size(); i++) {
            convert_to_deg_if_revolute(i, position[i], position[i]);
            convert_to_deg_if_revolute(i, velocity[i], velocity[i]);
            // Assuming that there is no conversion for linear joints
        }
    }
    effort = js->effort;
    timestamp.update(js->header.stamp.sec + js->header.stamp.nanosec * 1e-9);
    valid = true;
}

void yarp::dev::IsaacSimControlBoardNWCROS2::JointsState::convert_to_msg(sensor_msgs::msg::JointState& js) const
{
    bool useConversions = jointTypes.size() == name.size();
    js.name = name;
    js.position = position;
    js.velocity = velocity;
    if (useConversions) {
        for (size_t i = 0; i < position.size(); i++) {
            convert_to_rad_if_revolute(i, position[i], js.position[i]);
            convert_to_rad_if_revolute(i, velocity[i], js.velocity[i]);
            // Assuming that there is no conversion for linear joints
        }
    }
    js.effort = effort;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::JointsState::convert_to_deg_if_revolute(size_t index, double input, double& output) const
{
    if (index >= jointTypes.size())
    {
        return false;
    }
    if (jointTypes[index] == 0) // Revolute joint
    {
        output = input * rad2deg;
        return true;
    }
    output = input;
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::JointsState::convert_to_rad_if_revolute(size_t index, double input, double& output) const
{
    if (index >= jointTypes.size())
    {
        return false;
    }
    if (jointTypes[index] == 0) // Revolute joint
    {
        output = input * deg2rad;
        return true;
    }
    output = input;
    return true;
}

void yarp::dev::IsaacSimControlBoardNWCROS2::JointsState::resize()
{
    position.resize(name.size());
    velocity.resize(name.size());
    effort.resize(name.size());
    invalidate();
}

void yarp::dev::IsaacSimControlBoardNWCROS2::JointsState::invalidate()
{
    std::fill(position.begin(), position.end(), std::numeric_limits<double>::quiet_NaN());
    std::fill(velocity.begin(), velocity.end(), std::numeric_limits<double>::quiet_NaN());
    std::fill(effort.begin(), effort.end(), std::numeric_limits<double>::quiet_NaN());
    valid = false;
}

yarp::dev::IsaacSimControlBoardNWCROS2::CBNode::CBNode(const std::string& node_name,
                                                       const std::string& joint_state_topic_name,
                                                       const std::string& motor_state_topic_name,
                                                       const std::string& joint_references_topic_name,
                                                       const std::string& get_param_service_name,
                                                       const std::string& set_param_service_name,
                                                       double requests_timeout_sec,
                                                       IsaacSimControlBoardNWCROS2* parent)
: rclcpp::Node(node_name)
{
    m_jointStateSubscription = this->create_subscription<sensor_msgs::msg::JointState>(
        joint_state_topic_name, 10,
        [parent](const sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        parent->updateJointMeasurements(msg);
    });
    m_motorStateSubscription = this->create_subscription<sensor_msgs::msg::JointState>(
        motor_state_topic_name, 10,
        [parent](const sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        parent->updateMotorMeasurements(msg);
    });
    m_referencesPublisher = this->create_publisher<sensor_msgs::msg::JointState>(joint_references_topic_name, 10);
    m_getParamClient = this->create_client<rcl_interfaces::srv::GetParameters>(get_param_service_name);
    m_setParamClient = this->create_client<rcl_interfaces::srv::SetParameters>(set_param_service_name);

    m_requestsTimeout = std::chrono::duration<double>(requests_timeout_sec);
}

std::vector<rcl_interfaces::msg::ParameterValue> yarp::dev::IsaacSimControlBoardNWCROS2::CBNode::getParameters(const Parameters& parameters)
{
    std::string errorPrefix = "[getParameters] ";
    auto get_request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    get_request->names.resize(parameters.size());
    for (size_t i = 0; i < parameters.size(); i++)
    {
        get_request->names[i] = parameters[i].first;
    }

    auto result = m_getParamClient->async_send_request(get_request);

    if (result.wait_for(m_requestsTimeout) != std::future_status::ready)
    {
        yCError(CB) << errorPrefix << "Service call timed out";
        return std::vector<rcl_interfaces::msg::ParameterValue>();
    }

    auto& values = result.get()->values;
    if (values.size() != parameters.size())
    {
        yCError(CB) << errorPrefix << "Unexpected number of results. Expected" << parameters.size() << "but got" << values.size();
        return std::vector<rcl_interfaces::msg::ParameterValue>();
    }

    for (size_t i = 0; i < parameters.size(); i++)
    {
        if (values[i].type == Type::PARAMETER_NOT_SET)
        {
            yCError(CB) << errorPrefix << "Parameter" << parameters[i].first << "not set.";
            return std::vector<rcl_interfaces::msg::ParameterValue>();
        }

        if (parameters[i].second != Type::PARAMETER_NOT_SET && values[i].type != parameters[i].second)
        {
            yCError(CB) << errorPrefix << "Unexpected type for parameter" << parameters[i].first << ". Expected" << parameters[i].second << "but got" << values[i].type;
            return std::vector<rcl_interfaces::msg::ParameterValue>();
        }
    }
    return values;
}


std::vector<rcl_interfaces::msg::SetParametersResult> yarp::dev::IsaacSimControlBoardNWCROS2::CBNode::setParameters(const std::vector<rcl_interfaces::msg::Parameter>& params)
{
    auto set_request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    set_request->parameters = params;
    auto result = m_setParamClient->async_send_request(set_request);
    if (result.wait_for(m_requestsTimeout) != std::future_status::ready)
    {
        yCError(CB) << "[setParameters] Service call timed out";
        return std::vector<rcl_interfaces::msg::SetParametersResult>();
    }
    return result.get()->results;
}

void yarp::dev::IsaacSimControlBoardNWCROS2::CBNode::publishReferences(JointsState& msg)
{
    if (!msg.valid.load())
    {
        return;
    }
    std::lock_guard<std::mutex> lock(msg.mutex);
    msg.convert_to_msg(m_referencesMessageBuffer);
    m_referencesMessageBuffer.header.stamp = this->now();
    m_referencesPublisher->publish(m_referencesMessageBuffer);
    msg.invalidate();
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::CBNode::waitServicesAvailable()
{
    return m_getParamClient->wait_for_service(m_requestsTimeout * 10) &&
           m_setParamClient->wait_for_service(m_requestsTimeout * 10);
}
