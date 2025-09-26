// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-2-Clause
#include "IsaacSimControlBoardNWCROS2.h"
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>

YARP_DECLARE_LOG_COMPONENT(CB)
YARP_LOG_COMPONENT(CB, "yarp.device.IsaacSimControlBoardNWCROS2")

constexpr double rad2deg = 180.0 / M_PI;

yarp::dev::IsaacSimControlBoardNWCROS2::~IsaacSimControlBoardNWCROS2()
{
    close();
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::open(yarp::os::Searchable& config)
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
                                      m_paramsParser.m_get_parameters_service_name,
                                      m_paramsParser.m_set_parameters_service_name,
                                      m_paramsParser.m_service_request_timeout,
                                      this);
    m_executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    m_executor->add_node(m_node);
    m_executorThread = std::thread([this]() { m_executor->spin(); });

    //TODO check if the services are available and get the control modes and compliant states of the joints

    return true;
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
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setPid(const yarp::dev::PidControlTypeEnum& pidtype, int j, const yarp::dev::Pid& p)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setPids(const yarp::dev::PidControlTypeEnum& pidtype, const yarp::dev::Pid* ps)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setPidReference(const yarp::dev::PidControlTypeEnum& pidtype, int j, double ref)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setPidReferences(const yarp::dev::PidControlTypeEnum& pidtype, const double* refs)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setPidErrorLimit(const yarp::dev::PidControlTypeEnum& pidtype, int j, double limit)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setPidErrorLimits(const yarp::dev::PidControlTypeEnum& pidtype, const double* limits)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getPidError(const yarp::dev::PidControlTypeEnum& pidtype, int j, double* err)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getPidErrors(const yarp::dev::PidControlTypeEnum& pidtype, double* errs)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getPidOutput(const yarp::dev::PidControlTypeEnum& pidtype, int j, double* out)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getPidOutputs(const yarp::dev::PidControlTypeEnum& pidtype, double* outs)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setPidOffset(const yarp::dev::PidControlTypeEnum& pidtype, int j, double v)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getPid(const yarp::dev::PidControlTypeEnum& pidtype, int j, yarp::dev::Pid* p)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getPids(const yarp::dev::PidControlTypeEnum& pidtype, yarp::dev::Pid* pids)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getPidReference(const yarp::dev::PidControlTypeEnum& pidtype, int j, double* ref)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getPidReferences(const yarp::dev::PidControlTypeEnum& pidtype, double* refs)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getPidErrorLimit(const yarp::dev::PidControlTypeEnum& pidtype, int j, double* limit)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getPidErrorLimits(const yarp::dev::PidControlTypeEnum& pidtype, double* limits)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::resetPid(const yarp::dev::PidControlTypeEnum& pidtype, int j)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::disablePid(const yarp::dev::PidControlTypeEnum& pidtype, int j)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::enablePid(const yarp::dev::PidControlTypeEnum& pidtype, int j)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::isPidEnabled(const yarp::dev::PidControlTypeEnum& pidtype, int j, bool* enabled)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getAxes(int* ax)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getAxes] ";
    if (!m_jointState.valid.load())
    {
        yCError(CB) << errorPrefix << "No valid data received yet.";
        return false;
    }

    std::lock_guard<std::mutex> lock_measurements(m_jointState.mutex);

    *ax = static_cast<int>(m_jointState.name.size());
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::positionMove(int j, double ref)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::positionMove(const double* refs)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::positionMove(const int n_joints, const int* joints, const double* refs)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getTargetPosition(const int joint, double* ref)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getTargetPositions(double* refs)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getTargetPositions(const int n_joint, const int* joints, double* refs)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::relativeMove(int j, double delta)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::relativeMove(const double* deltas)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::relativeMove(const int n_joints, const int* joints, const double* deltas)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::checkMotionDone(int j, bool* flag)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::checkMotionDone(bool* flag)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::checkMotionDone(const int n_joints, const int* joints, bool* flags)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRefSpeed(int j, double sp)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRefSpeeds(const double* spds)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRefSpeeds(const int n_joints, const int* joints, const double* spds)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRefAcceleration(int j, double acc)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRefAccelerations(const double* accs)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRefAccelerations(const int n_joints, const int* joints, const double* accs)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefSpeed(int j, double* ref)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefSpeeds(double* spds)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefSpeeds(const int n_joints, const int* joints, double* spds)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefAcceleration(int j, double* acc)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefAccelerations(double* accs)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefAccelerations(const int n_joints, const int* joints, double* accs)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::stop(int j)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::stop()
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::stop(const int n_joints, const int* joints)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getLastJointFault(int j, int& fault, std::string& message)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::velocityMove(int j, double v)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::velocityMove(const double* v)
{
    return false;
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
    std::fill(t, t + m_jointState.position.size(), m_jointState.timestamp);

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
    *t = m_jointState.timestamp;
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
    //TODO: this could be the gear ratio
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getMotorEncoderCountsPerRevolution(int m, double* cpr)
{
    //TODO: this could be the gear ratio
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
    std::fill(t, t + m_motorState.position.size(), m_motorState.timestamp);
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
    *t = m_motorState.timestamp;
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
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getMaxCurrent(int j, double* v)
{
    return false;
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
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getLimits(int j, double* min, double* max)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setVelLimits(int j, double min, double max)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getVelLimits(int j, double* min, double* max)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRemoteVariable(std::string key, yarp::os::Bottle& val)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRemoteVariable(std::string key, const yarp::os::Bottle& val)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRemoteVariablesList(yarp::os::Bottle* listOfKeys)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::isCalibratorDevicePresent(bool* isCalib)
{
    return false;
}

yarp::dev::IRemoteCalibrator* yarp::dev::IsaacSimControlBoardNWCROS2::getCalibratorDevice()
{
    return nullptr;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::calibrateSingleJoint(int j)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::calibrateWholePart()
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::homingSingleJoint(int j)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::homingWholePart()
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::parkSingleJoint(int j, bool _wait)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::parkWholePart()
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::quitCalibrate()
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::quitPark()
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::calibrateAxisWithParams(int j, unsigned int ui, double v1, double v2, double v3)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setCalibrationParameters(int j, const yarp::dev::CalibrationParameters& params)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::calibrationDone(int j)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::abortPark()
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::abortCalibration()
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getNumberOfMotors(int* num)
{
    return false;
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
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setGearboxRatio(int m, const double val)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getAxisName(int j, std::string& name)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getAxisName] ";
    if (!m_jointState.valid.load())
    {
        yCError(CB) << errorPrefix << "No valid data received yet.";
        return false;
    }

    std::lock_guard<std::mutex> lock_measurements(m_jointState.mutex);
    if (j < 0 || j >= static_cast<int>(m_jointState.name.size()))
    {
        yCError(CB) << errorPrefix << "Index" << j << "out of range. Valid range is [0," << m_jointState.name.size() - 1 << "]";
        return false;
    }
    name = m_jointState.name[j];
    return true;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getJointType(int j, yarp::dev::JointTypeEnum& type)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefTorques(double* refs)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefTorque(int j, double* t)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRefTorques(const double* t)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRefTorque(int j, double t)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRefTorques(const int n_joint, const int* joints, const double* t)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getMotorTorqueParams(int j, yarp::dev::MotorTorqueParameters* params)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setMotorTorqueParams(int j, const yarp::dev::MotorTorqueParameters params)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setImpedance(int j, double stiff, double damp)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setImpedanceOffset(int j, double offset)
{
    return false;
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
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getTorqueRanges(double* min, double* max)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getImpedance(int j, double* stiff, double* damp)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getImpedanceOffset(int j, double* offset)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getCurrentImpedanceLimit(int j, double* min_stiff, double* max_stiff, double* min_damp, double* max_damp)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getControlMode(int j, int* mode)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getControlModes(int* modes)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getControlModes(const int n_joint, const int* joints, int* modes)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setControlMode(const int j, const int mode)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setControlModes(const int n_joints, const int* joints, int* modes)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setControlModes(int* modes)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setPosition(int j, double ref)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setPositions(const int n_joints, const int* joints, const double* dpos)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setPositions(const double* refs)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefPosition(const int joint, double* ref)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefPositions(double* refs)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefPositions(const int n_joint, const int* joints, double* refs)
{
    return false;
}

yarp::os::Stamp yarp::dev::IsaacSimControlBoardNWCROS2::getLastInputStamp()
{
    return yarp::os::Stamp();
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::velocityMove(const int n_joints, const int* joints, const double* spds)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefVelocity(const int joint, double* vel)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefVelocities(double* vels)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefVelocities(const int n_joint, const int* joints, double* vels)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getInteractionMode(int j, yarp::dev::InteractionModeEnum* mode)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getInteractionModes(int n_joints, int* joints, yarp::dev::InteractionModeEnum* modes)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getInteractionModes(yarp::dev::InteractionModeEnum* modes)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setInteractionMode(int j, yarp::dev::InteractionModeEnum mode)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setInteractionModes(int n_joints, int* joints, yarp::dev::InteractionModeEnum* modes)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setInteractionModes(yarp::dev::InteractionModeEnum* modes)
{
    return false;
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
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getCurrentRanges(double* min, double* max)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRefCurrents(const double* currs)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRefCurrent(int m, double curr)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::setRefCurrents(const int n_motor, const int* motors, const double* currs)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefCurrents(double* currs)
{
    return false;
}

bool yarp::dev::IsaacSimControlBoardNWCROS2::getRefCurrent(int m, double* curr)
{
    return false;
}

void yarp::dev::IsaacSimControlBoardNWCROS2::updateJointMeasurements(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> lock(m_jointState.mutex);

    m_jointState.convert_to_vectors(msg);
}

void yarp::dev::IsaacSimControlBoardNWCROS2::updateMotorMeasurements(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> lock(m_motorState.mutex);

    m_motorState.convert_to_vectors(msg);
}

void yarp::dev::IsaacSimControlBoardNWCROS2::JointsState::convert_to_vectors(const sensor_msgs::msg::JointState::ConstSharedPtr& js)
{
    name = js->name;
    position = js->position;
    for (auto& pos : position) {
        pos *= rad2deg;
    }
    velocity = js->velocity;
    for (auto& vel : velocity) {
        vel *= rad2deg;
    }
    effort = js->effort;
    timestamp = js->header.stamp.sec + js->header.stamp.nanosec * 1e-9;
    valid = true;
}
yarp::dev::IsaacSimControlBoardNWCROS2::CBNode::CBNode(const std::string& node_name,
                                                       const std::string& joint_state_topic_name,
                                                       const std::string& motor_state_topic_name,
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
    m_getParamClient = this->create_client<rcl_interfaces::srv::GetParameters>(get_param_service_name);
    m_setParamClient = this->create_client<rcl_interfaces::srv::SetParameters>(set_param_service_name);

    // convert double to chrono
    m_requestsTimeout = std::chrono::duration<double>(requests_timeout_sec);
}

std::vector<rcl_interfaces::msg::ParameterValue> yarp::dev::IsaacSimControlBoardNWCROS2::CBNode::getParameters(const std::vector<std::string>& names)
{
    auto get_request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    get_request->names = names;

    auto result = m_getParamClient->async_send_request(get_request);

    if (result.wait_for(m_requestsTimeout) != std::future_status::ready)
    {
        yCError(CB) << "[getParameters] Service call timed out";
        return std::vector<rcl_interfaces::msg::ParameterValue>();
    }

    return result.get()->values;
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
