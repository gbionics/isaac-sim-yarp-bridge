// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-2-Clause
#include "IsaacSimMultipleAnalogSensorsNWCROS2.h"
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Matrix.h>

YARP_DECLARE_LOG_COMPONENT(MAS)
YARP_LOG_COMPONENT(MAS, "yarp.device.IsaacSimMultipleAnalogSensorsNWCROS2")

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::waitForData(double timeout)
{
    double elapsed_time = 0.0;
    const double wait_time_s = 0.1;

    if (timeout < 0.0)
    {
        return true;
    }

    while (elapsed_time <= timeout)
    {
        {
            bool all_imus_valid = true;
            for (const auto& imu : m_imus)
            {
                if (!imu.valid)
                {
                    all_imus_valid = false;
                    break;
                }
            }
            bool all_fts_valid = true;
            for (const auto& ft : m_fts)
            {
                if (!ft.valid)
                {
                    all_fts_valid = false;
                    break;
                }
            }
            if (all_imus_valid && all_fts_valid)
            {
                return true;
            }
        }
        yarp::os::Time::delay(wait_time_s);
        elapsed_time += wait_time_s;
    }

    yCError(MAS) << "[waitForData] Did not receive data from one or more topic for" << timeout << "seconds.";
    return false;
}

yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::~IsaacSimMultipleAnalogSensorsNWCROS2()
{
    close();
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::open(yarp::os::Searchable& config)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[open] ";

    if (!m_paramsParser.parseParams(config))
    {
        yCError(MAS) << errorPrefix << "Failed to parse parameters for IsaacSimMultipleAnalogSensorsNWCROS2";
        return false;
    }

    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }

    m_subscriber = std::make_shared<MASSubscriber>(m_paramsParser.m_node_name, m_paramsParser.m_imu_topic_names,
                                                   m_paramsParser.m_ft_topic_names, this);
    m_executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    m_executor->add_node(m_subscriber);
    m_executorThread = std::thread([this]() { m_executor->spin(); });

    return waitForData(m_paramsParser.m_init_wait_time);
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::close()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_subscriber)
    {
        m_executor->cancel();
        m_executorThread.join();
        m_subscriber.reset();
    }
    return true;
}

size_t yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getNrOfThreeAxisGyroscopes() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_imus.size();
}

yarp::dev::MAS_status
yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getThreeAxisGyroscopeStatus(size_t sens_index) const
{
    std::lock_guard<std::mutex> lock(m_mutex);

    std::string errorPrefix = "[getThreeAxisGyroscopeStatus] ";
    if (sens_index >= m_imus.size())
    {
        yCError(MAS) << errorPrefix << "Index" << sens_index << "out of range. The number of available gyroscopes is"
                     << m_imus.size();
        return MAS_status::MAS_UNKNOWN;
    }

    if (m_imus[sens_index].valid)
    {
        return MAS_status::MAS_OK;
    }
    else
    {
        return MAS_status::MAS_WAITING_FOR_FIRST_READ;
    }

    return MAS_status::MAS_UNKNOWN;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getThreeAxisGyroscopeName(size_t sens_index,
                                                                                std::string& name) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getThreeAxisGyroscopeName] ";

    if (sens_index >= m_imus.size())
    {
        yCError(MAS) << errorPrefix << "Index" << sens_index << "out of range. The number of available gyroscopes is"
                     << m_imus.size();
        return false;
    }
    name = m_imus[sens_index].name;
    return true;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getThreeAxisGyroscopeFrameName(size_t sens_index,
                                                                                     std::string& frameName) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getThreeAxisGyroscopeFrameName] ";
    if (sens_index >= m_imus.size())
    {
        yCError(MAS) << errorPrefix << "Index" << sens_index << "out of range. The number of available gyroscopes is"
                     << m_imus.size();
        return false;
    }

    if (m_imus[sens_index].valid)
    {
        std::lock_guard<std::mutex> lock(m_imus[sens_index].mutex);
        frameName = m_imus[sens_index].frame;
    }
    else
    {
        yCError(MAS) << errorPrefix << "The IMU" << sens_index << "has not received any message yet.";
        return false;
    }
    return true;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getThreeAxisGyroscopeMeasure(size_t sens_index,
                                                                                   yarp::sig::Vector& out,
                                                                                   double& timestamp) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getThreeAxisGyroscopeMeasure] ";
    if (sens_index >= m_imus.size())
    {
        yCError(MAS) << errorPrefix << "Index" << sens_index << "out of range. The number of available gyroscopes is"
                     << m_imus.size();
        return false;
    }
    if (m_imus[sens_index].valid)
    {
        std::lock_guard<std::mutex> lock(m_imus[sens_index].mutex);
        out = m_imus[sens_index].angular_velocity_deg_s;
        timestamp = m_imus[sens_index].timestamp;
    }
    else
    {
        yCError(MAS) << errorPrefix << "The IMU" << sens_index << "has not received any message yet.";
        return false;
    }
    return true;
}

size_t yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getNrOfThreeAxisLinearAccelerometers() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_imus.size();
}

yarp::dev::MAS_status
yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getThreeAxisLinearAccelerometerStatus(size_t sens_index) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getThreeAxisLinearAccelerometerStatus] ";
    if (sens_index >= m_imus.size())
    {
        yCError(MAS) << errorPrefix << "Index" << sens_index
                     << "out of range. The number of available linear accelerometers is" << m_imus.size();
        return MAS_status::MAS_UNKNOWN;
    }
    if (m_imus[sens_index].valid)
    {
        return MAS_status::MAS_OK;
    }
    else
    {
        return MAS_status::MAS_WAITING_FOR_FIRST_READ;
    }
    return MAS_status::MAS_UNKNOWN;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getThreeAxisLinearAccelerometerName(size_t sens_index,
                                                                                          std::string& name) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getThreeAxisLinearAccelerometerName] ";
    if (sens_index >= m_imus.size())
    {
        yCError(MAS) << errorPrefix << "Index" << sens_index
                     << "out of range. The number of available linear accelerometers is" << m_imus.size();
        return false;
    }
    name = m_imus[sens_index].name;
    return true;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getThreeAxisLinearAccelerometerFrameName(
    size_t sens_index, std::string& frameName) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getThreeAxisLinearAccelerometerFrameName] ";
    if (sens_index >= m_imus.size())
    {
        yCError(MAS) << errorPrefix << "Index" << sens_index
                     << "out of range. The number of available linear accelerometers is" << m_imus.size();
        return false;
    }
    if (m_imus[sens_index].valid)
    {
        std::lock_guard<std::mutex> lock(m_imus[sens_index].mutex);
        frameName = m_imus[sens_index].frame;
    }
    else
    {
        yCError(MAS) << errorPrefix << "The IMU" << sens_index << "has not received any message yet.";
        return false;
    }
    return true;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getThreeAxisLinearAccelerometerMeasure(size_t sens_index,
                                                                                             yarp::sig::Vector& out,
                                                                                             double& timestamp) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getThreeAxisLinearAccelerometerMeasure] ";
    if (sens_index >= m_imus.size())
    {
        yCError(MAS) << errorPrefix << "Index" << sens_index
                     << "out of range. The number of available linear accelerometers is" << m_imus.size();
        return false;
    }
    if (m_imus[sens_index].valid)
    {
        std::lock_guard<std::mutex> lock(m_imus[sens_index].mutex);
        out = m_imus[sens_index].linear_acceleration_m_s2;
        timestamp = m_imus[sens_index].timestamp;
    }
    else
    {
        yCError(MAS) << errorPrefix << "The IMU" << sens_index << "has not received any message yet.";
        return false;
    }
    return true;
}

size_t yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getNrOfThreeAxisAngularAccelerometers() const
{
    return 0;
}

yarp::dev::MAS_status
yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getThreeAxisAngularAccelerometerStatus(size_t /*sens_index*/) const
{
    yCErrorOnce(MAS)
        << "[getThreeAxisAngularAccelerometerStatus] This device does not implement three axis angular accelerometers";
    return MAS_status::MAS_UNKNOWN;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getThreeAxisAngularAccelerometerName(size_t /*sens_index*/,
                                                                                           std::string& /*name*/) const
{
    yCErrorOnce(MAS)
        << "[getThreeAxisAngularAccelerometerName] This device does not implement three axis angular accelerometers";
    return false;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getThreeAxisAngularAccelerometerFrameName(
    size_t /*sens_index*/, std::string& /*frameName*/) const
{
    yCErrorOnce(MAS) << "[getThreeAxisAngularAccelerometerFrameName] This device does not implement three axis angular "
                        "accelerometers";
    return false;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getThreeAxisAngularAccelerometerMeasure(
    size_t /*sens_index*/, yarp::sig::Vector& /*out*/, double& /*timestamp*/) const
{
    yCErrorOnce(MAS)
        << "[getThreeAxisAngularAccelerometerMeasure] This device does not implement three axis angular accelerometers";
    return false;
}

size_t yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getNrOfThreeAxisMagnetometers() const
{
    return 0;
}

yarp::dev::MAS_status
yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getThreeAxisMagnetometerStatus(size_t /*sens_index*/) const
{
    yCErrorOnce(MAS) << "[getThreeAxisMagnetometerStatus] This device does not implement three axis magnetometers";
    return MAS_status::MAS_UNKNOWN;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getThreeAxisMagnetometerName(size_t /*sens_index*/,
                                                                                   std::string& /*name*/) const
{
    yCErrorOnce(MAS) << "[getThreeAxisMagnetometerName] This device does not implement three axis magnetometers";
    return false;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getThreeAxisMagnetometerFrameName(
    size_t /*sens_index*/, std::string& /*frameName*/) const
{
    yCErrorOnce(MAS) << "[getThreeAxisMagnetometerFrameName] This device does not implement three axis magnetometers";
    return false;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getThreeAxisMagnetometerMeasure(size_t /*sens_index*/,
                                                                                      yarp::sig::Vector& /*out*/,
                                                                                      double& /*timestamp*/) const
{
    yCErrorOnce(MAS) << "[getThreeAxisMagnetometerMeasure] This device does not implement three axis magnetometers";
    return false;
}

size_t yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getNrOfOrientationSensors() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_imus.size();
}

yarp::dev::MAS_status
yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getOrientationSensorStatus(size_t sens_index) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getOrientationSensorStatus] ";
    if (sens_index >= m_imus.size())
    {
        yCError(MAS) << errorPrefix << "Index" << sens_index
                     << "out of range. The number of available orientation sensors is" << m_imus.size();
        return MAS_status::MAS_UNKNOWN;
    }
    if (m_imus[sens_index].valid)
    {
        return MAS_status::MAS_OK;
    }
    else
    {
        return MAS_status::MAS_WAITING_FOR_FIRST_READ;
    }
    return MAS_status::MAS_UNKNOWN;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getOrientationSensorName(size_t sens_index,
                                                                               std::string& name) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getOrientationSensorName] ";
    if (sens_index >= m_imus.size())
    {
        yCError(MAS) << errorPrefix << "Index" << sens_index
                     << "out of range. The number of available orientation sensors is" << m_imus.size();
        return false;
    }
    name = m_imus[sens_index].name;
    return true;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getOrientationSensorFrameName(size_t sens_index,
                                                                                    std::string& frameName) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getOrientationSensorFrameName] ";
    if (sens_index >= m_imus.size())
    {
        yCError(MAS) << errorPrefix << "Index" << sens_index
                     << "out of range. The number of available orientation sensors is" << m_imus.size();
        return false;
    }
    if (m_imus[sens_index].valid)
    {
        std::lock_guard<std::mutex> lock(m_imus[sens_index].mutex);
        frameName = m_imus[sens_index].frame;
    }
    else
    {
        yCError(MAS) << errorPrefix << "The IMU" << sens_index << "has not received any message yet.";
        return false;
    }
    return true;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getOrientationSensorMeasureAsRollPitchYaw(size_t sens_index,
                                                                                                yarp::sig::Vector& rpy,
                                                                                                double& timestamp) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getOrientationSensorMeasureAsRollPitchYaw] ";
    if (sens_index >= m_imus.size())
    {
        yCError(MAS) << errorPrefix << "Index" << sens_index
                     << "out of range. The number of available orientation sensors is" << m_imus.size();
        return false;
    }
    if (m_imus[sens_index].valid)
    {
        std::lock_guard<std::mutex> lock(m_imus[sens_index].mutex);
        rpy = m_imus[sens_index].rpy_deg;
        timestamp = m_imus[sens_index].timestamp;
    }
    else
    {
        yCError(MAS) << errorPrefix << "The IMU" << sens_index << "has not received any message yet.";
        return false;
    }
    return true;
}

size_t yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getNrOfTemperatureSensors() const
{
    return 0;
}

yarp::dev::MAS_status
yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getTemperatureSensorStatus(size_t /*sens_index*/) const
{
    yCErrorOnce(MAS) << "[getTemperatureSensorStatus] This device does not implement temperature sensors";
    return MAS_status::MAS_UNKNOWN;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getTemperatureSensorName(size_t /*sens_index*/,
                                                                               std::string& /*name*/) const
{
    yCErrorOnce(MAS) << "[getTemperatureSensorName] This device does not implement temperature sensors";
    return false;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getTemperatureSensorFrameName(size_t /*sens_index*/,
                                                                                    std::string& /*frameName*/) const
{
    yCErrorOnce(MAS) << "[getTemperatureSensorFrameName] This device does not implement temperature sensors";
    return false;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getTemperatureSensorMeasure(size_t /*sens_index*/, double& out,
                                                                                  double& /*timestamp*/) const
{
    yCErrorOnce(MAS) << "[getTemperatureSensorMeasure] This device does not implement temperature sensors";
    return false;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getTemperatureSensorMeasure(size_t /*sens_index*/,
                                                                                  yarp::sig::Vector& /*out*/,
                                                                                  double& /*timestamp*/) const
{
    yCErrorOnce(MAS) << "[getTemperatureSensorMeasure] This device does not implement temperature sensors";
    return false;
}

size_t yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getNrOfSixAxisForceTorqueSensors() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_fts.size();
}

yarp::dev::MAS_status
yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getSixAxisForceTorqueSensorStatus(size_t sens_index) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getSixAxisForceTorqueSensorStatus] ";
    if (sens_index >= m_fts.size())
    {
        yCError(MAS) << errorPrefix << "Index" << sens_index
                     << "out of range. The number of available six axis force torque sensors is" << m_fts.size();
        return MAS_status::MAS_UNKNOWN;
    }
    if (m_fts[sens_index].valid)
    {
        return MAS_status::MAS_OK;
    }
    else
    {
        return MAS_status::MAS_WAITING_FOR_FIRST_READ;
    }
    return MAS_status::MAS_UNKNOWN;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getSixAxisForceTorqueSensorName(size_t sens_index,
                                                                                      std::string& name) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getSixAxisForceTorqueSensorName] ";
    if (sens_index >= m_fts.size())
    {
        yCError(MAS) << errorPrefix << "Index" << sens_index
                     << "out of range. The number of available six axis force torque sensors is" << m_fts.size();
        return false;
    }
    name = m_fts[sens_index].name;
    return true;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getSixAxisForceTorqueSensorFrameName(size_t sens_index,
                                                                                           std::string& frame) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getSixAxisForceTorqueSensorFrameName] ";
    if (sens_index >= m_fts.size())
    {
        yCError(MAS) << errorPrefix << "Index" << sens_index
                     << "out of range. The number of available six axis force torque sensors is" << m_fts.size();
        return false;
    }
    if (m_fts[sens_index].valid)
    {
        std::lock_guard<std::mutex> lock(m_fts[sens_index].mutex);
        frame = m_fts[sens_index].frame;
    }
    else
    {
        yCError(MAS) << errorPrefix << "The FT sensor" << sens_index << "has not received any message yet.";
        return false;
    }
    return true;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getSixAxisForceTorqueSensorMeasure(size_t sens_index,
                                                                                         yarp::sig::Vector& out,
                                                                                         double& timestamp) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[getSixAxisForceTorqueSensorMeasure] ";
    if (sens_index >= m_fts.size())
    {
        yCError(MAS) << errorPrefix << "Index" << sens_index
                     << "out of range. The number of available six axis force torque sensors is" << m_fts.size();
        return false;
    }
    if (m_fts[sens_index].valid)
    {
        std::lock_guard<std::mutex> lock(m_fts[sens_index].mutex);
        out = m_fts[sens_index].force_torque;
        timestamp = m_fts[sens_index].timestamp;
    }
    else
    {
        yCError(MAS) << errorPrefix << "The FT sensor" << sens_index << "has not received any message yet.";
        return false;
    }
    return true;
}

size_t yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getNrOfContactLoadCellArrays() const
{
    return 0;
}

yarp::dev::MAS_status
yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getContactLoadCellArrayStatus(size_t /*sens_index*/) const
{
    yCErrorOnce(MAS) << "[getContactLoadCellArrayStatus] This device does not implement contact load cell arrays";
    return MAS_status::MAS_UNKNOWN;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getContactLoadCellArrayName(size_t /*sens_index*/,
                                                                                  std::string& /*name*/) const
{
    yCErrorOnce(MAS) << "[getContactLoadCellArrayName] This device does not implement contact load cell arrays";
    return false;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getContactLoadCellArrayMeasure(size_t /*sens_index*/,
                                                                                     yarp::sig::Vector& /*out*/,
                                                                                     double& /*timestamp*/) const
{
    yCErrorOnce(MAS) << "[getContactLoadCellArrayMeasure] This device does not implement contact load cell arrays";
    return false;
}

size_t yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getContactLoadCellArraySize(size_t /*sens_index*/) const
{
    return 0;
}

size_t yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getNrOfEncoderArrays() const
{
    return 0;
}

yarp::dev::MAS_status
yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getEncoderArrayStatus(size_t /*sens_index*/) const
{
    yCErrorOnce(MAS) << "[getEncoderArrayStatus] This device does not implement encoder arrays";
    return MAS_status::MAS_UNKNOWN;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getEncoderArrayName(size_t /*sens_index*/,
                                                                          std::string& /*name*/) const
{
    yCErrorOnce(MAS) << "[getEncoderArrayName] This device does not implement encoder arrays";
    return false;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getEncoderArrayMeasure(size_t /*sens_index*/,
                                                                             yarp::sig::Vector& /*out*/,
                                                                             double& /*timestamp*/) const
{
    yCErrorOnce(MAS) << "[getEncoderArrayMeasure] This device does not implement encoder arrays";
    return false;
}

size_t yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getEncoderArraySize(size_t /*sens_index*/) const
{
    return 0;
}

size_t yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getNrOfSkinPatches() const
{
    return 0;
}

yarp::dev::MAS_status yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getSkinPatchStatus(size_t /*sens_index*/) const
{
    yCErrorOnce(MAS) << "[getSkinPatchStatus] This device does not implement skin patches";
    return MAS_status::MAS_UNKNOWN;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getSkinPatchName(size_t /*sens_index*/,
                                                                       std::string& /*name*/) const
{
    yCErrorOnce(MAS) << "[getSkinPatchName] This device does not implement skin patches";
    return false;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getSkinPatchMeasure(size_t /*sens_index*/,
                                                                          yarp::sig::Vector& /*out*/,
                                                                          double& /*timestamp*/) const
{
    yCErrorOnce(MAS) << "[getSkinPatchMeasure] This device does not implement skin patches";
    return false;
}

size_t yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getSkinPatchSize(size_t /*sens_index*/) const
{
    return 0;
}

size_t yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getNrOfPositionSensors() const
{
    return 0;
}

yarp::dev::MAS_status
yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getPositionSensorStatus(size_t /*sens_index*/) const
{
    yCErrorOnce(MAS) << "[getPositionSensorStatus] This device does not implement position sensors";
    return MAS_status::MAS_UNKNOWN;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getPositionSensorName(size_t /*sens_index*/,
                                                                            std::string& /*name*/) const
{
    yCErrorOnce(MAS) << "[getPositionSensorName] This device does not implement position sensors";
    return false;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getPositionSensorFrameName(size_t /*sens_index*/,
                                                                                 std::string& /*frameName*/) const
{
    yCErrorOnce(MAS) << "[getPositionSensorFrameName] This device does not implement position sensors";
    return false;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getPositionSensorMeasure(size_t /*sens_index*/,
                                                                               yarp::sig::Vector& /*xyz*/,
                                                                               double& /*timestamp*/) const
{
    yCErrorOnce(MAS) << "[getPositionSensorMeasure] This device does not implement position sensors";
    return false;
}

size_t yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getNrOfLinearVelocitySensors() const
{
    return 0;
}

yarp::dev::MAS_status
yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getLinearVelocitySensorStatus(size_t /*sens_index*/) const
{
    yCErrorOnce(MAS) << "[getLinearVelocitySensorStatus] This device does not implement linear velocity sensors";
    return MAS_status::MAS_UNKNOWN;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getLinearVelocitySensorName(size_t /*sens_index*/,
                                                                                  std::string& /*name*/) const
{
    yCErrorOnce(MAS) << "[getLinearVelocitySensorName] This device does not implement linear velocity sensors";
    return false;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getLinearVelocitySensorFrameName(size_t /*sens_index*/,
                                                                                       std::string& /*frameName*/) const
{
    yCErrorOnce(MAS) << "[getLinearVelocitySensorFrameName] This device does not implement linear velocity sensors";
    return false;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getLinearVelocitySensorMeasure(size_t /*sens_index*/,
                                                                                     yarp::sig::Vector& /*xyz*/,
                                                                                     double& /*timestamp*/) const
{
    yCErrorOnce(MAS) << "[getLinearVelocitySensorMeasure] This device does not implement linear velocity sensors";
    return false;
}

void yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::IMUMeasure::convert_to_yarp_vectors(
    const sensor_msgs::msg::Imu::ConstSharedPtr& imu)
{
    std::lock_guard<std::mutex> lock(mutex);
    yarp::sig::Matrix rot_matrix(3, 3);
    rot_matrix.zero();

    constexpr double rad2deg = 180.0 / M_PI;

    // Convert quaternion to rotation matrix
    double qw = imu->orientation.w;
    double qx = imu->orientation.x;
    double qy = imu->orientation.y;
    double qz = imu->orientation.z;

    // Copied from
    // https://github.com/robotology/idyntree/blob/e836c8b87ab1faafa4c97d55b1f989422c743efd/src/core/src/Rotation.cpp#L361-L429
    // See https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Quaternion-derived_rotation_matrix

    // To avoid memory allocation "unroll" the summation of Rodrigues' Formula
    //  R = I3 + 2s S(r) + 2S(r)^2,

    // The square of S(r) is symmetric, thus the diagonal elements are
    // filled only by that part
    rot_matrix(0, 0) = 1 - 2 * (qz * qz + qy * qy);
    rot_matrix(1, 1) = 1 - 2 * (qz * qz + qx * qx);
    rot_matrix(2, 2) = 1 - 2 * (qy * qy + qx * qx);

    // The off diagonal elements are filled by
    //(symmetrically) from the S(r)^2
    //(antisymmetrically) from the S(r)
    // Symmetric part
    rot_matrix(0, 1) = rot_matrix(1, 0) = 2 * qx * qy;
    rot_matrix(0, 2) = rot_matrix(2, 0) = 2 * qx * qz;
    rot_matrix(1, 2) = rot_matrix(2, 1) = 2 * qy * qz;

    // antisymmetric part
    double r01, r02, r12;

    r01 = 2 * qw * (-1) * qz;
    r02 = 2 * qw * (+1) * qy;
    r12 = 2 * qw * (-1) * qx;

    rot_matrix(0, 1) += r01;
    rot_matrix(0, 2) += r02;
    rot_matrix(1, 2) += r12;

    rot_matrix(1, 0) -= r01;
    rot_matrix(2, 0) -= r02;
    rot_matrix(2, 1) -= r12;

    // Convert the rotation from quaternion to roll-pitch-yaw (in radians)
    // assuming R = Rz(yaw)Ry(pitch)Rx(roll)
    // The code has been inspired from  https://www.geometrictools.com/Documentation/EulerAngles.pdf sec 2.6
    double r, p, y; // roll, pitch, yaw
    if (rot_matrix(2, 0) < 1.0)
    {
        if (rot_matrix(2, 0) > -1.0)
        {
            r = atan2(rot_matrix(2, 1), rot_matrix(2, 2));
            p = asin(-rot_matrix(2, 0));
            y = atan2(rot_matrix(1, 0), rot_matrix(0, 0));
        }
        else
        {
            // Not a unique solution
            r = 0.0;
            p = M_PI / 2.0;
            y = -atan2(-rot_matrix(1, 2), rot_matrix(1, 1));
        }
    }
    else
    {
        // Not a unique solution
        r = 0.0;
        p = -M_PI / 2.0;
        y = atan2(-rot_matrix(1, 2), rot_matrix(1, 1));
    }

    // Convert the rotation from quaternion to roll-pitch-yaw (in degrees)
    rpy_deg[0] = r * rad2deg;
    rpy_deg[1] = p * rad2deg;
    rpy_deg[2] = y * rad2deg;

    // Convert the angular velocity from rad/s to deg/s
    angular_velocity_deg_s[0] = imu->angular_velocity.x * rad2deg;
    angular_velocity_deg_s[1] = imu->angular_velocity.y * rad2deg;
    angular_velocity_deg_s[2] = imu->angular_velocity.z * rad2deg;

    // Linear acceleration is already in m/s^2
    linear_acceleration_m_s2[0] = imu->linear_acceleration.x;
    linear_acceleration_m_s2[1] = imu->linear_acceleration.y;
    linear_acceleration_m_s2[2] = imu->linear_acceleration.z;

    timestamp = imu->header.stamp.sec + imu->header.stamp.nanosec * 1e-9;
    frame = imu->header.frame_id;
}

yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::IMUMeasure::IMUMeasure()
{
    rpy_deg.resize(3, 0.0);
    angular_velocity_deg_s.resize(3, 0.0);
    linear_acceleration_m_s2.resize(3, 0.0);
    timestamp = 0.0;
}

yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::IMUMeasure::IMUMeasure(const IMUMeasure& other)
{
    rpy_deg = other.rpy_deg;
    angular_velocity_deg_s = other.angular_velocity_deg_s;
    linear_acceleration_m_s2 = other.linear_acceleration_m_s2;
    timestamp = other.timestamp;
    valid = other.valid.load();
}

void yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::FTMeasure::convert_to_yarp_vectors(
    const geometry_msgs::msg::WrenchStamped::ConstSharedPtr& ft)
{
    std::lock_guard<std::mutex> lock(mutex);
    force_torque[0] = ft->wrench.force.x;
    force_torque[1] = ft->wrench.force.y;
    force_torque[2] = ft->wrench.force.z;
    force_torque[3] = ft->wrench.torque.x;
    force_torque[4] = ft->wrench.torque.y;
    force_torque[5] = ft->wrench.torque.z;
    timestamp = ft->header.stamp.sec + ft->header.stamp.nanosec * 1e-9;
    frame = ft->header.frame_id;
}

yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::FTMeasure::FTMeasure()
{
    force_torque.resize(6, 0.0);
    timestamp = 0.0;
}

yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::FTMeasure::FTMeasure(const FTMeasure& other)
{
    force_torque = other.force_torque;
    timestamp = other.timestamp;
    valid = other.valid.load();
}

yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::MASSubscriber::MASSubscriber(
    const std::string& name, const std::vector<std::string>& imuTopics, const std::vector<std::string>& ftTopics,
    IsaacSimMultipleAnalogSensorsNWCROS2* parent)
    : Node(name)
{
    int queue_size = 10;

    parent->m_imus.resize(imuTopics.size());
    parent->m_fts.resize(ftTopics.size());

    for (size_t i = 0; i < imuTopics.size(); ++i)
    {
        IMUMeasure* imu_ptr = &parent->m_imus[i];
        // Use as name the last part of the topic after the last '/'
        size_t last_slash = imuTopics[i].find_last_of('/');
        if (last_slash != std::string::npos && last_slash + 1 < imuTopics[i].size())
        {
            imu_ptr->name = imuTopics[i].substr(last_slash + 1);
        }
        else
        {
            imu_ptr->name = imuTopics[i];
        }

        m_imu_subs.push_back(
            this->create_subscription<sensor_msgs::msg::Imu>(imuTopics[i], queue_size,
                                                             [imu_ptr](const sensor_msgs::msg::Imu::ConstSharedPtr& imu)
                                                             {
                                                                 imu_ptr->convert_to_yarp_vectors(imu);
                                                                 imu_ptr->valid = true;
                                                             }));
    }

    for (size_t i = 0; i < ftTopics.size(); ++i)
    {
        FTMeasure* ft_ptr = &parent->m_fts[i];
        // Use as name the last part of the topic after the last '/'
        size_t last_slash = ftTopics[i].find_last_of('/');
        if (last_slash != std::string::npos && last_slash + 1 < ftTopics[i].size())
        {
            ft_ptr->name = ftTopics[i].substr(last_slash + 1);
        }
        else
        {
            ft_ptr->name = ftTopics[i];
        }
        m_ft_subs.push_back(this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            ftTopics[i], queue_size,
            [ft_ptr](const geometry_msgs::msg::WrenchStamped::ConstSharedPtr& ft)
            {
                ft_ptr->convert_to_yarp_vectors(ft);
                ft_ptr->valid = true;
            }));
    }
}
