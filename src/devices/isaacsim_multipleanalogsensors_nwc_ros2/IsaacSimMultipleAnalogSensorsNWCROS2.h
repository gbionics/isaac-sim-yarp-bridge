// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-2-Clause

#ifndef ISAACSIM_MAS_SENSORS_NWC_ROS2_H
#define ISAACSIM_MAS_SENSORS_NWC_ROS2_H

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>

#include <rclcpp/node.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include <memory>
#include <mutex>
#include <atomic>
#include <string>
#include <thread>
#include <vector>

#include "IsaacSimMultipleAnalogSensorsNWCROS2_ParamsParser.h"

namespace yarp::dev
{
    class IsaacSimMultipleAnalogSensorsNWCROS2;
}


class yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2 :
    public yarp::dev::DeviceDriver,
    public yarp::dev::IThreeAxisGyroscopes,
    public yarp::dev::IThreeAxisLinearAccelerometers,
    public yarp::dev::IThreeAxisAngularAccelerometers,
    public yarp::dev::IThreeAxisMagnetometers,
    public yarp::dev::IOrientationSensors,
    public yarp::dev::ITemperatureSensors,
    public yarp::dev::ISixAxisForceTorqueSensors,
    public yarp::dev::IContactLoadCellArrays,
    public yarp::dev::IEncoderArrays,
    public yarp::dev::ISkinPatches,
    public yarp::dev::IPositionSensors,
    public yarp::dev::ILinearVelocitySensors
{

public:
    IsaacSimMultipleAnalogSensorsNWCROS2() = default;
    ~IsaacSimMultipleAnalogSensorsNWCROS2() override;

    /* DeviceDriver methods */
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    /* IThreeAxisGyroscopes methods */
    size_t getNrOfThreeAxisGyroscopes() const override;
    yarp::dev::MAS_status getThreeAxisGyroscopeStatus(size_t sens_index) const override;
    bool getThreeAxisGyroscopeName(size_t sens_index, std::string& name) const override;
    bool getThreeAxisGyroscopeFrameName(size_t sens_index, std::string& frameName) const override;
    bool getThreeAxisGyroscopeMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;

    /* IThreeAxisLinearAccelerometers methods */
    size_t getNrOfThreeAxisLinearAccelerometers() const override;
    yarp::dev::MAS_status getThreeAxisLinearAccelerometerStatus(size_t sens_index) const override;
    bool getThreeAxisLinearAccelerometerName(size_t sens_index, std::string& name) const override;
    bool getThreeAxisLinearAccelerometerFrameName(size_t sens_index, std::string& frameName) const override;
    bool getThreeAxisLinearAccelerometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;

    /* IThreeAxisAngularAccelerometers methods */
    size_t getNrOfThreeAxisAngularAccelerometers() const override;
    yarp::dev::MAS_status getThreeAxisAngularAccelerometerStatus(size_t /*sens_index*/) const override;
    bool getThreeAxisAngularAccelerometerName(size_t /*sens_index*/, std::string& /*name*/) const override;
    bool getThreeAxisAngularAccelerometerFrameName(size_t /*sens_index*/, std::string& /*frameName*/) const override;
    bool getThreeAxisAngularAccelerometerMeasure(size_t /*sens_index*/, yarp::sig::Vector& /*out*/, double& /*timestamp*/) const override;

    /* IThreeAxisMagnetometers methods */
    size_t getNrOfThreeAxisMagnetometers() const override;
    yarp::dev::MAS_status getThreeAxisMagnetometerStatus(size_t /*sens_index*/) const override;
    bool getThreeAxisMagnetometerName(size_t /*sens_index*/, std::string& /*name*/) const override;
    bool getThreeAxisMagnetometerFrameName(size_t /*sens_index*/, std::string& /*frameName*/) const override;
    bool getThreeAxisMagnetometerMeasure(size_t /*sens_index*/, yarp::sig::Vector& /*out*/, double& /*timestamp*/) const override;

    /* IOrientationSensors methods */
    size_t getNrOfOrientationSensors() const override;
    yarp::dev::MAS_status getOrientationSensorStatus(size_t sens_index) const override;
    bool getOrientationSensorName(size_t sens_index, std::string& name) const override;
    bool getOrientationSensorFrameName(size_t sens_index, std::string& frameName) const override;
    bool getOrientationSensorMeasureAsRollPitchYaw(size_t sens_index, yarp::sig::Vector& rpy, double& timestamp) const override;

    /* ITemperatureSensors methods */
    size_t getNrOfTemperatureSensors() const override;
    yarp::dev::MAS_status getTemperatureSensorStatus(size_t /*sens_index*/) const override;
    bool getTemperatureSensorName(size_t /*sens_index*/, std::string& /*name*/) const override;
    bool getTemperatureSensorFrameName(size_t /*sens_index*/, std::string& /*frameName*/) const override;
    bool getTemperatureSensorMeasure(size_t /*sens_index*/, double& out, double& /*timestamp*/) const override;
    bool getTemperatureSensorMeasure(size_t /*sens_index*/, yarp::sig::Vector& /*out*/, double& /*timestamp*/) const override;

    /* ISixAxisForceTorqueSensors */
    size_t getNrOfSixAxisForceTorqueSensors() const override;
    yarp::dev::MAS_status getSixAxisForceTorqueSensorStatus(size_t sens_index) const override;
    bool getSixAxisForceTorqueSensorName(size_t sens_index, std::string& name) const override;
    bool getSixAxisForceTorqueSensorFrameName(size_t sens_index, std::string& frame) const override;
    bool getSixAxisForceTorqueSensorMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;

    /* IContactLoadCellArrays */
    size_t getNrOfContactLoadCellArrays() const override;
    yarp::dev::MAS_status getContactLoadCellArrayStatus(size_t /*sens_index*/) const override;
    bool getContactLoadCellArrayName(size_t /*sens_index*/, std::string& /*name*/) const override;
    bool getContactLoadCellArrayMeasure(size_t /*sens_index*/, yarp::sig::Vector& /*out*/, double& /*timestamp*/) const override;
    size_t getContactLoadCellArraySize(size_t /*sens_index*/) const override;

    /* IEncoderArrays */
    size_t getNrOfEncoderArrays() const override;
    yarp::dev::MAS_status getEncoderArrayStatus(size_t /*sens_index*/) const override;
    bool getEncoderArrayName(size_t /*sens_index*/, std::string& /*name*/) const override;
    bool getEncoderArrayMeasure(size_t /*sens_index*/, yarp::sig::Vector& /*out*/, double& /*timestamp*/) const override;
    size_t getEncoderArraySize(size_t /*sens_index*/) const override;

    /* ISkinPatches */
    size_t getNrOfSkinPatches() const override;
    yarp::dev::MAS_status getSkinPatchStatus(size_t /*sens_index*/) const override;
    bool getSkinPatchName(size_t /*sens_index*/, std::string& /*name*/) const override;
    bool getSkinPatchMeasure(size_t /*sens_index*/, yarp::sig::Vector& /*out*/, double& /*timestamp*/) const override;
    size_t getSkinPatchSize(size_t /*sens_index*/) const override;

    /* IPositionSensors methods */
    size_t getNrOfPositionSensors() const override;
    yarp::dev::MAS_status getPositionSensorStatus(size_t /*sens_index*/) const override;
    bool getPositionSensorName(size_t /*sens_index*/, std::string& /*name*/) const override;
    bool getPositionSensorFrameName(size_t /*sens_index*/, std::string& /*frameName*/) const override;
    bool getPositionSensorMeasure(size_t /*sens_index*/, yarp::sig::Vector& /*xyz*/, double& /*timestamp*/) const override;

    /* ILinearVelocitySensors methods */
    size_t getNrOfLinearVelocitySensors() const override;
    yarp::dev::MAS_status getLinearVelocitySensorStatus(size_t /*sens_index*/) const override;
    bool getLinearVelocitySensorName(size_t /*sens_index*/, std::string& /*name*/) const override;
    bool getLinearVelocitySensorFrameName(size_t /*sens_index*/, std::string& /*frameName*/) const override;
    bool getLinearVelocitySensorMeasure(size_t /*sens_index*/, yarp::sig::Vector& xyz, double& /*timestamp*/) const override;

private:
    struct IMUMeasure
    {
        std::string name;
        std::string frame;
        yarp::sig::Vector rpy_deg;
        yarp::sig::Vector angular_velocity_deg_s;
        yarp::sig::Vector linear_acceleration_m_s2;
        double timestamp;
        std::atomic<bool> valid{ false };
        mutable std::mutex mutex;
        void convert_to_yarp_vectors(const sensor_msgs::msg::Imu::ConstSharedPtr& imu);

        IMUMeasure();
        IMUMeasure(const IMUMeasure& other);
    };

    struct FTMeasure
    {
        std::string name;
        std::string frame;
        yarp::sig::Vector force_torque;
        double timestamp;
        std::atomic<bool> valid{ false };
        mutable std::mutex mutex;
        void convert_to_yarp_vectors(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr& ft);

        FTMeasure();
        FTMeasure(const FTMeasure& other);
    };

    class MASSubscriber : public rclcpp::Node
    {
    public:
        MASSubscriber(const std::string& name,
                     const std::vector<std::string>& imuTopics,
                     const std::vector<std::string>& ftTopics,
                     IsaacSimMultipleAnalogSensorsNWCROS2* parent);
    private:
        std::vector<rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr> m_imu_subs;
        std::vector<rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr> m_ft_subs;
    };

    IsaacSimMultipleAnalogSensorsNWCROS2_ParamsParser m_paramsParser;
    std::shared_ptr<MASSubscriber> m_subscriber;
    std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> m_executor;
    std::thread m_executorThread;
    std::vector<IMUMeasure> m_imus;
    std::vector<FTMeasure> m_fts;
    mutable std::mutex m_mutex;

};
#endif
