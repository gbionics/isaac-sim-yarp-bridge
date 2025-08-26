// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-2-Clause
#include "IsaacSimMultipleAnalogSensorsNWCROS2.h"
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>

YARP_DECLARE_LOG_COMPONENT(MAS)
YARP_LOG_COMPONENT(MAS, "yarp.device.IsaacSimMultipleAnalogSensorsNWCROS2")

yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::~IsaacSimMultipleAnalogSensorsNWCROS2()
{
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::open(yarp::os::Searchable& config)
{
    return false;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::close()
{
    return false;
}

size_t yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getNrOfThreeAxisGyroscopes() const
{
    return size_t();
}

yarp::dev::MAS_status yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getThreeAxisGyroscopeStatus(size_t sens_index) const
{
    return yarp::dev::MAS_status();
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getThreeAxisGyroscopeName(size_t sens_index, std::string& name) const
{
    return false;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getThreeAxisGyroscopeFrameName(size_t sens_index, std::string& frameName) const
{
    return false;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getThreeAxisGyroscopeMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    return false;
}

size_t yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getNrOfThreeAxisLinearAccelerometers() const
{
    return size_t();
}

yarp::dev::MAS_status yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getThreeAxisLinearAccelerometerStatus(size_t sens_index) const
{
    return yarp::dev::MAS_status();
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getThreeAxisLinearAccelerometerName(size_t sens_index, std::string& name) const
{
    return false;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getThreeAxisLinearAccelerometerFrameName(size_t sens_index, std::string& frameName) const
{
    return false;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getThreeAxisLinearAccelerometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    return false;
}

size_t yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getNrOfThreeAxisAngularAccelerometers() const
{
    return size_t();
}

yarp::dev::MAS_status yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getThreeAxisAngularAccelerometerStatus(size_t sens_index) const
{
    return yarp::dev::MAS_status();
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getThreeAxisAngularAccelerometerName(size_t sens_index, std::string& name) const
{
    return false;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getThreeAxisAngularAccelerometerFrameName(size_t sens_index, std::string& frameName) const
{
    return false;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getThreeAxisAngularAccelerometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    return false;
}

size_t yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getNrOfThreeAxisMagnetometers() const
{
    return size_t();
}

yarp::dev::MAS_status yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getThreeAxisMagnetometerStatus(size_t sens_index) const
{
    return yarp::dev::MAS_status();
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getThreeAxisMagnetometerName(size_t sens_index, std::string& name) const
{
    return false;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getThreeAxisMagnetometerFrameName(size_t sens_index, std::string& frameName) const
{
    return false;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getThreeAxisMagnetometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    return false;
}

size_t yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getNrOfOrientationSensors() const
{
    return size_t();
}

yarp::dev::MAS_status yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getOrientationSensorStatus(size_t sens_index) const
{
    return yarp::dev::MAS_status();
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getOrientationSensorName(size_t sens_index, std::string& name) const
{
    return false;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getOrientationSensorFrameName(size_t sens_index, std::string& frameName) const
{
    return false;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getOrientationSensorMeasureAsRollPitchYaw(size_t sens_index, yarp::sig::Vector& rpy, double& timestamp) const
{
    return false;
}

size_t yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getNrOfTemperatureSensors() const
{
    return size_t();
}

yarp::dev::MAS_status yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getTemperatureSensorStatus(size_t sens_index) const
{
    return yarp::dev::MAS_status();
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getTemperatureSensorName(size_t sens_index, std::string& name) const
{
    return false;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getTemperatureSensorFrameName(size_t sens_index, std::string& frameName) const
{
    return false;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getTemperatureSensorMeasure(size_t sens_index, double& out, double& timestamp) const
{
    return false;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getTemperatureSensorMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    return false;
}

size_t yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getNrOfSixAxisForceTorqueSensors() const
{
    return size_t();
}

yarp::dev::MAS_status yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getSixAxisForceTorqueSensorStatus(size_t sens_index) const
{
    return yarp::dev::MAS_status();
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getSixAxisForceTorqueSensorName(size_t sens_index, std::string& name) const
{
    return false;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getSixAxisForceTorqueSensorFrameName(size_t sens_index, std::string& frame) const
{
    return false;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getSixAxisForceTorqueSensorMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    return false;
}

size_t yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getNrOfContactLoadCellArrays() const
{
    return size_t();
}

yarp::dev::MAS_status yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getContactLoadCellArrayStatus(size_t sens_index) const
{
    return yarp::dev::MAS_status();
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getContactLoadCellArrayName(size_t sens_index, std::string& name) const
{
    return false;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getContactLoadCellArrayMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    return false;
}

size_t yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getContactLoadCellArraySize(size_t sens_index) const
{
    return size_t();
}

size_t yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getNrOfEncoderArrays() const
{
    return size_t();
}

yarp::dev::MAS_status yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getEncoderArrayStatus(size_t sens_index) const
{
    return yarp::dev::MAS_status();
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getEncoderArrayName(size_t sens_index, std::string& name) const
{
    return false;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getEncoderArrayMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    return false;
}

size_t yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getEncoderArraySize(size_t sens_index) const
{
    return size_t();
}

size_t yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getNrOfSkinPatches() const
{
    return size_t();
}

yarp::dev::MAS_status yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getSkinPatchStatus(size_t sens_index) const
{
    return yarp::dev::MAS_status();
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getSkinPatchName(size_t sens_index, std::string& name) const
{
    return false;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getSkinPatchMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    return false;
}

size_t yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getSkinPatchSize(size_t sens_index) const
{
    return size_t();
}

size_t yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getNrOfPositionSensors() const
{
    return size_t();
}

yarp::dev::MAS_status yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getPositionSensorStatus(size_t sens_index) const
{
    return yarp::dev::MAS_status();
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getPositionSensorName(size_t sens_index, std::string& name) const
{
    return false;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getPositionSensorFrameName(size_t sens_index, std::string& frameName) const
{
    return false;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getPositionSensorMeasure(size_t sens_index, yarp::sig::Vector& xyz, double& timestamp) const
{
    return false;
}

size_t yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getNrOfLinearVelocitySensors() const
{
    return size_t();
}

yarp::dev::MAS_status yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getLinearVelocitySensorStatus(size_t sens_index) const
{
    return yarp::dev::MAS_status();
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getLinearVelocitySensorName(size_t sens_index, std::string& name) const
{
    return false;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getLinearVelocitySensorFrameName(size_t sens_index, std::string& frameName) const
{
    return false;
}

bool yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2::getLinearVelocitySensorMeasure(size_t sens_index, yarp::sig::Vector& xyz, double& timestamp) const
{
    return false;
}
