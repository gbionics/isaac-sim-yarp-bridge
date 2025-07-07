// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-2-Clause
#include "IsaacSimRGBDSensorNWCROS2.h"
#include <yarp/os/LogComponent.h>

YARP_DECLARE_LOG_COMPONENT(RGBD)
YARP_LOG_COMPONENT(RGBD, "yarp.device.IsaacSimRGBDSensorNWCROS2")


yarp::dev::IsaacSimRGBDSensorNWCROS2::IsaacSimRGBDSensorNWCROS2()
{
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::open(yarp::os::Searchable& config)
{
    if (!m_paramsParser.parseParams(config))
    {
        yCError(RGBD, "Failed to parse parameters for IsaacSimRGBDSensorNWCROS2");
        return false;
    }

    rclcpp::init(0, nullptr);
    m_subscriber = std::make_shared<RGBDSubscriber>(m_paramsParser.m_node_name, m_paramsParser.m_rgb_topic_name, m_paramsParser.m_depth_topic_name);
    rclcpp::spin(m_subscriber);

    return true;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::close()
{
    rclcpp::shutdown();
    return true;
}

int yarp::dev::IsaacSimRGBDSensorNWCROS2::getRgbHeight()
{
    return 0;
}

int yarp::dev::IsaacSimRGBDSensorNWCROS2::getRgbWidth()
{
    return 0;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::getRgbSupportedConfigurations(yarp::sig::VectorOf<yarp::dev::CameraConfig>& configurations)
{
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::getRgbResolution(int& width, int& height)
{
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::setRgbResolution(int width, int height)
{
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::getRgbFOV(double& horizontalFov, double& verticalFov)
{
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::setRgbFOV(double horizontalFov, double verticalFov)
{
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::getRgbMirroring(bool& mirror)
{
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::setRgbMirroring(bool mirror)
{
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::getRgbIntrinsicParam(yarp::os::Property& intrinsic)
{
    return false;
}

int yarp::dev::IsaacSimRGBDSensorNWCROS2::getDepthHeight()
{
    return 0;
}

int yarp::dev::IsaacSimRGBDSensorNWCROS2::getDepthWidth()
{
    return 0;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::setDepthResolution(int width, int height)
{
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::getDepthFOV(double& horizontalFov, double& verticalFov)
{
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::setDepthFOV(double horizontalFov, double verticalFov)
{
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::getDepthIntrinsicParam(yarp::os::Property& intrinsic)
{
    return false;
}

double yarp::dev::IsaacSimRGBDSensorNWCROS2::getDepthAccuracy()
{
    return 0.0;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::setDepthAccuracy(double accuracy)
{
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::getDepthClipPlanes(double& nearPlane, double& farPlane)
{
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::setDepthClipPlanes(double nearPlane, double farPlane)
{
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::getDepthMirroring(bool& mirror)
{
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::setDepthMirroring(bool mirror)
{
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::getExtrinsicParam(yarp::sig::Matrix& extrinsic)
{
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::getRgbImage(yarp::sig::FlexImage& rgbImage, yarp::os::Stamp* timeStamp)
{
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::getDepthImage(yarp::sig::ImageOf<yarp::sig::PixelFloat>& depthImage, yarp::os::Stamp* timeStamp)
{
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::getImages(yarp::sig::FlexImage& colorFrame, yarp::sig::ImageOf<yarp::sig::PixelFloat>& depthFrame, yarp::os::Stamp* colorStamp, yarp::os::Stamp* depthStamp)
{
    return false;
}

yarp::dev::IRGBDSensor::RGBDSensor_status yarp::dev::IsaacSimRGBDSensorNWCROS2::getSensorStatus()
{
    return RGBDSensor_status();
}

std::string yarp::dev::IsaacSimRGBDSensorNWCROS2::getLastErrorMsg(yarp::os::Stamp* timeStamp)
{
    return std::string();
}

yarp::dev::IsaacSimRGBDSensorNWCROS2::RGBDSubscriber::RGBDSubscriber(const std::string& name, const std::string& rgbTopic, const std::string& depthTopic)
    : Node(name)
{
    // Create message_filters subscribers
    m_rgb_sub.subscribe(this, rgbTopic);
    m_depth_sub.subscribe(this, depthTopic);

    const uint32_t queue_size = 10; // Default queue size for synchronization

    // ApproximateTimeSynchronizer setup
    m_sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(queue_size), m_rgb_sub, m_depth_sub);
    m_sync->registerCallback(std::bind(&RGBDSubscriber::callback, this, std::placeholders::_1, std::placeholders::_2));
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::RGBDSubscriber::getImages(yarp::sig::FlexImage& colorFrame, yarp::sig::ImageOf<yarp::sig::PixelFloat>& depthFrame, yarp::os::Stamp* colorStamp, yarp::os::Stamp* depthStamp)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return false;
}

void yarp::dev::IsaacSimRGBDSensorNWCROS2::RGBDSubscriber::callback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb, const sensor_msgs::msg::Image::ConstSharedPtr& depth)
{
    std::lock_guard<std::mutex> lock(m_mutex);

}
