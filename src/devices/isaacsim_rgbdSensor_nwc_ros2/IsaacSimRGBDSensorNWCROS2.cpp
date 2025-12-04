// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-2-Clause
#include "IsaacSimRGBDSensorNWCROS2.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>

YARP_DECLARE_LOG_COMPONENT(RGBD)
YARP_LOG_COMPONENT(RGBD, "yarp.device.IsaacSimRGBDSensorNWCROS2")

yarp::dev::IsaacSimRGBDSensorNWCROS2::~IsaacSimRGBDSensorNWCROS2()
{
    close();
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::open(yarp::os::Searchable& config)
{
    m_errorHandler.setPrefix("[open] ");
    if (!m_paramsParser.parseParams(config))
    {
        m_errorHandler << "Failed to parse parameters for IsaacSimRGBDSensorNWCROS2";
        return false;
    }
    if (!rclcpp::ok())
    {
        rclcpp::InitOptions options;
        options.shutdown_on_signal = false; // <-- disable ROS2 SIGINT handler
        rclcpp::init(0, nullptr, options);
    }
    m_subscriber = std::make_shared<RGBDSubscriber>(m_paramsParser.m_node_name, m_paramsParser.m_rgb_topic_name,
                                                    m_paramsParser.m_depth_topic_name, this);
    m_executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    m_executor->add_node(m_subscriber);
    m_executorThread = std::thread([this]() { m_executor->spin(); });

    return true;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::close()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_rgbReceivedOnce = false;
    m_depthReceivedOnce = false;
    m_rgbInfoReceivedOnce = false;
    m_depthInfoReceivedOnce = false;
    if (m_subscriber)
    {
        m_executor->cancel();
        m_executorThread.join();
        m_subscriber.reset();
    }
    return true;
}

int yarp::dev::IsaacSimRGBDSensorNWCROS2::getRgbHeight()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_rgbReceivedOnce)
    {
        m_errorHandler.setPrefix("[getRgbHeight] ");
        m_errorHandler << "RGB image not received yet. Please ensure the ROS2 topics are publishing data.";
        return 0;
    }

    return m_rgbImage.height();
}

int yarp::dev::IsaacSimRGBDSensorNWCROS2::getRgbWidth()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_rgbReceivedOnce)
    {
        m_errorHandler.setPrefix("[getRgbWidth] ");
        m_errorHandler << "RGB image not received yet. Please ensure the ROS2 topics are publishing data.";
        return 0;
    }
    return m_rgbImage.width();
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::getRgbSupportedConfigurations(
    yarp::sig::VectorOf<yarp::dev::CameraConfig>& configurations)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_rgbReceivedOnce)
    {
        m_errorHandler.setPrefix("[getRgbSupportedConfigurations] ");
        m_errorHandler << "RGB image not received yet. Please ensure the ROS2 topics are publishing data.";
        return false;
    }

    configurations.clear();
    yarp::dev::CameraConfig config;
    config.width = m_rgbImage.width();
    config.height = m_rgbImage.height();
    config.pixelCoding = VOCAB_PIXEL_RGB;
    config.framerate = m_estimatedRGBFrameRate;

    configurations.push_back(config);

    return true;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::getRgbResolution(int& width, int& height)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_errorHandler.setPrefix("[getRgbResolution] ");
    if (!m_rgbReceivedOnce)
    {
        m_errorHandler << "RGB image not received yet. Please ensure the ROS2 topics are publishing data.";
        return false;
    }
    width = m_rgbImage.width();
    height = m_rgbImage.height();
    return true;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::setRgbResolution(int width, int height)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_errorHandler.setPrefix("[setRgbResolution] ");
    m_errorHandler << "Setting RGB resolution is not supported. The resolution is determined by Isaac Sim.";
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::getRgbFOV(double& horizontalFov, double& verticalFov)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_errorHandler.setPrefix("[getRgbFOV] ");
    m_errorHandler << "Getting RGB FOV is not supported. The FOV is determined by Isaac Sim.";
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::setRgbFOV(double horizontalFov, double verticalFov)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_errorHandler.setPrefix("[setRgbFOV] ");
    m_errorHandler << "Setting RGB FOV is not supported. The FOV is determined by Isaac Sim.";
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::getRgbMirroring(bool& mirror)
{
    mirror = false; // Assuming no mirroring by default
    return true;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::setRgbMirroring(bool mirror)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_errorHandler.setPrefix("[setRgbMirroring] ");
    m_errorHandler << "Setting RGB mirroring is not supported. The mirroring is determined by Isaac Sim.";
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::getRgbIntrinsicParam(yarp::os::Property& intrinsic)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_errorHandler.setPrefix("[getRgbIntrinsicParam] ");
    if (!m_rgbInfoReceivedOnce)
    {
        m_errorHandler << "RGB camera info not received yet. Please ensure the ROS2 topics are publishing data.";
        return false;
    }
    intrinsic.clear();
    m_rgbIntrinsic.toProperty(intrinsic);
    return true;
}

int yarp::dev::IsaacSimRGBDSensorNWCROS2::getDepthHeight()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_depthReceivedOnce)
    {
        m_errorHandler.setPrefix("[getDepthHeight] ");
        m_errorHandler << "Depth image not received yet. Please ensure the ROS2 topics are publishing data.";
        return 0;
    }
    return m_depthImage.height();
}

int yarp::dev::IsaacSimRGBDSensorNWCROS2::getDepthWidth()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_depthReceivedOnce)
    {
        m_errorHandler.setPrefix("[getDepthWidth] ");
        m_errorHandler << "Depth image not received yet. Please ensure the ROS2 topics are publishing data.";
        return 0;
    }
    return m_depthImage.width();
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::setDepthResolution(int width, int height)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_errorHandler.setPrefix("[setDepthResolution] ");
    m_errorHandler << "Setting depth resolution is not supported. The resolution is determined by Isaac Sim.";
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::getDepthFOV(double& horizontalFov, double& verticalFov)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_errorHandler.setPrefix("[getDepthFOV] ");
    m_errorHandler << "Getting depth FOV is not supported. The FOV is determined by Isaac Sim.";
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::setDepthFOV(double horizontalFov, double verticalFov)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_errorHandler.setPrefix("[setDepthFOV] ");
    m_errorHandler << "Setting depth FOV is not supported. The FOV is determined by Isaac Sim.";
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::getDepthIntrinsicParam(yarp::os::Property& intrinsic)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_errorHandler.setPrefix("[getDepthIntrinsicParam] ");
    if (!m_depthInfoReceivedOnce)
    {
        m_errorHandler << "Depth camera info not received yet. Please ensure the ROS2 topics are publishing data.";
        return false;
    }
    intrinsic.clear();
    m_depthIntrinsic.toProperty(intrinsic);
    return true;
}

double yarp::dev::IsaacSimRGBDSensorNWCROS2::getDepthAccuracy()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_errorHandler.setPrefix("[getDepthAccuracy] ");
    m_errorHandler << "Getting depth accuracy is not supported. The accuracy is determined by Isaac Sim.";
    return 0.0; // Return 0.0 to indicate unsupported feature
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::setDepthAccuracy(double accuracy)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_errorHandler.setPrefix("[setDepthAccuracy] ");
    m_errorHandler << "Setting depth accuracy is not supported. The accuracy is determined by Isaac Sim.";
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::getDepthClipPlanes(double& nearPlane, double& farPlane)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_errorHandler.setPrefix("[getDepthClipPlanes] ");
    m_errorHandler << "Getting depth clip planes is not supported. The clip planes are determined by Isaac Sim.";
    nearPlane = 0.0; // Default value
    farPlane = 0.0;  // Default value
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::setDepthClipPlanes(double nearPlane, double farPlane)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_errorHandler.setPrefix("[setDepthClipPlanes] ");
    m_errorHandler << "Setting depth clip planes is not supported. The clip planes are determined by Isaac Sim.";
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::getDepthMirroring(bool& mirror)
{
    mirror = false; // Assuming no mirroring by default
    return true;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::setDepthMirroring(bool mirror)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_errorHandler.setPrefix("[setDepthMirroring] ");
    m_errorHandler << "Setting depth mirroring is not supported. The mirroring is determined by Isaac Sim.";
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::getExtrinsicParam(yarp::sig::Matrix& extrinsic)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_errorHandler.setPrefix("[getExtrinsicParam] ");
    m_errorHandler << "Getting extrinsic parameters is not supported. The extrinsics are determined by Isaac Sim.";
    return false;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::getRgbImage(yarp::sig::FlexImage& rgbImage, yarp::os::Stamp* timeStamp)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_errorHandler.setPrefix("[getRgbImage] ");
    if (!m_rgbReceivedOnce)
    {
        m_errorHandler << "No RGB image received yet. Please ensure the ROS2 topics are publishing data.";
        return false;
    }

    if (timeStamp != nullptr)
    {
        *timeStamp = m_rgbTimestamp;
    }

    rgbImage = m_rgbImage;

    return true;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::getDepthImage(yarp::sig::ImageOf<yarp::sig::PixelFloat>& depthImage,
                                                         yarp::os::Stamp* timeStamp)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_errorHandler.setPrefix("[getDepthImage] ");
    if (!m_depthReceivedOnce)
    {
        m_errorHandler << "No depth image received yet. Please ensure the ROS2 topics are publishing data.";
        return false;
    }

    if (timeStamp != nullptr)
    {
        *timeStamp = m_depthTimestamp;
    }

    depthImage = m_depthImage;

    return true;
}

bool yarp::dev::IsaacSimRGBDSensorNWCROS2::getImages(yarp::sig::FlexImage& colorFrame,
                                                     yarp::sig::ImageOf<yarp::sig::PixelFloat>& depthFrame,
                                                     yarp::os::Stamp* colorStamp, yarp::os::Stamp* depthStamp)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_errorHandler.setPrefix("[getImages] ");
    if (!m_rgbReceivedOnce)
    {
        m_errorHandler << "No RGB image received yet. Please ensure the ROS2 topics are publishing data.";
        return false;
    }

    if (!m_depthReceivedOnce)
    {
        m_errorHandler << "No Depth image received yet. Please ensure the ROS2 topics are publishing data.";
        return false;
    }

    if (colorStamp != nullptr)
    {
        *colorStamp = m_rgbTimestamp;
    }
    if (depthStamp != nullptr)
    {
        *depthStamp = m_depthTimestamp;
    }

    colorFrame = m_rgbImage;
    depthFrame = m_depthImage;

    return true;
}

yarp::dev::IRGBDSensor::RGBDSensor_status yarp::dev::IsaacSimRGBDSensorNWCROS2::getSensorStatus()
{
    if (!m_rgbReceivedOnce && !m_depthReceivedOnce)
    {
        return RGBDSensor_status::RGBD_SENSOR_NOT_READY;
    }
    return RGBDSensor_status::RGBD_SENSOR_OK_IN_USE;
}

std::string yarp::dev::IsaacSimRGBDSensorNWCROS2::getLastErrorMsg(yarp::os::Stamp* timeStamp)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_errorHandler.getLastErrorMsg();
}

void yarp::dev::IsaacSimRGBDSensorNWCROS2::updateRGB(const sensor_msgs::msg::Image::ConstSharedPtr& rgb)
{
    // The code of these conversions have been ispired from
    // https://github.com/robotology/yarp-devices-ros2/blob/e1b9c86aa91c0fb3a14c6d2415e75c3868e222dc/src/devices/ros2RGBDConversionUtils/Ros2RGBDConversionUtils.cpp

    std::lock_guard<std::mutex> lock(m_mutex);

    const auto& rosPixelType = rgb->encoding;
    int yarpPixelType = VOCAB_PIXEL_INVALID;

    if (rosPixelType == sensor_msgs::image_encodings::RGB8)
    {
        yarpPixelType = VOCAB_PIXEL_RGB;
    }
    else if (rosPixelType == sensor_msgs::image_encodings::BGR8)
    {
        yarpPixelType = VOCAB_PIXEL_BGR;
    }
    else
    {
        m_errorHandler.setPrefix("[updateRGB] ");
        m_errorHandler << "Unsupported RGB pixel type: " + rosPixelType;
        return;
    }
    m_rgbImage.setQuantum(0);
    m_rgbImage.setPixelCode(yarpPixelType);
    m_rgbImage.setPixelSize(rgb->step / rgb->width); // The step is a full row length in bytes
    m_rgbImage.resize(rgb->width, rgb->height);
    size_t c = 0;
    unsigned char* rgbData = m_rgbImage.getRawImage();
    for (auto it = rgb->data.begin(); it != rgb->data.end(); it++)
    {
        rgbData[c++] = *it;
    }

    double rgb_time = rgb->header.stamp.sec + (rgb->header.stamp.nanosec / 1e9);

    if (m_rgbReceivedOnce)
    {
        m_estimatedRGBFrameRate = 1.0 / (rgb_time - m_rgbTimestamp.getTime());
    }

    m_rgbTimestamp.update(rgb_time);
    m_rgbReceivedOnce = true;
}

void yarp::dev::IsaacSimRGBDSensorNWCROS2::updateDepth(const sensor_msgs::msg::Image::ConstSharedPtr& depth)
{
    // The code of these conversions have been ispired from
    // https://github.com/robotology/yarp-devices-ros2/blob/e1b9c86aa91c0fb3a14c6d2415e75c3868e222dc/src/devices/ros2RGBDConversionUtils/Ros2RGBDConversionUtils.cpp

    std::lock_guard<std::mutex> lock(m_mutex);
    const auto& rosPixelType = depth->encoding;

    if (depth->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
    {
        m_depthImage.resize(depth->width, depth->height);
        size_t c = 0;
        uint16_t* p = (uint16_t*)(depth->data.data());
        uint16_t* siz = (uint16_t*)(depth->data.data()) + (depth->data.size() / sizeof(uint16_t));
        unsigned char* depthData = m_depthImage.getRawImage();
        int count = 0;
        for (; p < siz; p++)
        {
            float value = static_cast<float>(*p) /
                          1000.0; // Convert from millimeters to meters, since the input is a 16-bit unsigned integer
            ((float*)(depthData))[c++] = value;
            count++;
        }
    }
    else if (depth->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
        m_depthImage.resize(depth->width, depth->height);
        unsigned char* depthData = m_depthImage.getRawImage();
        size_t c = 0;
        for (auto it = depth->data.begin(); it != depth->data.end(); it++)
        {
            depthData[c++] = *it;
        }
    }
    else
    {
        m_errorHandler.setPrefix("[updateDepth] ");
        m_errorHandler << "Unsupported depth pixel type: " + rosPixelType;
        return;
    }
    double depth_time = depth->header.stamp.sec + (depth->header.stamp.nanosec / 1e9);
    if (m_depthReceivedOnce)
    {
        m_estimatedDepthFrameRate = 1.0 / (depth_time - m_depthTimestamp.getTime());
    }
    m_depthTimestamp.update(depth_time);
    m_depthReceivedOnce = true;
}

yarp::sig::IntrinsicParams yarp::dev::IsaacSimRGBDSensorNWCROS2::convertCameraInfoToIntrinsic(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cameraInfo)
{
    yarp::sig::IntrinsicParams output;
    // See
    // https://docs.isaacsim.omniverse.nvidia.com/5.0.0/ros2_tutorials/tutorial_ros2_camera.html#camera-info-helper-node
    output.focalLengthX = cameraInfo->k[0];
    output.focalLengthY = cameraInfo->k[4];
    output.principalPointX = cameraInfo->k[2];
    output.principalPointY = cameraInfo->k[5];

    output.distortionModel.type = yarp::sig::YarpDistortion::YARP_DISTORTION_NONE;
    // Even though the distortion model is specified in the CameraInfo,
    // it seems that the inputs provided by IsaacSim cannot be converted
    // to a YARP distortion model, so we set it to none.
    return output;
}

void yarp::dev::IsaacSimRGBDSensorNWCROS2::updateRGBInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& rgbInfo)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_rgbIntrinsic = convertCameraInfoToIntrinsic(rgbInfo);
    m_rgbInfoReceivedOnce = true;
}

void yarp::dev::IsaacSimRGBDSensorNWCROS2::updateDepthInfo(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& depthInfo)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_depthIntrinsic = convertCameraInfoToIntrinsic(depthInfo);
    m_depthInfoReceivedOnce = true;
}

yarp::dev::IsaacSimRGBDSensorNWCROS2::RGBDSubscriber::RGBDSubscriber(const std::string& name,
                                                                     const std::string& rgbTopic,
                                                                     const std::string& depthTopic,
                                                                     IsaacSimRGBDSensorNWCROS2* parent)
    : Node(name)
{
    m_parent = parent; // Store the parent device pointer

    int queue_size = 10;

    // Subscribe to RGB and Depth topics
    m_rgb_sub = this->create_subscription<sensor_msgs::msg::Image>(
        rgbTopic, queue_size, std::bind(&RGBDSubscriber::callback_rgb, this, std::placeholders::_1));
    m_depth_sub = this->create_subscription<sensor_msgs::msg::Image>(
        depthTopic, queue_size, std::bind(&RGBDSubscriber::callback_depth, this, std::placeholders::_1));

    // Subscribe to CameraInfo topics
    m_rgb_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        rgbTopic + "/info", queue_size, std::bind(&RGBDSubscriber::callback_rgb_info, this, std::placeholders::_1));
    m_depth_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        depthTopic + "/info", queue_size, std::bind(&RGBDSubscriber::callback_depth_info, this, std::placeholders::_1));
}

void yarp::dev::IsaacSimRGBDSensorNWCROS2::RGBDSubscriber::callback_rgb(
    const sensor_msgs::msg::Image::ConstSharedPtr& rgb)
{
    m_parent->updateRGB(rgb);
}

void yarp::dev::IsaacSimRGBDSensorNWCROS2::RGBDSubscriber::callback_depth(
    const sensor_msgs::msg::Image::ConstSharedPtr& depth)
{
    m_parent->updateDepth(depth);
}

inline void yarp::dev::IsaacSimRGBDSensorNWCROS2::RGBDSubscriber::callback_rgb_info(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& rgbInfo)
{
    m_parent->updateRGBInfo(rgbInfo);
}

void yarp::dev::IsaacSimRGBDSensorNWCROS2::RGBDSubscriber::callback_depth_info(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& depthInfo)
{
    m_parent->updateDepthInfo(depthInfo);
}

void yarp::dev::IsaacSimRGBDSensorNWCROS2::ErrorHandler::setPrefix(const std::string& prefix)
{
    m_prefix = prefix;
    m_lastErrorMsg.clear();
}

void yarp::dev::IsaacSimRGBDSensorNWCROS2::ErrorHandler::operator<<(const std::string& errorMsg)
{
    m_lastErrorMsg = m_prefix + errorMsg;
    m_errorTimestamp.update();
    yCError(RGBD, "%s", m_lastErrorMsg.c_str());
}

void yarp::dev::IsaacSimRGBDSensorNWCROS2::ErrorHandler::operator<<(const std::stringstream& errorMsg)
{
    this->operator<<(errorMsg.str());
}

const std::string& yarp::dev::IsaacSimRGBDSensorNWCROS2::ErrorHandler::getLastErrorMsg() const
{
    return m_lastErrorMsg;
}
