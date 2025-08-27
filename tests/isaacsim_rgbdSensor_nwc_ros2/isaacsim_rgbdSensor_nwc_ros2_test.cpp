// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-2-Clause

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <IsaacSimRGBDSensorNWCROS2.h>
#include <yarp/sig/Image.h>
#include <yarp/os/Property.h>
#include <yarp/os/Network.h>

#include <chrono>
#include <vector>
#include <string>
#include <memory>

using namespace std::chrono_literals;

class TestNode : public rclcpp::Node {
public:
    TestNode() : Node("test_node") {
        m_rgb_pub = this->create_publisher<sensor_msgs::msg::Image>("/camera/rgb/image_raw", 10);
        m_depth_pub = this->create_publisher<sensor_msgs::msg::Image>("/camera/depth/image_raw", 10);
        m_rgb_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/rgb/image_raw/info", 10);
        m_depth_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/depth/image_raw/info", 10);
    }

    void publish_dummy_images() {
        auto rgb_msg = sensor_msgs::msg::Image();
        rgb_msg.height = 2;
        rgb_msg.width = 2;
        rgb_msg.encoding = "rgb8";
        rgb_msg.is_bigendian = false;
        rgb_msg.step = rgb_msg.width * 3;
        rgb_msg.data = { 255, 0, 0,   0, 255, 0,   0, 0, 255,   255, 255, 255 };
        rgb_msg.header.stamp = now();
        rgb_msg.header.frame_id = "rgb_frame";

        auto depth_msg = sensor_msgs::msg::Image();
        depth_msg.height = 2;
        depth_msg.width = 2;
        depth_msg.encoding = "32FC1";
        depth_msg.is_bigendian = false;
        depth_msg.step = depth_msg.width * 4;
        float depth_value = 1.23f;
        depth_msg.data.resize(depth_msg.step * depth_msg.height);
        std::memcpy(depth_msg.data.data(), &depth_value, sizeof(float));
        depth_msg.header.stamp = now();
        depth_msg.header.frame_id = "depth_frame";

        m_rgb_pub->publish(rgb_msg);
        m_depth_pub->publish(depth_msg);
    }

    void publish_dummy_camera_info() {
        auto rgb_info_msg = sensor_msgs::msg::CameraInfo();
        rgb_info_msg.height = 2;
        rgb_info_msg.width = 2;
        rgb_info_msg.k = { 100.0, 0.0, 1.0, 0.0, 100.0, 1.0, 0.0, 0.0, 1.0 };
        rgb_info_msg.header.stamp = now();
        rgb_info_msg.header.frame_id = "rgb_frame";
        m_rgb_info_pub->publish(rgb_info_msg);
        auto depth_info_msg = sensor_msgs::msg::CameraInfo();
        depth_info_msg.height = 2;
        depth_info_msg.width = 2;
        depth_info_msg.k = { 100.0, 0.0, 1.0, 0.0, 100.0, 1.0, 0.0, 0.0, 1.0 };
        depth_info_msg.header.stamp = now();
        depth_info_msg.header.frame_id = "depth_frame";
        m_depth_info_pub->publish(depth_info_msg);
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_rgb_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_depth_pub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr m_rgb_info_pub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr m_depth_info_pub;
};

TEST_CASE("RGB and Depth image reception", "[ros2]") {
    rclcpp::init(0, nullptr);
    yarp::os::Network::init();

    auto node = std::make_shared<TestNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    auto device = yarp::dev::IsaacSimRGBDSensorNWCROS2();

    yarp::os::Property config;
    config.put("node_name", "test_isaacsim_rgbd_sensor");
    config.put("rgb_topic_name", "/camera/rgb/image_raw");
    config.put("depth_topic_name", "/camera/depth/image_raw");

    REQUIRE(device.open(config));

    // Publish dummy messages
    node->publish_dummy_images();

    auto start = std::chrono::steady_clock::now();
    bool received = false;
    yarp::sig::FlexImage rgb_image;
    yarp::sig::ImageOf<yarp::sig::PixelFloat> depth_image;

    while (std::chrono::steady_clock::now() - start < 2s) {
        executor.spin_some();
        if (device.getSensorStatus() == yarp::dev::IRGBDSensor::RGBD_SENSOR_OK_IN_USE
            && device.getImages(rgb_image, depth_image)) {
            received = true;
            break;
        }
        std::this_thread::sleep_for(50ms);
    }

    REQUIRE(received);
    REQUIRE(rgb_image.getPixelCode() == VOCAB_PIXEL_RGB);
    REQUIRE(rgb_image.height() == 2);
    REQUIRE(rgb_image.width() == 2);
    REQUIRE(rgb_image.getRawImage()[0] == 255);
    REQUIRE(rgb_image.getRawImage()[1] == 0);
    REQUIRE(rgb_image.getRawImage()[2] == 0);
    REQUIRE(rgb_image.getRawImage()[3] == 0);
    REQUIRE(rgb_image.getRawImage()[4] == 255);
    REQUIRE(rgb_image.getRawImage()[5] == 0);
    REQUIRE(rgb_image.getRawImage()[6] == 0);
    REQUIRE(rgb_image.getRawImage()[7] == 0);
    REQUIRE(rgb_image.getRawImage()[8] == 255);
    REQUIRE(rgb_image.getRawImage()[9] == 255);
    REQUIRE(rgb_image.getRawImage()[10] == 255);
    REQUIRE(rgb_image.getRawImage()[11] == 255);


    REQUIRE(depth_image.height() == 2);
    REQUIRE(depth_image.width() == 2);

    float received_depth;
    std::memcpy(&received_depth, depth_image.getRawImage(), sizeof(float));
    REQUIRE_THAT(received_depth,
        Catch::Matchers::WithinULP(1.23f, 0));


    node->publish_dummy_camera_info();

    start = std::chrono::steady_clock::now();
    received = false;

    yarp::os::Property rgb_intrinsic;
    yarp::os::Property depth_intrinsic;

    while (std::chrono::steady_clock::now() - start < 2s) {
        executor.spin_some();
        if (device.getRgbIntrinsicParam(rgb_intrinsic)
            && device.getDepthIntrinsicParam(depth_intrinsic)) {
            received = true;
            break;
        }
        std::this_thread::sleep_for(50ms);
    }

    REQUIRE(received);
    REQUIRE_THAT(rgb_intrinsic.find("focalLengthX").asFloat64(),
        Catch::Matchers::WithinULP(100.0, 0));
    REQUIRE_THAT(rgb_intrinsic.find("focalLengthY").asFloat64(),
        Catch::Matchers::WithinULP(100.0, 0));
    REQUIRE_THAT(rgb_intrinsic.find("principalPointX").asFloat64(),
        Catch::Matchers::WithinULP(1.0, 0));
    REQUIRE_THAT(rgb_intrinsic.find("principalPointY").asFloat64(),
        Catch::Matchers::WithinULP(1.0, 0));

    REQUIRE_THAT(depth_intrinsic.find("focalLengthX").asFloat64(),
        Catch::Matchers::WithinULP(100.0, 0));
    REQUIRE_THAT(depth_intrinsic.find("focalLengthY").asFloat64(),
        Catch::Matchers::WithinULP(100.0, 0));
    REQUIRE_THAT(depth_intrinsic.find("principalPointX").asFloat64(),
        Catch::Matchers::WithinULP(1.0, 0));
    REQUIRE_THAT(depth_intrinsic.find("principalPointY").asFloat64(),
        Catch::Matchers::WithinULP(1.0, 0));

    REQUIRE(device.close());
    rclcpp::shutdown();
}
