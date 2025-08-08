// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-2-Clause

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <chrono>
#include <vector>
#include <string>
#include <memory>

using namespace std::chrono_literals;

class TestNode : public rclcpp::Node {
public:
    TestNode() : Node("test_node") {
        rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/rgb/image_raw", 10,
            [this](sensor_msgs::msg::Image::SharedPtr msg) {
            rgb_received_ = msg;
        }
        );

        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_raw", 10,
            [this](sensor_msgs::msg::Image::SharedPtr msg) {
            depth_received_ = msg;
        }
        );

        rgb_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/rgb/image_raw", 10);
        depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/depth/image_raw", 10);
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

        rgb_pub_->publish(rgb_msg);
        depth_pub_->publish(depth_msg);
    }

    sensor_msgs::msg::Image::SharedPtr rgb_received_;
    sensor_msgs::msg::Image::SharedPtr depth_received_;

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
};

TEST_CASE("RGB and Depth image reception", "[ros2]") {
    rclcpp::init(0, nullptr);

    auto node = std::make_shared<TestNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    // Publish dummy messages
    node->publish_dummy_images();

    auto start = std::chrono::steady_clock::now();
    bool got_both = false;

    while (std::chrono::steady_clock::now() - start < 2s) {
        executor.spin_some();
        if (node->rgb_received_ && node->depth_received_) {
            got_both = true;
            break;
        }
        std::this_thread::sleep_for(50ms);
    }

    REQUIRE(got_both);
    REQUIRE(node->rgb_received_->encoding == "rgb8");
    REQUIRE(node->rgb_received_->data.size() == 12); // 2x2 RGB

    REQUIRE(node->depth_received_->encoding == "32FC1");
    REQUIRE(node->depth_received_->data.size() == 16); // 2x2 float
    float received_depth;
    std::memcpy(&received_depth, node->depth_received_->data.data(), sizeof(float));
    REQUIRE_THAT(received_depth,
        Catch::Matchers::WithinULP(1.23f, 0));
    rclcpp::shutdown();
}
