// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-2-Clause

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include <IsaacSimClockROS2.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class TestNode : public rclcpp::Node
{
public:
    TestNode() : Node("test_node")
    {
        m_clock_pub = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
    }

    void publish_dummy_clock(rclcpp::Time timestamp)
    {
        auto clock_msg = rosgraph_msgs::msg::Clock();
        clock_msg.clock = timestamp;
        m_clock_pub->publish(clock_msg);
    }

private:
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr m_clock_pub;
};

TEST_CASE("Test use of clock in yarp (with nameserver)", "[ros2]")
{
    yarp::os::Network::init();

    if (!yarp::os::Network::checkNetwork(0.5))
    {
        WARN("YARP network is not available, skipping the test with the nameserver");
        return;
    }

    rclcpp::init(0, nullptr);

    auto node = std::make_shared<TestNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    auto device = yarp::dev::IsaacSimClockROS2();

    yarp::os::Property config;
    config.put("node_name", "test_isaacsim_clock");
    REQUIRE(device.open(config));

    REQUIRE(yarp::os::NetworkBase::exists("/clock"));

    // When explicitly setting YARP_CLOCK_NETWORK, it is not possible to use the local mode
    std::thread clock_setter_thread([]() { yarp::os::NetworkBase::yarpClockInit(yarp::os::YARP_CLOCK_NETWORK); });
    clock_setter_thread.detach();

    rclcpp::Time stamp(1, 23);
    // Publish dummy messages
    double timestamp_value = stamp.seconds();
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < 2s && std::abs(yarp::os::Time::now() - timestamp_value) > 1e-9)
    {
        node->publish_dummy_clock(stamp);
        executor.spin_some();
        std::this_thread::sleep_for(50ms);
    }

    REQUIRE_THAT(yarp::os::Time::now(), Catch::Matchers::WithinULP(timestamp_value, 1));

    REQUIRE(device.close());
    rclcpp::shutdown();
}

TEST_CASE("Test use of clock in yarp (local mode)", "[ros2]")
{
    rclcpp::init(0, nullptr);

    yarp::os::Network::init();
    if (!yarp::os::Network::checkNetwork(0.5))
    {
        // We will be opening yarp ports in the tests, but we do not want to run the yarpserver, so let's set YARP
        // in local mode (everything will work fine as long as all the ports are opened in the same process)
        yarp::os::NetworkBase::setLocalMode(true);
    }

    auto node = std::make_shared<TestNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    auto device = yarp::dev::IsaacSimClockROS2();

    yarp::os::Property config;
    config.put("node_name", "test_isaacsim_clock");
    REQUIRE(device.open(config));

    REQUIRE(yarp::os::NetworkBase::exists("/clock"));

    yarp::os::BufferedPort<yarp::os::Bottle> test_port;
    REQUIRE(test_port.open("/test_clock_port"));
    REQUIRE(yarp::os::Network::connect("/clock", "/test_clock_port"));

    rclcpp::Time stamp(1, 23);
    // Publish dummy messages
    double timestamp_value = stamp.seconds();
    yarp::os::Bottle* b = nullptr;
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < 2s && b == nullptr)
    {
        b = test_port.read(false);
        node->publish_dummy_clock(stamp);
        executor.spin_some();
        std::this_thread::sleep_for(50ms);
    }
    REQUIRE(b != nullptr);
    REQUIRE(b->size() == 2);
    REQUIRE(b->get(0).asInt32() == 1);
    REQUIRE(b->get(1).asInt32() == 23);
    REQUIRE(device.close());
    rclcpp::shutdown();
}
