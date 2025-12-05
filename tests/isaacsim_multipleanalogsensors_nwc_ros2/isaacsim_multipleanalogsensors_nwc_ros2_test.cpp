// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-2-Clause

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <IsaacSimMultipleAnalogSensorsNWCROS2.h>
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
        m_imu1_pub = this->create_publisher<sensor_msgs::msg::Imu>("/test/IMU/imu_1", 10);
        m_imu2_pub = this->create_publisher<sensor_msgs::msg::Imu>("/test/IMU/imu_2", 10);
        m_ft1_pub = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/test/FT/ft_1", 10);
        m_ft2_pub = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/test/FT/ft_2", 10);
    }

    void publish_dummy_values(rclcpp::Time timestamp)
    {
        constexpr double deg2rad = M_PI / 180.0;
        auto imu1_msg = sensor_msgs::msg::Imu();
        imu1_msg.angular_velocity.x = 0.1 * deg2rad;
        imu1_msg.angular_velocity.y = 0.2 * deg2rad;
        imu1_msg.angular_velocity.z = 0.3 * deg2rad;
        imu1_msg.linear_acceleration.x = 9.81;
        imu1_msg.linear_acceleration.y = 0.1;
        imu1_msg.linear_acceleration.z = 0.2;
        imu1_msg.orientation.w = 1.0;
        imu1_msg.orientation.x = 0.0;
        imu1_msg.orientation.y = 0.0;
        imu1_msg.orientation.z = 0.0;
        imu1_msg.header.stamp = timestamp;
        imu1_msg.header.frame_id = "imu_1_frame";
        m_imu1_pub->publish(imu1_msg);
        auto imu2_msg = sensor_msgs::msg::Imu();
        imu2_msg.angular_velocity.x = 0.4 * deg2rad;
        imu2_msg.angular_velocity.y = 0.5 * deg2rad;
        imu2_msg.angular_velocity.z = 0.6 * deg2rad;
        imu2_msg.linear_acceleration.x = 9.81;
        imu2_msg.linear_acceleration.y = 0.3;
        imu2_msg.linear_acceleration.z = 0.4;
        // The following quaternion should correspond to RPY = (10, 20, 30) degrees
        imu2_msg.orientation.w = 0.9515485;
        imu2_msg.orientation.x = 0.0381346;
        imu2_msg.orientation.y = 0.1893079;
        imu2_msg.orientation.z = 0.2392984;
        imu2_msg.header.stamp = timestamp;
        imu2_msg.header.frame_id = "imu_2_frame";
        m_imu2_pub->publish(imu2_msg);
        auto ft1_msg = geometry_msgs::msg::WrenchStamped();
        ft1_msg.wrench.force.x = 1.0;
        ft1_msg.wrench.force.y = 2.0;
        ft1_msg.wrench.force.z = 3.0;
        ft1_msg.wrench.torque.x = 0.1;
        ft1_msg.wrench.torque.y = 0.2;
        ft1_msg.wrench.torque.z = 0.3;
        ft1_msg.header.stamp = timestamp;
        ft1_msg.header.frame_id = "ft_1_frame";
        m_ft1_pub->publish(ft1_msg);
        auto ft2_msg = geometry_msgs::msg::WrenchStamped();
        ft2_msg.wrench.force.x = 4.0;
        ft2_msg.wrench.force.y = 5.0;
        ft2_msg.wrench.force.z = 6.0;
        ft2_msg.wrench.torque.x = 0.4;
        ft2_msg.wrench.torque.y = 0.5;
        ft2_msg.wrench.torque.z = 0.6;
        ft2_msg.header.stamp = timestamp;
        ft2_msg.header.frame_id = "ft_2_frame";
        m_ft2_pub->publish(ft2_msg);
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imu1_pub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imu2_pub;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr m_ft1_pub;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr m_ft2_pub;
};

TEST_CASE("IMU and FT messages reception", "[ros2]")
{
    rclcpp::init(0, nullptr);
    yarp::os::Network::init();

    auto node = std::make_shared<TestNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    auto device = yarp::dev::IsaacSimMultipleAnalogSensorsNWCROS2();

    yarp::os::Property config;
    config.put("node_name", "test_isaacsim_rgbd_sensor");
    yarp::os::Value imu_topics;
    yarp::os::Value ft_topics;
    auto* imu_topic_list = imu_topics.asList();
    auto* ft_topic_list = ft_topics.asList();
    imu_topic_list->addString("/test/IMU/imu_1");
    imu_topic_list->addString("/test/IMU/imu_2");
    ft_topic_list->addString("/test/FT/ft_1");
    ft_topic_list->addString("/test/FT/ft_2");

    config.put("imu_topic_names", imu_topics);
    config.put("ft_topic_names", ft_topics);
    config.put("init_wait_time", -1.0);

    REQUIRE(device.open(config));
    rclcpp::Time now = node->get_clock()->now();
    // Publish dummy messages
    node->publish_dummy_values(now);
    double timestamp_value = now.seconds();

    auto start = std::chrono::steady_clock::now();
    bool received = false;

    while (std::chrono::steady_clock::now() - start < 2s)
    {
        executor.spin_some();
        if (device.getOrientationSensorStatus(0) == yarp::dev::MAS_status::MAS_OK &&
            device.getOrientationSensorStatus(1) == yarp::dev::MAS_status::MAS_OK &&
            device.getThreeAxisGyroscopeStatus(0) == yarp::dev::MAS_status::MAS_OK &&
            device.getThreeAxisGyroscopeStatus(1) == yarp::dev::MAS_status::MAS_OK &&
            device.getThreeAxisLinearAccelerometerStatus(0) == yarp::dev::MAS_status::MAS_OK &&
            device.getThreeAxisLinearAccelerometerStatus(1) == yarp::dev::MAS_status::MAS_OK &&
            device.getSixAxisForceTorqueSensorStatus(0) == yarp::dev::MAS_status::MAS_OK &&
            device.getSixAxisForceTorqueSensorStatus(1) == yarp::dev::MAS_status::MAS_OK)
        {
            received = true;
            break;
        }
        std::this_thread::sleep_for(50ms);
    }

    REQUIRE(received);
    REQUIRE(device.getNrOfOrientationSensors() == 2);
    REQUIRE(device.getNrOfThreeAxisGyroscopes() == 2);
    REQUIRE(device.getNrOfThreeAxisLinearAccelerometers() == 2);
    REQUIRE(device.getNrOfSixAxisForceTorqueSensors() == 2);
    std::string name, frame;
    REQUIRE(device.getOrientationSensorName(0, name));
    REQUIRE(name == "imu_1");
    REQUIRE(device.getOrientationSensorName(1, name));
    REQUIRE(name == "imu_2");
    REQUIRE(device.getOrientationSensorFrameName(0, frame));
    REQUIRE(frame == "imu_1_frame");
    REQUIRE(device.getOrientationSensorFrameName(1, frame));
    REQUIRE(frame == "imu_2_frame");
    REQUIRE(device.getThreeAxisGyroscopeName(0, name));
    REQUIRE(name == "imu_1");
    REQUIRE(device.getThreeAxisGyroscopeName(1, name));
    REQUIRE(name == "imu_2");
    REQUIRE(device.getThreeAxisGyroscopeFrameName(0, frame));
    REQUIRE(frame == "imu_1_frame");
    REQUIRE(device.getThreeAxisGyroscopeFrameName(1, frame));
    REQUIRE(frame == "imu_2_frame");
    REQUIRE(device.getThreeAxisLinearAccelerometerName(0, name));
    REQUIRE(name == "imu_1");
    REQUIRE(device.getThreeAxisLinearAccelerometerName(1, name));
    REQUIRE(name == "imu_2");
    REQUIRE(device.getThreeAxisLinearAccelerometerFrameName(0, frame));
    REQUIRE(frame == "imu_1_frame");
    REQUIRE(device.getThreeAxisLinearAccelerometerFrameName(1, frame));
    REQUIRE(frame == "imu_2_frame");
    REQUIRE(device.getSixAxisForceTorqueSensorName(0, name));
    REQUIRE(name == "ft_1");
    REQUIRE(device.getSixAxisForceTorqueSensorName(1, name));
    REQUIRE(name == "ft_2");
    REQUIRE(device.getSixAxisForceTorqueSensorFrameName(0, frame));
    REQUIRE(frame == "ft_1_frame");
    REQUIRE(device.getSixAxisForceTorqueSensorFrameName(1, frame));
    REQUIRE(frame == "ft_2_frame");
    yarp::sig::Vector vec;
    double timestamp;
    REQUIRE(device.getOrientationSensorMeasureAsRollPitchYaw(0, vec, timestamp));
    REQUIRE(vec.size() == 3);
    REQUIRE_THAT(vec[0], Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(vec[1], Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(vec[2], Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(timestamp, Catch::Matchers::WithinAbs(timestamp_value, 1e-5));

    REQUIRE(device.getOrientationSensorMeasureAsRollPitchYaw(1, vec, timestamp));
    REQUIRE(vec.size() == 3);
    REQUIRE_THAT(vec[0], Catch::Matchers::WithinAbs(10.0, 1e-5));
    REQUIRE_THAT(vec[1], Catch::Matchers::WithinAbs(20.0, 1e-5));
    REQUIRE_THAT(vec[2], Catch::Matchers::WithinAbs(30.0, 1e-5));
    REQUIRE_THAT(timestamp, Catch::Matchers::WithinAbs(timestamp_value, 1e-5));

    REQUIRE(device.getThreeAxisGyroscopeMeasure(0, vec, timestamp));
    REQUIRE(vec.size() == 3);
    REQUIRE_THAT(vec[0], Catch::Matchers::WithinAbs(0.1, 1e-5));
    REQUIRE_THAT(vec[1], Catch::Matchers::WithinAbs(0.2, 1e-5));
    REQUIRE_THAT(vec[2], Catch::Matchers::WithinAbs(0.3, 1e-5));
    REQUIRE_THAT(timestamp, Catch::Matchers::WithinAbs(timestamp_value, 1e-5));

    REQUIRE(device.getThreeAxisGyroscopeMeasure(1, vec, timestamp));
    REQUIRE(vec.size() == 3);
    REQUIRE_THAT(vec[0], Catch::Matchers::WithinAbs(0.4, 1e-5));
    REQUIRE_THAT(vec[1], Catch::Matchers::WithinAbs(0.5, 1e-5));
    REQUIRE_THAT(vec[2], Catch::Matchers::WithinAbs(0.6, 1e-5));
    REQUIRE_THAT(timestamp, Catch::Matchers::WithinAbs(timestamp_value, 1e-5));

    REQUIRE(device.getThreeAxisLinearAccelerometerMeasure(0, vec, timestamp));
    REQUIRE(vec.size() == 3);
    REQUIRE_THAT(vec[0], Catch::Matchers::WithinAbs(9.81, 1e-5));
    REQUIRE_THAT(vec[1], Catch::Matchers::WithinAbs(0.1, 1e-5));
    REQUIRE_THAT(vec[2], Catch::Matchers::WithinAbs(0.2, 1e-5));
    REQUIRE_THAT(timestamp, Catch::Matchers::WithinAbs(timestamp_value, 1e-5));

    REQUIRE(device.getThreeAxisLinearAccelerometerMeasure(1, vec, timestamp));
    REQUIRE(vec.size() == 3);
    REQUIRE_THAT(vec[0], Catch::Matchers::WithinAbs(9.81, 1e-5));
    REQUIRE_THAT(vec[1], Catch::Matchers::WithinAbs(0.3, 1e-5));
    REQUIRE_THAT(vec[2], Catch::Matchers::WithinAbs(0.4, 1e-5));
    REQUIRE_THAT(timestamp, Catch::Matchers::WithinAbs(timestamp_value, 1e-5));

    REQUIRE(device.getSixAxisForceTorqueSensorMeasure(0, vec, timestamp));
    REQUIRE(vec.size() == 6);
    REQUIRE_THAT(vec[0], Catch::Matchers::WithinAbs(1.0, 1e-5));
    REQUIRE_THAT(vec[1], Catch::Matchers::WithinAbs(2.0, 1e-5));
    REQUIRE_THAT(vec[2], Catch::Matchers::WithinAbs(3.0, 1e-5));
    REQUIRE_THAT(vec[3], Catch::Matchers::WithinAbs(0.1, 1e-5));
    REQUIRE_THAT(vec[4], Catch::Matchers::WithinAbs(0.2, 1e-5));
    REQUIRE_THAT(vec[5], Catch::Matchers::WithinAbs(0.3, 1e-5));
    REQUIRE_THAT(timestamp, Catch::Matchers::WithinAbs(timestamp_value, 1e-5));

    REQUIRE(device.getSixAxisForceTorqueSensorMeasure(1, vec, timestamp));
    REQUIRE(vec.size() == 6);
    REQUIRE_THAT(vec[0], Catch::Matchers::WithinAbs(4.0, 1e-5));
    REQUIRE_THAT(vec[1], Catch::Matchers::WithinAbs(5.0, 1e-5));
    REQUIRE_THAT(vec[2], Catch::Matchers::WithinAbs(6.0, 1e-5));
    REQUIRE_THAT(vec[3], Catch::Matchers::WithinAbs(0.4, 1e-5));
    REQUIRE_THAT(vec[4], Catch::Matchers::WithinAbs(0.5, 1e-5));
    REQUIRE_THAT(vec[5], Catch::Matchers::WithinAbs(0.6, 1e-5));
    REQUIRE_THAT(timestamp, Catch::Matchers::WithinAbs(timestamp_value, 1e-5));

    REQUIRE(device.close());
    rclcpp::shutdown();
}
