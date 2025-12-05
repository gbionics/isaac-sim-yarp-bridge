#include <IsaacSimControlBoardNWCROS2.h>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/Vocab.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

using namespace std::chrono_literals;

class DummyCBNode : public rclcpp::Node
{
public:
    DummyCBNode(const std::string& node_name, const std::string& joint_state_topic,
                const std::string& motor_state_topic, const std::string& joint_references_topic,
                const std::string& get_param_service, const std::string& set_param_service)
        : Node(node_name)
    {
        joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>(joint_state_topic, 10);
        motor_state_pub = this->create_publisher<sensor_msgs::msg::JointState>(motor_state_topic, 10);
        joint_references_sub = this->create_subscription<sensor_msgs::msg::JointState>(
            joint_references_topic, 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg)
            {
                std::lock_guard<std::mutex> lock(joint_references_mutex);
                last_joint_references = *msg;
                joint_references_received = true;
            });

        get_param_srv = this->create_service<rcl_interfaces::srv::GetParameters>(
            get_param_service,
            [this](const std::shared_ptr<rcl_interfaces::srv::GetParameters::Request> request,
                   std::shared_ptr<rcl_interfaces::srv::GetParameters::Response> response)
            {
                // Return dummy values for required parameters
                response->values.resize(request->names.size());
                for (size_t i = 0; i < request->names.size(); ++i)
                {
                    if (request->names[i] == "joint_names")
                    {
                        response->values[i].type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
                        response->values[i].string_array_value = {"joint_1", "joint_2", "joint_3"};
                    }
                    else if (request->names[i] == "joint_types")
                    {
                        response->values[i].type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY;
                        // 0: revolute, 1: prismatic
                        response->values[i].integer_array_value = {0, 0, 0};
                    }
                    else if (request->names[i] == "compliant_modes")
                    {
                        response->values[i].type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY;
                        response->values[i].bool_array_value = {false, false, false};
                    }
                    else if (request->names[i].find("control_modes") != std::string::npos)
                    {
                        // Handle control_modes[X] requests
                        std::lock_guard<std::mutex> lock(control_modes_mutex);
                        size_t start = request->names[i].find('[');
                        size_t end = request->names[i].find(']');
                        if (start != std::string::npos && end != std::string::npos)
                        {
                            int joint_idx = std::stoi(request->names[i].substr(start + 1, end - start - 1));
                            if (joint_idx >= 0 && joint_idx < static_cast<int>(control_modes.size()))
                            {
                                response->values[i].type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
                                response->values[i].integer_value = control_modes[joint_idx];
                            }
                        }
                        else
                        {
                            response->values[i].type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY;
                            response->values[i].integer_array_value = control_modes;
                        }
                    }
                    else
                    {
                        response->values[i].type = rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET;
                    }
                }
            });

        set_param_srv = this->create_service<rcl_interfaces::srv::SetParameters>(
            set_param_service,
            [this](const std::shared_ptr<rcl_interfaces::srv::SetParameters::Request> request,
                   std::shared_ptr<rcl_interfaces::srv::SetParameters::Response> response)
            {
                response->results.resize(request->parameters.size());
                for (size_t i = 0; i < request->parameters.size(); ++i)
                {
                    if (request->parameters[i].name.find("control_modes") != std::string::npos)
                    {
                        std::lock_guard<std::mutex> lock(control_modes_mutex);
                        // Extract joint index from parameter name like "control_modes[0]"
                        size_t start = request->parameters[i].name.find('[');
                        size_t end = request->parameters[i].name.find(']');
                        if (start != std::string::npos && end != std::string::npos)
                        {
                            int joint_idx = std::stoi(request->parameters[i].name.substr(start + 1, end - start - 1));
                            if (joint_idx >= 0 && joint_idx < static_cast<int>(control_modes.size()))
                            {
                                control_modes[joint_idx] = request->parameters[i].value.integer_value;
                            }
                        }
                        else
                        {
                            // Handle setting all control modes at once
                            control_modes = request->parameters[i].value.integer_array_value;
                            REQUIRE(control_modes.size() == 3);
                        }
                    }
                    response->results[i].successful = true;
                }
            });
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motor_state_pub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_references_sub;
    rclcpp::Service<rcl_interfaces::srv::GetParameters>::SharedPtr get_param_srv;
    rclcpp::Service<rcl_interfaces::srv::SetParameters>::SharedPtr set_param_srv;

    // Store last received joint references
    sensor_msgs::msg::JointState last_joint_references;
    std::mutex joint_references_mutex;
    bool joint_references_received{false};

    // Store control modes
    std::vector<int64_t> control_modes{VOCAB_CM_POSITION, VOCAB_CM_POSITION,
                                       VOCAB_CM_POSITION}; // Default to VOCAB_CM_POSITION
    std::mutex control_modes_mutex;

    sensor_msgs::msg::JointState getLastJointReferences()
    {
        std::lock_guard<std::mutex> lock(joint_references_mutex);
        return last_joint_references;
    }

    bool hasReceivedJointReferences()
    {
        std::lock_guard<std::mutex> lock(joint_references_mutex);
        return joint_references_received;
    }

    void resetJointReferencesFlag()
    {
        std::lock_guard<std::mutex> lock(joint_references_mutex);
        joint_references_received = false;
    }
};

TEST_CASE("IsaacSimControlBoardNWCROS2", "[ros2][isaacsim_controlboard]")
{
    rclcpp::init(0, nullptr);
    yarp::os::Network::init();

    // Dummy ROS2 node with required topics/services
    auto dummy_node = std::make_shared<DummyCBNode>("dummy_cb_node", "/joint_states", "/motor_states",
                                                    "/joint_references", "/get_parameters", "/set_parameters");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(dummy_node);

    auto sleep_duration = 50ms;

    // Start executor in a thread
    std::thread exec_thread([&executor]() { executor.spin(); });

    // Give time for services to be available
    std::this_thread::sleep_for(sleep_duration);

    yarp::dev::IsaacSimControlBoardNWCROS2 device;
    yarp::os::Property config;
    config.put("streaming_node_name", "test_isaacsim_controlboard_streaming");
    config.put("service_node_name", "test_isaacsim_controlboard_service");
    config.put("joint_state_topic_name", "/joint_states");
    config.put("motor_state_topic_name", "/motor_states");
    config.put("joint_references_topic_name", "/joint_references");
    config.put("get_parameters_service_name", "/get_parameters");
    config.put("set_parameters_service_name", "/set_parameters");
    config.put("service_request_timeout", 1.0);

    REQUIRE(device.open(config));

    int numberOfJoints = 0;
    REQUIRE(device.getAxes(&numberOfJoints));
    REQUIRE(numberOfJoints == 3);

    constexpr double deg2rad = M_PI / 180.0;
    constexpr double rad2deg = 180.0 / M_PI;

    SECTION("Basic functionality")
    {
        // Test that the joint names are correctly retrieved
        for (int i = 0; i < numberOfJoints; ++i)
        {
            std::string joint_name;
            REQUIRE(device.getAxisName(i, joint_name));
            std::string expected_name = "joint_" + std::to_string(i + 1);
            REQUIRE(std::string(joint_name) == expected_name);
        }
    }

    SECTION("getEncoders function")
    {
        // Test getEncoders before receiving any data (should fail)
        std::vector<double> encoders(numberOfJoints);
        REQUIRE_FALSE(device.getEncoders(encoders.data()));

        // Publish joint state data (ROS2 publishes in radians)
        sensor_msgs::msg::JointState joint_state_msg;
        joint_state_msg.header.stamp = dummy_node->now();
        joint_state_msg.name = {"joint_1", "joint_2", "joint_3"};
        joint_state_msg.position = {0.1, 0.2, 0.3}; // radians
        joint_state_msg.velocity = {0.0, 0.0, 0.0};
        joint_state_msg.effort = {0.0, 0.0, 0.0};

        dummy_node->joint_state_pub->publish(joint_state_msg);

        // Give time for the message to be received
        std::this_thread::sleep_for(sleep_duration);

        // Test getEncoders after receiving data (should succeed)
        // Device returns in degrees for revolute joints
        REQUIRE(device.getEncoders(encoders.data()));
        REQUIRE_THAT(encoders[0], Catch::Matchers::WithinAbs(0.1 * rad2deg, 1e-5));
        REQUIRE_THAT(encoders[1], Catch::Matchers::WithinAbs(0.2 * rad2deg, 1e-5));
        REQUIRE_THAT(encoders[2], Catch::Matchers::WithinAbs(0.3 * rad2deg, 1e-5));

        // Publish updated joint state data (ROS2 publishes in radians)
        joint_state_msg.header.stamp = dummy_node->now();
        joint_state_msg.position = {1.5, -0.5, 2.0}; // radians
        dummy_node->joint_state_pub->publish(joint_state_msg);

        // Give time for the message to be received
        std::this_thread::sleep_for(sleep_duration);

        // Test getEncoders with updated data (device returns in degrees)
        REQUIRE(device.getEncoders(encoders.data()));
        REQUIRE_THAT(encoders[0], Catch::Matchers::WithinAbs(1.5 * rad2deg, 1e-5));
        REQUIRE_THAT(encoders[1], Catch::Matchers::WithinAbs(-0.5 * rad2deg, 1e-5));
        REQUIRE_THAT(encoders[2], Catch::Matchers::WithinAbs(2.0 * rad2deg, 1e-5));
    }

    SECTION("setRefSpeeds with positionMove - all joints")
    {
        dummy_node->resetJointReferencesFlag();

        // Set reference speeds for all joints (input in degrees/second for revolute joints)
        std::vector<double> ref_speeds = {10.0, 20.0, 30.0}; // deg/s
        REQUIRE(device.setRefSpeeds(ref_speeds.data()));

        // Set reference positions for all joints (input in degrees for revolute joints)
        // setRefSpeeds must be called before positionMove to work together
        std::vector<double> ref_positions = {30.0, 45.0, 60.0}; // degrees
        REQUIRE(device.positionMove(ref_positions.data()));

        // Give time for the message to be published
        std::this_thread::sleep_for(sleep_duration);

        // Check that joint references were published with both velocity and position fields set
        REQUIRE(dummy_node->hasReceivedJointReferences());
        auto joint_refs = dummy_node->getLastJointReferences();
        REQUIRE(joint_refs.name.size() == 3);
        REQUIRE(joint_refs.velocity.size() == 3);
        REQUIRE(joint_refs.position.size() == 3);
        // Device should publish velocities in radians/second
        REQUIRE_THAT(joint_refs.velocity[0], Catch::Matchers::WithinAbs(ref_speeds[0] * deg2rad, 1e-5));
        REQUIRE_THAT(joint_refs.velocity[1], Catch::Matchers::WithinAbs(ref_speeds[1] * deg2rad, 1e-5));
        REQUIRE_THAT(joint_refs.velocity[2], Catch::Matchers::WithinAbs(ref_speeds[2] * deg2rad, 1e-5));
        // Device should publish positions in radians
        REQUIRE_THAT(joint_refs.position[0], Catch::Matchers::WithinAbs(ref_positions[0] * deg2rad, 1e-5));
        REQUIRE_THAT(joint_refs.position[1], Catch::Matchers::WithinAbs(ref_positions[1] * deg2rad, 1e-5));
        REQUIRE_THAT(joint_refs.position[2], Catch::Matchers::WithinAbs(ref_positions[2] * deg2rad, 1e-5));
    }

    SECTION("setRefSpeed with positionMove - single joint")
    {
        dummy_node->resetJointReferencesFlag();

        // Set reference speed for joint 1 (input in degrees/second)
        double ref_speed = 45.0; // deg/s
        REQUIRE(device.setRefSpeed(1, ref_speed));

        // Set reference position for joint 1 (input in degrees)
        // setRefSpeed must be called before positionMove to work together
        double ref_position = 90.0; // degrees
        REQUIRE(device.positionMove(1, ref_position));

        // Give time for the message to be published
        std::this_thread::sleep_for(sleep_duration);

        // Check that joint references were published
        REQUIRE(dummy_node->hasReceivedJointReferences());
        auto joint_refs = dummy_node->getLastJointReferences();
        REQUIRE(joint_refs.velocity.size() >= 2);
        REQUIRE(joint_refs.position.size() >= 2);
        // Device should publish in radians/second and radians
        REQUIRE_THAT(joint_refs.velocity[1], Catch::Matchers::WithinAbs(ref_speed * deg2rad, 1e-5));
        REQUIRE_THAT(joint_refs.position[1], Catch::Matchers::WithinAbs(ref_position * deg2rad, 1e-5));
    }

    SECTION("positionMove - subset of joints")
    {
        dummy_node->resetJointReferencesFlag();

        // Set reference speeds for joints 0 and 2 (input in degrees/second)
        int n_joints = 2;
        int joints[] = {0, 2};
        double ref_speeds[] = {15.0, 25.0}; // deg/s
        REQUIRE(device.setRefSpeeds(n_joints, joints, ref_speeds));

        // Set reference positions for joints 0 and 2 (input in degrees)
        // setRefSpeeds must be called before positionMove to work together
        double ref_positions[] = {15.0, 75.0}; // degrees
        REQUIRE(device.positionMove(n_joints, joints, ref_positions));

        // Give time for the message to be published
        std::this_thread::sleep_for(sleep_duration);

        // Check that joint references were published
        REQUIRE(dummy_node->hasReceivedJointReferences());
        auto joint_refs = dummy_node->getLastJointReferences();
        // Should contain the specified joints
        REQUIRE(joint_refs.position.size() >= 2);
        REQUIRE(joint_refs.velocity.size() >= 2);
        // Find the positions and velocities for the specified joints
        for (int i = 0; i < n_joints; ++i)
        {
            bool found = false;
            for (size_t j = 0; j < joint_refs.name.size(); ++j)
            {
                if (joint_refs.name[j] == "joint_" + std::to_string(joints[i] + 1))
                {
                    // Device should publish velocities in radians/second and positions in radians
                    REQUIRE_THAT(joint_refs.velocity[j], Catch::Matchers::WithinAbs(ref_speeds[i] * deg2rad, 1e-5));
                    REQUIRE_THAT(joint_refs.position[j], Catch::Matchers::WithinAbs(ref_positions[i] * deg2rad, 1e-5));
                    found = true;
                    break;
                }
            }
            REQUIRE(found);
        }
    }

    SECTION("Control modes")
    {
        // Test setControlMode for single joint
        REQUIRE(device.setControlMode(0, VOCAB_CM_VELOCITY));
        std::this_thread::sleep_for(sleep_duration);

        // Test getControlMode for single joint
        int mode = 0;
        REQUIRE(device.getControlMode(0, &mode));
        REQUIRE(mode == VOCAB_CM_VELOCITY);

        // Test setControlModes for all joints
        std::vector<int> modes_to_set = {VOCAB_CM_POSITION, VOCAB_CM_TORQUE, VOCAB_CM_VELOCITY};
        REQUIRE(device.setControlModes(modes_to_set.data()));
        std::this_thread::sleep_for(sleep_duration);

        // Test getControlModes for all joints
        std::vector<int> modes_read(numberOfJoints);
        REQUIRE(device.getControlModes(modes_read.data()));
        REQUIRE(modes_read[0] == VOCAB_CM_POSITION);
        REQUIRE(modes_read[1] == VOCAB_CM_TORQUE);
        REQUIRE(modes_read[2] == VOCAB_CM_VELOCITY);

        // Test setControlModes for subset of joints
        int n_joints = 2;
        int joints[] = {0, 2};
        int modes_subset[] = {VOCAB_CM_TORQUE, VOCAB_CM_POSITION};
        REQUIRE(device.setControlModes(n_joints, joints, modes_subset));
        std::this_thread::sleep_for(sleep_duration);

        // Test getControlModes for subset of joints
        int modes_subset_read[2];
        REQUIRE(device.getControlModes(n_joints, joints, modes_subset_read));
        REQUIRE(modes_subset_read[0] == VOCAB_CM_TORQUE);
        REQUIRE(modes_subset_read[1] == VOCAB_CM_POSITION);
    }

    REQUIRE(device.close());

    executor.cancel();
    exec_thread.join();
    rclcpp::shutdown();
}
