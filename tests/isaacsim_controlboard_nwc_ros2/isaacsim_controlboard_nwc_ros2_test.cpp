#include <catch2/catch_test_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <yarp/os/Property.h>
#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <IsaacSimControlBoardNWCROS2.h>

#include <thread>
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

class DummyCBNode : public rclcpp::Node {
public:
    DummyCBNode(const std::string& node_name,
        const std::string& joint_state_topic,
        const std::string& motor_state_topic,
        const std::string& joint_references_topic,
        const std::string& get_param_service,
        const std::string& set_param_service)
        : Node(node_name)
    {
        joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>(joint_state_topic, 10);
        motor_state_pub = this->create_publisher<sensor_msgs::msg::JointState>(motor_state_topic, 10);
        joint_references_sub = this->create_subscription<sensor_msgs::msg::JointState>(
            joint_references_topic, 10,
            [](const sensor_msgs::msg::JointState::SharedPtr) { /* do nothing */ });

        get_param_srv = this->create_service<rcl_interfaces::srv::GetParameters>(
            get_param_service,
            [](const std::shared_ptr<rcl_interfaces::srv::GetParameters::Request> request,
                std::shared_ptr<rcl_interfaces::srv::GetParameters::Response> response) {
            // Return dummy values for required parameters
            response->values.resize(request->names.size());
            for (size_t i = 0; i < request->names.size(); ++i) {
                if (request->names[i] == "joint_names") {
                    response->values[i].type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
                    response->values[i].string_array_value = { "joint_1", "joint_2", "joint_3" };
                }
                else if (request->names[i] == "joint_types") {
                    response->values[i].type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY;
                    // 0: revolute, 1: prismatic
                    response->values[i].integer_array_value = { 0, 0, 0 };
                }
                else if (request->names[i] == "compliant_modes") {
                    response->values[i].type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY;
                    response->values[i].bool_array_value = { false, false, false};
                }
                else {
                    response->values[i].type = rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET;
                }
            }
            yInfo() << "GetParameters service ended successfully.";
        });

        set_param_srv = this->create_service<rcl_interfaces::srv::SetParameters>(
            set_param_service,
            [](const std::shared_ptr<rcl_interfaces::srv::SetParameters::Request>,
                std::shared_ptr<rcl_interfaces::srv::SetParameters::Response> response) {
            response->results.resize(1);
            response->results[0].successful = true;
        });
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motor_state_pub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_references_sub;
    rclcpp::Service<rcl_interfaces::srv::GetParameters>::SharedPtr get_param_srv;
    rclcpp::Service<rcl_interfaces::srv::SetParameters>::SharedPtr set_param_srv;
};

TEST_CASE("IsaacSimControlBoardNWCROS2 device basic functionality", "[ros2][isaacsim_controlboard]") {
    rclcpp::init(0, nullptr);
    yarp::os::Network::init();

    // Dummy ROS2 node with required topics/services
    auto dummy_node = std::make_shared<DummyCBNode>(
        "dummy_cb_node",
        "/joint_states",
        "/motor_states",
        "/joint_references",
        "/get_parameters",
        "/set_parameters"
    );
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(dummy_node);

    // Start executor in a thread
    std::thread exec_thread([&executor]() { executor.spin(); });

    // Give time for services to be available
    std::this_thread::sleep_for(500ms);

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
    // Test that the joint names are correctly retrieved
    int numberOfJoints = 0;
    REQUIRE(device.getAxes(&numberOfJoints));
    REQUIRE(numberOfJoints == 3);
    for (int i = 0; i < numberOfJoints; ++i) {
        std::string joint_name;
        REQUIRE(device.getAxisName(i, joint_name));
        std::string expected_name = "joint_" + std::to_string(i + 1);
        REQUIRE(std::string(joint_name) == expected_name);
    }

    REQUIRE(device.close());

    executor.cancel();
    exec_thread.join();
    rclcpp::shutdown();
}
