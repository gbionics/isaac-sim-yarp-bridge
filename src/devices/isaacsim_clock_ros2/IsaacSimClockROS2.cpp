// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-2-Clause
#include "IsaacSimClockROS2.h"
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>

YARP_DECLARE_LOG_COMPONENT(CLOCK)
YARP_LOG_COMPONENT(CLOCK, "yarp.device.IsaacSimClockROS2")

// This is inspired from
// https://github.com/robotology/gz-sim-yarp-plugins/blob/9155822ef1f6aae8c5aecf6472f11e345fca201a/plugins/clock/Clock.cc

yarp::dev::IsaacSimClockROS2::~IsaacSimClockROS2()
{
    close();
}

bool yarp::dev::IsaacSimClockROS2::open(yarp::os::Searchable& config)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string errorPrefix = "[open] ";
    if (!m_paramsParser.parseParams(config))
    {
        yCError(CLOCK) << errorPrefix << "Failed to parse parameters for IsaacSimClockROS2";
        return false;
    }

    // To avoid deadlock during initialization if YARP_CLOCK is set,
    // if the YARP network is not initialized we always initialize
    // it with system clock, and then we switch back to the default clock later
    bool networkIsNotInitialized = !yarp::os::NetworkBase::isNetworkInitialized();

    if (networkIsNotInitialized)
    {
        m_network = std::make_unique<yarp::os::Network>(yarp::os::YARP_CLOCK_SYSTEM);
        m_resetYARPClockAfterFirstPublish = true;
    }
    else
    {
        m_network = std::make_unique<yarp::os::Network>();
        m_resetYARPClockAfterFirstPublish = false;
    }

    if (!m_clockPort.open(m_paramsParser.m_port_name))
    {
        yCError(CLOCK) << errorPrefix << "Failed to open the port" << m_paramsParser.m_port_name;
        return false;
    }

    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }
    m_subscriber = std::make_shared<ClockSubscriber>(m_paramsParser.m_node_name, m_paramsParser.m_topic_name, this);
    m_executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    m_executor->add_node(m_subscriber);
    m_executorThread = std::thread([this]() { m_executor->spin(); });

    return true;
}

bool yarp::dev::IsaacSimClockROS2::close()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_subscriber)
    {
        m_executor->cancel();
        m_executorThread.join();
        m_subscriber.reset();
    }
    m_clockPort.close();

    return true;
}

void yarp::dev::IsaacSimClockROS2::updateClock(const rosgraph_msgs::msg::Clock::ConstSharedPtr& clock)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_resetYARPClockAfterFirstPublish)
    {
        // As the port is now created and contains streams data,
        // if necessary reset the YARP clock to YARP_CLOCK_DEFAULT
        // Unfortunately, the yarpClockInit blocks on the port until it
        // receives data, so we need to launch it in a different thread

        auto resetYARPNetworkClockLambda = []() { yarp::os::NetworkBase::yarpClockInit(yarp::os::YARP_CLOCK_DEFAULT); };
        std::thread resetYARPNetworkClockThread(resetYARPNetworkClockLambda);
        resetYARPNetworkClockThread.detach();
        m_resetYARPClockAfterFirstPublish = false;
    }
    yarp::os::Bottle& bottle = m_clockPort.prepare();
    bottle.clear();
    bottle.addInt32(clock->clock.sec);
    bottle.addInt32(clock->clock.nanosec);
    m_clockPort.write();
}

yarp::dev::IsaacSimClockROS2::ClockSubscriber::ClockSubscriber(const std::string& name, const std::string& topicName,
                                                               IsaacSimClockROS2* parent)
    : rclcpp::Node(name)
{
    int queue_size = 10;
    // Subscribe to Clock topic
    m_clock_sub = this->create_subscription<rosgraph_msgs::msg::Clock>(
        topicName, queue_size, std::bind(&IsaacSimClockROS2::updateClock, parent, std::placeholders::_1));
}
