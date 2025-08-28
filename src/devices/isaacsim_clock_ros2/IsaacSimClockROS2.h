// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-2-Clause

#ifndef ISAACSIM_CLOCK_ROS2_H
#define ISAACSIM_CLOCK_ROS2_H

#include <yarp/dev/DeviceDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>

#include <rclcpp/node.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "IsaacSimClockROS2_ParamsParser.h"

namespace yarp::dev
{
    class IsaacSimClockROS2;
}


class yarp::dev::IsaacSimClockROS2 : public yarp::dev::DeviceDriver
{

public:
    IsaacSimClockROS2() = default;
    ~IsaacSimClockROS2() override;

    /* DeviceDriver methods */
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

private:

    void updateClock(const rosgraph_msgs::msg::Clock::ConstSharedPtr& clock);

    class ClockSubscriber : public rclcpp::Node
    {
    public:
        ClockSubscriber(const std::string& name,
                        const std::string& topicName,
                        IsaacSimClockROS2* parent);
    private:
        rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr m_clock_sub;
    };

    yarp::os::BufferedPort<yarp::os::Bottle> m_clockPort;
    std::unique_ptr<yarp::os::Network> m_network = nullptr;
    bool m_resetYARPClockAfterFirstPublish =  false;
    IsaacSimClockROS2_ParamsParser m_paramsParser;
    std::shared_ptr<ClockSubscriber> m_subscriber;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> m_executor;
    std::thread m_executorThread;
    std::mutex m_mutex;
};
#endif
