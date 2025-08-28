# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-2-Clause

set(targetname isaacsim_clock_ros2_test)
add_executable(${targetname} isaacsim_clock_ros2_test.cpp)

target_link_libraries(${targetname} PRIVATE Catch2::Catch2WithMain IsaacSimClockROS2_lib YARP::YARP_init)
target_compile_definitions(${targetname} PRIVATE CATCH_CONFIG_FAST_COMPILE CATCH_CONFIG_DISABLE_MATCHERS)

add_test(NAME ${targetname} COMMAND ${targetname})
