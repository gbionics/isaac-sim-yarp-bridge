# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-2-Clause

set(plugin_name IsaacSimClockROS2)

yarp_prepare_plugin(${plugin_name}
  CATEGORY device
  TYPE yarp::dev::${plugin_name}
  INCLUDE IsaacSimClockROS2.h
  OPTION ENABLE_IsaacSimClockROS2
  DEFAULT ON
)

if (SKIP_${plugin_name})
  message(STATUS "Skipping ${plugin_name} plugin")
  return()
endif()


yarp_add_plugin(${plugin_name})
generateDeviceParamsParser(${plugin_name} ${plugin_name})

add_library(${plugin_name}_lib
    IsaacSimClockROS2.cpp
    IsaacSimClockROS2.h
    IsaacSimClockROS2_ParamsParser.h
    IsaacSimClockROS2_ParamsParser.cpp
)

target_link_libraries(${plugin_name}_lib
  PUBLIC
  YARP::YARP_os
  YARP::YARP_sig
  YARP::YARP_dev
  YARP::YARP_init
  rclcpp::rclcpp
  rosgraph_msgs::rosgraph_msgs__rosidl_typesupport_cpp
)

target_include_directories(${plugin_name}_lib
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

target_compile_features(${plugin_name}_lib PUBLIC cxx_std_17)

target_link_libraries(${plugin_name}
  PUBLIC
  ${plugin_name}_lib
)

yarp_install(
  TARGETS ${plugin_name}
  EXPORT ${plugin_name}
  COMPONENT ${plugin_name}
  LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
  ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
  YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}
)
