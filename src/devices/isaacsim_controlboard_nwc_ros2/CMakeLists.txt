# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-2-Clause

set(plugin_name IsaacSimControlBoardNWCROS2)

yarp_prepare_plugin(${plugin_name}
  CATEGORY device
  TYPE yarp::dev::${plugin_name}
  INCLUDE IsaacSimControlBoardNWCROS2.h
  OPTION ENABLE_IsaacSimControlBoardNWCROS2
  DEFAULT ON
)

if (SKIP_${plugin_name})
  message(STATUS "Skipping ${plugin_name} plugin")
  return()
endif()


yarp_add_plugin(${plugin_name})
generateDeviceParamsParser(${plugin_name} ${plugin_name})

add_library(${plugin_name}_lib
    IsaacSimControlBoardNWCROS2.cpp
    IsaacSimControlBoardNWCROS2.h
    IsaacSimControlBoardNWCROS2_ParamsParser.h
    IsaacSimControlBoardNWCROS2_ParamsParser.cpp
)

target_link_libraries(${plugin_name}_lib
  PUBLIC
  YARP::YARP_os
  YARP::YARP_sig
  YARP::YARP_dev
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  rcl_interfaces::rcl_interfaces__rosidl_typesupport_cpp
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
