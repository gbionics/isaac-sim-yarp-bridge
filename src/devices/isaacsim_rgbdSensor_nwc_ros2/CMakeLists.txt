# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-2-Clause

yarp_prepare_plugin(IsaacSimRGBDSensorNWCROS2
  CATEGORY device
  TYPE yarp::dev::IsaacSimRGBDSensorNWCROS2
  INCLUDE IsaacSimRGBDSensorNWCROS2.h
  OPTION ENABLE_IsaacSimRGBDSensorNWCROS2
  DEFAULT ON
)

if (SKIP_IsaacSimRGBDSensorNWCROS2)
  message(STATUS "Skipping IsaacSimRGBDSensorNWCROS2 plugin")
  return()
endif()

yarp_add_plugin(IsaacSimRGBDSensorNWCROS2)
generateDeviceParamsParser(IsaacSimRGBDSensorNWCROS2 IsaacSimRGBDSensorNWCROS2)

target_sources(IsaacSimRGBDSensorNWCROS2
  PRIVATE
    IsaacSimRGBDSensorNWCROS2.cpp
    IsaacSimRGBDSensorNWCROS2.h
    IsaacSimRGBDSensorNWCROS2_ParamsParser.h
    IsaacSimRGBDSensorNWCROS2_ParamsParser.cpp
)

target_link_libraries(IsaacSimRGBDSensorNWCROS2
  PRIVATE
  YARP::YARP_os
  YARP::YARP_sig
  YARP::YARP_dev
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
)

target_compile_features(IsaacSimRGBDSensorNWCROS2 PUBLIC cxx_std_17)

yarp_install(
  TARGETS IsaacSimRGBDSensorNWCROS2
  EXPORT IsaacSimRGBDSensorNWCROS2
  COMPONENT IsaacSimRGBDSensorNWCROS2
  LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
  ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
  YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}
)