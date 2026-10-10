cmake_minimum_required(VERSION 3.8)
project(falcon_pkg)

if(CMAKE_COMPILER_IS_GNUXX OR CMAKE_CXXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Beacon.msg"
  "msg/Config6Dof.msg"
  "msg/ConfigGyroCalibration.msg"
  "msg/ConfigRelativeAngle.msg"
  "msg/ConfigRelativeBeacon.msg"
  "msg/CurrentShutter.msg"
  "msg/DynamicMessage.msg"
  "msg/FieldOfViewReport.msg"
  "msg/GyroOffset.msg"
  "msg/Heartbeat.msg"
  "msg/NetworkAdvancedConfig.msg"
  "msg/NetworkConfig.msg"
  "msg/Pose6Dof.msg"
  "msg/PoseRelativeBeacon.msg"
  "msg/PoseRelativeBeaconCollection.msg"
  "msg/RelativeAngle.msg"
  "msg/RelativeAngleCollection.msg"
  "msg/ShutterSettings.msg"
  "msg/StatusMessage.msg"
  "msg/Version.msg"
  "srv/GetFirmwareVersion.srv"
  "srv/GetGyroCalibration.srv"
  "srv/GetVersion.srv"
  "srv/Initialize6Dof.srv"
  "srv/InitializeNetwork.srv"
  "srv/InitializeNetworkAdvanced.srv"
  "srv/InitializeRelativeAngle.srv"
  "srv/InitializeRelativeBeacon.srv"
  "srv/LoadDefaultShutterControl.srv"
  "srv/LoadShutterPlugin.srv"
  "srv/SetTrackingMode.srv"
  "srv/SwapMap.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# Including the SdsFalcon header
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include 
  ${CMAKE_CURRENT_SOURCE_DIR}/include/falcon_pkg
)

# Sets the environment variable LD_LIBRARY_PATH to include the shared library SdsFalcon so that the system can find it at runtime 
set(ENV{LD_LIBRARY_PATH} "$ENV{LD_LIBRARY_PATH}:${CMAKE_SOURCE_DIR}/src/falcon_pkg/lib")

# Find the SdsFalcon library
find_library(SDSFALCON_LIB libSdsFalcon.so PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib)
if(NOT SDSFALCON_LIB)
  message(FATAL_ERROR "libSdsFalcon.so not found")
endif()

# Add the SdsFalcon library
add_library(sdk_lib SHARED IMPORTED)
set_target_properties(sdk_lib PROPERTIES IMPORTED_LOCATION ${SDSFALCON_LIB})

# Add executables for each node
add_executable(falcon_manager_node src/falcon_manager_node.cpp)
ament_target_dependencies(falcon_manager_node rclcpp std_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(falcon_manager_node ${cpp_typesupport_target} sdk_lib)

install(TARGETS
  falcon_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
