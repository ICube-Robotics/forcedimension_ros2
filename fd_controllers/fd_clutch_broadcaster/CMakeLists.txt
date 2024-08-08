cmake_minimum_required(VERSION 3.5)
project(fd_clutch_broadcaster)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcutils REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

add_library(fd_clutch_broadcaster
  SHARED
  src/fd_clutch_broadcaster.cpp
)
target_include_directories(fd_clutch_broadcaster PRIVATE include)
ament_target_dependencies(fd_clutch_broadcaster
  builtin_interfaces
  controller_interface
  pluginlib
  rclcpp_lifecycle
  rcutils
  realtime_tools
  std_msgs
  eigen3_cmake_module
  Eigen3
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(fd_clutch_broadcaster PRIVATE "FD_CLUTCH_BROADCASTER_BUILDING_DLL")
# prevent pluginlib from using boost
target_compile_definitions(fd_clutch_broadcaster PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface fd_clutch_broadcaster_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
  fd_clutch_broadcaster
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_dependencies(
  controller_interface
  rclcpp_lifecycle
  geometry_msgs
  eigen3_cmake_module
  Eigen3
  example_interfaces
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  fd_clutch_broadcaster
)
ament_package()
