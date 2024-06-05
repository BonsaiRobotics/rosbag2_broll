cmake_minimum_required(VERSION 3.8)
project(rosbag2_storage_broll)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if("$ENV{ROS_DISTRO}" STREQUAL "humble")
  add_compile_definitions(ROS2_HUMBLE)
elseif("$ENV{ROS_DISTRO}" STREQUAL "iron")
  add_compile_definitions(ROS2_IRON)
elseif("$ENV{ROS_DISTRO}" STREQUAL "jazzy")
  add_compile_definitions(ROS2_JAZZY)
elseif("$ENV{ROS_DISTRO}" STREQUAL "rolling")
  add_compile_definitions(ROS2_ROLLING)
else()
  message(FATAL_ERROR "Unhandled ROS distro $ENV{ROS_DISTRO}")
endif()

# find dependencies
find_package(broll REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(bag_utils SHARED
  src/bag_utils.cpp
)
add_library(${PROJECT_NAME}::bag_utils ALIAS bag_utils)
target_include_directories(bag_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(bag_utils
  rclcpp::rclcpp
  rosbag2_storage::rosbag2_storage
  rosbag2_transport::rosbag2_transport
  ${sensor_msgs_TARGETS}
)

add_library(${PROJECT_NAME} SHARED
  src/broll_storage.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${PROJECT_NAME}::bag_utils
  broll::broll
  rclcpp::rclcpp
  rosbag2_storage::rosbag2_storage
  rosbag2_transport::rosbag2_transport
  ${sensor_msgs_TARGETS}
)
pluginlib_export_plugin_description_file(rosbag2_storage rosbag2_storage_plugin.xml)


install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME} bag_utils
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  rosbag2_transport
)
ament_export_libraries(
  bag_utils
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_package()
