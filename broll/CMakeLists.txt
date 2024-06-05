cmake_minimum_required(VERSION 3.8)
project(broll)
cmake_policy(SET CMP0074 NEW)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rcutils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBAV REQUIRED IMPORTED_TARGET
  libavcodec
  libavformat
  libavutil
  libswscale
)

### Utility library

list(APPEND all_local_targets broll)
add_library(broll
  src/frame_decoder.cpp
  src/msg_conversions.cpp
  src/video_reader.cpp
)
add_library(broll::broll ALIAS broll)
target_compile_features(broll PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(broll PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(broll PUBLIC
  avcodec
  avformat
  avutil
  swscale
  rcutils::rcutils
  ${sensor_msgs_TARGETS}
)

### Decoder node

list(APPEND all_local_targets decode_node_component)
add_library(decode_node_component SHARED
  src/decode_node.cpp
)
target_link_options(decode_node_component PRIVATE "-Wl,--no-undefined")
target_link_libraries(decode_node_component PUBLIC
  broll
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
)
rclcpp_components_register_node(decode_node_component
  PLUGIN "broll::DecodeNode"
  EXECUTABLE decode_node
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS ${all_local_targets}
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
  sensor_msgs
)
ament_export_include_directories(
  "include/${PROJECT_NAME}"
)
ament_export_libraries(
  broll
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
