// Copyright 2023 Bonsai Robotics, Inc - All Rights Reserved
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <vector>

#include "rclcpp/qos.hpp"
#include "rosbag2_storage/yaml.hpp"
#include "rosbag2_storage_broll/bag_utils.hpp"
#if defined(ROS2_JAZZY) || defined(ROS2_ROLLING)
#include "rosbag2_storage/qos.hpp"
using Rosbag2QoS = rosbag2_storage::Rosbag2QoS;
#else
#include "rosbag2_transport/qos.hpp"
using Rosbag2QoS = rosbag2_transport::Rosbag2QoS;
#endif

namespace rosbag2_storage_broll
{

std::string serialize_qos(const std::vector<rclcpp::QoS> & profiles)
{
  YAML::Node node;
  for (const auto & p : profiles) {
    node.push_back(Rosbag2QoS(p));
  }
  return YAML::Dump(node);
}

void scale_camera_info(
  sensor_msgs::msg::CameraInfo & info,
  double scale)
{
  int width = info.width * scale;
  width += width % 2;
  int height = info.height * scale;
  height += height % 2;

  info.width = width;
  info.height = height;
  for (auto & k : info.k) {
    k *= scale;
  }
  for (auto & p : info.p) {
    p *= scale;
  }
}

}  // namespace rosbag2_storage_broll
