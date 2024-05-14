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

#ifndef ROSBAG2_STORAGE_BROLL__BAG_UTILS_HPP_
#define ROSBAG2_STORAGE_BROLL__BAG_UTILS_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/time.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace rosbag2_storage_broll
{

void scale_camera_info(
  sensor_msgs::msg::CameraInfo & info,
  double scale);

// Provide the serialized format for rosbag2_storage::BagMetadata
std::string serialize_qos(const std::vector<rclcpp::QoS> & profiles);

// Serialize a message to return from read_next() API,
// when programmatically constructing messages in a storage plugin.
template<typename MessageT>
std::shared_ptr<rosbag2_storage::SerializedBagMessage>
serialize_msg(
  const MessageT & msg,
  const std::string & topic_name,
  rclcpp::Time timestamp)
{
  static rclcpp::Serialization<MessageT> serde;
  rclcpp::SerializedMessage serialized_msg;
  serde.serialize_message(&msg, &serialized_msg);

  auto bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
#if defined(ROS2_JAZZY) || defined(ROS2_ROLLING)
  bag_msg->recv_timestamp = timestamp.nanoseconds();
#else
  bag_msg->time_stamp = timestamp.nanoseconds();
#endif
  bag_msg->topic_name = topic_name;

  rcutils_uint8_array_t * serialized_data = new rcutils_uint8_array_t();
  *serialized_data = serialized_msg.release_rcl_serialized_message();
  bag_msg->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
    serialized_data,
    [](rcutils_uint8_array_t * msg) {
      auto fini_return = rcutils_uint8_array_fini(msg);
      delete msg;
      if (fini_return != RCUTILS_RET_OK) {
        RCLCPP_ERROR_STREAM(
          rclcpp::get_logger("avcodec_bag_utils"),
          "Failed to destroy serialized message: " << rcutils_get_error_string().str);
      }
    });
  return bag_msg;
}

template<typename MessageT>
void
deserialize_msg(
  rosbag2_storage::SerializedBagMessage bag_msg,
  MessageT & out_msg)
{
  static rclcpp::Serialization<MessageT> serde;
  rclcpp::SerializedMessage rclcpp_msg{*bag_msg.serialized_data};
  serde.deserialize_message(&rclcpp_msg, &out_msg);
  // Release ownership of the lent data so it doesn't delete it
  rclcpp_msg.release_rcl_serialized_message();
}

}  // namespace rosbag2_storage_broll

#endif  // ROSBAG2_STORAGE_BROLL__BAG_UTILS_HPP_
