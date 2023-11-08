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

#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "broll/frame_decoder.hpp"
#include "broll/msg_conversions.hpp"

namespace broll
{

class DecodeNode : public rclcpp::Node
{
public:
  explicit DecodeNode(const rclcpp::NodeOptions & options);
  virtual ~DecodeNode();

  void decode_and_republish(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

protected:
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr in_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr out_pub_;

  std::optional<FrameDecoder> frame_decoder_;
  size_t frame_idx_ = 0;

  // Node params
  double scale_;
  AVPixelFormat target_pix_fmt_;
};

DecodeNode::DecodeNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("broll_decode", options)
{
  const std::string in_topic = "video/compressed";
  const std::string out_topic = "video/raw";
  scale_ = declare_parameter("scale", 1.0);
  std::string pix_fmt_str = declare_parameter("pix_fmt", "bgr8");

  target_pix_fmt_ = pixel_format_from_ros_string(pix_fmt_str);
  if (target_pix_fmt_ == AV_PIX_FMT_NONE) {
    throw std::runtime_error("Unknown pixel format " + pix_fmt_str);
  }

  in_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
    in_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&DecodeNode::decode_and_republish, this, std::placeholders::_1));

  out_pub_ = create_publisher<sensor_msgs::msg::Image>(out_topic, 10);
}

DecodeNode::~DecodeNode()
{
}

void DecodeNode::decode_and_republish(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
  if (!frame_decoder_) {
    AVCodecID codec_id = AV_CODEC_ID_NONE;
    RCLCPP_INFO(get_logger(), "Initializing decoder on first msg");
    codec_id = codec_id_from_name(msg->format);
    if (codec_id == AV_CODEC_ID_NONE) {
      RCLCPP_ERROR_STREAM(get_logger(), "Unknown codec " << msg->format);
      return;
    }
    frame_decoder_.emplace(codec_id, target_pix_fmt_, scale_);
  }

  RCLCPP_INFO_ONCE(get_logger(), "Processing first image");
  RCLCPP_DEBUG(get_logger(), "Got msg %d", msg->header.stamp.nanosec);
  rclcpp::Time ts = msg->header.stamp;

  auto out_msg = std::make_unique<sensor_msgs::msg::Image>();
  frame_decoder_->decode(*msg, *out_msg);

  out_msg->header = msg->header;
  out_pub_->publish(std::move(out_msg));
}

}  // namespace broll

RCLCPP_COMPONENTS_REGISTER_NODE(broll::DecodeNode)
