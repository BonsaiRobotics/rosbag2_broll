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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "avcodec_msgs/msg/video_codec_parameters.hpp"
#include "broll/frame_decoder.hpp"

namespace broll
{

class DecodeNode : public rclcpp::Node
{
public:
  explicit DecodeNode(const rclcpp::NodeOptions & options);
  virtual ~DecodeNode();

  void decode_and_republish(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
  void on_codec_params(const avcodec_msgs::msg::VideoCodecParameters::SharedPtr msg);

protected:
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr in_sub_;
  rclcpp::Subscription<avcodec_msgs::msg::VideoCodecParameters>::SharedPtr param_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr out_pub_;

  std::unique_ptr<FrameDecoder> frame_decoder_;
  AVPacket * packet_;
  size_t frame_idx_ = 0;
  double scale_ = 1.0;
};

DecodeNode::DecodeNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("broll_decode", options)
{
  const std::string in_topic = "video/compressed";
  const std::string out_topic = "video/raw";
  const std::string codec_params_topic = "video/codec_params";
  scale_ = declare_parameter("scale", 1.0);

  in_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
    in_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&DecodeNode::decode_and_republish, this, std::placeholders::_1));

  rclcpp::QoS param_qos{1};
  param_qos.transient_local();
  param_sub_ = create_subscription<avcodec_msgs::msg::VideoCodecParameters>(
    codec_params_topic,
    param_qos,
    std::bind(&DecodeNode::on_codec_params, this, std::placeholders::_1));

  out_pub_ = create_publisher<sensor_msgs::msg::Image>(out_topic, 10);
  packet_ = av_packet_alloc();
  assert(packet_);
}

DecodeNode::~DecodeNode()
{
  av_packet_free(&packet_);
}


void DecodeNode::on_codec_params(const avcodec_msgs::msg::VideoCodecParameters::SharedPtr msg)
{
  if (frame_decoder_) {
    return;
  }

  RCLCPP_INFO(get_logger(), "Got codec params, initializing decoder");
  auto params = broll::parameters_from_message(*msg);
  frame_decoder_ = std::make_unique<FrameDecoder>(params, AV_PIX_FMT_BGR24, scale_);
  avcodec_parameters_free(&params);
}

void DecodeNode::decode_and_republish(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
  if (!frame_decoder_) {
    RCLCPP_WARN_ONCE(get_logger(), "No codec params received yet, ignoring images");
    // No parameters received yet to initialize decoder
    return;
  }

  RCLCPP_INFO_ONCE(get_logger(), "Processing first image");
  RCLCPP_DEBUG(get_logger(), "Got msg %d", msg->header.stamp.nanosec);
  rclcpp::Time ts = msg->header.stamp;

  packet_->size = msg->data.size();
  packet_->data = &msg->data[0];

  auto out_msg = std::make_unique<sensor_msgs::msg::Image>();
  frame_decoder_->decode(*packet_, *out_msg);

  out_msg->header = msg->header;
  out_pub_->publish(std::move(out_msg));
}

}  // namespace broll

RCLCPP_COMPONENTS_REGISTER_NODE(broll::DecodeNode)
