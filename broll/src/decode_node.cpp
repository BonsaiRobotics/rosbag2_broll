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

#include "avcodec_msgs/msg/video_codec_parameters.hpp"
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
  void on_codec_params(const avcodec_msgs::msg::VideoCodecParameters::SharedPtr msg);

protected:
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr in_sub_;
  rclcpp::Subscription<avcodec_msgs::msg::VideoCodecParameters>::SharedPtr param_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr out_pub_;

  std::optional<FrameDecoder> frame_decoder_;
  size_t frame_idx_ = 0;

  // Node params
  double scale_;
  AVPixelFormat target_pix_fmt_;
  bool wait_for_codec_params_ = false;
};

DecodeNode::DecodeNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("broll_decode", options)
{
  const std::string in_topic = "video/compressed";
  const std::string out_topic = "video/raw";
  const std::string codec_params_topic = "video/codec_params";
  scale_ = declare_parameter("scale", 1.0);
  std::string pix_fmt_str = declare_parameter("pix_fmt", "bgr8");
  wait_for_codec_params_ = declare_parameter("wait_for_codec_params", false);

  if (pix_fmt_str == "bgr8") {
    target_pix_fmt_ = AV_PIX_FMT_BGR24;
  } else if (pix_fmt_str == "rgb8") {
    target_pix_fmt_ = AV_PIX_FMT_RGB24;
  } else {
    throw std::runtime_error("Unknown pixel format " + pix_fmt_str);
  }

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
}

DecodeNode::~DecodeNode()
{
}


void DecodeNode::on_codec_params(const avcodec_msgs::msg::VideoCodecParameters::SharedPtr msg)
{
  if (frame_decoder_) {
    return;
  }

  RCLCPP_INFO(get_logger(), "Got codec params, initializing decoder");
  auto params = broll::parameters_from_message(*msg);
  frame_decoder_.emplace(params, target_pix_fmt_, scale_);
  avcodec_parameters_free(&params);
}

void DecodeNode::decode_and_republish(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
  if (!frame_decoder_) {
    if (wait_for_codec_params_) {
      RCLCPP_INFO_ONCE(get_logger(), "Received first message, waiting for codec params");
      return;
    }
    AVCodecParameters * params = avcodec_parameters_alloc();
    params->codec_type = AVMEDIA_TYPE_VIDEO;
    RCLCPP_INFO(get_logger(), "Initializing decoder on first msg");
    if (msg->format == "h264") {
      params->codec_id = AV_CODEC_ID_H264;
    } else if (msg->format == "h265") {
      params->codec_id = AV_CODEC_ID_H265;
    } else {
      RCLCPP_ERROR(get_logger(), "Unknown codec %s", msg->format.c_str());
      return;
    }

    frame_decoder_.emplace(params, target_pix_fmt_, scale_);
    avcodec_parameters_free(&params);
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
