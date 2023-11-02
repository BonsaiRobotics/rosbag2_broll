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

#ifndef BROLL__MSG_CONVERSIONS_HPP_
#define BROLL__MSG_CONVERSIONS_HPP_

#include <memory>

extern "C" {
#include "libavcodec/avcodec.h"
}

#include "avcodec_msgs/msg/video_codec_parameters.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace broll
{

avcodec_msgs::msg::VideoCodecParameters message_from_parameters(
  const AVCodecParameters * p);

// Note: caller gains ownership of the created object, and must later call avcodec_parameters_free
AVCodecParameters * parameters_from_message(
  const avcodec_msgs::msg::VideoCodecParameters & m);

/// @brief Converts a decoded AVFrame to a sensor_msgs::msg::Image
/// @param frame
/// @param img
/// @return True if conversion was successful, false otherwise
bool frame_to_image(const AVFrame & frame, sensor_msgs::msg::Image & img);

}  // namespace broll

#endif  // BROLL__MSG_CONVERSIONS_HPP_
