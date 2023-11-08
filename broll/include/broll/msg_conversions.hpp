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

#include <string>

extern "C" {
#include "libavcodec/avcodec.h"
}

#include "sensor_msgs/msg/image.hpp"

namespace broll
{

/// @brief Converts a decoded AVFrame to a sensor_msgs::msg::Image
/// @param frame
/// @param img
/// @return True if conversion was successful, false otherwise
bool frame_to_image(const AVFrame & frame, sensor_msgs::msg::Image & img);

/// @brief Convert a ROS pixel format string to equivalent AVPixelFormat
/// @param pix_fmt_str see sensor_msgs::image_encodings for valid strings
/// @return AV_PIX_FMT_NONE if no equivalent format was found
AVPixelFormat pixel_format_from_ros_string(const std::string & pix_fmt_str);

/// @brief Convert a ROS CompressedImage::format string to equivalent AVCodecID
/// @param codec_id_str
/// @return
AVCodecID codec_id_from_name(const std::string & codec_name);

}  // namespace broll

#endif  // BROLL__MSG_CONVERSIONS_HPP_
