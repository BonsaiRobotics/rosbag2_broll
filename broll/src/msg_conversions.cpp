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

#include <cassert>
#include <functional>
#include <string>

extern "C" {
#include "libavcodec/avcodec.h"
#include "libavutil/imgutils.h"
}

#include "broll/msg_conversions.hpp"

namespace broll
{

bool frame_to_image(const AVFrame & in, sensor_msgs::msg::Image & out)
{
  const int align_size = 16;

  out.height = in.height;
  out.width = in.width;
  out.is_bigendian = false;

  switch (in.format) {
    case AV_PIX_FMT_RGB24:
      out.encoding = "rgb8";
      break;
    case AV_PIX_FMT_BGR24:
      out.encoding = "bgr8";
      break;
    default: {
        int fourcc = avcodec_pix_fmt_to_codec_tag((AVPixelFormat)in.format);
        out.encoding.resize(4);
        for (int i = 0; i < 4; i++) {
          out.encoding[i] = 0xFF & (fourcc >> (i * 8));
        }
      } break;
  }

  int data_size = av_image_get_buffer_size(
    (AVPixelFormat)in.format,
    in.width,
    in.height,
    align_size);
  assert(data_size > 0);
  out.step = data_size / out.height;

  out.data.resize(data_size);
  av_image_copy_to_buffer(
    &out.data[0],
    data_size,
    in.data,
    in.linesize,
    (AVPixelFormat)in.format,
    in.width,
    in.height,
    align_size);
  return true;
}

AVPixelFormat pixel_format_from_ros_string(const std::string & pix_fmt_str)
{
  if (pix_fmt_str == "bgr8") {
    return AV_PIX_FMT_BGR24;
  } else if (pix_fmt_str == "rgb8") {
    return AV_PIX_FMT_RGB24;
  }
  return AV_PIX_FMT_NONE;
}

AVCodecID codec_id_from_name(const std::string & codec_name)
{
  const AVCodecDescriptor * desc = avcodec_descriptor_get_by_name(codec_name.c_str());
  if (desc) {
    return desc->id;
  }
  return AV_CODEC_ID_NONE;
}

}  // namespace broll
