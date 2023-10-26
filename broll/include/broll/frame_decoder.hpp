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

#ifndef BROLL__FRAME_DECODER_HPP_
#define BROLL__FRAME_DECODER_HPP_

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}

#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace broll
{

class FrameDecoder
{
public:
  /// @brief Constructor
  /// @param params Codec parameters to initialize the decoder
  /// @param target_fmt Pixel format to convert to, if necessary.
  ///   AV_PIX_FMT_NONE guarantees no conversion.
  /// @param scale Scale applied to image dimensions (by multiplication, 0.5 is half size)
  FrameDecoder(
    AVCodecParameters * params,
    AVPixelFormat target_fmt = AV_PIX_FMT_NONE,
    double scale = 1.0f);
  virtual ~FrameDecoder();

  /// @brief Decode a compressed image message into an image message
  /// @note This method does not set the header (timestamp, tf frame) of the output image
  bool decode(const AVPacket & in, sensor_msgs::msg::Image & out);

  /// @brief Convenience wrapper to represent CompressedImage as AVPacket
  /// and call decode(const AVPacket &, Image &)
  /// @param in Frame to decode
  /// @param out Decoded frame to fill
  /// @return True if decoding was successful, false otherwise
  bool decode(const sensor_msgs::msg::CompressedImage & in, sensor_msgs::msg::Image & out);

  static bool convertToImage(const AVFrame & frame_in, sensor_msgs::msg::Image & img_out);

protected:
  bool decodeFrame(const AVPacket & packet_in, AVFrame & frame_out, bool dbg_print = false);

  AVCodec * codec_ = nullptr;
  AVCodecContext * codecCtx_ = nullptr;
  AVPixelFormat targetPixFmt_ = AV_PIX_FMT_NONE;
  SwsContext * swsCtx_ = nullptr;
  AVFrame * decodedFrame_ = nullptr;
  AVFrame * convertedFrame_ = nullptr;
  float scale_ = 1.0f;
  uint scaled_width_ = 0u;
  uint scaled_height_ = 0u;
  uint consecutive_receive_failures_ = 0;
  AVPacket * packet_ = nullptr;
};

}  // namespace broll

#endif  // BROLL__FRAME_DECODER_HPP_
