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

#include <atomic>

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
  /// @param codec_id ID of the codec to initialize the video decoder
  /// @param target_fmt Pixel format to convert to, if necessary.
  ///   AV_PIX_FMT_NONE guarantees no conversion.
  /// @param scale Scale applied to image dimensions (by multiplication, 0.5 is half size)
  /// @param dbg_print Print info about each decoded frame to stdout
  FrameDecoder(
    AVCodecID codec_id,
    AVPixelFormat target_fmt = AV_PIX_FMT_NONE,
    double scale = 1.0f,
    bool dbg_print = false);
  virtual ~FrameDecoder();

  /// @brief Decode a compressed image packet into an image message
  /// @note This method does not set the header (timestamp, tf frame) of the output image
  bool decode(const AVPacket & in, sensor_msgs::msg::Image & out);

  /// @brief Convenience wrapper to represent CompressedImage as AVPacket
  /// and call decode(const AVPacket &, Image &)
  /// @param in Frame to decode
  /// @param out Decoded frame to fill
  /// @return True if decoding was successful, false otherwise
  bool decode(const sensor_msgs::msg::CompressedImage & in, sensor_msgs::msg::Image & out);

protected:
  void initializeSwsContext();
  bool decodeFrame(const AVPacket & packet_in, AVFrame & frame_out);

  /// @brief Pure-C function pointer to redirect libav log calls back to a class instance
  ///
  static void avLogCallbackWrapper(void * ptr, int level, const char * fmt, va_list vargs);

  /// @brief Skip new P-frames until the next I-frame
  ///
  /// This allows the application context to detect when the video stream has missing
  /// I-frames that lead to the gray diff-only frames that x265 produces in an attempt to
  /// gracefully keep the stream going.
  /// For the user, it is probably much better to have a stuttering video with only good
  /// frames, rather than a smoother one with bad gray decoded images.
  ///
  /// Ideally this decoder class would be able to detect the condition itself, but the way
  /// it is implemented now, we can detect the bad frames only via global libav log messages,
  /// which is not per-AVCodecContext, instead being program-global.
  /// The information is not available in the AVFrame, this condition is hidden from the libavcodec
  /// user, we would probably have to be using libx265 directly to intercept it.
  void startSkippingPFrames();

  AVPacket * packet_ = nullptr;
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
  bool dbg_print_ = false;

  std::atomic<bool> skip_pframes_{false};
  std::atomic<uint> skipped_pframes_{0};
};

}  // namespace broll

#endif  // BROLL__FRAME_DECODER_HPP_
