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

#include <atomic>

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
  /// @param hw_device_type Attempt to use hardware decoding device of this type.
  ///   AV_HWDEVICE_TYPE_NONE uses software decoding only.
  /// @param dbg_print Print info about each decoded frame to stdout
  /// @throws std::invalid_argument if hw_device_type is not supported
  FrameDecoder(
    AVCodecID codec_id,
    AVPixelFormat target_fmt = AV_PIX_FMT_NONE,
    double scale = 1.0f,
    AVHWDeviceType hw_device_type = AV_HWDEVICE_TYPE_NONE,
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
  bool decodeFrame(const AVPacket & packet_in, AVFrame & frame_out);

  /// @brief Pure-C function pointer to intercept libav log messages and direct
  /// callbacks to a class instance if necessary.
  /// Note: this handles multiple instances of FrameDecoder, but will be disabled if
  /// another part of the program also calls av_log_set_callback
  static void avLogCallbackWrapper(void * ptr, int level, const char * fmt, va_list vargs);

  /// @brief Skip new P-frames until the next I-frame
  ///
  /// This allows the decoder to detect when the video stream has missing I-frames that lead to
  /// the gray diff-only frames that x265 produces in an attempt to gracefully continue the stream.
  /// For the ROS user, it is probably better to have a stuttering video with only good frames,
  /// rather than a smoother one with bad gray decoded images.
  ///
  /// Ideally this decoder class would be able to detect the condition directly, but the way
  /// it is implemented now, we can detect the bad frames only via av_log messages.
  /// The information about this case is not available to the libavcodec user, being only
  /// available in HEVC decoder internals, so we would need to use libx265 directly to detect.
  void startSkippingPFrames();

  /// @brief Function to help AVCodecContext pick a pixel format that matches hwPixFmt_.
  ///
  /// Only used if hardware decoding is enabled.
  static AVPixelFormat getHardwarePixelFormat(AVCodecContext * ctx, const AVPixelFormat * pix_fmts);

  AVPacket * packet_ = nullptr;
  const AVCodec * codec_ = nullptr;
  AVCodecContext * codecCtx_ = nullptr;
  AVPixelFormat targetPixFmt_ = AV_PIX_FMT_NONE;
  SwsContext * swsCtx_ = nullptr;
  AVFrame * decodedFrame_ = nullptr;
  AVFrame * convertedFrame_ = nullptr;

  // Hardware decoding extras
  // Pixel format reported for an on-hardware DMA frame, such as "cuda", which isn't
  // a real pixel format but how the frame reports that it isn't in CPU memory.
  AVPixelFormat hwPixFmt_ = AV_PIX_FMT_NONE;
  // Pixel format to convert to when transferring the frame from hardware into CPU memory.
  // Hardware devices offer several options, but not all that we might want so this
  // is the pre-conversion format before sws.
  AVPixelFormat hwSoftwarePixFmt_ = AV_PIX_FMT_NONE;
  AVBufferRef * hwDeviceCtx_ = nullptr;
  AVFrame * hwFrame_ = nullptr;

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
