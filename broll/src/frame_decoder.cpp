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

extern "C" {
#include <libavcodec/avcodec.h>
#if LIBAVCODEC_VERSION_MAJOR >= 60
#include <libavcodec/bsf.h>
#endif
#include <libavcodec/version.h>
#include <libavutil/imgutils.h>
}

#include "broll/frame_decoder.hpp"
#include "broll/msg_conversions.hpp"
#include "sensor_msgs/image_encodings.hpp"

#include "logging.hpp"

namespace
{

static AVFrame * allocPicture(enum AVPixelFormat pix_fmt, int width, int height)
{
  AVFrame * picture = av_frame_alloc();
  if (!picture) {
    return NULL;
  }
  picture->format = pix_fmt;
  picture->width = width;
  picture->height = height;

  /* allocate the buffers for the frame data */
  int ret = av_frame_get_buffer(picture, 0);
  if (ret < 0) {
    fprintf(stderr, "Could not allocate frame data.\n");
    return nullptr;
  }
  return picture;
}

}  // namespace

namespace broll
{

FrameDecoder::FrameDecoder(
  AVCodecID codec_id,
  AVPixelFormat target_fmt,
  double scale,
  AVHWDeviceType hw_device_type,
  bool dbg_print)
: targetPixFmt_(target_fmt),
  scale_(scale),
  dbg_print_(dbg_print)
{
  AVCodecParameters * params = avcodec_parameters_alloc();
  assert(params);
  params->codec_type = AVMEDIA_TYPE_VIDEO;
  params->codec_id = codec_id;

  packet_ = av_packet_alloc();
  assert(packet_ && "failed to alloc packet");

  codec_ = avcodec_find_decoder(params->codec_id);
  if (!codec_) {
    BROLL_LOG_ERROR("Failed to find decoder");
    assert(false);
  }

  codecCtx_ = avcodec_alloc_context3(codec_);
  if (!codecCtx_) {
    BROLL_LOG_ERROR("Failed to alloc context");
    assert(false);
  }
  codecCtx_->opaque = this;
  av_log_set_callback(&FrameDecoder::avLogCallbackWrapper);

  if (avcodec_parameters_to_context(codecCtx_, params) < 0) {
    assert(false && "failed to copy codec params to codec context");
  }

  if (hw_device_type != AV_HWDEVICE_TYPE_NONE) {
    // Read HW configs 0->N until finding the one we're looking for or it returns nullptr
    for (size_t i = 0;; i++) {
      const AVCodecHWConfig * config = avcodec_get_hw_config(codec_, i);
      if (!config) {
        BROLL_LOG_ERROR(
          "Decoder %s does not support device type %s.\n",
          codec_->name, av_hwdevice_get_type_name(hw_device_type));
        throw std::runtime_error("Unsupported hardware device type");
      }
      if (
        config->methods & AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX &&
        config->device_type == hw_device_type)
      {
        hwPixFmt_ = config->pix_fmt;
        break;
      }
    }

    BROLL_LOG_INFO(
      "Hardware decoding enabled. Pixel format \"%s\"",
      av_get_pix_fmt_name(hwPixFmt_));
    codecCtx_->get_format = FrameDecoder::getHardwarePixelFormat;
    int err = av_hwdevice_ctx_create(&hwDeviceCtx_, hw_device_type, nullptr, nullptr, 0);
    if (err < 0) {
      assert(false && "Failed to initialize hardware decoder.");
    }
    codecCtx_->hw_device_ctx = av_buffer_ref(hwDeviceCtx_);
    {
      // Check what software-side pixel formats the hardware transfer supports.
      // Print all for visibility, but select just the first one.
      // This should probably be configurable, to let user target best pre-sws fmt.
      AVHWFramesConstraints * hw_frames_const = av_hwdevice_get_hwframe_constraints(
        hwDeviceCtx_, nullptr);
      assert(hw_frames_const && "Couldn't retrieve hardware device frame constraints.");
      hwSoftwarePixFmt_ = AV_PIX_FMT_NONE;
      for (AVPixelFormat * p = hw_frames_const->valid_sw_formats; *p != AV_PIX_FMT_NONE; p++) {
        if (sws_isSupportedInput(*p)) {
          BROLL_LOG_INFO(
            "Supported hardware-to-software pixel format: \"%s\"",
            av_get_pix_fmt_name(*p));
          if (hwSoftwarePixFmt_ == AV_PIX_FMT_NONE) {
            hwSoftwarePixFmt_ = *p;
          }
        }
      }
      av_hwframe_constraints_free(&hw_frames_const);
      BROLL_LOG_INFO("Selected hw/sw pixel format: \"%s\"", av_get_pix_fmt_name(hwSoftwarePixFmt_));
    }

    BROLL_LOG_INFO("Succeeded to init hardware decoder");
    hwFrame_ = av_frame_alloc();
    assert(hwFrame_ && "failed to alloc hardwareFrame");
  }

  if (avcodec_open2(codecCtx_, codec_, nullptr) < 0) {
    assert(false && "failed to open codec through avcodec_open2");
  }

  decodedFrame_ = av_frame_alloc();
  assert(decodedFrame_ && "failed to alloc decodedFrame");
  avcodec_parameters_free(&params);
}

FrameDecoder::~FrameDecoder()
{
  // final flush - seen in examples, necessary?
  avcodec_send_packet(codecCtx_, nullptr);
  avcodec_free_context(&codecCtx_);
  av_frame_free(&decodedFrame_);
  if (convertedFrame_) {
    av_frame_free(&convertedFrame_);
  }
  sws_freeContext(swsCtx_);
  av_packet_free(&packet_);
}

bool FrameDecoder::decodeFrame(const AVPacket & packet_in, AVFrame & frame_out)
{
  const int send_pkt_resp = avcodec_send_packet(codecCtx_, &packet_in);
  if (send_pkt_resp < 0) {
    char errStr[128] = {};
    av_strerror(send_pkt_resp, errStr, sizeof(errStr));
    BROLL_LOG_ERROR("avcodec_send_packet failed: %s", errStr);
    return false;
  }

  int recv_frame_resp;
  if (hwFrame_) {
    recv_frame_resp = avcodec_receive_frame(codecCtx_, hwFrame_);
  } else {
    recv_frame_resp = avcodec_receive_frame(codecCtx_, &frame_out);
  }
  if (recv_frame_resp == AVERROR(EAGAIN)) {
    BROLL_LOG_DEBUG("avcodec_receive_frame returned EAGAIN");
    return false;
  } else if (recv_frame_resp == AVERROR_EOF) {
    BROLL_LOG_ERROR("avcodec_receive_frame returned EOF");
    return false;
  } else if (recv_frame_resp < 0) {
    assert(false && "Error while receiving a frame from the decoder");
    return false;
  }
  if (recv_frame_resp < 0) {
    return false;
  }

  if (hwFrame_) {
    if (hwFrame_->format != hwPixFmt_) {
      BROLL_LOG_ERROR("Received hardware frame was not in expected pixel format.");
      return false;
    }
    // Move the frame from hardware device memory to CPU memory.
    // Setting the pixel format on frame_out prompts the transfer to convert to a specific format.
    frame_out.format = hwSoftwarePixFmt_;
    if (av_hwframe_transfer_data(&frame_out, hwFrame_, 0) < 0) {
      BROLL_LOG_ERROR("Error transferring the data to system memory");
      return false;
    }
  }

#if LIBAVCODEC_VERSION_MAJOR >= 60
  int64_t frame_num = codecCtx_->frame_num;
  bool is_key_frame = frame_out.flags & AV_FRAME_FLAG_KEY;
#else
  int64_t frame_num = codecCtx_->frame_number;
  bool is_key_frame = frame_out.key_frame;
#endif

  if (dbg_print_) {
    BROLL_LOG_INFO(
      "Frame %ld (type=%c, size=%d bytes, format=%d) pts %ld key_frame %d [DTS %ld]",
      frame_num,
      av_get_picture_type_char(frame_out.pict_type),
      packet_in.size,
      frame_out.format,
      frame_out.pts,
      is_key_frame,
      frame_out.pkt_dts
    );
  }
  if (skip_pframes_.load()) {
    if (is_key_frame) {
      BROLL_LOG_INFO(
        "Recovered next I-frame after skipping %d P-frames without a reference.",
        skipped_pframes_.load());
      skip_pframes_.store(false);
    } else {
      skipped_pframes_++;
    }
    return false;
  }
  return true;
}

void FrameDecoder::avLogCallbackWrapper(void * ptr, int level, const char * fmt, va_list vargs)
{
  // First forward to default libav callback to print properly
  av_log_default_callback(ptr, level, fmt, vargs);

  static const char * const badref_line = "Could not find ref with POC";

  AVClass * avc = ptr ? *reinterpret_cast<AVClass **>(ptr) : nullptr;
  if (avc == avcodec_get_class()) {
    // This log message is from/for an AVCodecContext
    auto * ctx = reinterpret_cast<AVCodecContext *>(ptr);
    if (
      ctx->opaque != nullptr &&
      strlen(fmt) >= strlen(badref_line) &&
      strncmp(fmt, badref_line, strlen(badref_line)) == 0)
    {
      // The user-set opaque pointer is there, and the log message matched the badref_line
      reinterpret_cast<broll::FrameDecoder *>(ctx->opaque)->startSkippingPFrames();
    }
  }
}

void FrameDecoder::startSkippingPFrames()
{
  if (!skip_pframes_.load()) {
    skipped_pframes_.store(0);
    skip_pframes_.store(true);
    BROLL_LOG_WARN("Skipping P-frames because of missing reference I-frame.");
  }
}

bool FrameDecoder::decode(const AVPacket & in, sensor_msgs::msg::Image & out)
{
  if (!decodeFrame(in, *decodedFrame_)) {
    if (++consecutive_receive_failures_ % 20 == 0) {
      BROLL_LOG_ERROR("Failed to decode 20 frames");
    }
    return false;
  }
  if (consecutive_receive_failures_ > 0) {
    BROLL_LOG_INFO(
      "Recovered from %d frame decode failures",
      consecutive_receive_failures_);
    consecutive_receive_failures_ = 0;
  }

  if (!convertedFrame_) {
    // Initialize converted frame and sws context on first decode

    int width = decodedFrame_->width;
    int height = decodedFrame_->height;
    // Round up to nearest multiple of 2
    scaled_width_ = width * scale_;
    scaled_width_ += (scaled_width_ % 2);
    scaled_height_ = height * scale_;
    scaled_height_ += (scaled_height_ % 2);

    BROLL_LOG_INFO(
      "Frame Decoder initialized: resolution in %d x %d, resolution out %d x %d",
      width, height, scaled_width_, scaled_height_);
    BROLL_LOG_INFO("\tCodec %d ('%s')", codec_->id, codec_->name);
    BROLL_LOG_INFO(
      "\tCodec pixfmt '%s', decoded pixfmt '%s'",
      av_get_pix_fmt_name(codecCtx_->pix_fmt),
      av_get_pix_fmt_name(static_cast<AVPixelFormat>(decodedFrame_->format)));
    BROLL_LOG_INFO("\tTarget pixfmt '%s'", av_get_pix_fmt_name(targetPixFmt_));

    convertedFrame_ = allocPicture(targetPixFmt_, scaled_width_, scaled_height_);
    assert(convertedFrame_ && "failed to alloc convertedFrame");

    bool set_extended_color_range = false;
    AVPixelFormat sws_pix_fmt = static_cast<AVPixelFormat>(decodedFrame_->format);
    switch (sws_pix_fmt) {
      case AV_PIX_FMT_YUVJ420P:
        sws_pix_fmt = AV_PIX_FMT_YUV420P;
        set_extended_color_range = true;
        break;
      case AV_PIX_FMT_YUVJ422P:
        sws_pix_fmt = AV_PIX_FMT_YUV422P;
        set_extended_color_range = true;
        break;
      case AV_PIX_FMT_YUVJ444P:
        sws_pix_fmt = AV_PIX_FMT_YUV444P;
        set_extended_color_range = true;
        break;
      default:
        break;
    }
    swsCtx_ = sws_getContext(
      width, height, sws_pix_fmt,
      scaled_width_, scaled_height_, targetPixFmt_,
      0, nullptr, nullptr, nullptr);
    assert(swsCtx_ && "Failed to created sws context for conversion.");
    if (set_extended_color_range) {
      int contrast, saturation, brightness, dstRange, srcRange;
      int * inv_table, * table;
      sws_getColorspaceDetails(
        swsCtx_, &inv_table, &srcRange, &table, &dstRange, &brightness,
        &contrast, &saturation);

      sws_setColorspaceDetails(
        swsCtx_, inv_table, 1, table, 1,
        brightness, contrast, saturation);
    }
  }
  sws_scale(
    swsCtx_,
    decodedFrame_->data, decodedFrame_->linesize, 0, decodedFrame_->height,
    convertedFrame_->data, convertedFrame_->linesize);

  if (!frame_to_image(*convertedFrame_, out)) {
    BROLL_LOG_ERROR("Failed to convert frame to img");
    return false;
  }
  return true;
}

bool FrameDecoder::decode(
  const sensor_msgs::msg::CompressedImage & in,
  sensor_msgs::msg::Image & out)
{
  packet_->pts = AV_NOPTS_VALUE;
  packet_->dts = AV_NOPTS_VALUE;
  packet_->size = in.data.size();
  // Packet is only used as const in the decode call
  packet_->data = const_cast<uint8_t *>(&in.data[0]);
  bool res = decode(*packet_, out);
  // Then remove the reference to the held data so it can't be used non-const by accident
  packet_->size = 0;
  packet_->data = nullptr;
  return res;
}

AVPixelFormat FrameDecoder::getHardwarePixelFormat(
  AVCodecContext * ctx, const AVPixelFormat * pix_fmts)
{
  const AVPixelFormat * p;
  const AVPixelFormat hwPixFmt = static_cast<FrameDecoder *>(ctx->opaque)->hwPixFmt_;
  for (p = pix_fmts; *p != AV_PIX_FMT_NONE; p++) {
    if (*p == hwPixFmt) {
      return *p;
    }
  }
  BROLL_LOG_ERROR("Failed to get HW surface format.");
  return AV_PIX_FMT_NONE;
}


}  // namespace broll
