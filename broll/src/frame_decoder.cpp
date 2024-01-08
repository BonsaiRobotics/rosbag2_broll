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
    exit(1);
  }
  return picture;
}

static const int64_t NS_TO_S = 1000000000;

}  // namespace

namespace broll
{

FrameDecoder::FrameDecoder(
  AVCodecID codec_id,
  AVPixelFormat target_fmt,
  double scale,
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
  int send_pkt_resp = avcodec_send_packet(codecCtx_, &packet_in);
  if (send_pkt_resp < 0) {
    char errStr[128] = {};
    av_strerror(send_pkt_resp, errStr, sizeof(errStr));
    BROLL_LOG_ERROR("avcodec_send_packet failed: %s", errStr);
    return false;
  }
  int recv_frame_resp = avcodec_receive_frame(codecCtx_, &frame_out);
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

  if (dbg_print_) {
    BROLL_LOG_INFO(
      "Frame %d (type=%c, size=%d bytes, format=%d) pts %ld key_frame %d [DTS %d]",
      codecCtx_->frame_number,
      av_get_picture_type_char(frame_out.pict_type),
      frame_out.pkt_size,
      frame_out.format,
      frame_out.pts,
      frame_out.key_frame,
      frame_out.coded_picture_number
    );
  }
  if (skip_pframes_.load()) {
    if (frame_out.key_frame) {
      BROLL_LOG_INFO(
        "Recovered next I frame after skipping %d P frames without a reference.",
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
  static const char * const badref_line = "Could not find ref with POC";

  AVClass * avc = ptr ? reinterpret_cast<AVClass *>(ptr) : nullptr;
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

  // Always forward to default libav callback after doing interception
  return av_log_default_callback(ptr, level, fmt, vargs);
}

void FrameDecoder::startSkippingPFrames()
{
  if (!skip_pframes_.load()) {
    skipped_pframes_.store(0);
    skip_pframes_.store(true);
    BROLL_LOG_WARN("Skipping P frames since couldn't find a reference I frame.");
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
    BROLL_LOG_INFO("\tCodec %s ID %d", codec_->name, codec_->id);

    convertedFrame_ = allocPicture(targetPixFmt_, scaled_width_, scaled_height_);
    assert(convertedFrame_ && "failed to alloc convertedFrame");

    swsCtx_ = sws_getContext(
      width, height, codecCtx_->pix_fmt,
      scaled_width_, scaled_height_, targetPixFmt_,
      0, nullptr, nullptr, nullptr);
    assert(swsCtx_ && "Failed to created sws context for conversion.");
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

}  // namespace broll
