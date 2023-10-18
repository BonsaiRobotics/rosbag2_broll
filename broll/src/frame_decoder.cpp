// Copyright (C) 2023 Bonsai Robotics, Inc - All Rights Reserved
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
  AVCodecParameters * params,
  AVPixelFormat target_fmt,
  double scale)
: targetPixFmt_(target_fmt),
  scale_(scale),
  packet_(av_packet_alloc())
{
  assert(params);

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

  if (avcodec_parameters_to_context(codecCtx_, params) < 0) {
    assert(false && "failed to copy codec params to codec context");
  }

  if (avcodec_open2(codecCtx_, codec_, nullptr) < 0) {
    assert(false && "failed to open codec through avcodec_open2");
  }
  const int width = codecCtx_->width;
  const int height = codecCtx_->height;

  // Round up to nearest multiple of 2
  scaled_width_ = width * scale_;
  scaled_width_ += (scaled_width_ % 2);
  scaled_height_ = height * scale_;
  scaled_height_ += (scaled_height_ % 2);

  BROLL_LOG_INFO(
    "Frame Decoder: resolution in %d x %d, resolution out %d x %d",
    width, height, scaled_width_, scaled_height_);
  BROLL_LOG_INFO("\tCodec %s ID %d bit_rate %ld", codec_->name, codec_->id, params->bit_rate);

  decodedFrame_ = allocPicture(codecCtx_->pix_fmt, width, height);
  assert(decodedFrame_ && "failed to alloc decodedFrame");
  convertedFrame_ = allocPicture(targetPixFmt_, scaled_width_, scaled_height_);
  assert(convertedFrame_ && "failed to alloc convertedFrame");

  swsCtx_ = sws_getContext(
    width, height, codecCtx_->pix_fmt,
    scaled_width_, scaled_height_, targetPixFmt_,
    0, nullptr, nullptr, nullptr);
  assert(swsCtx_ && "Failed to created sws context for conversion.");
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
  if (swsCtx_) {
    sws_freeContext(swsCtx_);
  }
  av_packet_free(&packet_);
}

bool FrameDecoder::decodeFrame(const AVPacket & packet_in, AVFrame & frame_out, bool dbg_print)
{
  int response = avcodec_send_packet(codecCtx_, &packet_in);
  if (response < 0) {
    char errStr[128] = {};
    av_strerror(response, errStr, sizeof(errStr));
    BROLL_LOG_ERROR("avcodec_send_packet failed: %s", errStr);
    return false;
  }
  response = avcodec_receive_frame(codecCtx_, &frame_out);
  if (response == AVERROR(EAGAIN)) {
    BROLL_LOG_DEBUG("avcodec_receive_frame returned EAGAIN");
    return false;
  } else if (response == AVERROR_EOF) {
    BROLL_LOG_ERROR("avcodec_receive_frame returned EOF");
    return false;
  } else if (response < 0) {
    assert(false && "Error while receiving a frame from the decoder");
    return false;
  }
  if (response >= 0) {
    if (dbg_print) {
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
    return true;
  }
  return false;
}

bool FrameDecoder::convertToImage(const AVFrame & in, sensor_msgs::msg::Image & out)
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
  AVFrame * finalFrame = decodedFrame_;
  if (swsCtx_) {
    sws_scale(
      swsCtx_,
      decodedFrame_->data, decodedFrame_->linesize, 0, decodedFrame_->height,
      convertedFrame_->data, convertedFrame_->linesize);
    finalFrame = convertedFrame_;
  }
  if (!convertToImage(*finalFrame, out)) {
    BROLL_LOG_ERROR("Failed to convert frame to img");
    return false;
  }

  return true;
}

bool FrameDecoder::decode(
  const sensor_msgs::msg::CompressedImage & in,
  sensor_msgs::msg::Image & out)
{
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
