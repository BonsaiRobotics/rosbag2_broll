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

#include "broll/msg_conversions.hpp"
#include "broll/video_reader.hpp"

#include "logging.hpp"

static const uint64_t NS_TO_S = 1000000000;

namespace broll
{

VideoReader::VideoReader(const std::filesystem::path & videoPath)
{
  if (avformat_open_input(&formatCtx_, videoPath.c_str(), nullptr, nullptr) != 0) {
    throw std::runtime_error("Failed to open " + videoPath.string());
  }

  if (avformat_find_stream_info(formatCtx_, nullptr) < 0) {
    avformat_close_input(&formatCtx_);
    throw std::runtime_error("Failed to find stream info for " + videoPath.string());
  }

  videoStreamIndex_ =
    av_find_best_stream(formatCtx_, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
  if (videoStreamIndex_ < 0) {
    throw std::runtime_error("Failed to find video stream in " + videoPath.string());
  }

  stream_ = formatCtx_->streams[videoStreamIndex_];
  codecId_ = stream_->codecpar->codec_id;
  codecParams_ = stream_->codecpar;

  nextPacket_ = av_packet_alloc();
  assert(nextPacket_);

  BROLL_LOG_INFO(
    "Video Reader: resolution %d x %d",
    codecParams_->width, codecParams_->height);
  BROLL_LOG_INFO(
    "\tCodec ID %d bit_rate %ld",
    codecParams_->codec_id, codecParams_->bit_rate);

  switch (codecId_) {
    case AV_CODEC_ID_HEVC:
      format_name_ = "hevc";
      break;
    case AV_CODEC_ID_H264:
      format_name_ = "h264";
      break;
    default:
      throw std::runtime_error("Unsupported codec" + std::to_string(codecId_));
  }

  ts_scale_ = std::chrono::nanoseconds{
    static_cast<uint64_t>(stream_->time_base.num * NS_TO_S / stream_->time_base.den)};
}

VideoReader::~VideoReader()
{
  if (nextPacket_) {
    av_packet_unref(nextPacket_);
    av_packet_free(&nextPacket_);
  }
  avformat_close_input(&formatCtx_);
}

bool VideoReader::has_next()
{
  if (nextPacket_->data) {
    return true;
  }

  while (av_read_frame(formatCtx_, nextPacket_) >= 0) {
    if (nextPacket_->stream_index == videoStreamIndex_) {
      // TODO(emersonknapp) unref used packets?
      return true;
    }
    av_packet_unref(nextPacket_);
  }
  return false;
}

AVPacket * VideoReader::read_next()
{
  AVPacket * ret = nullptr;
  if (has_next()) {
    ret = nextPacket_;
    nextPacket_ = av_packet_alloc();
    assert(nextPacket_);
  }
  return ret;
}

AVCodecParameters * VideoReader::codec_parameters() const
{
  return codecParams_;
}

avcodec_msgs::msg::VideoCodecParameters VideoReader::codec_parameters_msg() const
{
  return message_from_parameters(codecParams_);
}

std::chrono::nanoseconds VideoReader::ts_scale() const
{
  return ts_scale_;
}

std::chrono::nanoseconds VideoReader::duration() const
{
  return std::chrono::nanoseconds{stream_->duration * ts_scale_.count()};
}

uint64_t VideoReader::frame_count() const
{
  return stream_->nb_frames;
}

std::string VideoReader::format_name() const
{
  return format_name_;
}

}  // namespace broll
