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

#include "broll/msg_conversions.hpp"
#include "broll/video_reader.hpp"

#include "logging.hpp"

static const uint64_t NS_TO_S = 1000000000;

namespace broll
{

VideoReader::VideoReader(
  const std::filesystem::path & videoPath,
  bool do_annexb)
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

  const char * bsf_name = nullptr;
  switch (codecId_) {
    case AV_CODEC_ID_HEVC:
      format_name_ = "hevc";
      bsf_name = "hevc_mp4toannexb";
      break;
    case AV_CODEC_ID_H264:
      format_name_ = "h264";
      bsf_name = "h264_mp4toannexb";
      break;
    default:
      throw std::runtime_error("Unsupported codec" + std::to_string(codecId_));
  }

  if (do_annexb) {
    bitstreamFilter_ = av_bsf_get_by_name(bsf_name);
    if (!bitstreamFilter_) {
      throw std::runtime_error("Failed to find bitstream filter " + std::string(bsf_name));
    }
    if (av_bsf_alloc(bitstreamFilter_, &bsfCtx_) < 0) {
      throw std::runtime_error("Failed to allocate bitstream filter context");
    }
    bsfCtx_->par_in = codecParams_;
    if (av_bsf_init(bsfCtx_) < 0) {
      throw std::runtime_error("Failed to initialize bitstream filter context");
    }
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

AVPacket * VideoReader::read_next()
{
  while (av_read_frame(formatCtx_, nextPacket_) >= 0) {
    if (nextPacket_->stream_index == videoStreamIndex_) {
      return nextPacket_;
    }
    av_packet_unref(nextPacket_);
  }
  return nullptr;
}

const AVCodecParameters * VideoReader::codec_parameters() const
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
