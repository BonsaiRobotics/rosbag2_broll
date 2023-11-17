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

#include <fstream>

#include "broll/msg_conversions.hpp"
#include "broll/video_reader.hpp"

#include "logging.hpp"

static const uint64_t NS_TO_S = 1000000000;

namespace broll
{

VideoReader::VideoReader(const std::filesystem::path & videoPath)
{
  {
    // Note(emersonknapp) this is a little kludgy, but we just want to hard-disable opening an
    // MCAP file, this will say it's opened successfully but then fail to read anything
    const char mcap_header[] = "\x89MCAP\x30\r\n";
    char buf[sizeof(mcap_header)];
    std::fstream file(videoPath, std::ios::in | std::ios::binary);
    file.read(buf, sizeof(mcap_header));
    if (memcmp(buf, mcap_header, sizeof(mcap_header) - 1) == 0) {
      throw std::runtime_error("MCAP files are not supported by broll::VideoReader");
    }
  }

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
  bsfPacket_ = av_packet_alloc();
  assert(bsfPacket_);

  BROLL_LOG_INFO(
    "Video Reader: resolution %d x %d",
    codecParams_->width, codecParams_->height);
  BROLL_LOG_INFO(
    "\tCodec ID %d bit_rate %ld",
    codecParams_->codec_id, codecParams_->bit_rate);

  const char * bsf_name = nullptr;
  format_name_ = avcodec_get_name(codecId_);
  switch (codecId_) {
    case AV_CODEC_ID_HEVC:
      bsf_name = "hevc_mp4toannexb";
      break;
    case AV_CODEC_ID_H264:
      bsf_name = "h264_mp4toannexb";
      break;
    default:
      throw std::runtime_error("Unsupported codec" + std::to_string(codecId_));
  }

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
  BROLL_LOG_INFO("Bitstream filter %s initialized for format %s", bsf_name, format_name_.c_str());

  ts_scale_ = std::chrono::nanoseconds{
    static_cast<uint64_t>(stream_->time_base.num * NS_TO_S / stream_->time_base.den)};
}

VideoReader::~VideoReader()
{
  if (nextPacket_->data) {
    av_packet_unref(nextPacket_);
  }
  av_packet_free(&nextPacket_);

  if (bsfPacket_->data) {
    av_packet_unref(bsfPacket_);
  }
  av_packet_free(&bsfPacket_);

  avformat_close_input(&formatCtx_);
}

AVPacket * VideoReader::read_next()
{
  if (nextPacket_->data) {
    av_packet_unref(nextPacket_);
  }
  if (bsfPacket_->data) {
    av_packet_unref(bsfPacket_);
  }

  // First, if the bitstream filter already has data, return that before adding more
  if (av_bsf_receive_packet(bsfCtx_, bsfPacket_) >= 0) {
    return bsfPacket_;
  }
  // Next, check for a new frame from format
  // and if enabled send to bsf, else return the new frame
  while (av_read_frame(formatCtx_, nextPacket_) >= 0) {
    if (nextPacket_->stream_index == videoStreamIndex_) {
      if (av_bsf_send_packet(bsfCtx_, nextPacket_) < 0) {
        BROLL_LOG_ERROR("Failed to send packet to bitstream filter.");
        return nullptr;
      }
      break;
    }
  }
  // Finally check for bsf packets if data was sent to the bsf just now
  if (av_bsf_receive_packet(bsfCtx_, bsfPacket_) >= 0) {
    return bsfPacket_;
  }
  return nullptr;
}

AVCodecID VideoReader::codec_id() const
{
  return codecParams_->codec_id;
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
