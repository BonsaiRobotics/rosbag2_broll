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

#ifndef BROLL__VIDEO_READER_HPP_
#define BROLL__VIDEO_READER_HPP_

extern "C" {
#include <libavcodec/avcodec.h>
#if LIBAVCODEC_VERSION_MAJOR >= 60
#include <libavcodec/bsf.h>
#endif
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}

#include <filesystem>
#include <string>

namespace broll
{

class VideoReader
{
public:
  explicit VideoReader(const std::filesystem::path & videoFilename);
  virtual ~VideoReader();

  // Core API
  AVPacket * read_next();

  // Information accessors
  AVCodecID codec_id() const;
  std::chrono::nanoseconds ts_scale() const;
  std::chrono::nanoseconds duration() const;
  uint64_t frame_count() const;
  std::string format_name() const;

protected:
  AVPacket * nextPacket_ = nullptr;
  AVPacket * bsfPacket_ = nullptr;
  AVFormatContext * formatCtx_ = nullptr;
  const AVBitStreamFilter * bitstreamFilter_ = nullptr;
  AVBSFContext * bsfCtx_ = nullptr;
  AVStream * stream_ = nullptr;
  AVCodecID codecId_ = AV_CODEC_ID_NONE;
  int videoStreamIndex_ = -1;
  AVCodecParameters * codecParams_ = nullptr;
  std::string format_name_;
  std::chrono::nanoseconds ts_scale_;  // nanoseconds per pts unit
};

}  // namespace broll

#endif  // BROLL__VIDEO_READER_HPP_
