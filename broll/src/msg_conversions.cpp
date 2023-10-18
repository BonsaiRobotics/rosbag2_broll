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

#include <functional>
#include <string>

extern "C" {
#include "libavcodec/avcodec.h"
}

#include "broll/msg_conversions.hpp"

namespace broll
{

avcodec_msgs::msg::VideoCodecParameters message_from_parameters(const AVCodecParameters * p)
{
  avcodec_msgs::msg::VideoCodecParameters m;
  m.codec_id = p->codec_id;
  m.codec_tag = p->codec_tag;
  m.extradata.resize(p->extradata_size);
  memcpy(&m.extradata[0], p->extradata, p->extradata_size);
  m.format = p->format;
  m.bit_rate = p->bit_rate;
  m.bits_per_coded_sample = p->bits_per_coded_sample;
  m.bits_per_raw_sample = p->bits_per_raw_sample;
  m.profile = p->profile;
  m.level = p->level;
  m.width = p->width;
  m.height = p->height;
  m.sample_aspect_ratio.num = p->sample_aspect_ratio.num;
  m.sample_aspect_ratio.den = p->sample_aspect_ratio.den;
  m.field_order = p->field_order;
  m.color_range = p->color_range;
  m.color_primaries = p->color_primaries;
  m.color_trc = p->color_trc;
  m.color_space = p->color_space;
  m.chroma_location = p->chroma_location;
  m.video_delay = p->video_delay;
  return m;
}

void free_avcodec_parameters(AVCodecParameters * p)
{
  avcodec_parameters_free(&p);
}

AVCodecParameters * parameters_from_message(
  const avcodec_msgs::msg::VideoCodecParameters & m)
{
  AVCodecParameters * p = avcodec_parameters_alloc();
  p->codec_type = AVMEDIA_TYPE_VIDEO;
  p->codec_id = (AVCodecID)m.codec_id;
  p->codec_tag = m.codec_tag;
  p->extradata = reinterpret_cast<uint8_t *>(av_malloc(m.extradata.size()));
  memcpy(p->extradata, &m.extradata[0], m.extradata.size());
  p->extradata_size = m.extradata.size();
  p->format = m.format;
  p->bit_rate = m.bit_rate;
  p->bits_per_coded_sample = m.bits_per_coded_sample;
  p->bits_per_raw_sample = m.bits_per_raw_sample;
  p->profile = m.profile;
  p->level = m.level;
  p->width = m.width;
  p->height = m.height;
  p->sample_aspect_ratio.num = m.sample_aspect_ratio.num;
  p->sample_aspect_ratio.den = m.sample_aspect_ratio.den;
  p->field_order = (AVFieldOrder)m.field_order;
  p->color_range = (AVColorRange)m.color_range;
  p->color_primaries = (AVColorPrimaries)m.color_primaries;
  p->color_trc = (AVColorTransferCharacteristic)m.color_trc;
  p->color_space = (AVColorSpace)m.color_space;
  p->chroma_location = (AVChromaLocation)m.chroma_location;
  p->video_delay = m.video_delay;
  return p;
}

}  // namespace broll
