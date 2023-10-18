# Stores fields of AVCodecParameters struct relevant to video codecs
int32 codec_id # enum
uint32 codec_tag
uint8[] extradata
int32 format
int64 bit_rate
int32 bits_per_coded_sample
int32 bits_per_raw_sample
int32 profile
int32 level
int32 width
int32 height
Rational sample_aspect_ratio
int32 field_order # enum
int32 color_range # enum
int32 color_primaries # enum
int32 color_trc # enum
int32 color_space # enum
int32 chroma_location # enum
int32 video_delay
