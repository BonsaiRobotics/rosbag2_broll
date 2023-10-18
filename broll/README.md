# B-Roll

Provides utilities for dealing with video stream data in the context of Rosbag2.

Provides wrapper utilities for `libavcodec` (ffmpeg).

### broll

Library providing wrapper classes for common `libavcodec` <--> `sensor_msgs` operations

### decode_node

`decode_node` node takes in `CompressedImage` with some encoding and republishes as raw `Image`

Topics Subscribed:
* `video/compressed: sensor_msgs/msg/CompressedImage`
* `video/codec_params: avcodec_msgs/msg/AVCodecParameters` (NOTE: this shouldn't be strictly necessary but it is in this first pass implementation)

Topics Published:
* `video/raw: sensor_msgs/msg/Image`

Parameters:
* `scale: float` scale to apply to image before republishing. `0.5` will result in half size, rounds up to nearest even number on both width and height

Examples:

```
ros2 run broll decode_node --ros-args -r video/compressed:=camera0/compressed -r video/codec_params:=camera0/codec_params -r video/raw:=/camera0/raw_bgr -p scale:=0.5
```

### Note on decoding

These components currently rely (when decoding remotely from the video file being read) on sending `AVCodecParameters` struct over the wire as a message, to help initialize the codec on the other side.

This shouldn't be strictly necessary for H264/H265, as their bitstream protocol should allow for codec initialization fully from the sent packets.
However when reading from an MP4 file, the special information (SPS/PPS) are not encoded in frame packets, and I do not know (yet) how to do that.
That single-stream approach would be a little neater, but for now with latched publishers, the side channel codec params are still quite reliable.
