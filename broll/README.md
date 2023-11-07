# B-Roll

Provides utilities for dealing with video stream data in the context of Rosbag2.

Provides wrapper utilities for `libavcodec` (ffmpeg).

### broll

Library providing wrapper classes for common `libavcodec` <--> `sensor_msgs` operations

### decode_node

`decode_node` node takes in `CompressedImage` with some encoding and republishes as raw `Image`

Topics Subscribed:
* `video/compressed: sensor_msgs/msg/CompressedImage`

Topics Published:
* `video/raw: sensor_msgs/msg/Image`

Parameters:
* `scale: float` scale to apply to image before republishing. `0.5` will result in half size, rounds up to nearest even number on both width and height

Examples:

```
ros2 run broll decode_node --ros-args -r video/compressed:=camera0/compressed -r video/raw:=/camera0/raw_bgr -p scale:=0.5
```
