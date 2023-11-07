# rosbag2_storage_broll

Rosbag2 storage plugin that can read videos using B-Roll libraries.

Example:

```
ros2 bag play data/camera_0.mp4 --loop
```

Feature in this package:
* `rosbag2_storage_broll` storage implementation
* `rosbag2_storage_broll::bag_utils` exported utility library that contains some useful functions for manipulating bags

## Storage Plugin `rosbag2_storage_broll`

Rosbag2 allows passing arbitrary configuration to a storage plugin via the `--storage-config-file` option.
`rosbag2_storage_broll` takes a YAML file with the following fields:

```
ns: optional<string>  # namespace for published topics, must end with a /, default "video/"
compressed_topic: optional<string>  # compressed frame topic, default "compressed"
frame_id: optional<string>  # tf frame ID to apply to message headers
```

TODO(emersonknapp): This plugin should also handle standard bags with `CompressedImage` topic inside. There is no fundamental blocker for this.

## `bag_utils` library

See [bag_utils.hpp](./include/rosbag2_storage_broll/bag_utils.hpp) for details
