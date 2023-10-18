# B-Roll: Video data in rosbag2

This repository contains plugins and libraries for interacting effectively with video data using rosbag2.

NOTE: This project is in an early stage and is incomplete - please forgive any issues, pull requests welcome!

Contains the following packages, see their respective `README.md` for more information:
* [`avcodec_msgs`](./avcodec_msgs/) Contains ROS message definitions for conveying necessary `libavcodec` data structures
* [`broll`](./broll/) The main `broll` utility library
* [`rosbag2_storage_broll`](./rosbag2_storage_broll/) A Rosbag2 storage implementation that can read encoded video frames from video files, and optionally decode them into raw images
