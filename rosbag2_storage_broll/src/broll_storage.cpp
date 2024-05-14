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

#include <optional>
#include <queue>

#include "broll/frame_decoder.hpp"
#include "broll/msg_conversions.hpp"
#include "broll/video_reader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/time.hpp"
#include "rosbag2_storage/storage_interfaces/read_only_interface.hpp"
#include "rosbag2_storage/yaml.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

#include "rosbag2_storage_broll/bag_utils.hpp"

namespace
{
rosbag2_storage::TopicInformation create_topic_info(
  const std::string & topic_name,
  const std::string & type_name,
  const rclcpp::QoS & qos_profile)
{
  rosbag2_storage::TopicInformation topic_info;
  topic_info.topic_metadata.name = topic_name;
  topic_info.topic_metadata.type = type_name;
  topic_info.topic_metadata.serialization_format = "cdr";
  #if defined(ROS2_JAZZY) || defined(ROS2_ROLLING)
  topic_info.topic_metadata.offered_qos_profiles.push_back(qos_profile);
  #else
  topic_info.topic_metadata.offered_qos_profiles = rosbag2_storage_broll::serialize_qos(
    {
      qos_profile
    });
  #endif
  topic_info.message_count = 1;
  return topic_info;
}

struct BRollStorageConfig
{
  std::string tf_frame_id = "camera_link";

  bool pub_compressed = false;
  std::string compressed_topic = "video/compressed";

  bool pub_decoded = true;
  double decoded_scale = 1.0f;
  std::string decoded_topic = "video/raw";
  AVPixelFormat decoded_format = AV_PIX_FMT_BGR24;
};
}  // namespace

namespace YAML
{
template<>
struct convert<BRollStorageConfig>
{
  static bool decode(const Node & node, BRollStorageConfig & config)
  {
    // NOTE: if you update this, also update README.md please
    optional_assign<std::string>(node, "frame_id", config.tf_frame_id);

    optional_assign<bool>(node, "pub_compressed", config.pub_compressed);
    optional_assign<std::string>(node, "compressed_topic", config.compressed_topic);

    optional_assign<bool>(node, "pub_decoded", config.pub_decoded);
    optional_assign<double>(node, "scale", config.decoded_scale);
    optional_assign<std::string>(node, "decoded_topic", config.decoded_topic);

    std::string decoded_format_str = "bgr8";
    optional_assign<std::string>(node, "pix_fmt", decoded_format_str);
    config.decoded_format = broll::pixel_format_from_ros_string(decoded_format_str);
    if (config.decoded_format == AV_PIX_FMT_NONE) {
      throw std::runtime_error("Unknown pixel format " + decoded_format_str);
    }

    return true;
  }
};
}  // namespace YAML

namespace rosbag2_storage_broll
{

class BRollStorage : public rosbag2_storage::storage_interfaces::ReadOnlyInterface
{
public:
  BRollStorage();
  ~BRollStorage() override;

  /** BaseIOInterface **/
  void open(
    const rosbag2_storage::StorageOptions & storage_options,
    rosbag2_storage::storage_interfaces::IOFlag io_flag) override;

  /** BaseInfoInterface **/
  rosbag2_storage::BagMetadata get_metadata() override;
  std::string get_relative_file_path() const override;
  uint64_t get_bagfile_size() const override;
  std::string get_storage_identifier() const override;

  /** BaseReadInterface **/
  bool has_next() override;
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> read_next() override;
  std::vector<rosbag2_storage::TopicMetadata> get_all_topics_and_types() override;
  #if defined(ROS2_IRON) || defined(ROS2_JAZZY) || defined(ROS2_ROLLING)
  // Iron interface additions
  bool set_read_order(const rosbag2_storage::ReadOrder & order) override;
  void get_all_message_definitions(
    std::vector<rosbag2_storage::MessageDefinition> & definitions) override;
  #endif


  /** ReadOnlyInterface **/
  void set_filter(const rosbag2_storage::StorageFilter & storage_filter) override;
  void reset_filter() override;
  void seek(const rcutils_time_point_value_t & time_stamp) override;

protected:
  rosbag2_storage::BagMetadata metadata_{};

  // Configs
  rosbag2_storage::StorageOptions options_;
  rosbag2_storage::storage_interfaces::IOFlag io_flag_;
  rosbag2_storage::StorageFilter filter_;
  BRollStorageConfig config_;

  rclcpp::Logger logger_;
  std::optional<broll::VideoReader> video_reader_;
  std::optional<broll::FrameDecoder> frame_decoder_;
  AVPacket * next_frame_ = nullptr;
};


BRollStorage::BRollStorage()
: logger_(rclcpp::get_logger("BRollStorage"))
{
  metadata_.storage_identifier = get_storage_identifier();
}

BRollStorage::~BRollStorage()
{
}

void BRollStorage::open(
  const rosbag2_storage::StorageOptions & storage_options,
  rosbag2_storage::storage_interfaces::IOFlag io_flag)
{
  options_ = storage_options;
  io_flag_ = io_flag;

  if (io_flag != rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY) {
    RCLCPP_ERROR(logger_, "BRollStorage only supports read only");
    throw std::runtime_error("");
  }

  if (!storage_options.storage_config_uri.empty()) {
    YAML::Node node = YAML::LoadFile(storage_options.storage_config_uri);
    config_ = node.as<BRollStorageConfig>();
  }
  if (config_.pub_compressed && config_.pub_decoded) {
    RCLCPP_ERROR(
      logger_,
      "BRollStorage cannot publish both compressed and decoded at the same time (TODO)");
    throw std::runtime_error("");
  }

  video_reader_.emplace(storage_options.uri);
  frame_decoder_.emplace(video_reader_->codec_id(), config_.decoded_format, config_.decoded_scale);
  RCLCPP_INFO(logger_, "Successfully opened %s with BRollStorage", storage_options.uri.c_str());

  metadata_ = rosbag2_storage::BagMetadata{};
  metadata_.bag_size = get_bagfile_size();
  metadata_.storage_identifier = get_storage_identifier();
  metadata_.relative_file_paths = {get_relative_file_path()};
  metadata_.duration = video_reader_->duration();
  // TODO(emersonknapp) starting time, message count
  metadata_.starting_time = std::chrono::time_point<std::chrono::high_resolution_clock>{
    std::chrono::nanoseconds{0}};
  metadata_.message_count = video_reader_->frame_count();
  metadata_.compression_format = "";
  metadata_.compression_mode = "";

  if (config_.pub_compressed) {
    metadata_.topics_with_message_count.push_back(
      create_topic_info(
        config_.compressed_topic,
        "sensor_msgs/msg/CompressedImage",
        rclcpp::QoS(4)));
  }
  if (config_.pub_decoded) {
    metadata_.topics_with_message_count.push_back(
      create_topic_info(
        config_.decoded_topic,
        "sensor_msgs/msg/Image",
        rclcpp::QoS(4)));
  }
}

rosbag2_storage::BagMetadata BRollStorage::get_metadata()
{
  return metadata_;
}

std::string BRollStorage::get_relative_file_path() const
{
  return options_.uri;
}

uint64_t BRollStorage::get_bagfile_size() const
{
  return std::filesystem::file_size(get_relative_file_path());
}

std::string BRollStorage::get_storage_identifier() const
{
  return "avcodec";
}

bool BRollStorage::has_next()
{
  // TODO(emersonknapp) topic_filter
  if (next_frame_) {
    return true;
  }
  next_frame_ = video_reader_->read_next();
  return next_frame_ != nullptr;
}

std::shared_ptr<rosbag2_storage::SerializedBagMessage> BRollStorage::read_next()
{
  rclcpp::Time starting_time = rclcpp::Time{0};  // TODO(emersonknapp) starting time offset
  // TODO(emersonknapp) topic_filter

  if (!next_frame_) {
    next_frame_ = video_reader_->read_next();
    if (!next_frame_) {
      return nullptr;
    }
  }
  AVPacket * packet = next_frame_;

  rclcpp::Time ts =
    starting_time +
    rclcpp::Duration::from_nanoseconds(packet->pts * video_reader_->ts_scale().count());
  auto tsd = rclcpp::Duration::from_nanoseconds(ts.nanoseconds());

  std::shared_ptr<rosbag2_storage::SerializedBagMessage> ret = nullptr;
  if (config_.pub_compressed) {
    // TODO(emersonknapp) use preallocated pools to avoid dynamic alloc
    sensor_msgs::msg::CompressedImage ci_msg;
    ci_msg.header.stamp = ts;
    ci_msg.header.frame_id = config_.tf_frame_id;
    ci_msg.format = video_reader_->format_name();
    ci_msg.data.resize(packet->size);
    memcpy(&ci_msg.data[0], packet->data, packet->size);
    ret = serialize_msg(ci_msg, config_.compressed_topic, ts);
  } else if (config_.pub_decoded) {
    // TODO(emersonknapp) use preallocated pools to avoid dynamic alloc
    sensor_msgs::msg::Image image_msg;
    frame_decoder_->decode(*packet, image_msg);
    image_msg.header.stamp = ts;
    image_msg.header.frame_id = config_.tf_frame_id;
    ret = serialize_msg(image_msg, config_.decoded_topic, ts);
  }
  next_frame_ = nullptr;
  return ret;
}

std::vector<rosbag2_storage::TopicMetadata> BRollStorage::get_all_topics_and_types()
{
  std::vector<rosbag2_storage::TopicMetadata> topics;
  for (const auto & topic_info : metadata_.topics_with_message_count) {
    topics.push_back(topic_info.topic_metadata);
  }
  return topics;
}

#if defined(ROS2_IRON) || defined(ROS2_JAZZY) || defined(ROS2_ROLLING)
// Iron interface additions
bool BRollStorage::set_read_order(const rosbag2_storage::ReadOrder & order)
{
  if (order.reverse) {
    return false;
  }
  switch (order.sort_by) {
    case rosbag2_storage::ReadOrder::ReceivedTimestamp:
    case rosbag2_storage::ReadOrder::File:
      return true;
    default:
      return false;
  }
}
void BRollStorage::get_all_message_definitions(
  std::vector<rosbag2_storage::MessageDefinition> & definitions)
{
  definitions.clear();
}
#endif

void BRollStorage::set_filter(const rosbag2_storage::StorageFilter & storage_filter)
{
  filter_ = storage_filter;
}

void BRollStorage::reset_filter()
{
  filter_ = rosbag2_storage::StorageFilter{};
}

void BRollStorage::seek(const rcutils_time_point_value_t & /* time_stamp */)
{
  // TODO(emersonknapp) should be possible to implement this correctly with
  // av_seek_frame(formatCtx_, ...)
  if (video_reader_) {
    RCLCPP_WARN(logger_, "BRollStorage::seek not implemented fully, reopening at beginning");
    open(options_, io_flag_);
  } else {
    RCLCPP_WARN(logger_, "BRollStorage::seek called before open");
  }
}

}  // namespace rosbag2_storage_broll

PLUGINLIB_EXPORT_CLASS(
  rosbag2_storage_broll::BRollStorage,
  rosbag2_storage::storage_interfaces::ReadOnlyInterface)
