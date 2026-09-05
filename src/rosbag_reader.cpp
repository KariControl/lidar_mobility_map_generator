#include "lidar_mobility_map_generator/rosbag_reader.hpp"

#include "lidar_mobility_map_generator/trajectory.hpp"
#include "lidar_mobility_map_generator/transform_graph.hpp"
#include "lidar_mobility_map_generator/voxel_map.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

namespace lidar_mobility_map_generator
{

bool acceptsDirectTfPoseEndpoint(
  const std::string & frame_id,
  const RosbagInputConfig & input_config)
{
  const std::string endpoint = normalizeFrameId(frame_id);
  const std::string base = normalizeFrameId(input_config.base_frame);
  if (input_config.pose_reference_frame == "base") {
    return endpoint == base;
  }
  if (input_config.pose_reference_frame == "sensor") {
    const std::string sensor = normalizeFrameId(input_config.sensor_frame);
    return endpoint == sensor || endpoint == base;
  }
  return false;
}

namespace
{

struct RawPose
{
  std::int64_t stamp_ns{0};
  std::string parent_frame;
  std::string child_frame;
  Transform parent_from_child{};
};

std::int64_t stampToNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

template<typename MessageT>
auto bagTimestampValue(const MessageT & message, int) -> decltype(message.recv_timestamp)
{
  return message.recv_timestamp;
}

template<typename MessageT>
auto bagTimestampValue(const MessageT & message, long) -> decltype(message.time_stamp)
{
  return message.time_stamp;
}

std::int64_t bagTimestampNanoseconds(
  const rosbag2_storage::SerializedBagMessage & message)
{
  return static_cast<std::int64_t>(bagTimestampValue(message, 0));
}

Transform transformFromMessage(const geometry_msgs::msg::Transform & message)
{
  Quaternion quaternion{
    message.rotation.x, message.rotation.y,
    message.rotation.z, message.rotation.w};
  if (quaternion.squaredNorm() < 1.0e-15) {
    quaternion = {};
  }
  return {{message.translation.x, message.translation.y, message.translation.z}, quaternion.normalized()};
}

Transform poseFromMessage(const geometry_msgs::msg::Pose & message)
{
  Quaternion quaternion{
    message.orientation.x, message.orientation.y,
    message.orientation.z, message.orientation.w};
  if (quaternion.squaredNorm() < 1.0e-15) {
    quaternion = {};
  }
  return {{message.position.x, message.position.y, message.position.z}, quaternion.normalized()};
}

template<typename MessageT>
MessageT deserialize(
  const std::shared_ptr<rosbag2_storage::SerializedBagMessage> & bag_message)
{
  rclcpp::SerializedMessage serialized_message(*bag_message->serialized_data);
  rclcpp::Serialization<MessageT> serialization;
  MessageT result;
  serialization.deserialize_message(&serialized_message, &result);
  return result;
}

std::unique_ptr<rosbag2_cpp::Reader> openReader(const RosbagInputConfig & config)
{
  auto reader = std::make_unique<rosbag2_cpp::Reader>();
  if (config.storage_id.empty()) {
    // Let rosbag2 inspect metadata.yaml and select the storage plugin.
    reader->open(config.bag_path.string());
  } else {
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = config.bag_path.string();
    storage_options.storage_id = config.storage_id;
    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";
    reader->open(storage_options, converter_options);
  }
  return reader;
}

std::unordered_map<std::string, std::string> topicTypes(rosbag2_cpp::Reader & reader)
{
  std::unordered_map<std::string, std::string> result;
  for (const auto & topic : reader.get_all_topics_and_types()) {
    result[topic.name] = topic.type;
  }
  return result;
}

void requireTopicType(
  const std::unordered_map<std::string, std::string> & topics,
  const std::string & topic,
  const std::string & expected)
{
  const auto found = topics.find(topic);
  if (found == topics.end()) {
    throw std::runtime_error("rosbag2 does not contain required topic: " + topic);
  }
  if (found->second != expected) {
    throw std::runtime_error(
            "topic " + topic + " has type " + found->second + ", expected " + expected);
  }
}

std::vector<TimedPose> resolveRawPoses(
  const std::vector<RawPose> & raw,
  const StaticTransformGraph & static_transforms,
  const RosbagInputConfig & config,
  std::vector<std::string> & warnings)
{
  const std::string world = normalizeFrameId(config.world_frame);
  const std::string base = normalizeFrameId(config.base_frame);
  const std::string sensor = normalizeFrameId(config.sensor_frame);
  const bool sensor_reference = config.pose_reference_frame == "sensor";
  std::vector<TimedPose> result;
  result.reserve(raw.size());
  std::size_t unresolved = 0U;
  std::size_t overridden_child_labels = 0U;
  for (const RawPose & observation : raw) {
    const std::string parent = normalizeFrameId(observation.parent_frame);
    const std::string child = normalizeFrameId(observation.child_frame);
    const std::optional<Transform> world_from_parent =
      parent == world ? std::optional<Transform>(Transform{}) :
      static_transforms.resolve(world, parent);
    const std::optional<Transform> child_from_reference = sensor_reference ?
      std::optional<Transform>(Transform{}) :
      (child == base ? std::optional<Transform>(Transform{}) :
      static_transforms.resolve(child, base));
    if (!world_from_parent || !child_from_reference) {
      ++unresolved;
      continue;
    }
    if (sensor_reference && child != sensor) {
      ++overridden_child_labels;
    }
    result.push_back({
      observation.stamp_ns,
      *world_from_parent * observation.parent_from_child * *child_from_reference});
  }
  result = normalizeTrajectory(result);
  if (unresolved > 0U) {
    warnings.push_back(
      std::to_string(unresolved) +
      " pose observations could not be transformed to the configured world/reference frames");
  }
  if (overridden_child_labels > 0U) {
    warnings.push_back(
      std::to_string(overridden_child_labels) +
      " pose child-frame labels were overridden because pose_reference_frame=sensor");
  }
  if (result.size() < 2U) {
    throw std::runtime_error(
            "fewer than two self-localization poses could be resolved to the configured "
            "world/reference frames");
  }
  return result;
}

class PointCloudFieldReader
{
public:
  explicit PointCloudFieldReader(
    const sensor_msgs::msg::PointCloud2 & message,
    const std::string & requested_time_field)
  : message_(message)
  {
    const std::size_t point_count =
      static_cast<std::size_t>(message_.width) * message_.height;
    if (message_.point_step == 0U && point_count > 0U) {
      throw std::runtime_error("PointCloud2 point_step is zero");
    }
    const std::size_t required_row_step =
      static_cast<std::size_t>(message_.point_step) * message_.width;
    if (message_.height > 0U && message_.row_step < required_row_step) {
      throw std::runtime_error("PointCloud2 row_step is smaller than width * point_step");
    }
    const std::size_t required_data_size =
      static_cast<std::size_t>(message_.row_step) * message_.height;
    if (message_.data.size() < required_data_size) {
      throw std::runtime_error("PointCloud2 data buffer is smaller than row_step * height");
    }
    x_ = requireField({"x"});
    y_ = requireField({"y"});
    z_ = requireField({"z"});
    intensity_ = findField({"intensity", "reflectivity", "reflection", "i"});
    if (!requested_time_field.empty()) {
      time_ = findField({requested_time_field});
    }
  }

  [[nodiscard]] bool hasTime() const {return time_.has_value();}

  template<typename CallbackT>
  void forEach(const std::size_t point_stride, CallbackT && callback) const
  {
    const std::size_t stride = std::max<std::size_t>(1U, point_stride);
    std::size_t linear_index = 0U;
    for (std::size_t row = 0U; row < message_.height; ++row) {
      const std::uint8_t * row_data = message_.data.data() + row * message_.row_step;
      for (std::size_t column = 0U; column < message_.width; ++column, ++linear_index) {
        if (linear_index % stride != 0U) {
          continue;
        }
        const std::uint8_t * point = row_data + column * message_.point_step;
        const double x = read(point, x_);
        const double y = read(point, y_);
        const double z = read(point, z_);
        const double intensity = intensity_ ? read(point, *intensity_) : 0.0;
        const std::optional<double> time = time_ ?
          std::optional<double>(read(point, *time_)) : std::nullopt;
        callback(x, y, z, intensity, time);
      }
    }
  }

private:
  struct Field
  {
    std::size_t offset{0U};
    std::uint8_t datatype{0U};
  };

  [[nodiscard]] static std::size_t datatypeSize(const std::uint8_t datatype)
  {
    switch (datatype) {
      case sensor_msgs::msg::PointField::INT8:
      case sensor_msgs::msg::PointField::UINT8:
        return 1U;
      case sensor_msgs::msg::PointField::INT16:
      case sensor_msgs::msg::PointField::UINT16:
        return 2U;
      case sensor_msgs::msg::PointField::INT32:
      case sensor_msgs::msg::PointField::UINT32:
      case sensor_msgs::msg::PointField::FLOAT32:
        return 4U;
      case sensor_msgs::msg::PointField::FLOAT64:
        return 8U;
      default:
        throw std::runtime_error("unsupported PointCloud2 field datatype");
    }
  }

  [[nodiscard]] std::optional<Field> findField(const std::vector<std::string> & names) const
  {
    for (const std::string & name : names) {
      for (const sensor_msgs::msg::PointField & field : message_.fields) {
        if (field.name == name) {
          if (field.count == 0U ||
            static_cast<std::size_t>(field.offset) + datatypeSize(field.datatype) >
            message_.point_step)
          {
            throw std::runtime_error("PointCloud2 field is outside point_step: " + field.name);
          }
          return Field{field.offset, field.datatype};
        }
      }
    }
    return std::nullopt;
  }

  [[nodiscard]] Field requireField(const std::vector<std::string> & names) const
  {
    const auto field = findField(names);
    if (!field) {
      throw std::runtime_error("PointCloud2 is missing required x/y/z field");
    }
    return *field;
  }

  template<typename T>
  [[nodiscard]] T readValue(const std::uint8_t * data) const
  {
    T value{};
    std::memcpy(&value, data, sizeof(T));
    const bool host_little_endian = []() {
        const std::uint16_t test = 1U;
        return *reinterpret_cast<const std::uint8_t *>(&test) == 1U;
      }();
    if (message_.is_bigendian == host_little_endian && sizeof(T) > 1U) {
      std::uint8_t * begin = reinterpret_cast<std::uint8_t *>(&value);
      std::reverse(begin, begin + sizeof(T));
    }
    return value;
  }

  [[nodiscard]] double read(const std::uint8_t * point, const Field & field) const
  {
    const std::uint8_t * data = point + field.offset;
    switch (field.datatype) {
      case sensor_msgs::msg::PointField::INT8:
        return static_cast<double>(readValue<std::int8_t>(data));
      case sensor_msgs::msg::PointField::UINT8:
        return static_cast<double>(readValue<std::uint8_t>(data));
      case sensor_msgs::msg::PointField::INT16:
        return static_cast<double>(readValue<std::int16_t>(data));
      case sensor_msgs::msg::PointField::UINT16:
        return static_cast<double>(readValue<std::uint16_t>(data));
      case sensor_msgs::msg::PointField::INT32:
        return static_cast<double>(readValue<std::int32_t>(data));
      case sensor_msgs::msg::PointField::UINT32:
        return static_cast<double>(readValue<std::uint32_t>(data));
      case sensor_msgs::msg::PointField::FLOAT32:
        return static_cast<double>(readValue<float>(data));
      case sensor_msgs::msg::PointField::FLOAT64:
        return readValue<double>(data);
      default:
        throw std::runtime_error("unsupported PointCloud2 field datatype");
    }
  }

  const sensor_msgs::msg::PointCloud2 & message_;
  Field x_{};
  Field y_{};
  Field z_{};
  std::optional<Field> intensity_;
  std::optional<Field> time_;
};

std::int64_t cloudTimestamp(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const rosbag2_storage::SerializedBagMessage & bag_message,
  const bool use_header_stamp)
{
  const std::int64_t header = stampToNanoseconds(cloud.header.stamp);
  return use_header_stamp && header != 0 ? header : bagTimestampNanoseconds(bag_message);
}

std::optional<std::int64_t> deskewPointTimestamp(
  const std::int64_t cloud_stamp,
  const double raw_point_time,
  const DeskewConfig & config)
{
  const long double scaled_time_ns =
    (static_cast<long double>(raw_point_time) *
    static_cast<long double>(config.point_time_scale_sec) +
    static_cast<long double>(config.point_time_offset_sec)) * 1.0e9L;
  long double point_stamp_ns = scaled_time_ns;
  if (config.point_time_reference == "relative") {
    point_stamp_ns += static_cast<long double>(cloud_stamp);
  } else if (config.point_time_reference != "absolute") {
    throw std::runtime_error("deskew point_time_reference must be 'relative' or 'absolute'");
  }
  if (!std::isfinite(point_stamp_ns)) {
    return std::nullopt;
  }
  const long double rounded_stamp_ns = std::round(point_stamp_ns);
  if (rounded_stamp_ns <
    static_cast<long double>(std::numeric_limits<std::int64_t>::lowest()) ||
    rounded_stamp_ns > static_cast<long double>(std::numeric_limits<std::int64_t>::max()))
  {
    return std::nullopt;
  }
  return static_cast<std::int64_t>(rounded_stamp_ns);
}

std::optional<Transform> baseFromCloudFrame(
  const std::string & cloud_frame_value,
  const StaticTransformGraph & static_transforms,
  const RosbagInputConfig & config,
  const ExtrinsicsConfig & extrinsics)
{
  const std::string cloud_frame = normalizeFrameId(cloud_frame_value.empty() ?
      config.sensor_frame : cloud_frame_value);
  const std::string base = normalizeFrameId(config.base_frame);
  const std::string world = normalizeFrameId(config.world_frame);
  const std::string configured_sensor = normalizeFrameId(config.sensor_frame);
  if (cloud_frame == base || cloud_frame == world) {
    return Transform{};
  }
  if (extrinsics.source == "tf_static") {
    const auto resolved = static_transforms.resolve(base, cloud_frame);
    if (resolved) {
      return resolved;
    }
  } else if (extrinsics.source == "parameters") {
    if (cloud_frame == configured_sensor || !config.strict_frame_check) {
      return extrinsics.base_from_sensor;
    }
  } else {
    throw std::runtime_error("extrinsics.source must be 'tf_static' or 'parameters'");
  }
  return std::nullopt;
}

void addStaticTransforms(
  const tf2_msgs::msg::TFMessage & message,
  StaticTransformGraph & graph)
{
  for (const geometry_msgs::msg::TransformStamped & transform : message.transforms) {
    graph.add(
      transform.header.frame_id,
      transform.child_frame_id,
      transformFromMessage(transform.transform));
  }
}

}  // namespace

MappingDataset readRosbagDataset(
  const RosbagInputConfig & config,
  const ExtrinsicsConfig & extrinsics,
  const MapBuilderConfig & map_config)
{
  if (config.bag_path.empty() || !std::filesystem::exists(config.bag_path)) {
    throw std::runtime_error("rosbag2 path does not exist: " + config.bag_path.string());
  }
  if (config.pointcloud_mode != "scan" && config.pointcloud_mode != "accumulated_map") {
    throw std::runtime_error("pointcloud_mode must be 'scan' or 'accumulated_map'");
  }
  if (normalizeFrameId(config.world_frame).empty() || normalizeFrameId(config.base_frame).empty()) {
    throw std::runtime_error("world_frame and base_frame must not be empty");
  }
  if (!(config.maximum_pose_gap_sec > 0.0) || !std::isfinite(config.maximum_pose_gap_sec)) {
    throw std::runtime_error("maximum_pose_gap_sec must be finite and greater than zero");
  }
  if (config.deskew.enabled) {
    if (config.deskew.point_time_field.empty()) {
      throw std::runtime_error("deskew point_time_field must not be empty when deskew is enabled");
    }
    if (config.deskew.point_time_reference != "relative" &&
      config.deskew.point_time_reference != "absolute")
    {
      throw std::runtime_error("deskew point_time_reference must be 'relative' or 'absolute'");
    }
    if (!std::isfinite(config.deskew.point_time_scale_sec) ||
      config.deskew.point_time_scale_sec == 0.0 ||
      !std::isfinite(config.deskew.point_time_offset_sec))
    {
      throw std::runtime_error("deskew time scale must be nonzero and time values must be finite");
    }
  }
  if (!(map_config.voxel_size > 0.0) || map_config.scan_stride == 0U ||
    map_config.point_stride == 0U || map_config.minimum_observations_per_voxel == 0U)
  {
    throw std::runtime_error("invalid map builder voxel size, stride, or observation threshold");
  }

  MappingDataset dataset;
  dataset.world_frame = normalizeFrameId(config.world_frame);
  StaticTransformGraph static_transforms;
  std::vector<RawPose> raw_poses;
  std::optional<nav_msgs::msg::Path> latest_path;

  {
    std::unique_ptr<rosbag2_cpp::Reader> reader = openReader(config);
    const auto topics = topicTypes(*reader);
    requireTopicType(topics, config.pointcloud_topic, "sensor_msgs/msg/PointCloud2");
    if (config.pose_source == "odometry") {
      requireTopicType(topics, config.pose_topic, "nav_msgs/msg/Odometry");
    } else if (config.pose_source == "pose_stamped") {
      requireTopicType(topics, config.pose_topic, "geometry_msgs/msg/PoseStamped");
    } else if (config.pose_source == "path") {
      requireTopicType(topics, config.pose_topic, "nav_msgs/msg/Path");
    } else if (config.pose_source != "tf") {
      throw std::runtime_error("pose_source must be odometry, pose_stamped, path, or tf");
    }

    while (reader->has_next()) {
      const auto bag_message = reader->read_next();
      if (bag_message->topic_name == config.tf_static_topic) {
        addStaticTransforms(deserialize<tf2_msgs::msg::TFMessage>(bag_message), static_transforms);
        continue;
      }
      if (config.pose_source == "odometry" && bag_message->topic_name == config.pose_topic) {
        const nav_msgs::msg::Odometry message = deserialize<nav_msgs::msg::Odometry>(bag_message);
        const std::int64_t header_stamp = stampToNanoseconds(message.header.stamp);
        raw_poses.push_back({
          header_stamp != 0 ? header_stamp : bagTimestampNanoseconds(*bag_message),
          message.header.frame_id.empty() ? config.world_frame : message.header.frame_id,
          message.child_frame_id.empty() ? config.base_frame : message.child_frame_id,
          poseFromMessage(message.pose.pose)});
        ++dataset.statistics.pose_messages;
      } else if (
        config.pose_source == "pose_stamped" && bag_message->topic_name == config.pose_topic)
      {
        const geometry_msgs::msg::PoseStamped message =
          deserialize<geometry_msgs::msg::PoseStamped>(bag_message);
        const std::int64_t header_stamp = stampToNanoseconds(message.header.stamp);
        raw_poses.push_back({
          header_stamp != 0 ? header_stamp : bagTimestampNanoseconds(*bag_message),
          message.header.frame_id.empty() ? config.world_frame : message.header.frame_id,
          config.base_frame,
          poseFromMessage(message.pose)});
        ++dataset.statistics.pose_messages;
      } else if (config.pose_source == "path" && bag_message->topic_name == config.pose_topic) {
        // A SLAM node may republish the complete, newly optimized path. Retain only the
        // latest revision instead of mixing obsolete and corrected historical poses.
        latest_path = deserialize<nav_msgs::msg::Path>(bag_message);
      } else if (config.pose_source == "tf" && bag_message->topic_name == config.tf_topic) {
        const tf2_msgs::msg::TFMessage message = deserialize<tf2_msgs::msg::TFMessage>(bag_message);
        const std::string world = normalizeFrameId(config.world_frame);
        for (const geometry_msgs::msg::TransformStamped & transform : message.transforms) {
          const std::string parent = normalizeFrameId(transform.header.frame_id);
          const std::string child = normalizeFrameId(transform.child_frame_id);
          const bool accepted_child = acceptsDirectTfPoseEndpoint(child, config);
          const bool accepted_parent = acceptsDirectTfPoseEndpoint(parent, config);
          if (parent == world && accepted_child) {
            raw_poses.push_back({
              stampToNanoseconds(transform.header.stamp), parent, child,
              transformFromMessage(transform.transform)});
            ++dataset.statistics.pose_messages;
          } else if (accepted_parent && child == world) {
            raw_poses.push_back({
              stampToNanoseconds(transform.header.stamp), world, parent,
              transformFromMessage(transform.transform).inverse()});
            ++dataset.statistics.pose_messages;
          }
        }
      }
    }

    if (config.pose_source == "path") {
      if (!latest_path || latest_path->poses.size() < 2U) {
        throw std::runtime_error("the latest Path contains fewer than two poses");
      }
      const std::int64_t path_header_stamp = stampToNanoseconds(latest_path->header.stamp);
      raw_poses.reserve(latest_path->poses.size());
      for (std::size_t index = 0U; index < latest_path->poses.size(); ++index) {
        const geometry_msgs::msg::PoseStamped & pose = latest_path->poses[index];
        std::int64_t stamp = stampToNanoseconds(pose.header.stamp);
        if (stamp == 0) {
          if (config.pointcloud_mode == "scan") {
            throw std::runtime_error(
                    "Path poses require valid timestamps when pointcloud_mode is 'scan'");
          }
          // An accumulated map needs pose order for route generation but does not need
          // scan synchronization. Preserve order with deterministic synthetic stamps.
          stamp = std::max<std::int64_t>(1, path_header_stamp) +
            static_cast<std::int64_t>(index);
        }
        const std::string parent = !pose.header.frame_id.empty() ? pose.header.frame_id :
          (!latest_path->header.frame_id.empty() ? latest_path->header.frame_id :
          config.world_frame);
        raw_poses.push_back({stamp, parent, config.base_frame, poseFromMessage(pose.pose)});
      }
      dataset.statistics.pose_messages = latest_path->poses.size();
    }
  }

  dataset.trajectory = resolveRawPoses(raw_poses, static_transforms, config, dataset.warnings);
  if (config.pose_reference_frame == "sensor") {
    const std::optional<Transform> base_from_sensor = baseFromCloudFrame(
      config.sensor_frame, static_transforms, config, extrinsics);
    if (!base_from_sensor) {
      throw std::runtime_error(
              "cannot convert sensor-reference localization poses without "
              "a resolvable base <- sensor extrinsic");
    }
    dataset.trajectory = sensorPosesToBase(dataset.trajectory, *base_from_sensor);
    dataset.warnings.push_back(
      "self-localization poses were interpreted as sensor-origin poses and converted "
      "to the configured body base using T_base_sensor");
  }
  PoseBuffer poses(dataset.trajectory);
  if (config.pose_source == "tf") {
    dataset.warnings.push_back(
      "TF pose mode currently reads only direct world<->base or world<->sensor transforms; "
      "use Odometry for dynamic TF chains");
  }

  VoxelMapAccumulator voxel_map(map_config.voxel_size);
  std::optional<sensor_msgs::msg::PointCloud2> last_accumulated_map;
  std::int64_t last_accumulated_stamp = 0;
  std::uint64_t observation_id = 0U;
  bool warned_missing_time = false;
  bool warned_unresolved_cloud_frame = false;

  auto process_cloud = [&](
      const sensor_msgs::msg::PointCloud2 & cloud,
      const std::int64_t cloud_stamp,
      const std::uint64_t current_observation) {
      const std::string cloud_frame = normalizeFrameId(
        cloud.header.frame_id.empty() ? config.sensor_frame : cloud.header.frame_id);
      const std::string world = normalizeFrameId(config.world_frame);
      const bool already_world = cloud_frame == world;
      if (config.pointcloud_mode == "accumulated_map" && !already_world) {
        throw std::runtime_error(
                "accumulated_map PointCloud2 must already be expressed in world_frame ('" +
                world + "'), but header.frame_id is '" + cloud_frame + "'");
      }
      const std::optional<Transform> base_from_cloud = already_world ?
        std::optional<Transform>(Transform{}) :
        baseFromCloudFrame(cloud_frame, static_transforms, config, extrinsics);
      if (!base_from_cloud) {
        if (config.strict_frame_check) {
          throw std::runtime_error(
                  "cannot resolve static transform " + config.base_frame + " <- " + cloud_frame);
        }
        if (!warned_unresolved_cloud_frame) {
          dataset.warnings.push_back(
            "point cloud frame could not be resolved; configured parameter extrinsic was assumed");
          warned_unresolved_cloud_frame = true;
        }
      }
      const Transform fallback_base_from_cloud = base_from_cloud.value_or(extrinsics.base_from_sensor);
      PointCloudFieldReader fields(
        cloud, config.deskew.enabled ? config.deskew.point_time_field : std::string{});
      if (config.deskew.enabled && !fields.hasTime() && !warned_missing_time) {
        dataset.warnings.push_back(
          "deskew was enabled but the configured point-time field was absent; cloud-level poses were used");
        warned_missing_time = true;
      }

      const bool use_point_poses =
        !already_world && config.deskew.enabled && fields.hasTime();
      const std::optional<Transform> cloud_pose = already_world ?
        std::optional<Transform>(Transform{}) : poses.lookup(cloud_stamp, config.maximum_pose_gap_sec);
      if (!already_world && !use_point_poses && !cloud_pose) {
        ++dataset.statistics.dropped_pointcloud_messages;
        return false;
      }

      bool synchronized_point = !use_point_poses;
      fields.forEach(map_config.point_stride,
        [&](const double x, const double y, const double z, const double intensity,
          const std::optional<double> point_time) {
          ++dataset.statistics.decoded_points;
          if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
            !std::isfinite(intensity))
          {
            return;
          }
          const bool apply_sensor_range_filter =
            config.pointcloud_mode == "scan" && !already_world;
          const double range = std::sqrt(x * x + y * y + z * z);
          if ((apply_sensor_range_filter &&
            (range < map_config.minimum_range || range > map_config.maximum_range)) ||
            z < map_config.minimum_z || z > map_config.maximum_z)
          {
            return;
          }
          Transform world_from_cloud{};
          if (already_world) {
            world_from_cloud = Transform{};
          } else if (use_point_poses) {
            if (!point_time) {
              return;
            }
            const std::optional<std::int64_t> point_stamp = deskewPointTimestamp(
              cloud_stamp, *point_time, config.deskew);
            if (!point_stamp) {
              return;
            }
            const auto point_pose = poses.lookup(*point_stamp, config.maximum_pose_gap_sec);
            if (!point_pose) {
              return;
            }
            synchronized_point = true;
            world_from_cloud = *point_pose * fallback_base_from_cloud;
          } else {
            world_from_cloud = *cloud_pose * fallback_base_from_cloud;
          }
          const Vec3 world_point = world_from_cloud.apply({x, y, z});
          voxel_map.add(world_point, intensity, current_observation);
          ++dataset.statistics.accepted_points;
        });
      if (!synchronized_point) {
        ++dataset.statistics.dropped_pointcloud_messages;
        return false;
      }
      ++dataset.statistics.used_pointcloud_messages;
      return true;
    };

  {
    std::unique_ptr<rosbag2_cpp::Reader> reader = openReader(config);
    std::size_t selected_cloud_index = 0U;
    while (reader->has_next()) {
      const auto bag_message = reader->read_next();
      if (bag_message->topic_name != config.pointcloud_topic) {
        continue;
      }
      ++dataset.statistics.pointcloud_messages;
      const sensor_msgs::msg::PointCloud2 cloud =
        deserialize<sensor_msgs::msg::PointCloud2>(bag_message);
      const std::int64_t stamp = cloudTimestamp(cloud, *bag_message, config.use_header_stamp);
      if (config.pointcloud_mode == "accumulated_map") {
        last_accumulated_map = cloud;
        last_accumulated_stamp = stamp;
        continue;
      }
      if (selected_cloud_index++ % std::max<std::size_t>(1U, map_config.scan_stride) != 0U) {
        continue;
      }
      ++observation_id;
      process_cloud(cloud, stamp, observation_id);
    }
  }

  if (config.pointcloud_mode == "accumulated_map") {
    if (!last_accumulated_map) {
      throw std::runtime_error("accumulated_map mode found no point cloud message");
    }
    ++observation_id;
    process_cloud(*last_accumulated_map, last_accumulated_stamp, observation_id);
  }

  std::size_t minimum_observations = map_config.minimum_observations_per_voxel;
  if (config.pointcloud_mode == "accumulated_map" && minimum_observations > 1U) {
    dataset.warnings.push_back(
      "minimum_observations_per_voxel was reduced to 1 for accumulated_map input");
    minimum_observations = 1U;
  }
  dataset.map_points = voxel_map.points(minimum_observations);
  dataset.statistics.map_voxels = dataset.map_points.size();
  if (dataset.map_points.empty()) {
    throw std::runtime_error(
            "point cloud map is empty after synchronization, filtering, and voxelization");
  }
  if (dataset.statistics.dropped_pointcloud_messages > 0U) {
    dataset.warnings.push_back(
      std::to_string(dataset.statistics.dropped_pointcloud_messages) +
      " point cloud messages were dropped because no synchronized pose was available");
  }
  return dataset;
}

}  // namespace lidar_mobility_map_generator
