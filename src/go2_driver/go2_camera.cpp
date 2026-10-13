// Copyright (c) 2026 SoftBank Corp.
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

#include <cstddef>
#include <go2_driver/go2_camera.hpp>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "sensor_msgs/msg/camera_info.hpp"

namespace go2_driver
{

namespace
{

constexpr const char * g_CODEC = "h264";

// Raw 1280x720 bgr8 frames are 2.7 MB each, so keep the queue short.
constexpr std::size_t g_IMAGE_QUEUE_DEPTH = 5;

// H.264 NAL unit types (ITU-T H.264 table 7-1) the node has to tell apart.
constexpr uint8_t g_NAL_NON_IDR_SLICE = 1;
constexpr uint8_t g_NAL_IDR_SLICE = 5;
constexpr uint8_t g_NAL_SPS = 7;
constexpr uint8_t g_NAL_PPS = 8;

// The firmware interleaves several resolutions on one topic, so give it this long to
// deliver a matching frame before complaining that `resolution` matches nothing.
constexpr double g_RESOLUTION_GRACE_SEC = 2.0;

auto joinResolutions(const std::set<int> & resolutions) -> std::string
{
  std::string joined;
  for (const int resolution : resolutions) {
    if (!joined.empty()) {
      joined += ", ";
    }
    joined += std::to_string(resolution);
  }
  return joined;
}

struct NalUnit
{
  std::size_t offset_;  // of the Annex-B start code
  std::size_t length_;  // start code included
  uint8_t type_;
};

// Splits an Annex-B byte stream into its NAL units; each keeps its own start code.
auto splitNalUnits(const std::vector<uint8_t> & data) -> std::vector<NalUnit>
{
  std::vector<NalUnit> units;
  const std::size_t size = data.size();
  for (std::size_t i = 0; i + 3 < size; ++i) {
    if (data[i] != 0 || data[i + 1] != 0 || data[i + 2] != 1) {
      continue;
    }
    // A four-byte start code (00 00 00 01) begins one byte earlier.
    const std::size_t start = (i > 0 && data[i - 1] == 0) ? i - 1 : i;
    if (!units.empty()) {
      units.back().length_ = start - units.back().offset_;
    }
    units.push_back(NalUnit{start, 0, static_cast<uint8_t>(data[i + 3] & 0x1F)});
    i += 2;
  }
  if (!units.empty()) {
    units.back().length_ = size - units.back().offset_;
  }
  return units;
}

auto copyNalUnit(const std::vector<uint8_t> & data, const NalUnit & unit) -> std::vector<uint8_t>
{
  const auto begin = data.begin() + static_cast<std::ptrdiff_t>(unit.offset_);
  return std::vector<uint8_t>(begin, begin + static_cast<std::ptrdiff_t>(unit.length_));
}

}  // namespace

Go2Camera::Go2Camera(const rclcpp::NodeOptions & options) : Node("go2_camera", options)
{
  input_video_topic_ = declare_parameter<std::string>("input_video_topic", "/frontvideostream");
  resolution_ = declare_parameter<int>("resolution", 720);
  frame_id_ = declare_parameter<std::string>("frame_id", "go2_camera_optical_frame");
  decoder_name_ = declare_parameter<std::string>("decoder", "h264");
  const auto camera_name = declare_parameter<std::string>("camera_name", "go2_camera");
  const auto camera_info_url =
    declare_parameter<std::string>("camera_info_url", "package://go2_driver/config/go2_camera.yaml");

  camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name, camera_info_url);

  rmw_qos_profile_t qos = rmw_qos_profile_default;
  qos.depth = g_IMAGE_QUEUE_DEPTH;
  camera_pub_ = image_transport::create_camera_publisher(this, "go2_camera/image_raw", qos);

  decoder_.setLogger(get_logger());
  if (!initDecoder()) {
    RCLCPP_ERROR(
      get_logger(), "Decoder '%s' could not be initialised; no images will be published", decoder_name_.c_str());
    return;
  }

  // Best effort so the subscription matches whichever reliability the firmware publishes with.
  video_sub_ = create_subscription<unitree_go::msg::Go2FrontVideoData>(
    input_video_topic_, rclcpp::QoS(10).best_effort(),
    [this](unitree_go::msg::Go2FrontVideoData::SharedPtr msg) { videoCallback(std::move(msg)); });

  RCLCPP_INFO(get_logger(), "go2_camera started");
  RCLCPP_INFO(get_logger(), "input_video_topic: %s", input_video_topic_.c_str());
  RCLCPP_INFO(get_logger(), "resolution: %d", resolution_);
  RCLCPP_INFO(get_logger(), "frame_id: %s", frame_id_.c_str());
  RCLCPP_INFO(get_logger(), "decoder: %s", decoder_name_.c_str());
  RCLCPP_INFO(get_logger(), "camera_info: %s", camera_info_manager_->isCalibrated() ? "calibrated" : "uncalibrated");
}

auto Go2Camera::initDecoder() -> bool
{
  waiting_for_keyframe_ = true;
  return decoder_.initialize(
    g_CODEC,
    [this](const sensor_msgs::msg::Image::ConstSharedPtr & image, bool, const std::string &) { publishImage(image); },
    decoder_name_);
}

auto Go2Camera::videoCallback(unitree_go::msg::Go2FrontVideoData::SharedPtr msg) -> void
{
  if (decoder_failed_) {
    return;
  }
  if (first_frame_time_.nanoseconds() == 0) {
    first_frame_time_ = now();
  }
  if (msg->resolution != resolution_) {
    seen_resolutions_.insert(msg->resolution);
    if (!resolution_matched_ && (now() - first_frame_time_).seconds() > g_RESOLUTION_GRACE_SEC) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "No frames with resolution %d; the stream carries %s", resolution_,
        joinResolutions(seen_resolutions_).c_str());
    }
    return;
  }
  resolution_matched_ = true;
  if (camera_pub_.getNumSubscribers() == 0) {
    // Nobody is listening: skip the decode and resync on the next keyframe once someone connects.
    waiting_for_keyframe_ = true;
    return;
  }

  bool has_slice = false;
  bool has_idr = false;
  bool has_sps = false;
  bool has_pps = false;
  for (const auto & unit : splitNalUnits(msg->data)) {
    switch (unit.type_) {
      case g_NAL_SPS:
        sps_ = copyNalUnit(msg->data, unit);
        has_sps = true;
        break;
      case g_NAL_PPS:
        pps_ = copyNalUnit(msg->data, unit);
        has_pps = true;
        break;
      case g_NAL_IDR_SLICE:
        has_idr = true;
        has_slice = true;
        break;
      case g_NAL_NON_IDR_SLICE:
        has_slice = true;
        break;
      default:
        break;
    }
  }
  // Parameter sets without a slice are cached above; feeding them alone would leave
  // an orphan entry in the decoder's pts bookkeeping.
  if (!has_slice) {
    return;
  }
  if (waiting_for_keyframe_) {
    if (!has_idr || sps_.empty() || pps_.empty()) {
      return;
    }
    waiting_for_keyframe_ = false;
  }

  const std::vector<uint8_t> * packet = &msg->data;
  std::vector<uint8_t> prefixed;
  if (has_idr && (!has_sps || !has_pps)) {
    if (!has_sps) {
      prefixed.insert(prefixed.end(), sps_.begin(), sps_.end());
    }
    if (!has_pps) {
      prefixed.insert(prefixed.end(), pps_.begin(), pps_.end());
    }
    prefixed.insert(prefixed.end(), msg->data.begin(), msg->data.end());
    packet = &prefixed;
  }

  // The decoder hands the frame_id and stamp back through its pts map, so the pts
  // only has to be unique.
  if (!decoder_.decodePacket(g_CODEC, packet->data(), packet->size(), next_pts_++, frame_id_, now())) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Decoding failed; resetting the decoder and waiting for the next keyframe");
    decoder_.reset();
    if (!initDecoder()) {
      RCLCPP_ERROR(
        get_logger(), "Decoder '%s' could not be re-initialised; no more images will be published",
        decoder_name_.c_str());
      decoder_failed_ = true;
    }
  }
}

auto Go2Camera::publishImage(const sensor_msgs::msg::Image::ConstSharedPtr & image) -> void
{
  auto info = std::make_shared<sensor_msgs::msg::CameraInfo>(camera_info_manager_->getCameraInfo());
  info->header = image->header;
  if (info->width == 0 && info->height == 0) {
    // Uncalibrated: at least report the frame size so consumers can reject the zero intrinsics.
    info->width = image->width;
    info->height = image->height;
  }
  camera_pub_.publish(image, info);
}

}  // namespace go2_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(go2_driver::Go2Camera)
