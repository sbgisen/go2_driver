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

#ifndef GO2_DRIVER__GO2_CAMERA_HPP_
#define GO2_DRIVER__GO2_CAMERA_HPP_

#include <camera_info_manager/camera_info_manager.hpp>
#include <cstdint>
#include <ffmpeg_encoder_decoder/decoder.hpp>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <unitree_go/msg/go2_front_video_data.hpp>
#include <vector>

namespace go2_driver
{

// Decodes the H.264 stream of the Go2 front camera into sensor_msgs/Image.
class Go2Camera : public rclcpp::Node
{
public:
  explicit Go2Camera(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  auto videoCallback(unitree_go::msg::Go2FrontVideoData::SharedPtr msg) -> void;
  auto publishImage(const sensor_msgs::msg::Image::ConstSharedPtr & image) -> void;

  // (Re)creates the libav decoder and puts the stream back into keyframe-wait.
  auto initDecoder() -> bool;

  rclcpp::Subscription<unitree_go::msg::Go2FrontVideoData>::SharedPtr video_sub_;
  image_transport::CameraPublisher camera_pub_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  ffmpeg_encoder_decoder::Decoder decoder_;

  // Most recent SPS / PPS NAL units, start code included. Repeated in front of
  // every IDR slice so decoding can start (or restart) at any keyframe.
  std::vector<uint8_t> sps_;
  std::vector<uint8_t> pps_;

  // Set until a decodable IDR slice arrives; non-IDR slices are dropped meanwhile.
  bool waiting_for_keyframe_{true};
  bool decoder_failed_{false};
  uint64_t next_pts_{0};

  std::string input_video_topic_;
  std::string frame_id_;
  std::string decoder_name_;
  int resolution_;
};

}  // namespace go2_driver

#endif  // GO2_DRIVER__GO2_CAMERA_HPP_
