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
//
// Original work: Copyright (c) 2024 Intelligent Robotics Lab (URJC).

#ifndef GO2_DRIVER__GO2_DRIVER_HPP_
#define GO2_DRIVER__GO2_DRIVER_HPP_

#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

#include "tf2_ros/transform_broadcaster.hpp"
#include "unitree_go/msg/low_state.hpp"

namespace go2_driver
{

class Go2Driver : public rclcpp::Node
{
public:
  Go2Driver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void publishLidar(sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void odomCallback(nav_msgs::msg::Odometry::SharedPtr msg);
  void publishJointStates(unitree_go::msg::LowState::SharedPtr msg);

  // Returns msg_stamp when use_msg_stamp_ is set, otherwise the current node clock.
  auto resolveStamp(const builtin_interfaces::msg::Time & msg_stamp) const -> builtin_interfaces::msg::Time;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr planar_odom_pub_;

  tf2_ros::TransformBroadcaster tf_broadcaster_;

  std::string input_odom_topic_;
  std::string output_planar_odom_topic_;
  std::string pointcloud_frame_;
  std::string odom_frame_;
  std::string base_footprint_frame_;
  std::string base_link_frame_;

  double body_z_offset_;
  bool use_msg_stamp_;
  bool publish_tf_;
  bool publish_planar_odom_;
};

}  // namespace go2_driver

#endif  // GO2_DRIVER__GO2_DRIVER_HPP_
