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

#include <array>
#include <cstddef>
#include <functional>
#include <go2_driver/go2_driver.hpp>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace go2_driver
{

namespace
{

// Joint names in the order expected by the GO2 URDF: legs FL, FR, RL, RR, each
// with hip, thigh and calf joints.
const std::vector<std::string> g_JOINT_NAMES = {"FL_hip_joint",   "FL_thigh_joint", "FL_calf_joint",  "FR_hip_joint",
                                                "FR_thigh_joint", "FR_calf_joint",  "RL_hip_joint",   "RL_thigh_joint",
                                                "RL_calf_joint",  "RR_hip_joint",   "RR_thigh_joint", "RR_calf_joint"};

// Unitree orders its motor_state array as legs FR, FL, RR, RL. These indices map
// each entry of g_JOINT_NAMES (URDF order) to the matching motor_state index.
constexpr std::array<std::size_t, 12> g_MOTOR_STATE_INDEX = {3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8};

}  // namespace

Go2Driver::Go2Driver(const rclcpp::NodeOptions & options) : Node("go2_driver", options), tf_broadcaster_(this)
{
  input_odom_topic_ = declare_parameter<std::string>("input_odom_topic", "/utlidar/robot_odom");
  output_planar_odom_topic_ = declare_parameter<std::string>("output_planar_odom_topic", "/pochi/odom_planar");
  pointcloud_frame_ = declare_parameter<std::string>("pointcloud_frame", "radar");

  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  base_footprint_frame_ = declare_parameter<std::string>("base_footprint_frame", "base_footprint");
  base_link_frame_ = declare_parameter<std::string>("base_link_frame", "base_link");

  body_z_offset_ = declare_parameter<double>("body_z_offset", 0.0);
  use_msg_stamp_ = declare_parameter<bool>("use_msg_stamp", false);
  publish_tf_ = declare_parameter<bool>("publish_tf", true);
  publish_planar_odom_ = declare_parameter<bool>("publish_planar_odom", true);

  pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);
  joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  planar_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_planar_odom_topic_, rclcpp::QoS(20));

  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/utlidar/cloud", 10,
    std::bind(&Go2Driver::publishLidar, this, std::placeholders::_1));  // NOLINT(modernize-avoid-bind)

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    input_odom_topic_, rclcpp::QoS(50).best_effort(),
    std::bind(&Go2Driver::odomCallback, this, std::placeholders::_1));  // NOLINT(modernize-avoid-bind)

  low_state_sub_ = create_subscription<unitree_go::msg::LowState>(
    "lowstate", 10,
    std::bind(&Go2Driver::publishJointStates, this, std::placeholders::_1));  // NOLINT(modernize-avoid-bind)

  RCLCPP_INFO(get_logger(), "go2_driver state bridge started");
  RCLCPP_INFO(get_logger(), "input_odom_topic: %s", input_odom_topic_.c_str());
  RCLCPP_INFO(get_logger(), "output_planar_odom_topic: %s", output_planar_odom_topic_.c_str());
  RCLCPP_INFO(get_logger(), "pointcloud_frame: %s", pointcloud_frame_.c_str());
  RCLCPP_INFO(get_logger(), "odom_frame: %s", odom_frame_.c_str());
  RCLCPP_INFO(get_logger(), "base_footprint_frame: %s", base_footprint_frame_.c_str());
  RCLCPP_INFO(get_logger(), "base_link_frame: %s", base_link_frame_.c_str());
  RCLCPP_INFO(get_logger(), "body_z_offset: %.3f", body_z_offset_);
}

void Go2Driver::publishLidar(sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  msg->header.stamp = now();
  msg->header.frame_id = pointcloud_frame_;
  pointcloud_pub_->publish(*msg);
}

void Go2Driver::odomCallback(nav_msgs::msg::Odometry::SharedPtr msg)
{
  builtin_interfaces::msg::Time stamp;
  if (use_msg_stamp_) {
    stamp = msg->header.stamp;
  } else {
    stamp = get_clock()->now();
  }

  const auto & p = msg->pose.pose.position;
  const auto & q_msg = msg->pose.pose.orientation;

  tf2::Quaternion q_in;
  tf2::fromMsg(q_msg, q_in);

  double roll;
  double pitch;
  double yaw;
  tf2::Matrix3x3(q_in).getRPY(roll, pitch, yaw);

  tf2::Quaternion q_yaw;
  q_yaw.setRPY(0.0, 0.0, yaw);
  q_yaw.normalize();

  tf2::Quaternion q_rp;
  q_rp.setRPY(roll, pitch, 0.0);
  q_rp.normalize();

  const double body_z = p.z + body_z_offset_;

  if (publish_tf_) {
    geometry_msgs::msg::TransformStamped tf_odom_to_footprint;
    tf_odom_to_footprint.header.stamp = stamp;
    tf_odom_to_footprint.header.frame_id = odom_frame_;
    tf_odom_to_footprint.child_frame_id = base_footprint_frame_;
    tf_odom_to_footprint.transform.translation.x = p.x;
    tf_odom_to_footprint.transform.translation.y = p.y;
    tf_odom_to_footprint.transform.translation.z = 0.0;
    tf_odom_to_footprint.transform.rotation = tf2::toMsg(q_yaw);

    geometry_msgs::msg::TransformStamped tf_footprint_to_link;
    tf_footprint_to_link.header.stamp = stamp;
    tf_footprint_to_link.header.frame_id = base_footprint_frame_;
    tf_footprint_to_link.child_frame_id = base_link_frame_;
    tf_footprint_to_link.transform.translation.x = 0.0;
    tf_footprint_to_link.transform.translation.y = 0.0;
    tf_footprint_to_link.transform.translation.z = body_z;
    tf_footprint_to_link.transform.rotation = tf2::toMsg(q_rp);

    tf_broadcaster_.sendTransform(tf_odom_to_footprint);
    tf_broadcaster_.sendTransform(tf_footprint_to_link);
  }

  if (publish_planar_odom_) {
    nav_msgs::msg::Odometry planar;
    planar.header.stamp = stamp;
    planar.header.frame_id = odom_frame_;
    planar.child_frame_id = base_footprint_frame_;

    planar.pose.pose.position.x = p.x;
    planar.pose.pose.position.y = p.y;
    planar.pose.pose.position.z = 0.0;
    planar.pose.pose.orientation = tf2::toMsg(q_yaw);
    planar.pose.covariance = msg->pose.covariance;

    planar.twist.twist.linear.x = msg->twist.twist.linear.x;
    planar.twist.twist.linear.y = msg->twist.twist.linear.y;
    planar.twist.twist.linear.z = 0.0;
    planar.twist.twist.angular.x = 0.0;
    planar.twist.twist.angular.y = 0.0;
    planar.twist.twist.angular.z = msg->twist.twist.angular.z;
    planar.twist.covariance = msg->twist.covariance;

    planar_odom_pub_->publish(planar);
  }
}

void Go2Driver::publishJointStates(unitree_go::msg::LowState::SharedPtr msg)
{
  sensor_msgs::msg::JointState joint_state;
  joint_state.header.stamp = now();
  joint_state.name = g_JOINT_NAMES;

  joint_state.position.reserve(g_MOTOR_STATE_INDEX.size());
  for (const auto index : g_MOTOR_STATE_INDEX) {
    joint_state.position.push_back(msg->motor_state[index].q);
  }

  joint_state_pub_->publish(joint_state);
}

}  // namespace go2_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(go2_driver::Go2Driver)
