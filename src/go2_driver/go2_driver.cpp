// BSD 3-Clause License

// Copyright (c) 2024, Intelligent Robotics Lab
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.

// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.

// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <functional>
#include <go2_driver/go2_driver.hpp>

namespace go2_driver
{

Go2Driver::Go2Driver(const rclcpp::NodeOptions & options)
: Node("go2_driver", options), tf_broadcaster_(this)
{
  rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);
  joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", qos_profile);

  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/utlidar/cloud", 10,
    std::bind(&Go2Driver::publishLidar, this, std::placeholders::_1));  // NOLINT(modernize-avoid-bind)

  robot_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/utlidar/robot_pose", 10,
    std::bind(&Go2Driver::publishPoseStamped, this, std::placeholders::_1));  // NOLINT(modernize-avoid-bind)

  low_state_sub_ = create_subscription<unitree_go::msg::LowState>(
    "lowstate", 10,
    std::bind(&Go2Driver::publishJointStates, this, std::placeholders::_1));  // NOLINT(modernize-avoid-bind)
}

void Go2Driver::publishLidar(sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  msg->header.stamp = now();
  msg->header.frame_id = "radar";
  pointcloud_pub_->publish(*msg);
}

void Go2Driver::publishPoseStamped(geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = now();
  transform.header.frame_id = "odom";
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = msg->pose.position.x;
  transform.transform.translation.y = msg->pose.position.y;
  transform.transform.translation.z = msg->pose.position.z + 0.07;
  transform.transform.rotation.x = msg->pose.orientation.x;
  transform.transform.rotation.y = msg->pose.orientation.y;
  transform.transform.rotation.z = msg->pose.orientation.z;
  transform.transform.rotation.w = msg->pose.orientation.w;
  tf_broadcaster_.sendTransform(transform);

  if (!odom_published_) {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = msg->pose.position.x;
    odom.pose.pose.position.y = msg->pose.position.y;
    odom.pose.pose.position.z = msg->pose.position.z + 0.07;
    odom.pose.pose.orientation.x = msg->pose.orientation.x;
    odom.pose.pose.orientation.y = msg->pose.orientation.y;
    odom.pose.pose.orientation.z = msg->pose.orientation.z;
    odom.pose.pose.orientation.w = msg->pose.orientation.w;
    odom_pub_->publish(odom);
    odom_published_ = true;
  }
}

void Go2Driver::publishJointStates(unitree_go::msg::LowState::SharedPtr msg)
{
  sensor_msgs::msg::JointState joint_state;
  joint_state.header.stamp = now();
  joint_state.name = {"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", "FR_hip_joint",
    "FR_thigh_joint", "FR_calf_joint", "RL_hip_joint", "RL_thigh_joint",
    "RL_calf_joint", "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};

  joint_state.position = {msg->motor_state[3].q, msg->motor_state[4].q, msg->motor_state[5].q,
    msg->motor_state[0].q,
    msg->motor_state[1].q, msg->motor_state[2].q, msg->motor_state[9].q, msg->motor_state[10].q,
    msg->motor_state[11].q, msg->motor_state[6].q, msg->motor_state[7].q, msg->motor_state[8].q};

  joint_state_pub_->publish(joint_state);
}

}  // namespace go2_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(go2_driver::Go2Driver)
