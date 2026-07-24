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

#ifndef GO2_DRIVER__GO2_SPORT_BRIDGE_HPP_
#define GO2_DRIVER__GO2_SPORT_BRIDGE_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <go2_interfaces/srv/mode.hpp>
#include <go2_interfaces/srv/speed_level.hpp>
#include <go2_interfaces/srv/switch_joystick.hpp>
#include <memory>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unitree_api/msg/request.hpp>
#include <unordered_map>
#include <vector>

namespace go2_driver
{

struct SportCommandStep
{
  int32_t api_id_;
  nlohmann::json parameter_;
  int wait_ms_after_;
};

class Go2SportBridge : public rclcpp::Node
{
public:
  explicit Go2SportBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  auto initPresets() -> void;

  auto publishRequest(int32_t api_id, const nlohmann::json & parameter, std::string & message) -> bool;

  auto executeSequence(const std::vector<SportCommandStep> & steps, std::string & message) -> bool;

  auto cmdVelCallback(geometry_msgs::msg::Twist::SharedPtr msg) -> void;

  auto handleMode(
    std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<go2_interfaces::srv::Mode::Request> request,
    std::shared_ptr<go2_interfaces::srv::Mode::Response> response) -> void;

  auto handleSpeedLevel(
    std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<go2_interfaces::srv::SpeedLevel::Request> request,
    std::shared_ptr<go2_interfaces::srv::SpeedLevel::Response> response) -> void;

  auto handleSwitchJoystick(
    std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<go2_interfaces::srv::SwitchJoystick::Request> request,
    std::shared_ptr<go2_interfaces::srv::SwitchJoystick::Response> response) -> void;

  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr request_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  rclcpp::Service<go2_interfaces::srv::Mode>::SharedPtr mode_service_;
  rclcpp::Service<go2_interfaces::srv::SpeedLevel>::SharedPtr speed_level_service_;
  rclcpp::Service<go2_interfaces::srv::SwitchJoystick>::SharedPtr switch_joystick_service_;

  std::unordered_map<std::string, std::vector<SportCommandStep>> presets_;
};

}  // namespace go2_driver

#endif  // GO2_DRIVER__GO2_SPORT_BRIDGE_HPP_
