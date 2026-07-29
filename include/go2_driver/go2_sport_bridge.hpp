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
#include <go2_driver/unitree_api_client.hpp>
#include <go2_interfaces/srv/euler.hpp>
#include <go2_interfaces/srv/get_auto_recovery.hpp>
#include <go2_interfaces/srv/mode.hpp>
#include <go2_interfaces/srv/pose.hpp>
#include <go2_interfaces/srv/speed_level.hpp>
#include <go2_interfaces/srv/switch_joystick.hpp>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
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
  // A mode preset in the middle of being executed, one step per robot reply.
  struct SequenceRun
  {
    std::string name_;
    std::vector<SportCommandStep> steps_;
    std::size_t index_;
    rmw_request_id_t request_id_;
    std::string last_message_;
    bool ok_;
  };

  auto initPresets() -> void;

  // Sends one Sport API request. Invokes on_result once, with the robot's
  // reply, or immediately when wait_for_response is disabled.
  auto sendRequest(int32_t api_id, const nlohmann::json & parameter, ApiResponseCallback on_result) -> bool;

  auto startSequence(
    const rmw_request_id_t & request_id, const std::string & name, const std::vector<SportCommandStep> & steps) -> void;

  auto advanceSequence() -> void;

  auto onStepResult(const ApiResult & result) -> void;

  auto finishSequence() -> void;

  auto cmdVelCallback(geometry_msgs::msg::Twist::SharedPtr msg) -> void;

  auto handleMode(std::shared_ptr<rmw_request_id_t> header, go2_interfaces::srv::Mode::Request::SharedPtr request)
    -> void;

  auto handleSpeedLevel(
    std::shared_ptr<rmw_request_id_t> header, go2_interfaces::srv::SpeedLevel::Request::SharedPtr request) -> void;

  auto handleSwitchJoystick(
    std::shared_ptr<rmw_request_id_t> header, go2_interfaces::srv::SwitchJoystick::Request::SharedPtr request) -> void;

  auto handleEuler(std::shared_ptr<rmw_request_id_t> header, go2_interfaces::srv::Euler::Request::SharedPtr request)
    -> void;

  auto handlePose(std::shared_ptr<rmw_request_id_t> header, go2_interfaces::srv::Pose::Request::SharedPtr request)
    -> void;

  auto handleGetAutoRecovery(
    std::shared_ptr<rmw_request_id_t> header, go2_interfaces::srv::GetAutoRecovery::Request::SharedPtr request) -> void;

  std::unique_ptr<UnitreeApiClient> api_client_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  rclcpp::Service<go2_interfaces::srv::Mode>::SharedPtr mode_service_;
  rclcpp::Service<go2_interfaces::srv::SpeedLevel>::SharedPtr speed_level_service_;
  rclcpp::Service<go2_interfaces::srv::SwitchJoystick>::SharedPtr switch_joystick_service_;
  rclcpp::Service<go2_interfaces::srv::Euler>::SharedPtr euler_service_;
  rclcpp::Service<go2_interfaces::srv::Pose>::SharedPtr pose_service_;
  rclcpp::Service<go2_interfaces::srv::GetAutoRecovery>::SharedPtr get_auto_recovery_service_;

  rclcpp::TimerBase::SharedPtr sequence_timer_;
  std::optional<SequenceRun> active_sequence_;

  bool wait_for_response_{true};

  std::unordered_map<std::string, std::vector<SportCommandStep>> presets_;
};

}  // namespace go2_driver

#endif  // GO2_DRIVER__GO2_SPORT_BRIDGE_HPP_
