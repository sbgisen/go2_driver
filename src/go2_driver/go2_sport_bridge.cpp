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

#include "go2_driver/go2_sport_bridge.hpp"

#include <algorithm>
#include <chrono>
#include <thread>
#include <utility>

#include "go2_driver/sport_api_id.hpp"

namespace go2_driver
{

namespace
{

// Inclusive bounds accepted by the Sport API SpeedLevel command.
constexpr int32_t g_MIN_SPEED_LEVEL = -1;
constexpr int32_t g_MAX_SPEED_LEVEL = 1;

auto emptyJson() -> nlohmann::json { return nlohmann::json::object(); }

template <typename T>
auto dataJson(T value) -> nlohmann::json
{
  nlohmann::json js;
  js["data"] = value;
  return js;
}

auto availableModes(const std::unordered_map<std::string, std::vector<SportCommandStep>> & presets) -> std::string
{
  std::vector<std::string> names;
  names.reserve(presets.size());
  for (const auto & entry : presets) {
    names.push_back(entry.first);
  }
  std::sort(names.begin(), names.end());

  std::string joined;
  for (const auto & name : names) {
    if (!joined.empty()) {
      joined += ", ";
    }
    joined += name;
  }
  return joined;
}

auto moveJson(double x, double y, double z) -> nlohmann::json
{
  nlohmann::json js;
  js["x"] = x;
  js["y"] = y;
  js["z"] = z;
  return js;
}

auto step(SportApiId id, nlohmann::json parameter = emptyJson(), int wait_ms_after = 0) -> SportCommandStep
{
  return SportCommandStep{static_cast<int32_t>(id), std::move(parameter), wait_ms_after};
}

}  // namespace

Go2SportBridge::Go2SportBridge(const rclcpp::NodeOptions & options) : Node("go2_sport_bridge", options)
{
  initPresets();

  request_pub_ = create_publisher<unitree_api::msg::Request>("api/sport/request", 10);

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, [this](geometry_msgs::msg::Twist::SharedPtr msg) { cmdVelCallback(std::move(msg)); });

  mode_service_ = create_service<go2_interfaces::srv::Mode>(
    "mode", [this](
              std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<go2_interfaces::srv::Mode::Request> request,
              std::shared_ptr<go2_interfaces::srv::Mode::Response> response) {
      handleMode(std::move(header), std::move(request), std::move(response));
    });

  speed_level_service_ = create_service<go2_interfaces::srv::SpeedLevel>(
    "speed_level",
    [this](
      std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<go2_interfaces::srv::SpeedLevel::Request> request,
      std::shared_ptr<go2_interfaces::srv::SpeedLevel::Response> response) {
      handleSpeedLevel(std::move(header), std::move(request), std::move(response));
    });

  switch_joystick_service_ = create_service<go2_interfaces::srv::SwitchJoystick>(
    "switch_joystick",
    [this](
      std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<go2_interfaces::srv::SwitchJoystick::Request> request,
      std::shared_ptr<go2_interfaces::srv::SwitchJoystick::Response> response) {
      handleSwitchJoystick(std::move(header), std::move(request), std::move(response));
    });

  RCLCPP_INFO(get_logger(), "go2_sport_bridge started");
}

void Go2SportBridge::initPresets()
{
  presets_ = {
    {"damp", {step(SportApiId::DAMP)}},
    {"balance_stand", {step(SportApiId::BALANCE_STAND)}},
    {"stop_move", {step(SportApiId::STOP_MOVE)}},
    // {"stand_up", {step(SportApiId::STAND_UP)}},
    {"stand_down", {step(SportApiId::STAND_DOWN)}},
    {"recovery_stand", {step(SportApiId::RECOVERY_STAND)}},
    {"sit", {step(SportApiId::SIT)}},
    {"rise_sit", {step(SportApiId::RISE_SIT)}},
    {"hello", {step(SportApiId::HELLO)}},
    {"stretch", {step(SportApiId::STRETCH)}},
    {"dance1", {step(SportApiId::DANCE1)}},
    {"dance2", {step(SportApiId::DANCE2)}},
    {"finger_heart", {step(SportApiId::FINGER_HEART)}},

    {"static_walk", {step(SportApiId::STATIC_WALK)}},
    {"trot_run", {step(SportApiId::TROT_RUN)}},
    {"economic_gait", {step(SportApiId::ECONOMIC_GAIT)}},

    {"classic_walk", {step(SportApiId::CLASSIC_WALK, dataJson(true))}},
    {"walk_upright", {step(SportApiId::WALK_UPRIGHT, dataJson(true))}},
    {"cross_step", {step(SportApiId::CROSS_STEP, dataJson(true))}},

    {"free_walk", {step(SportApiId::FREE_WALK)}},
    {"free_bound", {step(SportApiId::FREE_BOUND, dataJson(true))}},
    {"free_jump", {step(SportApiId::FREE_JUMP, dataJson(true))}},
    {"free_avoid", {step(SportApiId::FREE_AVOID, dataJson(true))}},
    {"switch_avoid_mode", {step(SportApiId::SWITCH_AVOID_MODE)}},
    {"auto_recovery_set", {step(SportApiId::AUTO_RECOVERY_SET, dataJson(true))}},

    // WARNING: acrobatic moves. Run only with clear space and a safe surface.
    {"left_flip", {step(SportApiId::LEFT_FLIP)}},
    {"back_flip", {step(SportApiId::BACK_FLIP)}},
    {"hand_stand", {step(SportApiId::HAND_STAND, dataJson(true))}},

    {"stand_down_damp",
     {
       step(SportApiId::STAND_DOWN, emptyJson(), 2000),
       step(SportApiId::DAMP),
     }},
  };
}

auto Go2SportBridge::publishRequest(int32_t api_id, const nlohmann::json & parameter) -> std::string
{
  unitree_api::msg::Request req;
  req.header.identity.api_id = api_id;
  req.parameter = parameter.dump();

  request_pub_->publish(req);

  return "published api_id=" + std::to_string(api_id) + ", parameter=" + req.parameter;
}

auto Go2SportBridge::executeSequence(const std::vector<SportCommandStep> & steps, std::string & message) -> bool
{
  if (steps.empty()) {
    message = "sequence is empty";
    return false;
  }

  std::string last_message;

  for (const auto & s : steps) {
    last_message = publishRequest(s.api_id_, s.parameter_);

    if (s.wait_ms_after_ > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(s.wait_ms_after_));
    }
  }

  message = "sequence completed: " + last_message;
  return true;
}

void Go2SportBridge::cmdVelCallback(geometry_msgs::msg::Twist::SharedPtr msg)
{
  const auto js = moveJson(msg->linear.x, msg->linear.y, msg->angular.z);

  static_cast<void>(publishRequest(static_cast<int32_t>(SportApiId::MOVE), js));
}

void Go2SportBridge::handleMode(
  std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<go2_interfaces::srv::Mode::Request> request,
  std::shared_ptr<go2_interfaces::srv::Mode::Response> response)
{
  (void)header;

  const auto it = presets_.find(request->mode);
  if (it == presets_.end()) {
    response->success = false;
    response->message = "invalid mode: " + request->mode + ". available: " + availableModes(presets_);
    return;
  }

  response->success = executeSequence(it->second, response->message);
}

void Go2SportBridge::handleSpeedLevel(
  std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<go2_interfaces::srv::SpeedLevel::Request> request,
  std::shared_ptr<go2_interfaces::srv::SpeedLevel::Response> response)
{
  (void)header;

  if (request->level < g_MIN_SPEED_LEVEL || request->level > g_MAX_SPEED_LEVEL) {
    response->success = false;
    response->message =
      "level is out of range [" + std::to_string(g_MIN_SPEED_LEVEL) + ", " + std::to_string(g_MAX_SPEED_LEVEL) + "]";
    return;
  }

  response->message = publishRequest(static_cast<int32_t>(SportApiId::SPEED_LEVEL), dataJson(request->level));
  response->success = true;
}

void Go2SportBridge::handleSwitchJoystick(
  std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<go2_interfaces::srv::SwitchJoystick::Request> request,
  std::shared_ptr<go2_interfaces::srv::SwitchJoystick::Response> response)
{
  (void)header;

  // go2_interfaces/SwitchJoystick has no message field, so discard the description.
  static_cast<void>(publishRequest(static_cast<int32_t>(SportApiId::SWITCH_JOYSTICK), dataJson(request->flag)));
  response->success = true;
}

}  // namespace go2_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(go2_driver::Go2SportBridge)
