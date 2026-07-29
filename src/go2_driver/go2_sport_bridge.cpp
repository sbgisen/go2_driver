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
#include <functional>
#include <utility>

#include "go2_driver/sport_api_id.hpp"

namespace go2_driver
{

namespace
{

// The Sport API SpeedLevel command takes -1 or 1, not the whole [-1, 1] range:
// the firmware answers 0 with status.code = -1 (measured on the robot, 6/6).
// Neither unitree_sdk2 nor unitree_ros2 documents or range-checks this.
constexpr int32_t g_SPEED_LEVEL_LOW = -1;
constexpr int32_t g_SPEED_LEVEL_HIGH = 1;

// Seconds the robot is given to acknowledge a Sport API request.
constexpr double g_DEFAULT_RESPONSE_TIMEOUT = 2.0;

constexpr int g_QOS_DEPTH = 10;

// rclcpp picks the deferred service callback by exact signature: it takes no
// response argument, and that is what stops rclcpp from answering the caller
// as soon as the handler returns.
using ModeCallback =
  std::function<void(std::shared_ptr<rmw_request_id_t>, go2_interfaces::srv::Mode::Request::SharedPtr)>;
using SpeedLevelCallback =
  std::function<void(std::shared_ptr<rmw_request_id_t>, go2_interfaces::srv::SpeedLevel::Request::SharedPtr)>;
using SwitchJoystickCallback =
  std::function<void(std::shared_ptr<rmw_request_id_t>, go2_interfaces::srv::SwitchJoystick::Request::SharedPtr)>;
using EulerCallback =
  std::function<void(std::shared_ptr<rmw_request_id_t>, go2_interfaces::srv::Euler::Request::SharedPtr)>;
using PoseCallback =
  std::function<void(std::shared_ptr<rmw_request_id_t>, go2_interfaces::srv::Pose::Request::SharedPtr)>;
using GetAutoRecoveryCallback =
  std::function<void(std::shared_ptr<rmw_request_id_t>, go2_interfaces::srv::GetAutoRecovery::Request::SharedPtr)>;

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

// Both Move and Euler take their three values under x / y / z.
auto xyzJson(double x, double y, double z) -> nlohmann::json
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

auto describe(int32_t api_id, const ApiResult & result, bool waited) -> std::string
{
  const auto prefix = "api_id=" + std::to_string(api_id);
  if (!waited) {
    // Nothing was checked, so the wording must not imply the robot agreed.
    return prefix + " published";
  }
  if (result.ok_) {
    return prefix + " accepted";
  }
  if (result.status_code_ < 0) {
    return prefix + " got no reply";
  }
  return prefix + " rejected with status " + std::to_string(result.status_code_);
}

}  // namespace

Go2SportBridge::Go2SportBridge(const rclcpp::NodeOptions & options) : Node("go2_sport_bridge", options)
{
  initPresets();

  wait_for_response_ = declare_parameter<bool>("wait_for_response", true);
  const auto timeout = declare_parameter<double>("response_timeout", g_DEFAULT_RESPONSE_TIMEOUT);

  api_client_ = std::make_unique<UnitreeApiClient>(
    this, "api/sport/request", "api/sport/response", std::chrono::milliseconds(static_cast<int64_t>(timeout * 1000.0)));

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", g_QOS_DEPTH, [this](geometry_msgs::msg::Twist::SharedPtr msg) { cmdVelCallback(std::move(msg)); });

  mode_service_ = create_service<go2_interfaces::srv::Mode>(
    "mode",
    ModeCallback([this](std::shared_ptr<rmw_request_id_t> header, go2_interfaces::srv::Mode::Request::SharedPtr req) {
      handleMode(std::move(header), std::move(req));
    }));

  speed_level_service_ = create_service<go2_interfaces::srv::SpeedLevel>(
    "speed_level",
    SpeedLevelCallback(
      [this](std::shared_ptr<rmw_request_id_t> header, go2_interfaces::srv::SpeedLevel::Request::SharedPtr req) {
        handleSpeedLevel(std::move(header), std::move(req));
      }));

  switch_joystick_service_ = create_service<go2_interfaces::srv::SwitchJoystick>(
    "switch_joystick",
    SwitchJoystickCallback(
      [this](std::shared_ptr<rmw_request_id_t> header, go2_interfaces::srv::SwitchJoystick::Request::SharedPtr req) {
        handleSwitchJoystick(std::move(header), std::move(req));
      }));

  euler_service_ = create_service<go2_interfaces::srv::Euler>(
    "euler",
    EulerCallback([this](std::shared_ptr<rmw_request_id_t> header, go2_interfaces::srv::Euler::Request::SharedPtr req) {
      handleEuler(std::move(header), std::move(req));
    }));

  pose_service_ = create_service<go2_interfaces::srv::Pose>(
    "pose",
    PoseCallback([this](std::shared_ptr<rmw_request_id_t> header, go2_interfaces::srv::Pose::Request::SharedPtr req) {
      handlePose(std::move(header), std::move(req));
    }));

  get_auto_recovery_service_ = create_service<go2_interfaces::srv::GetAutoRecovery>(
    "get_auto_recovery",
    GetAutoRecoveryCallback(
      [this](std::shared_ptr<rmw_request_id_t> header, go2_interfaces::srv::GetAutoRecovery::Request::SharedPtr req) {
        handleGetAutoRecovery(std::move(header), std::move(req));
      }));

  RCLCPP_INFO(get_logger(), "go2_sport_bridge started");
  RCLCPP_INFO(
    get_logger(), "wait_for_response: %s",
    wait_for_response_ ? "true (services report the status code the robot replied with)"
                       : "false (services report only that the request was published)");
}

void Go2SportBridge::initPresets()
{
  presets_ = {
    // Basic postures and one-shot gestures.
    {"damp", {step(SportApiId::DAMP)}},
    {"balance_stand", {step(SportApiId::BALANCE_STAND)}},
    {"stop_move", {step(SportApiId::STOP_MOVE)}},
    // stand_up is intentionally not exposed; use recovery_stand / balance_stand instead.
    // {"stand_up", {step(SportApiId::STAND_UP)}},
    {"stand_down", {step(SportApiId::STAND_DOWN)}},
    {"recovery_stand", {step(SportApiId::RECOVERY_STAND)}},
    {"sit", {step(SportApiId::SIT)}},
    {"rise_sit", {step(SportApiId::RISE_SIT)}},
    {"hello", {step(SportApiId::HELLO)}},
    {"stretch", {step(SportApiId::STRETCH)}},
    {"content", {step(SportApiId::CONTENT)}},
    {"scrape", {step(SportApiId::SCRAPE)}},
    {"dance1", {step(SportApiId::DANCE1)}},
    {"dance2", {step(SportApiId::DANCE2)}},
    {"finger_heart", {step(SportApiId::FINGER_HEART)}},

    // Locomotion gaits.
    {"static_walk", {step(SportApiId::STATIC_WALK)}},
    {"trot_run", {step(SportApiId::TROT_RUN)}},
    {"economic_gait", {step(SportApiId::ECONOMIC_GAIT)}},

    // Advanced gaits (enabled with a data flag).
    {"classic_walk", {step(SportApiId::CLASSIC_WALK, dataJson(true))}},
    {"walk_upright", {step(SportApiId::WALK_UPRIGHT, dataJson(true))}},
    {"cross_step", {step(SportApiId::CROSS_STEP, dataJson(true))}},

    // Free / autonomous modes.
    {"free_walk", {step(SportApiId::FREE_WALK)}},
    {"free_bound", {step(SportApiId::FREE_BOUND, dataJson(true))}},
    {"free_jump", {step(SportApiId::FREE_JUMP, dataJson(true))}},
    {"free_avoid", {step(SportApiId::FREE_AVOID, dataJson(true))}},
    {"switch_avoid_mode", {step(SportApiId::SWITCH_AVOID_MODE)}},
    {"auto_recovery_set", {step(SportApiId::AUTO_RECOVERY_SET, dataJson(true))}},

    // WARNING: acrobatic moves. Run only with clear space and a safe surface.
    {"front_flip", {step(SportApiId::FRONT_FLIP)}},
    {"front_jump", {step(SportApiId::FRONT_JUMP)}},
    {"front_pounce", {step(SportApiId::FRONT_POUNCE)}},
    {"left_flip", {step(SportApiId::LEFT_FLIP)}},
    {"back_flip", {step(SportApiId::BACK_FLIP)}},
    {"hand_stand", {step(SportApiId::HAND_STAND, dataJson(true))}},

    // Composite sequences (multiple steps with inter-step delays).
    {"stand_down_damp",
     {
       step(SportApiId::STAND_DOWN, emptyJson(), 2000),
       step(SportApiId::DAMP),
     }},
  };
}

auto Go2SportBridge::sendRequest(int32_t api_id, const nlohmann::json & parameter, ApiResponseCallback on_result)
  -> bool
{
  if (wait_for_response_) {
    return api_client_->call(api_id, parameter, std::move(on_result));
  }

  api_client_->send(api_id, parameter);
  on_result(ApiResult{true, 0, ""});
  return true;
}

void Go2SportBridge::startSequence(
  const rmw_request_id_t & request_id, const std::string & name, const std::vector<SportCommandStep> & steps)
{
  active_sequence_ = SequenceRun{name, steps, 0, request_id, "", true};
  advanceSequence();
}

void Go2SportBridge::advanceSequence()
{
  auto & run = *active_sequence_;

  if (run.index_ >= run.steps_.size()) {
    finishSequence();
    return;
  }

  const auto api_id = run.steps_[run.index_].api_id_;
  const auto sent =
    sendRequest(api_id, run.steps_[run.index_].parameter_, [this](const ApiResult & result) { onStepResult(result); });

  if (!sent) {
    run.ok_ = false;
    run.last_message_ = "api_id=" + std::to_string(api_id) + " could not be sent";
    finishSequence();
  }
}

void Go2SportBridge::onStepResult(const ApiResult & result)
{
  auto & run = *active_sequence_;

  const auto wait = std::chrono::milliseconds(run.steps_[run.index_].wait_ms_after_);
  run.ok_ = run.ok_ && result.ok_;
  run.last_message_ = describe(run.steps_[run.index_].api_id_, result, wait_for_response_);
  ++run.index_;

  // Even a zero wait goes back through the timer rather than recursing, so the
  // executor stays free between the steps of a sequence.
  sequence_timer_ = create_wall_timer(wait, [this] {
    sequence_timer_->cancel();
    advanceSequence();
  });
}

void Go2SportBridge::finishSequence()
{
  auto run = std::move(*active_sequence_);
  active_sequence_.reset();
  sequence_timer_.reset();

  go2_interfaces::srv::Mode::Response response;
  response.success = run.ok_;
  response.message = "mode " + run.name_ + ": " + run.last_message_;

  mode_service_->send_response(run.request_id_, response);
}

void Go2SportBridge::cmdVelCallback(geometry_msgs::msg::Twist::SharedPtr msg)
{
  // Velocity commands arrive far more often than the robot answers them, so
  // they are never correlated.
  api_client_->send(static_cast<int32_t>(SportApiId::MOVE), xyzJson(msg->linear.x, msg->linear.y, msg->angular.z));
}

void Go2SportBridge::handleMode(
  std::shared_ptr<rmw_request_id_t> header, go2_interfaces::srv::Mode::Request::SharedPtr request)
{
  go2_interfaces::srv::Mode::Response response;
  response.success = false;

  const auto it = presets_.find(request->mode);
  if (it == presets_.end()) {
    response.message = "invalid mode: " + request->mode + ". available: " + availableModes(presets_);
    mode_service_->send_response(*header, response);
    return;
  }

  if (active_sequence_.has_value()) {
    // Only reachable now that a sequence no longer blocks the executor.
    response.message = "another mode sequence is in progress: " + active_sequence_->name_;
    mode_service_->send_response(*header, response);
    return;
  }

  startSequence(*header, request->mode, it->second);
}

void Go2SportBridge::handleSpeedLevel(
  std::shared_ptr<rmw_request_id_t> header, go2_interfaces::srv::SpeedLevel::Request::SharedPtr request)
{
  // send_response takes the id by non-const reference, so every copy that
  // reaches it is a mutable one.
  auto request_id = *header;
  const auto api_id = static_cast<int32_t>(SportApiId::SPEED_LEVEL);

  // The two-value check comes from this branch's base; only the way the answer
  // is sent changes here.
  if (request->level != g_SPEED_LEVEL_LOW && request->level != g_SPEED_LEVEL_HIGH) {
    go2_interfaces::srv::SpeedLevel::Response response;
    response.success = false;
    response.message = "level must be " + std::to_string(g_SPEED_LEVEL_LOW) + " or " +
                       std::to_string(g_SPEED_LEVEL_HIGH) + ", got " + std::to_string(request->level);
    speed_level_service_->send_response(request_id, response);
    return;
  }

  const auto sent = sendRequest(api_id, dataJson(request->level), [this, request_id](const ApiResult & result) mutable {
    go2_interfaces::srv::SpeedLevel::Response response;
    response.success = result.ok_;
    response.message = describe(api_id, result, wait_for_response_);
    speed_level_service_->send_response(request_id, response);
  });

  if (!sent) {
    go2_interfaces::srv::SpeedLevel::Response response;
    response.success = false;
    response.message = "api_id=" + std::to_string(api_id) + " could not be sent";
    speed_level_service_->send_response(request_id, response);
  }
}

void Go2SportBridge::handleSwitchJoystick(
  std::shared_ptr<rmw_request_id_t> header, go2_interfaces::srv::SwitchJoystick::Request::SharedPtr request)
{
  auto request_id = *header;
  const auto api_id = static_cast<int32_t>(SportApiId::SWITCH_JOYSTICK);

  const auto sent = sendRequest(api_id, dataJson(request->flag), [this, request_id](const ApiResult & result) mutable {
    if (!result.ok_) {
      // go2_interfaces/SwitchJoystick carries no message field, so the reason
      // only reaches the log.
      RCLCPP_WARN(get_logger(), "switch_joystick failed: %s", describe(api_id, result, wait_for_response_).c_str());
    }

    go2_interfaces::srv::SwitchJoystick::Response response;
    response.success = result.ok_;
    switch_joystick_service_->send_response(request_id, response);
  });

  if (!sent) {
    go2_interfaces::srv::SwitchJoystick::Response response;
    response.success = false;
    switch_joystick_service_->send_response(request_id, response);
  }
}

void Go2SportBridge::handleEuler(
  std::shared_ptr<rmw_request_id_t> header, go2_interfaces::srv::Euler::Request::SharedPtr request)
{
  auto request_id = *header;
  const auto api_id = static_cast<int32_t>(SportApiId::EULER);

  const auto sent = sendRequest(
    api_id, xyzJson(request->roll, request->pitch, request->yaw), [this, request_id](const ApiResult & result) mutable {
      go2_interfaces::srv::Euler::Response response;
      response.success = result.ok_;
      response.message = describe(api_id, result, wait_for_response_);
      euler_service_->send_response(request_id, response);
    });

  if (!sent) {
    go2_interfaces::srv::Euler::Response response;
    response.success = false;
    response.message = "api_id=" + std::to_string(api_id) + " could not be sent";
    euler_service_->send_response(request_id, response);
  }
}

void Go2SportBridge::handlePose(
  std::shared_ptr<rmw_request_id_t> header, go2_interfaces::srv::Pose::Request::SharedPtr request)
{
  auto request_id = *header;
  const auto api_id = static_cast<int32_t>(SportApiId::POSE);

  const auto sent = sendRequest(api_id, dataJson(request->flag), [this, request_id](const ApiResult & result) mutable {
    if (!result.ok_) {
      // go2_interfaces/Pose carries no message field, so the reason only
      // reaches the log.
      RCLCPP_WARN(get_logger(), "pose failed: %s", describe(api_id, result, wait_for_response_).c_str());
    }

    go2_interfaces::srv::Pose::Response response;
    response.success = result.ok_;
    pose_service_->send_response(request_id, response);
  });

  if (!sent) {
    go2_interfaces::srv::Pose::Response response;
    response.success = false;
    pose_service_->send_response(request_id, response);
  }
}

void Go2SportBridge::handleGetAutoRecovery(
  std::shared_ptr<rmw_request_id_t> header, go2_interfaces::srv::GetAutoRecovery::Request::SharedPtr request)
{
  (void)request;

  auto request_id = *header;
  const auto api_id = static_cast<int32_t>(SportApiId::AUTO_RECOVERY_GET);

  go2_interfaces::srv::GetAutoRecovery::Response failure;
  failure.success = false;

  if (!wait_for_response_) {
    // The answer is the reply itself, so there is nothing to report without it.
    failure.message = "get_auto_recovery needs wait_for_response to be enabled";
    get_auto_recovery_service_->send_response(request_id, failure);
    return;
  }

  const auto sent = sendRequest(api_id, emptyJson(), [this, request_id](const ApiResult & result) mutable {
    go2_interfaces::srv::GetAutoRecovery::Response response;
    response.success = result.ok_;
    response.message = describe(api_id, result, true);

    if (result.ok_) {
      // The robot answers {"data": <bool>}; a malformed reply must not throw
      // out of the subscription callback.
      const auto parsed = nlohmann::json::parse(result.data_, nullptr, false);
      if (parsed.is_discarded() || !parsed.contains("data")) {
        response.success = false;
        response.message = "could not read a flag out of the reply: " + result.data_;
      } else {
        response.enable = parsed["data"].get<bool>();
      }
    }

    get_auto_recovery_service_->send_response(request_id, response);
  });

  if (!sent) {
    failure.message = "api_id=" + std::to_string(api_id) + " could not be sent";
    get_auto_recovery_service_->send_response(request_id, failure);
  }
}

}  // namespace go2_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(go2_driver::Go2SportBridge)
