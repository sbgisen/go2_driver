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

#include "go2_driver/go2_robot_state_bridge.hpp"

#include <functional>
#include <nlohmann/json.hpp>
#include <string>
#include <utility>

#include "go2_driver/robot_state_api_id.hpp"

namespace go2_driver
{

namespace
{

// Seconds the robot is given to answer. Listing its services takes noticeably
// longer than acknowledging a switch, so this is more generous than the sport
// bridge's default.
constexpr double g_DEFAULT_RESPONSE_TIMEOUT = 5.0;

using ServiceSwitchCallback =
  std::function<void(std::shared_ptr<rmw_request_id_t>, go2_interfaces::srv::ServiceSwitch::Request::SharedPtr)>;
using SetReportFreqCallback =
  std::function<void(std::shared_ptr<rmw_request_id_t>, go2_interfaces::srv::SetReportFreq::Request::SharedPtr)>;
using ServiceListCallback =
  std::function<void(std::shared_ptr<rmw_request_id_t>, go2_interfaces::srv::ServiceList::Request::SharedPtr)>;

auto describe(int32_t api_id, const ApiResult & result) -> std::string
{
  const auto prefix = "api_id=" + std::to_string(api_id);
  if (result.ok_) {
    return prefix + " accepted";
  }
  if (result.status_code_ < 0) {
    return prefix + " got no reply";
  }
  return prefix + " rejected with status " + std::to_string(result.status_code_);
}

}  // namespace

Go2RobotStateBridge::Go2RobotStateBridge(const rclcpp::NodeOptions & options) : Node("go2_robot_state_bridge", options)
{
  const auto timeout = declare_parameter<double>("response_timeout", g_DEFAULT_RESPONSE_TIMEOUT);

  api_client_ = std::make_unique<UnitreeApiClient>(
    this, "api/robot_state/request", "api/robot_state/response",
    std::chrono::milliseconds(static_cast<int64_t>(timeout * 1000.0)));

  service_switch_service_ = create_service<go2_interfaces::srv::ServiceSwitch>(
    "service_switch",
    ServiceSwitchCallback(
      [this](std::shared_ptr<rmw_request_id_t> header, go2_interfaces::srv::ServiceSwitch::Request::SharedPtr req) {
        handleServiceSwitch(std::move(header), std::move(req));
      }));

  set_report_freq_service_ = create_service<go2_interfaces::srv::SetReportFreq>(
    "set_report_freq",
    SetReportFreqCallback(
      [this](std::shared_ptr<rmw_request_id_t> header, go2_interfaces::srv::SetReportFreq::Request::SharedPtr req) {
        handleSetReportFreq(std::move(header), std::move(req));
      }));

  service_list_service_ = create_service<go2_interfaces::srv::ServiceList>(
    "service_list",
    ServiceListCallback(
      [this](std::shared_ptr<rmw_request_id_t> header, go2_interfaces::srv::ServiceList::Request::SharedPtr req) {
        handleServiceList(std::move(header), std::move(req));
      }));

  RCLCPP_INFO(get_logger(), "go2_robot_state_bridge started");
}

void Go2RobotStateBridge::handleServiceSwitch(
  std::shared_ptr<rmw_request_id_t> header, go2_interfaces::srv::ServiceSwitch::Request::SharedPtr request)
{
  auto request_id = *header;
  const auto api_id = static_cast<int32_t>(RobotStateApiId::SERVICE_SWITCH);

  nlohmann::json parameter;
  parameter["name"] = request->name;
  // The firmware spells this key "switch", which cannot be a field name in a
  // ROS interface, and it wants an int rather than a bool.
  parameter["switch"] = request->enable ? 1 : 0;

  const auto sent = api_client_->call(api_id, parameter, [this, request_id](const ApiResult & result) mutable {
    go2_interfaces::srv::ServiceSwitch::Response response;
    response.success = result.ok_;
    response.message = describe(api_id, result);

    if (result.ok_) {
      const auto parsed = nlohmann::json::parse(result.data_, nullptr, false);
      if (parsed.is_discarded() || !parsed.contains("status")) {
        response.success = false;
        response.message = "could not read a status out of the reply: " + result.data_;
      } else {
        response.status = parsed["status"].get<int32_t>();
      }
    }

    service_switch_service_->send_response(request_id, response);
  });

  if (!sent) {
    go2_interfaces::srv::ServiceSwitch::Response response;
    response.success = false;
    response.message = "api_id=" + std::to_string(api_id) + " could not be sent";
    service_switch_service_->send_response(request_id, response);
  }
}

void Go2RobotStateBridge::handleSetReportFreq(
  std::shared_ptr<rmw_request_id_t> header, go2_interfaces::srv::SetReportFreq::Request::SharedPtr request)
{
  auto request_id = *header;
  const auto api_id = static_cast<int32_t>(RobotStateApiId::SET_REPORT_FREQ);

  nlohmann::json parameter;
  parameter["interval"] = request->interval;
  parameter["duration"] = request->duration;

  // The robot does not answer this one, so waiting for a reply would only ever
  // time out.
  api_client_->send(api_id, parameter, true);

  go2_interfaces::srv::SetReportFreq::Response response;
  response.success = true;
  response.message = "api_id=" + std::to_string(api_id) + " published; the robot sends no reply to it";
  set_report_freq_service_->send_response(request_id, response);
}

void Go2RobotStateBridge::handleServiceList(
  std::shared_ptr<rmw_request_id_t> header, go2_interfaces::srv::ServiceList::Request::SharedPtr request)
{
  (void)request;

  auto request_id = *header;
  const auto api_id = static_cast<int32_t>(RobotStateApiId::SERVICE_LIST);

  const auto sent =
    api_client_->call(api_id, nlohmann::json::object(), [this, request_id](const ApiResult & result) mutable {
      go2_interfaces::srv::ServiceList::Response response;
      response.success = result.ok_;
      response.message = describe(api_id, result);

      if (result.ok_) {
        const auto parsed = nlohmann::json::parse(result.data_, nullptr, false);
        if (parsed.is_discarded() || !parsed.is_array()) {
          response.success = false;
          response.message = "could not read a service list out of the reply: " + result.data_;
        } else {
          for (const auto & entry : parsed) {
            go2_interfaces::msg::ServiceState state;
            // value() rather than at(): a firmware that stops reporting one of
            // these should still yield a usable list.
            state.name = entry.value("name", "");
            state.status = entry.value("status", 0);
            state.protect = entry.value("protect", 0);
            state.version = entry.value("version", "");
            response.services.push_back(state);
          }
        }
      }

      service_list_service_->send_response(request_id, response);
    });

  if (!sent) {
    go2_interfaces::srv::ServiceList::Response response;
    response.success = false;
    response.message = "api_id=" + std::to_string(api_id) + " could not be sent";
    service_list_service_->send_response(request_id, response);
  }
}

}  // namespace go2_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(go2_driver::Go2RobotStateBridge)
