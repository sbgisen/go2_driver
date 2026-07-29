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

#ifndef GO2_DRIVER__GO2_ROBOT_STATE_BRIDGE_HPP_
#define GO2_DRIVER__GO2_ROBOT_STATE_BRIDGE_HPP_

#include <go2_driver/unitree_api_client.hpp>
#include <go2_interfaces/srv/service_list.hpp>
#include <go2_interfaces/srv/service_switch.hpp>
#include <go2_interfaces/srv/set_report_freq.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace go2_driver
{

// Starts, stops and lists the services running inside the robot.
//
// Kept apart from Go2SportBridge on purpose: this is configuration of the
// robot rather than motion, it talks on a different topic pair, and its api
// ids collide numerically with the sport ones.
class Go2RobotStateBridge : public rclcpp::Node
{
public:
  explicit Go2RobotStateBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  auto handleServiceSwitch(
    std::shared_ptr<rmw_request_id_t> header, go2_interfaces::srv::ServiceSwitch::Request::SharedPtr request) -> void;

  auto handleSetReportFreq(
    std::shared_ptr<rmw_request_id_t> header, go2_interfaces::srv::SetReportFreq::Request::SharedPtr request) -> void;

  auto handleServiceList(
    std::shared_ptr<rmw_request_id_t> header, go2_interfaces::srv::ServiceList::Request::SharedPtr request) -> void;

  std::unique_ptr<UnitreeApiClient> api_client_;

  rclcpp::Service<go2_interfaces::srv::ServiceSwitch>::SharedPtr service_switch_service_;
  rclcpp::Service<go2_interfaces::srv::SetReportFreq>::SharedPtr set_report_freq_service_;
  rclcpp::Service<go2_interfaces::srv::ServiceList>::SharedPtr service_list_service_;
};

}  // namespace go2_driver

#endif  // GO2_DRIVER__GO2_ROBOT_STATE_BRIDGE_HPP_
