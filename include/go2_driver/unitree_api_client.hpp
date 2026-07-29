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

#ifndef GO2_DRIVER__UNITREE_API_CLIENT_HPP_
#define GO2_DRIVER__UNITREE_API_CLIENT_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unitree_api/msg/request.hpp>
#include <unitree_api/msg/response.hpp>
#include <unordered_map>

namespace go2_driver
{

// Outcome of a correlated Unitree API call.
struct ApiResult
{
  // True when the robot replied and reported status code 0.
  bool ok_;

  // Status code the robot reported, or -1 when the call timed out.
  int32_t status_code_;

  // Response.data, the reply's JSON payload. Empty when the call timed out.
  std::string data_;
};

using ApiResponseCallback = std::function<void(const ApiResult &)>;

// Publishes unitree_api requests and matches the robot's replies back to them
// on header.identity.id.
//
// Not thread-safe by design: every method, and every callback it invokes, must
// run on a single executor thread. Both bridges in this package are hosted in
// their own single-threaded process, so no locking is needed. Hosting one of
// them in component_container_mt would require guarding pending_ with a mutex.
class UnitreeApiClient
{
public:
  UnitreeApiClient(
    rclcpp::Node * node, const std::string & request_topic, const std::string & response_topic,
    std::chrono::milliseconds timeout);

  // Publishes without asking for a reply. Use it for commands sent at a high
  // rate, and for api ids the firmware never answers.
  auto send(int32_t api_id, const nlohmann::json & parameter, bool noreply = false) -> void;

  // Publishes and invokes on_response exactly once, either with the robot's
  // reply or, after the timeout, with a failed ApiResult.
  //
  // Returns false without publishing when too many calls are already in
  // flight, which is what happens when the robot answers nothing at all.
  auto call(int32_t api_id, const nlohmann::json & parameter, ApiResponseCallback on_response) -> bool;

  auto pendingCount() const -> std::size_t;

private:
  struct PendingRequest
  {
    int32_t api_id_;
    rclcpp::Time deadline_;
    ApiResponseCallback callback_;
  };

  auto publish(int64_t id, int32_t api_id, const nlohmann::json & parameter, bool noreply) -> void;

  auto responseCallback(unitree_api::msg::Response::SharedPtr msg) -> void;

  auto sweepTimeouts() -> void;

  auto nextId() -> int64_t;

  rclcpp::Node * node_;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr request_pub_;
  rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr response_sub_;
  rclcpp::TimerBase::SharedPtr sweep_timer_;

  std::unordered_map<int64_t, PendingRequest> pending_;
  std::chrono::milliseconds timeout_;
  int64_t last_id_{0};
};

}  // namespace go2_driver

#endif  // GO2_DRIVER__UNITREE_API_CLIENT_HPP_
