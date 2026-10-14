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

#include "go2_driver/unitree_api_client.hpp"

#include <algorithm>
#include <utility>
#include <vector>

namespace go2_driver
{

namespace
{

// Status code the robot reports when it accepted a request.
constexpr int32_t g_STATUS_OK = 0;

// Status code reported to the caller when the robot never replied.
constexpr int32_t g_STATUS_TIMEOUT = -1;

// Only bounds how late a timeout is noticed; replies are answered at once.
constexpr int g_SWEEP_PERIOD_MS = 100;

// Bounds pending_ if the robot stops answering entirely.
constexpr std::size_t g_MAX_PENDING = 64;

constexpr int g_QOS_DEPTH = 10;

}  // namespace

UnitreeApiClient::UnitreeApiClient(
  rclcpp::Node * node, const std::string & request_topic, const std::string & response_topic,
  std::chrono::milliseconds timeout)
: node_(node), timeout_(timeout)
{
  request_pub_ = node_->create_publisher<unitree_api::msg::Request>(request_topic, g_QOS_DEPTH);

  response_sub_ = node_->create_subscription<unitree_api::msg::Response>(
    response_topic, g_QOS_DEPTH,
    [this](unitree_api::msg::Response::SharedPtr msg) { responseCallback(std::move(msg)); });

  sweep_timer_ = node_->create_wall_timer(std::chrono::milliseconds(g_SWEEP_PERIOD_MS), [this] { sweepTimeouts(); });
}

void UnitreeApiClient::send(int32_t api_id, const nlohmann::json & parameter, bool noreply)
{
  publish(nextId(), api_id, parameter, noreply);
}

auto UnitreeApiClient::call(int32_t api_id, const nlohmann::json & parameter, ApiResponseCallback on_response) -> bool
{
  if (pending_.size() >= g_MAX_PENDING) {
    RCLCPP_ERROR(
      node_->get_logger(), "dropping api_id=%d: %zu requests are already awaiting a reply", api_id, pending_.size());
    return false;
  }

  const auto id = nextId();
  pending_.emplace(id, PendingRequest{api_id, node_->now() + rclcpp::Duration(timeout_), std::move(on_response)});

  publish(id, api_id, parameter, false);
  return true;
}

auto UnitreeApiClient::pendingCount() const -> std::size_t { return pending_.size(); }

void UnitreeApiClient::publish(int64_t id, int32_t api_id, const nlohmann::json & parameter, bool noreply)
{
  unitree_api::msg::Request req;
  req.header.identity.id = id;
  req.header.identity.api_id = api_id;
  req.header.policy.noreply = noreply;
  req.parameter = parameter.dump();

  request_pub_->publish(req);
}

void UnitreeApiClient::responseCallback(unitree_api::msg::Response::SharedPtr msg)
{
  const auto it = pending_.find(msg->header.identity.id);
  if (it == pending_.end()) {
    // Every node on the robot's DDS domain sees every reply.
    RCLCPP_DEBUG(node_->get_logger(), "ignoring reply for id=%ld", msg->header.identity.id);
    return;
  }

  if (it->second.api_id_ != msg->header.identity.api_id) {
    RCLCPP_WARN(
      node_->get_logger(), "reply for id=%ld carries api_id=%ld, expected %d", msg->header.identity.id,
      msg->header.identity.api_id, it->second.api_id_);
  }

  const auto callback = it->second.callback_;
  // Erase first: the callback may start another call, and a duplicate reply
  // must not answer the same request twice.
  pending_.erase(it);

  const auto code = msg->header.status.code;
  callback(ApiResult{code == g_STATUS_OK, code, msg->data});
}

void UnitreeApiClient::sweepTimeouts()
{
  if (pending_.empty()) {
    return;
  }

  const auto now = node_->now();

  std::vector<ApiResponseCallback> expired;
  for (auto it = pending_.begin(); it != pending_.end();) {
    if (it->second.deadline_ > now) {
      ++it;
      continue;
    }

    RCLCPP_WARN(node_->get_logger(), "no reply for api_id=%d within the timeout", it->second.api_id_);
    expired.push_back(it->second.callback_);
    it = pending_.erase(it);
  }

  // After the sweep: a callback may start another call.
  for (const auto & callback : expired) {
    callback(ApiResult{false, g_STATUS_TIMEOUT, ""});
  }
}

auto UnitreeApiClient::nextId() -> int64_t
{
  // Nanoseconds since boot, like unitree_ros2's clients: ids must be unique
  // across processes on the shared DDS domain, so a counter will not do.
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  const auto id = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();

  last_id_ = std::max(id, last_id_ + 1);
  return last_id_;
}

}  // namespace go2_driver
