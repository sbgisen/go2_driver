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

#ifndef GO2_DRIVER__ROBOT_STATE_API_ID_HPP_
#define GO2_DRIVER__ROBOT_STATE_API_ID_HPP_

#include <cstdint>

namespace go2_driver
{

// Robot State API, added in unitree_ros2 v0.2.0:
//   https://github.com/unitreerobotics/unitree_ros2/releases/tag/v0.2.0
//
// WARNING: these values overlap numerically with SportApiId (DAMP is also
// 1001). They are only ever published on api/robot_state/request, never on
// api/sport/request, which is why the two enums live in separate components.
// Sending 1001 to the wrong topic makes the robot collapse.
//
// NOLINTNEXTLINE(performance-enum-size)
enum class RobotStateApiId : int32_t
{
  SERVICE_SWITCH = 1001,
  SET_REPORT_FREQ = 1002,
  SERVICE_LIST = 1003,
};

}  // namespace go2_driver

#endif  // GO2_DRIVER__ROBOT_STATE_API_ID_HPP_
