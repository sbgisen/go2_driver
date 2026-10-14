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
// WARNING: these overlap numerically with SportApiId -- 1001 is ServiceSwitch
// here and Damp there. Publish them on api/robot_state/request only.
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
