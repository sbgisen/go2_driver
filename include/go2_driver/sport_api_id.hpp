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

#ifndef GO2_DRIVER__SPORT_API_ID_HPP_
#define GO2_DRIVER__SPORT_API_ID_HPP_

#include <cstdint>

namespace go2_driver
{

// The underlying type is fixed to int32_t to match Unitree's Sport API ID width
// (the value is sent as unitree_api Request api_id); do not narrow it.
// NOLINTNEXTLINE(performance-enum-size)
enum class SportApiId : int32_t
{
  DAMP = 1001,
  BALANCE_STAND = 1002,
  STOP_MOVE = 1003,
  STAND_UP = 1004,
  STAND_DOWN = 1005,
  RECOVERY_STAND = 1006,

  MOVE = 1008,
  SIT = 1009,
  RISE_SIT = 1010,
  HELLO = 1016,
  STRETCH = 1017,
  DANCE1 = 1022,
  DANCE2 = 1023,
  FINGER_HEART = 1036,

  SPEED_LEVEL = 1015,
  SWITCH_JOYSTICK = 1027,

  STATIC_WALK = 1061,
  TROT_RUN = 1062,
  ECONOMIC_GAIT = 1063,

  LEFT_FLIP = 2041,
  BACK_FLIP = 2043,
  HAND_STAND = 2044,
  FREE_WALK = 2045,
  FREE_BOUND = 2046,
  FREE_JUMP = 2047,
  FREE_AVOID = 2048,
  CLASSIC_WALK = 2049,
  WALK_UPRIGHT = 2050,
  CROSS_STEP = 2051,
  AUTO_RECOVERY_SET = 2054,
  AUTO_RECOVERY_GET = 2055,  // getter: returns a value, not used by the publish-only controller
  SWITCH_AVOID_MODE = 2058,
};

// API IDs that are no longer supported by Unitree.
//
// These were removed as a BREAKING CHANGE in unitree_ros2 v0.2.0 (2025-07-30):
//   https://github.com/unitreerobotics/unitree_ros2/releases/tag/v0.2.0
//
// Do NOT add these to SportApiId or publish them: the robot ignores the
// request on current firmware. They are listed here only so the values are
// not accidentally reused.
//
//   | API ID | Macro Definition                      | Function Name        |
//   | 1011   | ROBOT_SPORT_API_ID_SWITCHGAIT         | SwitchGait()         |
//   | 1012   | ROBOT_SPORT_API_ID_TRIGGER            | Trigger()            |
//   | 1013   | ROBOT_SPORT_API_ID_BODYHEIGHT         | BodyHeight()         |
//   | 1014   | ROBOT_SPORT_API_ID_FOOTRAISEHEIGHT    | FootRaiseHeight()    |
//   | 1018   | ROBOT_SPORT_API_ID_TRAJECTORYFOLLOW   | TrajectoryFollow()   |
//   | 1019   | ROBOT_SPORT_API_ID_CONTINUOUSGAIT     | ContinuousGait()     |
//   | 1021   | ROBOT_SPORT_API_ID_WALLOW             | Wallow()             |
//   | 1024   | ROBOT_SPORT_API_ID_GETBODYHEIGHT      | (no direct function) |
//   | 1025   | ROBOT_SPORT_API_ID_GETFOOTRAISEHEIGHT | (no direct function) |
//   | 1026   | ROBOT_SPORT_API_ID_GETSPEEDLEVEL      | (no direct function) |

}  // namespace go2_driver

#endif  // GO2_DRIVER__SPORT_API_ID_HPP_
