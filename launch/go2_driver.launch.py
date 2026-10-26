# Copyright (c) 2026 SoftBank Corp.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Original work: Copyright (c) 2024 Intelligent Robotics Lab (URJC),
# licensed under the BSD 3-Clause License. See the NOTICE file for its terms.

from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.utilities.type_utils import AllowedTypesType
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _param(name: str, value_type: AllowedTypesType = str) -> ParameterValue:
    # value_type coerces the launch-argument string so the node's parameter type
    # checks (declare_parameter<std::string> / <double> / <bool> / <vector<string>>)
    # pass. List arguments are written in yaml flow style: '[a, b]'.
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


def generate_launch_description() -> LaunchDescription:
    """Generate launch descriptions.

    Returns:
        Launch descriptions
    """
    args = []
    args.append(
        DeclareLaunchArgument('pointcloud_frame',
                              default_value='utlidar_lidar',
                              description='Frame ID assigned to the republished Unitree L1 point cloud.'))
    args.append(
        DeclareLaunchArgument('odom_frame',
                              default_value='odom',
                              description='Parent frame of the published odometry and TF chain.'))
    args.append(
        DeclareLaunchArgument('base_footprint_frame',
                              default_value='base_footprint',
                              description='Ground-projected frame published below odom_frame.'))
    args.append(
        DeclareLaunchArgument('base_link_frame',
                              default_value='base_link',
                              description='Robot body frame published below base_footprint_frame.'))
    args.append(
        DeclareLaunchArgument('body_z_offset',
                              default_value='0.0',
                              description='Offset added to the base_link TF body height [m].'))
    args.append(
        DeclareLaunchArgument('use_msg_stamp',
                              default_value='true',
                              description='Use the incoming message stamp instead of the current node clock.'))
    args.append(
        DeclareLaunchArgument('publish_tf',
                              default_value='true',
                              description='Whether to broadcast the odom -> base_link TF chain.'))
    args.append(
        DeclareLaunchArgument('publish_odom',
                              default_value='true',
                              description='Whether to republish the Unitree odometry.'))
    args.append(
        DeclareLaunchArgument(
            'wait_for_response',
            default_value='true',
            description="Sport bridge: report the robot's status code instead of only that the request was sent."))
    args.append(
        DeclareLaunchArgument(
            'blocked_modes',
            default_value='[front_flip, back_flip, left_flip, hand_stand]',
            description='Sport bridge: mode presets refused without being sent, as a yaml list of names. '
            '[none] blocks nothing.'))
    args.append(
        DeclareLaunchArgument('sport_response_timeout',
                              default_value='2.0',
                              description='Sport bridge: seconds the robot is given to reply to a request [s].'))
    args.append(
        DeclareLaunchArgument('robot_state_response_timeout',
                              default_value='5.0',
                              description='Robot state bridge: seconds the robot is given to reply to a request [s].'))
    args.append(
        DeclareLaunchArgument('use_camera',
                              default_value='false',
                              description='Whether to start the front camera decoder.'))
    args.append(
        DeclareLaunchArgument('camera_resolution',
                              default_value='720',
                              description='Resolution tag of the /frontvideostream frames to decode.'))

    go2_driver = Node(package='go2_driver',
                      executable='go2_driver_node',
                      name='go2_driver',
                      output='screen',
                      parameters=[{
                          'input_pointcloud_topic': '/utlidar/cloud',
                          'input_odom_topic': '/utlidar/robot_odom',
                          'pointcloud_frame': _param('pointcloud_frame'),
                          'odom_frame': _param('odom_frame'),
                          'base_footprint_frame': _param('base_footprint_frame'),
                          'base_link_frame': _param('base_link_frame'),
                          'body_z_offset': _param('body_z_offset', float),
                          'use_msg_stamp': _param('use_msg_stamp', bool),
                          'publish_tf': _param('publish_tf', bool),
                          'publish_odom': _param('publish_odom', bool),
                      }])

    # A separate process from go2_driver on purpose: the state bridge must keep
    # publishing TF and odometry even if the command bridge is not wanted.
    # The two bridges' response_timeout parameters have different defaults
    # (listing services takes longer than acknowledging a command), so each gets
    # its own launch argument instead of one shared value.
    go2_sport_bridge = Node(package='go2_driver',
                            executable='go2_sport_bridge_node',
                            name='go2_sport_bridge',
                            output='screen',
                            parameters=[{
                                'wait_for_response': _param('wait_for_response', bool),
                                'blocked_modes': _param('blocked_modes', List[str]),
                                'response_timeout': _param('sport_response_timeout', float),
                            }])

    # No wait_for_response here: Go2RobotStateBridge does not declare one. It
    # always waits, because every one of its services is a query whose answer is
    # the point of calling it.
    go2_robot_state_bridge = Node(package='go2_driver',
                                  executable='go2_robot_state_bridge_node',
                                  name='go2_robot_state_bridge',
                                  output='screen',
                                  parameters=[{
                                      'response_timeout': _param('robot_state_response_timeout', float),
                                  }])

    go2_camera = Node(package='go2_driver',
                      executable='go2_camera_node',
                      name='go2_camera',
                      output='screen',
                      condition=IfCondition(LaunchConfiguration('use_camera')),
                      parameters=[{
                          'resolution': _param('camera_resolution', int),
                      }])

    return LaunchDescription(args + [
        go2_driver,
        go2_sport_bridge,
        go2_robot_state_bridge,
        go2_camera,
    ])
