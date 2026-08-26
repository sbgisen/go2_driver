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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _param(name: str, value_type: type = str) -> ParameterValue:
    # value_type coerces the launch-argument string so the node's parameter type
    # checks (declare_parameter<std::string> / <double> / <bool>) pass.
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
                              default_value='false',
                              description='Use the incoming message stamp instead of the current node clock.'))
    args.append(
        DeclareLaunchArgument('publish_tf',
                              default_value='true',
                              description='Whether to broadcast the odom -> base_link TF chain.'))
    args.append(
        DeclareLaunchArgument('publish_planar_odom',
                              default_value='true',
                              description='Whether to publish the flattened planar odometry.'))

    go2_driver = Node(package='go2_driver',
                      executable='go2_driver_node',
                      name='go2_driver',
                      output='screen',
                      parameters=[{
                          'input_pointcloud_topic': '/utlidar/cloud',
                          'input_odom_topic': '/utlidar/robot_odom',
                          'output_planar_odom_topic': 'odom_planar',
                          'pointcloud_frame': _param('pointcloud_frame'),
                          'odom_frame': _param('odom_frame'),
                          'base_footprint_frame': _param('base_footprint_frame'),
                          'base_link_frame': _param('base_link_frame'),
                          'body_z_offset': _param('body_z_offset', float),
                          'use_msg_stamp': _param('use_msg_stamp', bool),
                          'publish_tf': _param('publish_tf', bool),
                          'publish_planar_odom': _param('publish_planar_odom', bool),
                      }])

    return LaunchDescription(args + [
        go2_driver,
    ])
