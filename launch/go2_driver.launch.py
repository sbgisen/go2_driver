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

# Original work: Copyright (c) 2024 Intelligent Robotics Lab (URJC).

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description() -> LaunchDescription:
    args = []
    args.append(
        DeclareLaunchArgument('pointcloud_frame',
                              default_value='radar',
                              description='Frame ID assigned to the republished Unitree L1 point cloud.'))

    pointcloud_frame = LaunchConfiguration('pointcloud_frame')

    composable_nodes = []

    composable_node = ComposableNode(
        package='go2_driver',
        plugin='go2_driver::Go2Driver',
        name='go2_driver',
        namespace='',
        parameters=[{
            'input_odom_topic': '/utlidar/robot_odom',
            'output_planar_odom_topic': '/pochi/odom_planar',
            'pointcloud_frame': pointcloud_frame,
            'odom_frame': 'odom',
            'base_footprint_frame': 'base_footprint',
            'base_link_frame': 'base_link',
            'body_z_offset': 0.0,
            'use_msg_stamp': False,
            'publish_tf': True,
            'publish_planar_odom': True,
        }],
    )
    composable_nodes.append(composable_node)

    container = ComposableNodeContainer(
        name='go2_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen',
    )

    return LaunchDescription(args + [
        container,
    ])
