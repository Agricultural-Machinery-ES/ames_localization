# Copyright 2019 Samsung Research America
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

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("robot_localization")
    config_file = os.path.join(pkg_share, "params", "navsat_transform.yaml")

    # 1. NavSat Transform Node: 将 GPS 经纬度转换为 Map 坐标系下的米制坐标
    navsat_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform_node",
        output="screen",
        parameters=[config_file],
        remappings=[
            ("imu/data", "/imu/data"),
            ("gps/fix", "/gps/fix"),
            ("odometry/filtered", "/odom"),  # 订阅局部EKF
            ("odometry/gps", "/odometry/gps"),  # 输出转换后的位姿
        ],
    )

    # 2. Global EKF Node: 融合 GPS、IMU 和局部里程计，发布 map -> odom
    global_ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="global_ekf_node",
        output="screen",
        parameters=[config_file],
    )

    return LaunchDescription([navsat_node, global_ekf_node])
