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
from ames_utils.config_injector import with_global_config
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
        parameters=with_global_config([config_file]),
        remappings=[
            ("imu", "/imu/data"),
            ("gps/fix", "/gps/fix"),
            ("odometry/filtered", "/odom"),  # 订阅实机 GPS 里程计
            ("odometry/gps", "/odometry/gps"),  # 输出转换后的位姿
        ],
    )

    static_base_link_imu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "imu_link"],
    )

    static_base_link_navsat = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "navsat_link"],
    )

    # 2. Global EKF Node: 融合 GPS、IMU 和局部里程计，发布 map -> odom
    global_ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="global_ekf_node",
        output="screen",
        parameters=with_global_config([config_file]),
    )

    return LaunchDescription([
        navsat_node,
        static_base_link_imu,
        static_base_link_navsat,
        global_ekf_node,
    ])
