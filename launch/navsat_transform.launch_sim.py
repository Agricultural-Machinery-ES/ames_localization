import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("robot_localization")
    config_file = os.path.join(pkg_share, "config", "navsat_transform_sim.yaml")

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

    static_map_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )
    return LaunchDescription([navsat_node, static_map_odom])
