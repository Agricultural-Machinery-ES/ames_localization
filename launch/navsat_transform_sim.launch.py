import os
from ament_index_python.packages import get_package_share_directory
from ames_utils.config_injector import with_global_config
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("robot_localization")
    config_file = os.path.join(pkg_share, "params", "navsat_transform_sim.yaml")

    local_ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="local_ekf_node",
        output="screen",
        parameters=with_global_config([config_file]),
        remappings=[("odometry/filtered", "/odometry/local")],
    )

    navsat_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform_node",
        output="screen",
        parameters=with_global_config([config_file]),
        remappings=[
            ("imu", "/imu/data"),
            ("gps/fix", "/gps/fix"),
            ("odometry/filtered", "/odometry/local"),
            ("odometry/gps", "/odometry/gps"),
            ("gps/filtered", "/gps/filtered"),
        ],
    )

    global_ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="global_ekf_node",
        output="screen",
        parameters=with_global_config([config_file]),
        remappings=[("odometry/filtered", "/odometry/global")],
    )

    return LaunchDescription([local_ekf_node, navsat_node, global_ekf_node])
