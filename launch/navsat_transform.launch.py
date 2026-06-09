import os
from ames_utils.config_injector import with_global_config
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    config_file = os.path.join(package_root, "params", "navsat_transform.yaml")

    local_ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="local_ekf_node",
        output="screen",
        parameters=with_global_config([config_file]),
        remappings=[("odometry/filtered", "/odometry/local")],
    )

    global_ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="global_ekf_node",
        output="screen",
        parameters=with_global_config([config_file]),
        remappings=[("odometry/filtered", "/odometry/global")],
    )

    navsat_transform_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform_node",
        output="screen",
        parameters=with_global_config([config_file]),
        remappings=[
            ("imu", "/imu/data"),
            ("gps/fix", "/gps/fix"),
            ("odometry/filtered", "/odometry/global"),
            ("odometry/gps", "/odometry/gps"),
        ],
    )
    static_base_link_navsat = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "navsat_link"],
    )

    return LaunchDescription([
        local_ekf_node,
        global_ekf_node,
        navsat_transform_node,
        static_base_link_navsat,
    ])
