import os
from ament_index_python.packages import get_package_share_directory
from ames_utils.config_injector import with_global_config
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("robot_localization")
    config_file = os.path.join(pkg_share, "params", "navsat_transform.yaml")

    # 1. Local EKF: 用实机 /gps/odom 作为局部运动输入，生成 /odometry/local
    local_ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="local_ekf_node",
        output="screen",
        parameters=with_global_config([config_file]),
        remappings=[("odometry/filtered", "/odometry/local")],
    )

    # 2. NavSat Transform: 把 /gps/fix 转成 map 系下的平面观测，航向来自 /odometry/global
    navsat_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform_node",
        output="screen",
        parameters=with_global_config([config_file]),
        remappings=[
            ("gps/fix", "/gps/fix"),
            ("odometry/filtered", "/odometry/global"),
            ("odometry/gps", "/odometry/gps"),
            ("gps/filtered", "/gps/filtered"),
        ],
    )

    static_base_link_navsat = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "navsat_link"],
    )

    # 3. Global EKF: 融合 /odometry/local 和 /odometry/gps，发布 map -> odom
    global_ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="global_ekf_node",
        output="screen",
        parameters=with_global_config([config_file]),
        remappings=[("odometry/filtered", "/odometry/global")],
    )

    return LaunchDescription([
        local_ekf_node,
        navsat_node,
        static_base_link_navsat,
        global_ekf_node,
    ])