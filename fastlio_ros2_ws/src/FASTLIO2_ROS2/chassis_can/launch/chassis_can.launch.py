from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_share = get_package_share_directory("chassis_can")
    params = os.path.join(pkg_share, "config", "chassis_can.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("interface", default_value="can0"),
        DeclareLaunchArgument("listen_only", default_value="true"),
        DeclareLaunchArgument("enable_on_start", default_value="false"),
        DeclareLaunchArgument("query_on_start", default_value="false"),
        DeclareLaunchArgument("query_period_s", default_value="1.0"),
        DeclareLaunchArgument("wheel_base", default_value="0.80"),
        DeclareLaunchArgument("wheel_radius", default_value="0.22"),
        DeclareLaunchArgument("gear_ratio", default_value="1.0"),
        DeclareLaunchArgument("speed_scale", default_value="1.0"),
        DeclareLaunchArgument("left_sign", default_value="1.0"),
        DeclareLaunchArgument("right_sign", default_value="1.0"),
        DeclareLaunchArgument("feedback_byte_order", default_value="big"),
        DeclareLaunchArgument("feedback_crc_enabled", default_value="false"),
        DeclareLaunchArgument("feedback_crc_mode", default_value="modbus"),
        DeclareLaunchArgument("feedback_parse_mode", default_value="small_board"),
        DeclareLaunchArgument("combined_left_speed_offset", default_value="0"),
        DeclareLaunchArgument("combined_right_speed_offset", default_value="2"),
        DeclareLaunchArgument("publish_odom_tf", default_value="false"),
        Node(
            package="chassis_can",
            executable="chassis_can_comm_node.py",
            name="chassis_can_node",
            output="screen",
            parameters=[
                params,
                {
                    "interface": LaunchConfiguration("interface"),
                    "listen_only": ParameterValue(LaunchConfiguration("listen_only"), value_type=bool),
                    "enable_on_start": ParameterValue(LaunchConfiguration("enable_on_start"), value_type=bool),
                    "query_on_start": ParameterValue(LaunchConfiguration("query_on_start"), value_type=bool),
                    "query_period_s": ParameterValue(LaunchConfiguration("query_period_s"), value_type=float),
                    "wheel_base": ParameterValue(LaunchConfiguration("wheel_base"), value_type=float),
                    "wheel_radius": ParameterValue(LaunchConfiguration("wheel_radius"), value_type=float),
                    "gear_ratio": ParameterValue(LaunchConfiguration("gear_ratio"), value_type=float),
                    "speed_scale": ParameterValue(LaunchConfiguration("speed_scale"), value_type=float),
                    "left_sign": ParameterValue(LaunchConfiguration("left_sign"), value_type=float),
                    "right_sign": ParameterValue(LaunchConfiguration("right_sign"), value_type=float),
                    "feedback_byte_order": LaunchConfiguration("feedback_byte_order"),
                    "feedback_crc_enabled": ParameterValue(LaunchConfiguration("feedback_crc_enabled"), value_type=bool),
                    "feedback_crc_mode": LaunchConfiguration("feedback_crc_mode"),
                    "feedback_parse_mode": LaunchConfiguration("feedback_parse_mode"),
                    "combined_left_speed_offset": ParameterValue(LaunchConfiguration("combined_left_speed_offset"), value_type=int),
                    "combined_right_speed_offset": ParameterValue(LaunchConfiguration("combined_right_speed_offset"), value_type=int),
                    "publish_odom_tf": ParameterValue(LaunchConfiguration("publish_odom_tf"), value_type=bool),
                },
            ],
        )
    ])
