from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration("port")
    baudrate = LaunchConfiguration("baudrate")
    address = LaunchConfiguration("address")
    rate_hz = LaunchConfiguration("rate_hz")
    accel_range_g = LaunchConfiguration("accel_range_g")

    return LaunchDescription(
        [
            DeclareLaunchArgument("port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("baudrate", default_value="115200"),
            DeclareLaunchArgument("address", default_value="80"),
            DeclareLaunchArgument("rate_hz", default_value="150.0"),
            DeclareLaunchArgument("accel_range_g", default_value="4.0"),
            Node(
                package="fast_livo2_ros2",
                executable="hwt601_485_imu_node.py",
                name="hwt601_485_imu_node",
                output="screen",
                parameters=[
                    {
                        "port": port,
                        "baudrate": baudrate,
                        "address": address,
                        "rate_hz": rate_hz,
                        "accel_range_g": accel_range_g,
                        "topic": "/imu/data",
                        "frame_id": "imu_link",
                    }
                ],
            ),
        ]
    )
