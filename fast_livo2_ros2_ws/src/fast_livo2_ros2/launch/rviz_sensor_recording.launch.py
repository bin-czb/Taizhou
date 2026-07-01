from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = LaunchConfiguration("rviz_config")
    default_rviz_config = PathJoinSubstitution(
        [FindPackageShare("fast_livo2_ros2"), "rviz", "sensor_recording.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="RViz2 configuration for raw sensor recording checks.",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="sensor_recording_rviz",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )
