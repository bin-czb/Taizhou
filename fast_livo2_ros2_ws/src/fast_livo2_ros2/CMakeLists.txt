cmake_minimum_required(VERSION 3.8)
project(fast_livo2_ros2)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY config docs launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/analyze_imu_static.py
  scripts/apriltag_lidar_extrinsic.py
  scripts/benchmark_hwt601_poll_rate.py
  scripts/calibrate_lidar_camera_planes.py
  scripts/calibrate_checkerboard_from_topic.py
  scripts/check_livox_mid360_link.sh
  scripts/check_record_topics.py
  scripts/fix_ros1_bag_type_names.py
  scripts/hwt601_485_imu_node.py
  scripts/probe_hwt601_485.py
  scripts/record_fast_livo_site_bag.sh
  scripts/record_livox_builtin_imu_bag.sh
  scripts/record_validation_bag.sh
  scripts/record_lidar_imu_init_bag.sh
  scripts/record_camera_intrinsic_bag.sh
  scripts/start_usb_camera.sh
  scripts/validate_intrinsics_with_apriltag.py
  scripts/visualize_lidar_camera_projection.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
