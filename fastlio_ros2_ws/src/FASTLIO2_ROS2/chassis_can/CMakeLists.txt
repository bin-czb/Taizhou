cmake_minimum_required(VERSION 3.8)
project(chassis_can)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/can_protocol_probe.py
  scripts/chassis_can_comm_node.py
  scripts/setup_can.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
