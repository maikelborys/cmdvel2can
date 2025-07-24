


cmake_minimum_required(VERSION 3.8)
project(cmdvel2can)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(can_msgs REQUIRED)

include_directories(include)


# CAN Bridge Node
add_executable(can_bridge_node src/can_bridge_node.cpp)
ament_target_dependencies(can_bridge_node rclcpp can_msgs)
install(TARGETS can_bridge_node DESTINATION lib/${PROJECT_NAME})

# Build the main modular node
add_executable(cmdvel2can_node src/main.cpp src/cmdvel_to_can_bridge.cpp src/can_interface.cpp src/velocity_converter.cpp src/can_message_builder.cpp src/safety_system.cpp src/velocity_monitor.cpp src/incremental_controller.cpp)
ament_target_dependencies(cmdvel2can_node rclcpp geometry_msgs std_msgs diagnostic_msgs std_srvs can_msgs)
install(TARGETS cmdvel2can_node DESTINATION lib/${PROJECT_NAME})

# Interactive /cmd_vel publisher node
add_executable(cmdvel_pub_node src/cmdvel_pub_node.cpp)
ament_target_dependencies(cmdvel_pub_node rclcpp geometry_msgs)
install(TARGETS cmdvel_pub_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
