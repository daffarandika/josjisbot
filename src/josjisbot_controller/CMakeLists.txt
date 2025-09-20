cmake_minimum_required(VERSION 3.8)
project(josjisbot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

add_executable(joystick_republisher src/joystick_republisher.cpp)
ament_target_dependencies(joystick_republisher rclcpp sensor_msgs)

add_executable(robot_centric_controller src/robot_centric_controller.cpp)
ament_target_dependencies(robot_centric_controller rclcpp std_msgs sensor_msgs geometry_msgs tf2 tf2_ros)

install(
	DIRECTORY include
	DESTINATION include
)

install(TARGETS
	joystick_republisher
	robot_centric_controller
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()
