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
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

add_executable(robot_centric_controller src/robot_centric_controller.cpp)
ament_target_dependencies(robot_centric_controller rclcpp std_msgs sensor_msgs geometry_msgs tf2 tf2_ros nav_msgs)

add_executable(world_centric_controller src/world_centric_controller.cpp)
ament_target_dependencies(world_centric_controller rclcpp std_msgs sensor_msgs geometry_msgs tf2 tf2_ros nav_msgs)

install(
	DIRECTORY include
	DESTINATION include
)

install(TARGETS
	robot_centric_controller
	world_centric_controller
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()
