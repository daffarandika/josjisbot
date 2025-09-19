cmake_minimum_required(VERSION 3.8)
project(josjisbot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_executable(joystick_republisher src/joystick_republisher.cpp)
ament_target_dependencies(joystick_republisher rclcpp sensor_msgs)

install(
	DIRECTORY include
	DESTINATION include
)

install(TARGETS
	joystick_republisher
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()
