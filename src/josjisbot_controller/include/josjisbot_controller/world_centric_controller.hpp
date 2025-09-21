#ifndef WORLD_CENTRIC_CONTROLLER_H
#define WORLD_CENTRIC_CONTROLLER_H

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "josjisbot_controller/robot_controller.hpp"
#include <rclcpp/time.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp/node.hpp>
#include <string>
#include <array>

class WorldCentricController:
	public rclcpp::Node,
	public RobotController
{
public:
	WorldCentricController(const std::string& name);

private:
	rclcpp::Publisher<Float32MultiArray>::SharedPtr wheel_cmd_pub_;
	rclcpp::Subscription<TwistStamped>::SharedPtr cmd_sub_;

	const std::array<float, 4> wheel_angles_ = {45.0, 135.0, 225.0, 315.0}; // alpha 1-4
	Float32MultiArray wheel_cmd_;
	float x_, y_, heading_;

	void cmdCallback(const TwistStamped& msg);
	Float32MultiArray inverseKinematics(const TwistStamped& cmd) override;
	TwistStamped forwardKinematics(const Float32MultiArray& wheel_cmd) override;
};

#endif // !WORLD_CENTRIC_CONTROLLER_H
