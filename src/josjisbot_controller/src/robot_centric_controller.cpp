#include "josjisbot_controller/robot_centric_controller.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <rclcpp/logging.hpp>
#include <string>
#include <math.h>

#define CURRENT_TIME this->get_clock()->now()
#define PI M_PI

using namespace std::chrono_literals;
using namespace std::placeholders;

RobotCentricController::RobotCentricController(const std::string& name)
: Node(name)
{
	this->wheel_cmd_pub_ = this->create_publisher<Float32MultiArray>(
		"/josjisbot_controller/wheel_cmd",
		5
	);

	this->cmd_sub_ = this->create_subscription<TwistStamped>(
		"/cmd_vel",
		5,
		std::bind(&RobotCentricController::cmdCallback, this, _1)
	);

	this->transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

	this->tf_.header.set__frame_id("odom");
	this->tf_.set__child_frame_id("base_footprint");
	this->wheel_cmd_.set__data({0.0, 0.0, 0.0, 0.0});
	this->prev_time_ = CURRENT_TIME;
}

void RobotCentricController::cmdCallback(const TwistStamped& msg)
{
	auto wheel_cmd = this->inverseKinematics(msg);
	RCLCPP_INFO_STREAM(this->get_logger(), "w1: " << wheel_cmd.data[0] << " w2: " << wheel_cmd.data[1] << " w3: " << wheel_cmd.data[2] << " w4: " << wheel_cmd.data[3]);
}

Float32MultiArray RobotCentricController::inverseKinematics(const TwistStamped& cmd)
{

	for (int i = 0; i < 4; i++) {
		this->wheel_cmd_.data[i] = (
			// x dan y dibalik karena ROS menggunakan REP 103
			-sin(PI/180 * this->wheel_angles_[i]) * cmd.twist.linear.y +
			cos(PI/180 * this->wheel_angles_[i]) * cmd.twist.linear.x +
			cmd.twist.angular.z
		);
	}
	for (int i  = 0; i < 4; i++) {
		RCLCPP_INFO(this->get_logger(), "w%d: (%f) %f", i, wheel_angles_[i], wheel_cmd_.data[i]);
	}
	return this->wheel_cmd_;
}

TwistStamped  RobotCentricController::forwardKinematics(const Float32MultiArray& wheel_cmd)
{
	return TwistStamped();
}

// Float32MultiArray RobotCetricController::inverseKine

int main (int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<RobotCentricController>("robot_centric_controller");
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
