#include "josjisbot_controller/robot_centric_controller.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <math.h>

#define PI M_PI

using namespace std::chrono_literals;
using namespace std::placeholders;

RobotCentricController::RobotCentricController(const std::string& name)
: Node(name)
, x_(0.0)
, y_(0.0)
, heading_(0.0)
{
	this->wheel_cmd_pub_ = this->create_publisher<Float32MultiArray>(
		"/josjisbot_controllers/commands",
		5
	);

	this->cmd_sub_ = this->create_subscription<TwistStamped>(
		"/cmd_vel",
		5,
		std::bind(&RobotCentricController::cmdCallback, this, _1)
	);
	this->wheel_cmd_.set__data({0.0, 0.0, 0.0, 0.0});
}

void RobotCentricController::cmdCallback(const TwistStamped& msg)
{
	this->wheel_cmd_ = this->inverseKinematics(msg);

	this->x_ += msg.twist.linear.x + simulateNoise();
	this->y_ += msg.twist.linear.y + simulateNoise();
	this->heading_ += msg.twist.angular.z + simulateNoise();
	this->heading_ += msg.twist.angular.z + simulateNoise();
	this->heading_ = this->normalizeAngle(this->heading_);


	RCLCPP_INFO_STREAM(this->get_logger(),
										"x: " << this->x_ <<
										" y: " << this->y_ <<
										" w: " << this->heading_
										);

	this->wheel_cmd_pub_->publish(this->wheel_cmd_);
}

Float32MultiArray RobotCentricController::inverseKinematics(const TwistStamped& cmd)
{
	auto wheel_cmd = Float32MultiArray();
	wheel_cmd.data = {0,0,0,0};

	for (int i = 0; i < 4; i++) {
		wheel_cmd.data[i] = (
			// x dan y dibalik karena ROS menggunakan REP 103
			-sin(PI/180 * this->wheel_angles_[i]) * cmd.twist.linear.y +
			cos(PI/180 * this->wheel_angles_[i]) * cmd.twist.linear.x +
			cmd.twist.angular.z
		);
	}
	for (int i  = 0; i < 4; i++) {
		RCLCPP_INFO(this->get_logger(), "w%d: (%f) %f", i, wheel_angles_[i], wheel_cmd_.data[i]);
	}
	return wheel_cmd;
}

TwistStamped  RobotCentricController::forwardKinematics(const Float32MultiArray& wheel_cmd)
{
	auto cmd = TwistStamped();
	float vy = (
		-sin(PI/180 * this->wheel_angles_[0]) * wheel_cmd.data[0]+
		-sin(PI/180 * this->wheel_angles_[1]) * wheel_cmd.data[1]+
		-sin(PI/180 * this->wheel_angles_[2]) * wheel_cmd.data[2]+
		-sin(PI/180 * this->wheel_angles_[3]) * wheel_cmd.data[3] 
	)/2;

	float vx = (
		cos(PI/180 * this->wheel_angles_[0]) * wheel_cmd.data[0]+
		cos(PI/180 * this->wheel_angles_[1]) * wheel_cmd.data[1]+
		cos(PI/180 * this->wheel_angles_[2]) * wheel_cmd.data[2]+
		cos(PI/180 * this->wheel_angles_[3])* wheel_cmd.data[3] 
	)/2;
	float omega = (wheel_cmd.data[0] + wheel_cmd.data[1] + wheel_cmd.data[2] + wheel_cmd.data[3])/4;
	RCLCPP_INFO_STREAM(this->get_logger(),
										" vx: " << vx << 
										" vy: " << vy << 
										" omega: " << omega
										);
	cmd.twist.linear.x = vx;
	cmd.twist.linear.y = vy;
	cmd.twist.angular.z = omega;
	return cmd;
}

// Float32MultiArray RobotCetricController::inverseKine

int main (int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<RobotCentricController>("robot_centric_controller");
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
