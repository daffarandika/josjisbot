#include "josjisbot_controller/robot_centric_controller.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <string>

#define CURRENT_TIME this->get_clock()->now()

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

}

Float32MultiArray RobotCentricController::inverseKinematics(const TwistStamped& cmd)
{
	return Float32MultiArray();
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
