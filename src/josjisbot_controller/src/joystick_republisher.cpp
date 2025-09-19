#include "josjisbot_controller/joystick_republisher.hpp"
#include <functional>
#include <memory>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>
using namespace std::chrono_literals;

JoystickRepublisher::JoystickRepublisher(const std::string& name)
: Node(name)
{

	this->declare_parameter("mode", "a"); // a: auto, e: external, t: testing

	// jika menggunakan mode external
	this->declare_parameter("client_ip", "192.162.1.1");
	this->declare_parameter("client_port", 4444);

	// jika menggunakan mode testing
	this->declare_parameter("rx", 126.0);
	this->declare_parameter("ry", 0.0);
	this->declare_parameter("lx", 0.0);
	this->declare_parameter("ly", 0.0);

	this->joy_msgs_.header.set__frame_id("joy");
	this->joy_msgs_.header.set__stamp(this->get_clock()->now());
	this->joy_msgs_.set__axes({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

	char mode = this->get_parameter("mode").as_string().c_str()[0];
	switch (mode) {
		case 'a':
			RCLCPP_INFO(this->get_logger(), "mode auto dipilih");
			this->exec_method_ = ExecMethod::AUTO;
			break;

		case 'e':
			this->client_ip_ = this->get_parameter("client_ip").as_string();
			this->client_port_ = this->get_parameter("client_port").as_int();
			RCLCPP_INFO_STREAM(this->get_logger(),
											"mode external dipilih dengan ip: "
											<< this->client_ip_
											<< " dan port: "
											<< this->client_port_
											);
			this->exec_method_ = ExecMethod::EXTERNAL;
			break;

		case 't':
			this->testing_data_.RX = this->get_parameter("rx").as_double();
			this->testing_data_.RY = this->get_parameter("ry").as_double();
			this->testing_data_.LX = this->get_parameter("lx").as_double();
			this->testing_data_.LY = this->get_parameter("ly").as_double();
			RCLCPP_INFO_STREAM(this->get_logger(),
											"mode testing dipilih dengan data:"
											<< " RX: " << this->testing_data_.RX
											<< " RY: " << this->testing_data_.RY
											<< " LX: " << this->testing_data_.LX
											<< " LY: " << this->testing_data_.LY
											);
			this->exec_method_ = ExecMethod::TESTING;
			break;

		default:
			RCLCPP_ERROR(this->get_logger(), "argumen tidak valid, program akan berjalan menggunakan mode auto");
			this->exec_method_ = ExecMethod::AUTO;
			break;
	}

	this->joy_pub_ = this->create_publisher<Joy>(
		"josjisbot/joy",
		5
	);

	this->timer_ = this->create_wall_timer(
		5ms,
		std::bind(&JoystickRepublisher::timerCallback, this)
	);

}

void JoystickRepublisher::autoModeCallback() {
}

void JoystickRepublisher::externalModeCallback() {

}

void JoystickRepublisher::testingModeCallback() {
	this->joy_msgs_.header.set__stamp(this->get_clock()->now());
	this->joy_msgs_.axes[0] = this->testing_data_.LX;
	this->joy_msgs_.axes[1] = this->testing_data_.LY;
	this->joy_msgs_.axes[2] = this->testing_data_.RX;
	this->joy_msgs_.axes[3] = this->testing_data_.RY;
	this->joy_pub_->publish(this->joy_msgs_);
}

void JoystickRepublisher::timerCallback() {
	switch (this->exec_method_) {
		case ExecMethod::AUTO:
			this->autoModeCallback();
			break;
		case ExecMethod::EXTERNAL:
			this->externalModeCallback();
			break;
		case ExecMethod::TESTING:
			this->testingModeCallback();
			break;
	}
}


int main (int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<JoystickRepublisher>("josjisbot_joy_listener");
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
