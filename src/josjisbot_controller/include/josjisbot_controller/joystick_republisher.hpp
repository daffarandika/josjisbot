#ifndef JOYSTICK_H
#define JOYSTICK_H
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

using Joy = sensor_msgs::msg::Joy;

// Opsi metode menjalankan node ini
enum ExecMethod {
	/* Dalam mode auto, node akan mempublish LX, LY, RX, dan RY dari range -128 sampai 128 secara berkala */
	AUTO,
	/* Dalam mode external, node akan bertindak sebagai UDP server dan data yang diterima dari UDP akan dipublish */
	EXTERNAL,
	/* Dalam mode testing, node akan mempublish data statis yang diberikan sebagai parameter node, mode ini digunakan untuk mengecek apakah hasil perhitungan kinematika sudah benar atau belum */
	TESTING
};

typedef struct {
	double RX;
	double RY;
	double LX;
	double LY;
} JoystickTestData;

class JoystickRepublisher : public rclcpp::Node {

public:
	JoystickRepublisher(const std::string& name);

private:
	void timerCallback();
	void autoModeCallback();
	void externalModeCallback();
	void testingModeCallback();

	rclcpp::Publisher<Joy>::SharedPtr joy_pub_;
	rclcpp::TimerBase::SharedPtr timer_;
	Joy joy_msgs_;
	ExecMethod exec_method_;

	std::string client_ip_;
	int client_port_;
	JoystickTestData testing_data_;

};

#endif // !JOYSTICK_H
