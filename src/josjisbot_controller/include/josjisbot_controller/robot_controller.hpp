#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H
#include <chrono>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <random>

using namespace std_msgs::msg;
using namespace geometry_msgs::msg;

class RobotController {
protected:
	virtual Float32MultiArray inverseKinematics(const TwistStamped& cmd) = 0;
	virtual TwistStamped forwardKinematics(const Float32MultiArray& wheel_cmd) = 0;
	float simulateNoise() {
		unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
		std::default_random_engine noise_generator(seed);
		std::normal_distribution<double> noise(0.0, 0.005);
		return noise(noise_generator);
	}
	double normalizeAngle(double angle) {
		while (angle > M_PI) angle -= 2.0 * M_PI;
		while (angle < -M_PI) angle += 2.0 * M_PI;
		return angle;
	}
};

#endif // !ROBOT_CONTROLLER_H
