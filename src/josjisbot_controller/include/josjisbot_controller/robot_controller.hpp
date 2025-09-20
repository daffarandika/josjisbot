#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

using namespace std_msgs::msg;
using namespace geometry_msgs::msg;

class RobotController {
protected:
	virtual Float32MultiArray inverseKinematics(const TwistStamped& cmd) = 0;
	virtual TwistStamped forwardKinematics(const Float32MultiArray& wheel_cmd) = 0;
};

#endif // !ROBOT_CONTROLLER_H
