#include "rio_control_ros2_node/rio_control_ros2_node.hpp"
#include <cstdio>
#include <string>
#include <iostream>
#include <vector>
#include <thread>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.h"
#include "std_msgs/msg/float32.h"

#define ZMQ_BUILD_DRAFT_API

#include <zmq.h>
#include <thread>
#include <string>
#include <map>
#include <mutex>
#include <iostream>
#include <atomic>
#include <list>
#include <signal.h>
#include <math.h>

#include "RobotStatus.pb.h"
#include "JoystickStatus.pb.h"
#include "SolenoidControl.pb.h"
#include "SolenoidStatus.pb.h"
#include "MotorControl.pb.h"
#include "MotorStatus.pb.h"
#include "MotorConfiguration.pb.h"
#include "IMUConfig.pb.h"
#include "IMUData.pb.h"
#include "EncoderConfig.pb.h"
#include "EncoderData.pb.h"
#include "LEDControl.pb.h"

#include <ck_ros2_base_msgs_node/msg/joystick_status.hpp>
#include <ck_ros2_base_msgs_node/msg/joystick_status_array.hpp>
#include <ck_ros2_base_msgs_node/msg/robot_status.hpp>
#include <ck_ros2_base_msgs_node/msg/solenoid_control.hpp>
#include <ck_ros2_base_msgs_node/msg/solenoid_control_array.hpp>


#define NODE_NAME "rio_control_ros2_node"

class LocalNode : public ParameterizedNode
{
public:
    LocalNode() : ParameterizedNode(NODE_NAME)
    {

    }

    ~LocalNode()
    {

    }

private:


};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
