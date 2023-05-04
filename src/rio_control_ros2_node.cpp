#include "rio_control_ros2_node/rio_control_ros2_node.hpp"
#include <cstdio>
#include <string>
#include <iostream>
#include <vector>
#include <thread>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

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
