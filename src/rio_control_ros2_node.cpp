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

#include <ck_ros2_base_msgs_node/msg/joystick_status.hpp>
#include <ck_ros2_base_msgs_node/msg/joystick_status_array.hpp>
#include <ck_ros2_base_msgs_node/msg/robot_status.hpp>
#include <ck_ros2_base_msgs_node/msg/solenoid_control.hpp>
#include <ck_ros2_base_msgs_node/msg/solenoid_control_array.hpp>

#include "ck_utilities_ros2_node/node_handle.hpp"

#define NODE_NAME "rio_control_ros2_node"

const std::vector<std::string> ZMQ_TOPICS = {"robotstatus", "joystickstatus"};

constexpr unsigned int str2int(const char *str, int h = 0)
{
	return !str[h] ? 5381 : (str2int(str, h + 1) * 33) ^ str[h];
}

class SolenoidTracker
{
public:
	ck_ros2_base_msgs_node::msg::SolenoidControl solenoid;
	rclcpp::Time active_time;
};

class LocalNode : public ParameterizedNode
{
public:
    LocalNode() : ParameterizedNode(NODE_NAME)
    {
        context = zmq_ctx_new();

        solenoid_control_subscriber = this->create_subscription<ck_ros2_base_msgs_node::msg::SolenoidControlArray>("/SolenoidControl", 1, std::bind(&LocalNode::solenoid_control_msg_callback, this, std::placeholders::_1));
        robot_status_publisher = this->create_publisher<ck_ros2_base_msgs_node::msg::RobotStatus>("/RobotStatus", 10);
        joystick_status_publisher = this->create_publisher<ck_ros2_base_msgs_node::msg::JoystickStatusArray>("/JoystickStatus", 10);

        rio_receive_thread = std::thread(std::bind(&LocalNode::rio_receive, this));
        rio_send_thread = std::thread(std::bind(&LocalNode::rio_send, this));
    }

    ~LocalNode()
    {

    }

private:
    rclcpp::Subscription<ck_ros2_base_msgs_node::msg::SolenoidControlArray>::SharedPtr solenoid_control_subscriber;
    rclcpp::Publisher<ck_ros2_base_msgs_node::msg::RobotStatus>::SharedPtr robot_status_publisher;
    rclcpp::Publisher<ck_ros2_base_msgs_node::msg::JoystickStatusArray>::SharedPtr joystick_status_publisher;

    std::thread rio_receive_thread;
    std::thread rio_send_thread;

    void* context;

    std::recursive_mutex solenoid_control_mutex;
    std::map<int32_t, SolenoidTracker> solenoid_control_map;

    void solenoid_control_msg_callback(const ck_ros2_base_msgs_node::msg::SolenoidControlArray msg)
    {
        std::scoped_lock<std::recursive_mutex> lock(solenoid_control_mutex);
        for (size_t i = 0; i < msg.solenoids.size(); i++)
        {
            ck_ros2_base_msgs_node::msg::SolenoidControl updated_solenoid;
            updated_solenoid.id = msg.solenoids[i].id;
            updated_solenoid.output_value = msg.solenoids[i].output_value;
            updated_solenoid.module_type = msg.solenoids[i].module_type;
            updated_solenoid.solenoid_type = msg.solenoids[i].solenoid_type;

            SolenoidTracker updated_tracked_solenoid;
            updated_tracked_solenoid.solenoid = updated_solenoid;
            updated_tracked_solenoid.active_time = this->get_clock()->now() + rclcpp::Duration::from_seconds(Parameters.roborio_control_timeout);

            solenoid_control_map[msg.solenoids[i].id] = updated_tracked_solenoid;
        }
    }

    void rio_receive()
    {
        void *subscriber = zmq_socket(context, ZMQ_DISH);

        bool success = false;
        if (zmq_bind(subscriber, "udp://*:5801") >= 0)
        {
            success = true;
            for (const std::string& s : ZMQ_TOPICS)
            {
                success &= zmq_join(subscriber, s.c_str()) >= 0;
            }
        }

        if (!success)
        {
            std::string error_msg = "Failed to bind/subscribe to topics";
            RCLCPP_ERROR(this->get_logger(), error_msg.c_str());
            throw std::runtime_error(error_msg);
        }

        char buffer[10000];
        memset(buffer, 0, 10000);

        while (rclcpp::ok())
        {
            zmq_msg_t message;
            zmq_msg_init(&message);
            zmq_msg_recv(&message, subscriber, 0);

            std::string message_group(zmq_msg_group(&message));

            switch (str2int(message_group.c_str()))
            {
                case str2int("joystickstatus"):
                {
                    process_joystick_status(message);
                    break;
                }
                case str2int("robotstatus"):
                {
                    process_robot_status(message);
                    break;
                }
                default:
                {
                    RCLCPP_ERROR(this->get_logger(), "Got unrecognized message: %s", message_group.c_str());
                    break;
                }
            }

            zmq_msg_close(&message);
        }
    }

    void process_joystick_status(zmq_msg_t &message)
    {
        static ck::JoystickStatus status_protobuf;

        void *data = zmq_msg_data(&message);
        bool parse_result = status_protobuf.ParseFromArray(data, zmq_msg_size(&message));
        if (parse_result)
        {

            ck_ros2_base_msgs_node::msg::JoystickStatusArray joystick_status_array;

            for (int i = 0; i < status_protobuf.joysticks_size(); i++)
            {
                const ck::JoystickStatus::Joystick &joystick = status_protobuf.joysticks(i);
                ck_ros2_base_msgs_node::msg::JoystickStatus stick;

                stick.index = joystick.index();

                uint32_t buttons = joystick.buttons();
                for (int j = 0; j < 32; j++)
                {
                    stick.buttons.push_back((buttons >> j) & 0x0001);
                }

                for (int j = 0; j < joystick.axes_size(); j++)
                {
                    stick.axes.push_back(joystick.axes(j));
                }

                for (int j = 0; j < joystick.povs_size(); j++)
                {
                    stick.povs.push_back(joystick.povs(j));
                }

                joystick_status_array.joysticks.push_back(stick);
            }
            joystick_status_publisher->publish(joystick_status_array);
        }
    }

    void process_robot_status(zmq_msg_t &message)
    {
        static ck::RobotStatus status_protobuf;

        void *data = zmq_msg_data(&message);
        bool parse_result = status_protobuf.ParseFromArray(data, zmq_msg_size(&message));

        if (parse_result)
        {
            ck_ros2_base_msgs_node::msg::RobotStatus robot_status;
            robot_status.alliance = status_protobuf.alliance();
            robot_status.robot_state = status_protobuf.robot_state();
            robot_status.match_time = status_protobuf.match_time();
            robot_status.game_data = status_protobuf.game_data().c_str();
            robot_status.selected_auto = status_protobuf.selected_auto();
            robot_status.is_connected = status_protobuf.is_connected();
            robot_status_publisher->publish(robot_status);
        }
    }

    void rio_send()
    {
        void *publisher = zmq_socket(context, ZMQ_RADIO);

        if (zmq_connect(publisher, Parameters.roborio_connection_string.c_str()) < 0)
        {
            std::string error_msg = "Failed to initialize solenoid publisher";
            RCLCPP_ERROR(this->get_logger(), error_msg.c_str());
            throw std::runtime_error(error_msg);
        }

        char buffer[10000];
        memset(buffer, 0, 10000);

        rclcpp::Rate rate(100);

        while (rclcpp::ok())
        {
            {
                std::lock_guard<std::recursive_mutex> lock(solenoid_control_mutex);
                static ck::SolenoidControl solenoid_control;
                solenoid_control.clear_solenoids();
                solenoid_control.Clear();
                solenoid_control.set_compressor_is_enabled_for_auto(Parameters.compressor_enabled_in_auto);

                std::vector<std::map<int32_t, SolenoidTracker>::iterator> timed_out_solenoid_list;

                for (std::map<int32_t, SolenoidTracker>::iterator i = solenoid_control_map.begin();
                    i != solenoid_control_map.end();
                    i++)
                {
                    ck::SolenoidControl::Solenoid *new_solenoid = solenoid_control.add_solenoids();

                    new_solenoid->set_id((*i).second.solenoid.id);
                    new_solenoid->set_module_type((ck::SolenoidControl::Solenoid::ModuleType)(*i).second.solenoid.module_type);
                    new_solenoid->set_solenoid_type((ck::SolenoidControl::Solenoid::SolenoidType)(*i).second.solenoid.solenoid_type);
                    new_solenoid->set_output_value((ck::SolenoidControl::Solenoid::SolenoidValue)((*i).second.solenoid.output_value));

                    if ((*i).second.active_time < this->get_clock()->now())
                    {
                        timed_out_solenoid_list.push_back(i);
                    }
                }

                for (std::vector<std::map<int32_t, SolenoidTracker>::iterator>::iterator i = timed_out_solenoid_list.begin();
                    i != timed_out_solenoid_list.end();
                    i++)
                {
                    solenoid_control_map.erase((*i));
                }

                bool serialize_status = solenoid_control.SerializeToArray(buffer, 10000);

                if (!serialize_status)
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to serialize solenoid status!!");
                }
                else
                {
                    zmq_msg_t message;
                    zmq_msg_init_size(&message, solenoid_control.ByteSizeLong());
                    memcpy(zmq_msg_data(&message), buffer, solenoid_control.ByteSizeLong());
                    zmq_msg_set_group(&message, "solenoidcontrol");
                    // std::cout << "Sending message..." << std::endl;
                    zmq_msg_send(&message, publisher, 0);
                    zmq_msg_close(&message);
                }
            }

            rate.sleep();
        }
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
