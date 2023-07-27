#include "rclcpp/rclcpp.hpp"
#include "bfc_msgs/msg/button.hpp"
#include "std_msgs/msg/string.hpp"
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

rclcpp::Subscription<bfc_msgs::msg::Button>::SharedPtr button_;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr motion_;

void runLuaProgram() { system("cd /home/tegra/bfc_ros2/src/Player; screen -S dcm lua run_dcm.lua; screen -S player lua walk_server.lua;");}

int last_state = 0;
void button_callback(const bfc_msgs::msg::Button::SharedPtr msg)
{
    auto msg_mot = std_msgs::msg::String();
        if (msg->kill != last_state)
        {
            if (msg->kill == 1)
            {
                if (msg->strategy == 4)
                {
                    system("killall screen");
                }
            }
            else if (msg->kill == 0)
            {
                if (msg->strategy == 4)
                {
                    system("killall screen");
                    sleep(1);
                    runLuaProgram();
                    // msg_mot.data = "8";
                } else {
                    // msg_mot.data = "0";
                }
            }
            motion_->publish(msg_mot);
            last_state = msg->kill;
        }
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    system("killall screen");
    auto node = std::make_shared<rclcpp::Node>("run_lua");
    button_ = node->create_subscription<bfc_msgs::msg::Button>("/robot_3/button", 1, button_callback);
    motion_ = node->create_publisher<std_msgs::msg::String>("/robot_3/motion", 1);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}