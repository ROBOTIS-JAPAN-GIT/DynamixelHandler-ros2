#include "rclcpp/rclcpp.hpp"

#include "dynamixel_handler_msgs/msg/dxl_states.hpp"
#include "dynamixel_handler_msgs/msg/dxl_commands_x.hpp"

#include <chrono>
using namespace std::chrono_literals;

#include <map>
using std::map;
#include <string>
using std::string;

int main() {
    rclcpp::init(0, nullptr);

    auto dxl = dynamixel_handler_msgs::msg::DxlStates();
    auto updated_time = std::map<std::string, rclcpp::Time>();

    auto node  = std::make_shared<rclcpp::Node>("example1_node");
    auto pub_cmd = node->create_publisher<dynamixel_handler_msgs::msg::DxlCommandsX>("dynamixel/commands/x", 10);
    auto sub_st = node->create_subscription<dynamixel_handler_msgs::msg::DxlStates>("dynamixel/states", 10, 
        [&](const dynamixel_handler_msgs::msg::DxlStates::SharedPtr msg){
            if(!msg->status .id_list.empty()) {dxl.status  = msg->status ; updated_time["status" ]=msg->stamp;}
            if(!msg->present.id_list.empty()) {dxl.present = msg->present; updated_time["present"]=msg->stamp;}
            if(!msg->goal   .id_list.empty()) {dxl.goal    = msg->goal   ; updated_time["goal"   ]=msg->stamp;}
            if(!msg->gain   .id_list.empty()) {dxl.gain    = msg->gain   ; updated_time["gain"   ]=msg->stamp;} 
            if(!msg->limit  .id_list.empty()) {dxl.limit   = msg->limit  ; updated_time["limit"  ]=msg->stamp;}
            if(!msg->error  .id_list.empty()) {dxl.error   = msg->error  ; updated_time["error"  ]=msg->stamp;}
    });


    auto timer = node->create_wall_timer(1s, [&](){
        RCLCPP_INFO(node->get_logger(), "Updated time");
        RCLCPP_INFO(node->get_logger(), " status : %f", updated_time["status" ].seconds());
        RCLCPP_INFO(node->get_logger(), " present: %f", updated_time["present"].seconds());
        RCLCPP_INFO(node->get_logger(), " goal   : %f", updated_time["goal"   ].seconds());
        RCLCPP_INFO(node->get_logger(), " gain   : %f", updated_time["gain"   ].seconds());
        RCLCPP_INFO(node->get_logger(), " limit  : %f", updated_time["limit"  ].seconds());
        RCLCPP_INFO(node->get_logger(), " error  : %f", updated_time["error"  ].seconds());

        auto cmd = dynamixel_handler_msgs::msg::DxlCommandsX();
    });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
