#include "rclcpp/rclcpp.hpp"

#include "dynamixel_handler_msgs/msg/dxl_states.hpp"
#include "dynamixel_handler_msgs/msg/dxl_commands_x.hpp"
using namespace dynamixel_handler_msgs::msg;

#include <map>
#include <chrono>
using namespace std::chrono_literals;

int main() {
    rclcpp::init(0, nullptr);
    auto node  = std::make_shared<rclcpp::Node>("example1_node");

    rclcpp::Time updated_time;
    std::map<uint8_t, double> dxl_pos, dxl_vel, dxl_cur;
    auto sub_st = node->create_subscription<DxlStates>("dynamixel/states", 10, 
        [&](const DxlStates::SharedPtr msg){
            for (size_t i = 0; i < msg->status.id_list.size(); i++) {
                RCLCPP_INFO(node->get_logger(), "- servo [%d], torque %s, has %s, ping is %s, mode is %s", 
                    msg->status.id_list[i],
                    msg->status.torque[i] ? "on" : "off",
                    msg->status.error[i] ? "error" : "no error",
                    msg->status.ping[i] ? "response" : "no response",
                    msg->status.mode[i].c_str()
                ); 
            }

            if(!msg->present.id_list.empty()) updated_time = msg->stamp;
            for (size_t i = 0; i < msg->present.id_list.size(); i++) {
                auto id = msg->present.id_list[i];
                dxl_pos[id] = msg->present.position_deg[i];
                dxl_vel[id] = msg->present.velocity_deg_s[i];
                dxl_cur[id] = msg->present.current_ma[i];
            }
    });

    auto pub_cmd = node->create_publisher<DxlCommandsX>("dynamixel/commands/x", 10);
    auto timer = node->create_wall_timer(1.0s, [&](){
        RCLCPP_INFO(node->get_logger(), "* Present value updated time %f", updated_time.seconds());
        for (const auto& [id, _] : dxl_pos) {
            RCLCPP_INFO(node->get_logger(), "* servo [%d], pos %f, vel %f, cur %f", id, dxl_pos[id], dxl_vel[id], dxl_cur[id]);
        }

        auto cmd = DxlCommandsX();
        for (const auto& [id, pos] : dxl_pos) {
            // トルクをオンに．
            cmd.status.id_list.push_back(id);
            cmd.status.torque.push_back(true);
            // 電流を300mAに制限しつつ， +-45degで往復運動させる．
            auto target = (pos < 0) ? 45 : -45;
            auto& cmd_cpos = cmd.current_base_position_control;
            cmd_cpos.id_list.push_back(id);
            cmd_cpos.current_ma.push_back(300/*mA*/);
            cmd_cpos.position_deg.push_back(target/*deg*/);
        }
        if (!cmd.status.id_list.empty()) pub_cmd->publish(cmd);
    });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
