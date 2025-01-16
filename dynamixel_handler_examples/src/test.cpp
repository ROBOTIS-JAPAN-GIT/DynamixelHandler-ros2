#include "rclcpp/rclcpp.hpp"

#include "dynamixel_handler_msgs/msg/dxl_states.hpp"
#include "dynamixel_handler_msgs/msg/dxl_commands_x.hpp"
#include "dynamixel_handler_msgs/msg/dxl_commands_p.hpp"
#include "dynamixel_handler_msgs/msg/dxl_commands_all.hpp"
#include "dynamixel_handler_msgs/msg/dxl_external_port.hpp"

#include "dynamixel_handler_msgs/msg/dynamixel_shortcut.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_debug.hpp"

#include <chrono>
using namespace std::chrono_literals;

#include <map>
#include <string>

template <typename... Args>
void ROS_INFO(const rclcpp::Node::SharedPtr node,  const char* format, Args&&... args) {
    RCLCPP_INFO(node->get_logger(), format, std::forward<Args>(args)...);
};

template <typename... Args>
void ROS_ERROR(const rclcpp::Node::SharedPtr node,  const char* format, Args&&... args) {
    RCLCPP_ERROR(node->get_logger(), format, std::forward<Args>(args)...);
};

int main() {
    rclcpp::init(0, nullptr);

    auto dxl = dynamixel_handler_msgs::msg::DxlStates();
    auto updated_time = std::map<std::string, rclcpp::Time>();

    auto node  = std::make_shared<rclcpp::Node>("test_node");
    auto pub_cmd = node->create_publisher<dynamixel_handler_msgs::msg::DxlCommandsX>("dynamixel/commands/x", 10);
    auto pub_sht = node->create_publisher<dynamixel_handler_msgs::msg::DynamixelShortcut>("dynamixel/shortcut", 10);
    auto sub_states = node->create_subscription<dynamixel_handler_msgs::msg::DxlStates>("dynamixel/states", 10, 
        [&](const dynamixel_handler_msgs::msg::DxlStates::SharedPtr msg){
            if ( !updated_time.count("start") ) updated_time["start"] = msg->stamp;
            if(!msg->status .id_list.empty()) {dxl.status  = msg->status ; updated_time["status" ]=msg->stamp;}
            if(!msg->present.id_list.empty()) {dxl.present = msg->present; updated_time["present"]=msg->stamp;}
            if(!msg->goal   .id_list.empty()) {dxl.goal    = msg->goal   ; updated_time["goal"   ]=msg->stamp;}
            if(!msg->gain   .id_list.empty()) {dxl.gain    = msg->gain   ; updated_time["gain"   ]=msg->stamp;} 
            if(!msg->limit  .id_list.empty()) {dxl.limit   = msg->limit  ; updated_time["limit"  ]=msg->stamp;}
            if(!msg->error  .id_list.empty()) {dxl.error   = msg->error  ; updated_time["error"  ]=msg->stamp;}
    });


    auto timer = node->create_wall_timer(1s, [&](){
        if (
            updated_time["status" ].seconds() == 0 ||
            updated_time["present"].seconds() == 0 ||
            updated_time["goal"   ].seconds() == 0 ||
            updated_time["gain"   ].seconds() == 0 ||
            updated_time["limit"  ].seconds() == 0 ||
            updated_time["error"  ].seconds() == 0 
        ) {
            RCLCPP_INFO(node->get_logger(), "Updated time");
            RCLCPP_INFO(node->get_logger(), " status : %f", updated_time["status" ].seconds()-updated_time["start" ].seconds());
            RCLCPP_INFO(node->get_logger(), " present: %f", updated_time["present"].seconds()-updated_time["start" ].seconds());
            RCLCPP_INFO(node->get_logger(), " goal   : %f", updated_time["goal"   ].seconds()-updated_time["start" ].seconds());
            RCLCPP_INFO(node->get_logger(), " gain   : %f", updated_time["gain"   ].seconds()-updated_time["start" ].seconds());
            RCLCPP_INFO(node->get_logger(), " limit  : %f", updated_time["limit"  ].seconds()-updated_time["start" ].seconds());
            RCLCPP_INFO(node->get_logger(), " error  : %f", updated_time["error"  ].seconds()-updated_time["start" ].seconds());
            return;
        } 
        auto sht = dynamixel_handler_msgs::msg::DynamixelShortcut();
        auto cmd_x = dynamixel_handler_msgs::msg::DxlCommandsX();

        static size_t cnt = 0;
        if ( cnt == 0 ) {
            ROS_INFO(node, "START TEST");
            ROS_INFO(node, "Red characters are errors");
        }

        RCLCPP_INFO(node->get_logger(), " ");
        switch ( cnt++ % 100 ) {
            // # トルクのオンオフ
            // ##  shortcut msg でまとめてオフ
            case 0: RCLCPP_INFO(node->get_logger(), "torque, shortcut で全てOFF");
                sht.set__command(sht.TORQUE_OFF);
                pub_sht->publish(sht); return;
            case 1: // ## 結果の確認
                for (size_t i=0; i<dxl.status.id_list.size(); i++) {
                    (dxl.status.torque[i]) ? ROS_ERROR(node, "ID [%d] torque is ON", dxl.status.id_list[i])
                                           : ROS_INFO(node, "ID [%d] torque is OFF", dxl.status.id_list[i]);
                } return;
            // command で 個別に オンオフ
            case 2: ROS_INFO(node, "torque, command で個別にON");
                cmd_x.status.set__id_list({1, 6, 3});
                cmd_x.status.set__torque({true, true, true});
                pub_cmd->publish(cmd_x); return;
            case 3: // ## 結果の確認
                for (size_t i=0; i<dxl.status.id_list.size(); i++) {
                    auto id = dxl.status.id_list[i];
                    if ( dxl.status.torque[i] ) 
                        ( id==1 || id==6 || id==3 ) ? ROS_INFO(node, "ID [%d] torque is ON", id)
                                                    : ROS_ERROR(node, "ID [%d] torque is ON", id);
                    else 
                        ( id==1 || id==6 || id==3 ) ? ROS_ERROR(node, "ID [%d] torque is OFF", id)
                                                    : ROS_INFO(node, "ID [%d] torque is OFF", id);
                } return;
            // # error の解除 能動的に発生させられないからなぁ
            // ## shortcut msg でまとめて解除
            case 4: RCLCPP_INFO(node->get_logger(), "error, shortcut で全て解除");
                sht.set__command(sht.CLEAR_ERROR);
                pub_sht->publish(sht); return;
            case 5: // ## 結果の確認
                for (size_t i=0; i<dxl.status.id_list.size(); i++) {
                    (dxl.status.error[i]) ? ROS_ERROR(node, "ID [%d] has error", dxl.status.id_list[i])
                                          : ROS_INFO(node, "ID [%d] has no error", dxl.status.id_list[i]);
                } return;
            // ## command で個別に解除
            case 6: ROS_INFO(node, "error, command で個別に解除");
                cmd_x.status.set__id_list({1, 6, 3});
                cmd_x.status.set__error({false, false, false});
                pub_cmd->publish(cmd_x); return;
            case 7: // ## 結果の確認
                for (size_t i=0; i<dxl.status.id_list.size(); i++) {
                    auto id = dxl.status.id_list[i];
                    if ( dxl.status.error[i] ) ROS_ERROR(node, "ID [%d] has error", id);
                    else                       ROS_INFO(node, "ID [%d] has no error", id);
                } return;
            // # ping の追加 / 削除
            // ## shortcut msg で個別に削除
            case 8: RCLCPP_INFO(node->get_logger(), "ping, shortcut で 1, 2, 7 を削除");
                sht.set__command(sht.REMOVE_ID).set__id_list({1, 2, 7});
                pub_sht->publish(sht); return;
            case 9: // ## 結果の確認
                for (size_t i=0; i<dxl.status.id_list.size(); i++) {
                    auto id = dxl.status.id_list[i];
                    if ( dxl.status.ping[i] ) 
                        ( id==1 || id==2 || id==7 ) ? ROS_ERROR(node, "ID [%d] is added", id)
                                                    : ROS_INFO(node, "ID [%d] is added", id);
                    else 
                        ( id==1 || id==2 || id==7 ) ? ROS_INFO(node, "ID [%d] is not added", id)
                                                    : ROS_ERROR(node, "ID [%d] is not added", id);
                } return;
            // ## command で一部追加
            case 10: ROS_INFO(node, "ping, command で一部追加");
                cmd_x.status.set__id_list({1, 2, 6, 7});
                cmd_x.status.set__ping({true, true, true, true});
                pub_cmd->publish(cmd_x); return;
            case 11: // ## 結果の確認
                for (size_t i=0; i<dxl.status.id_list.size(); i++) {
                    auto id = dxl.status.id_list[i];
                    if ( dxl.status.ping[i] ) 
                        ( id==1 || id==2 || id==6 || id==7 ) ? ROS_INFO(node, "ID [%d] is added", id)
                                                             : ROS_ERROR(node, "ID [%d] is added", id);
                    else 
                        ( id==1 || id==2 || id==6 || id==7 ) ? ROS_ERROR(node, "ID [%d] is not added", id)
                                                             : ROS_INFO(node, "ID [%d] is not added", id);
                } return;


            default: ROS_INFO(node, "END_TEST");
        }

    });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
