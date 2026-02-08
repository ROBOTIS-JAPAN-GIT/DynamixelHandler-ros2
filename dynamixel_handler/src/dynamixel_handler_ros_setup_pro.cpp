#include "dynamixel_handler.hpp"

#include "dynamixel_handler_msgs/msg/dynamixel_control_pro_current.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_pro_extended_position.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_pro_position.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_pro_velocity.hpp"

using std::bind;
using std::placeholders::_1;

void DynamixelHandler::SetupRosInterfacesPro() {
    sub_ctrl_pro_cur_ = create_subscription<DynamixelControlProCurrent>(
        "dynamixel/command/pro/current_control", 4, bind(&DynamixelHandler::CallbackCmd_Pro_Current, this, _1));
    sub_ctrl_pro_vel_ = create_subscription<DynamixelControlProVelocity>(
        "dynamixel/command/pro/velocity_control", 4, bind(&DynamixelHandler::CallbackCmd_Pro_Velocity, this, _1));
    sub_ctrl_pro_pos_ = create_subscription<DynamixelControlProPosition>(
        "dynamixel/command/pro/position_control", 4, bind(&DynamixelHandler::CallbackCmd_Pro_Position, this, _1));
    sub_ctrl_pro_epos_ = create_subscription<DynamixelControlProExtendedPosition>(
        "dynamixel/command/pro/extended_position_control", 4, bind(&DynamixelHandler::CallbackCmd_Pro_ExtendedPosition, this, _1));
}
