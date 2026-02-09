#include "dynamixel_handler.hpp"

void DynamixelHandler::SetupRosInterfacesPro() {
    sub_ctrl_pro_cur_ = create_subscription<DynamixelControlProCurrent>(
        "dynamixel/command/pro/current_control", 4,
        [this](const DynamixelControlProCurrent& msg) { CallbackCmd_Pro_Current(msg); });
    sub_ctrl_pro_vel_ = create_subscription<DynamixelControlProVelocity>(
        "dynamixel/command/pro/velocity_control", 4,
        [this](const DynamixelControlProVelocity& msg) { CallbackCmd_Pro_Velocity(msg); });
    sub_ctrl_pro_pos_ = create_subscription<DynamixelControlProPosition>(
        "dynamixel/command/pro/position_control", 4,
        [this](const DynamixelControlProPosition& msg) { CallbackCmd_Pro_Position(msg); });
    sub_ctrl_pro_epos_ = create_subscription<DynamixelControlProExtendedPosition>(
        "dynamixel/command/pro/extended_position_control", 4,
        [this](const DynamixelControlProExtendedPosition& msg) { CallbackCmd_Pro_ExtendedPosition(msg); });
}
