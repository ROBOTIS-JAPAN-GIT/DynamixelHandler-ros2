#include "dynamixel_handler.hpp"

void DynamixelHandler::SetupRosInterfacesX() {
    sub_ctrl_x_pwm_ = create_subscription<DynamixelControlXPwm>(
        "dynamixel/command/x/pwm_control", 4,
        [this](const DynamixelControlXPwm& msg) { CallbackCmd_X_Pwm(msg); });
    sub_ctrl_x_cur_ = create_subscription<DynamixelControlXCurrent>(
        "dynamixel/command/x/current_control", 4,
        [this](const DynamixelControlXCurrent& msg) { CallbackCmd_X_Current(msg); });
    sub_ctrl_x_vel_ = create_subscription<DynamixelControlXVelocity>(
        "dynamixel/command/x/velocity_control", 4,
        [this](const DynamixelControlXVelocity& msg) { CallbackCmd_X_Velocity(msg); });
    sub_ctrl_x_pos_ = create_subscription<DynamixelControlXPosition>(
        "dynamixel/command/x/position_control", 4,
        [this](const DynamixelControlXPosition& msg) { CallbackCmd_X_Position(msg); });
    sub_ctrl_x_epos_ = create_subscription<DynamixelControlXExtendedPosition>(
        "dynamixel/command/x/extended_position_control", 4,
        [this](const DynamixelControlXExtendedPosition& msg) { CallbackCmd_X_ExtendedPosition(msg); });
    sub_ctrl_x_cpos_ = create_subscription<DynamixelControlXCurrentBasePosition>(
        "dynamixel/command/x/current_base_position_control", 4,
        [this](const DynamixelControlXCurrentBasePosition& msg) { CallbackCmd_X_CurrentBasePosition(msg); });
}
