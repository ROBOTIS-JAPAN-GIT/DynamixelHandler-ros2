#include "dynamixel_handler.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_p_current.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_p_extended_position.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_p_position.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_p_pwm.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_p_velocity.hpp"

void DynamixelHandler::SetupRosInterfacesP() {
    sub_ctrl_p_pwm_ = create_subscription<DynamixelControlPPwm>(
        "dynamixel/command/p/pwm_control", 4,
        [this](const DynamixelControlPPwm& msg) { CallbackCmd_P_Pwm(msg); });
    sub_ctrl_p_cur_ = create_subscription<DynamixelControlPCurrent>(
        "dynamixel/command/p/current_control", 4,
        [this](const DynamixelControlPCurrent& msg) { CallbackCmd_P_Current(msg); });
    sub_ctrl_p_vel_ = create_subscription<DynamixelControlPVelocity>(
        "dynamixel/command/p/velocity_control", 4,
        [this](const DynamixelControlPVelocity& msg) { CallbackCmd_P_Velocity(msg); });
    sub_ctrl_p_pos_ = create_subscription<DynamixelControlPPosition>(
        "dynamixel/command/p/position_control", 4,
        [this](const DynamixelControlPPosition& msg) { CallbackCmd_P_Position(msg); });
    sub_ctrl_p_epos_ = create_subscription<DynamixelControlPExtendedPosition>(
        "dynamixel/command/p/extended_position_control", 4,
        [this](const DynamixelControlPExtendedPosition& msg) { CallbackCmd_P_ExtendedPosition(msg); });
}
