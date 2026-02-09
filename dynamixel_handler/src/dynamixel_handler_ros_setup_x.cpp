#include "dynamixel_handler.hpp"
#include "dynamixel_handler_msgs/msg/dxl_commands_x.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_x_current.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_x_current_base_position.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_x_extended_position.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_x_position.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_x_pwm.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_x_velocity.hpp"
#include "myUtils/generic_subscription_helper.hpp"

void DynamixelHandler::SetupRosInterfacesX(bool no_use_command_line) {
    sub_dxl_x_cmds_ = create_subscription<DxlCommandsX>(
        "dynamixel/commands/x", 10,
        [this](std::shared_ptr<DxlCommandsX> msg) { CallbackCmdsX(msg); });

    if ( no_use_command_line ) return;

    sub_ctrl_x_pwm_ = dynamixel_handler_detail::CreateGenericSubscriptionWithTypedCallback<DynamixelControlXPwm>(
        this, "dynamixel/command/x/pwm_control", "dynamixel_handler_msgs/msg/DynamixelControlXPwm", 4,
        [this](const DynamixelControlXPwm& msg) { CallbackCmd_X_Pwm(msg); });
    sub_ctrl_x_cur_ = dynamixel_handler_detail::CreateGenericSubscriptionWithTypedCallback<DynamixelControlXCurrent>(
        this, "dynamixel/command/x/current_control", "dynamixel_handler_msgs/msg/DynamixelControlXCurrent", 4,
        [this](const DynamixelControlXCurrent& msg) { CallbackCmd_X_Current(msg); });
    sub_ctrl_x_vel_ = dynamixel_handler_detail::CreateGenericSubscriptionWithTypedCallback<DynamixelControlXVelocity>(
        this, "dynamixel/command/x/velocity_control", "dynamixel_handler_msgs/msg/DynamixelControlXVelocity", 4,
        [this](const DynamixelControlXVelocity& msg) { CallbackCmd_X_Velocity(msg); });
    sub_ctrl_x_pos_ = dynamixel_handler_detail::CreateGenericSubscriptionWithTypedCallback<DynamixelControlXPosition>(
        this, "dynamixel/command/x/position_control", "dynamixel_handler_msgs/msg/DynamixelControlXPosition", 4,
        [this](const DynamixelControlXPosition& msg) { CallbackCmd_X_Position(msg); });
    sub_ctrl_x_epos_ = dynamixel_handler_detail::CreateGenericSubscriptionWithTypedCallback<DynamixelControlXExtendedPosition>(
        this, "dynamixel/command/x/extended_position_control", "dynamixel_handler_msgs/msg/DynamixelControlXExtendedPosition", 4,
        [this](const DynamixelControlXExtendedPosition& msg) { CallbackCmd_X_ExtendedPosition(msg); });
    sub_ctrl_x_cpos_ = dynamixel_handler_detail::CreateGenericSubscriptionWithTypedCallback<DynamixelControlXCurrentBasePosition>(
        this, "dynamixel/command/x/current_base_position_control", "dynamixel_handler_msgs/msg/DynamixelControlXCurrentBasePosition", 4,
        [this](const DynamixelControlXCurrentBasePosition& msg) { CallbackCmd_X_CurrentBasePosition(msg); });
}
