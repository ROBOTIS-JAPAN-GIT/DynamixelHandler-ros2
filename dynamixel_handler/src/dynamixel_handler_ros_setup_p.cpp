#include "dynamixel_handler.hpp"
#include "dynamixel_handler_msgs/msg/dxl_commands_p.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_p_current.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_p_extended_position.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_p_position.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_p_pwm.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_p_velocity.hpp"
#include "myUtils/generic_subscription_helper.hpp"

void DynamixelHandler::SetupRosInterfacesP(bool no_use_command_line) {
    sub_dxl_p_cmds_ = create_subscription<DxlCommandsP>(
        "dynamixel/commands/p", 10,
        [this](std::shared_ptr<DxlCommandsP> msg) { CallbackCmdsP(msg); });

    if ( no_use_command_line ) return;

    sub_ctrl_p_pwm_ = dynamixel_handler_detail::CreateGenericSubscriptionWithTypedCallback<DynamixelControlPPwm>(
        this, "dynamixel/command/p/pwm_control", "dynamixel_handler_msgs/msg/DynamixelControlPPwm", 4,
        [this](const DynamixelControlPPwm& msg) { CallbackCmd_P_Pwm(msg); });
    sub_ctrl_p_cur_ = dynamixel_handler_detail::CreateGenericSubscriptionWithTypedCallback<DynamixelControlPCurrent>(
        this, "dynamixel/command/p/current_control", "dynamixel_handler_msgs/msg/DynamixelControlPCurrent", 4,
        [this](const DynamixelControlPCurrent& msg) { CallbackCmd_P_Current(msg); });
    sub_ctrl_p_vel_ = dynamixel_handler_detail::CreateGenericSubscriptionWithTypedCallback<DynamixelControlPVelocity>(
        this, "dynamixel/command/p/velocity_control", "dynamixel_handler_msgs/msg/DynamixelControlPVelocity", 4,
        [this](const DynamixelControlPVelocity& msg) { CallbackCmd_P_Velocity(msg); });
    sub_ctrl_p_pos_ = dynamixel_handler_detail::CreateGenericSubscriptionWithTypedCallback<DynamixelControlPPosition>(
        this, "dynamixel/command/p/position_control", "dynamixel_handler_msgs/msg/DynamixelControlPPosition", 4,
        [this](const DynamixelControlPPosition& msg) { CallbackCmd_P_Position(msg); });
    sub_ctrl_p_epos_ = dynamixel_handler_detail::CreateGenericSubscriptionWithTypedCallback<DynamixelControlPExtendedPosition>(
        this, "dynamixel/command/p/extended_position_control", "dynamixel_handler_msgs/msg/DynamixelControlPExtendedPosition", 4,
        [this](const DynamixelControlPExtendedPosition& msg) { CallbackCmd_P_ExtendedPosition(msg); });
}
