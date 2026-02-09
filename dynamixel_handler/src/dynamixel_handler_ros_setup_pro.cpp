#include "dynamixel_handler.hpp"
#include "dynamixel_handler_msgs/msg/dxl_commands_pro.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_pro_current.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_pro_extended_position.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_pro_position.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_pro_velocity.hpp"
#include "myUtils/generic_subscription_helper.hpp"

void DynamixelHandler::SetupRosInterfacesPro(bool no_use_command_line) {
    sub_dxl_pro_cmds_ = create_subscription<DxlCommandsPro>(
        "dynamixel/commands/pro", 10,
        [this](std::shared_ptr<DxlCommandsPro> msg) { CallbackCmdsPro(msg); });

    if ( no_use_command_line ) return;

    sub_ctrl_pro_cur_ = dynamixel_handler_detail::CreateGenericSubscriptionWithTypedCallback<DynamixelControlProCurrent>(
        this, "dynamixel/command/pro/current_control", "dynamixel_handler_msgs/msg/DynamixelControlProCurrent", 4,
        [this](const DynamixelControlProCurrent& msg) { CallbackCmd_Pro_Current(msg); });
    sub_ctrl_pro_vel_ = dynamixel_handler_detail::CreateGenericSubscriptionWithTypedCallback<DynamixelControlProVelocity>(
        this, "dynamixel/command/pro/velocity_control", "dynamixel_handler_msgs/msg/DynamixelControlProVelocity", 4,
        [this](const DynamixelControlProVelocity& msg) { CallbackCmd_Pro_Velocity(msg); });
    sub_ctrl_pro_pos_ = dynamixel_handler_detail::CreateGenericSubscriptionWithTypedCallback<DynamixelControlProPosition>(
        this, "dynamixel/command/pro/position_control", "dynamixel_handler_msgs/msg/DynamixelControlProPosition", 4,
        [this](const DynamixelControlProPosition& msg) { CallbackCmd_Pro_Position(msg); });
    sub_ctrl_pro_epos_ = dynamixel_handler_detail::CreateGenericSubscriptionWithTypedCallback<DynamixelControlProExtendedPosition>(
        this, "dynamixel/command/pro/extended_position_control", "dynamixel_handler_msgs/msg/DynamixelControlProExtendedPosition", 4,
        [this](const DynamixelControlProExtendedPosition& msg) { CallbackCmd_Pro_ExtendedPosition(msg); });
}
