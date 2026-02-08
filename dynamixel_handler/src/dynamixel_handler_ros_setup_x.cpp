#include "dynamixel_handler.hpp"

#include "dynamixel_handler_msgs/msg/dynamixel_control_x_current.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_x_current_base_position.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_x_extended_position.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_x_position.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_x_pwm.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_x_velocity.hpp"

using std::bind;
using std::placeholders::_1;

void DynamixelHandler::SetupRosInterfacesX() {
    sub_ctrl_x_pwm_ = create_subscription<DynamixelControlXPwm>(
        "dynamixel/command/x/pwm_control", 4, bind(&DynamixelHandler::CallbackCmd_X_Pwm, this, _1));
    sub_ctrl_x_cur_ = create_subscription<DynamixelControlXCurrent>(
        "dynamixel/command/x/current_control", 4, bind(&DynamixelHandler::CallbackCmd_X_Current, this, _1));
    sub_ctrl_x_vel_ = create_subscription<DynamixelControlXVelocity>(
        "dynamixel/command/x/velocity_control", 4, bind(&DynamixelHandler::CallbackCmd_X_Velocity, this, _1));
    sub_ctrl_x_pos_ = create_subscription<DynamixelControlXPosition>(
        "dynamixel/command/x/position_control", 4, bind(&DynamixelHandler::CallbackCmd_X_Position, this, _1));
    sub_ctrl_x_epos_ = create_subscription<DynamixelControlXExtendedPosition>(
        "dynamixel/command/x/extended_position_control", 4, bind(&DynamixelHandler::CallbackCmd_X_ExtendedPosition, this, _1));
    sub_ctrl_x_cpos_ = create_subscription<DynamixelControlXCurrentBasePosition>(
        "dynamixel/command/x/current_base_position_control", 4, bind(&DynamixelHandler::CallbackCmd_X_CurrentBasePosition, this, _1));
}
