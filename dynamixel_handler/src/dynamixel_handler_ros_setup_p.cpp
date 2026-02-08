#include "dynamixel_handler.hpp"

#include "dynamixel_handler_msgs/msg/dynamixel_control_p_current.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_p_extended_position.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_p_position.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_p_pwm.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_p_velocity.hpp"

using std::bind;
using std::placeholders::_1;

void DynamixelHandler::SetupRosInterfacesP() {
    sub_ctrl_p_pwm_ = create_subscription<DynamixelControlPPwm>(
        "dynamixel/command/p/pwm_control", 4, bind(&DynamixelHandler::CallbackCmd_P_Pwm, this, _1));
    sub_ctrl_p_cur_ = create_subscription<DynamixelControlPCurrent>(
        "dynamixel/command/p/current_control", 4, bind(&DynamixelHandler::CallbackCmd_P_Current, this, _1));
    sub_ctrl_p_vel_ = create_subscription<DynamixelControlPVelocity>(
        "dynamixel/command/p/velocity_control", 4, bind(&DynamixelHandler::CallbackCmd_P_Velocity, this, _1));
    sub_ctrl_p_pos_ = create_subscription<DynamixelControlPPosition>(
        "dynamixel/command/p/position_control", 4, bind(&DynamixelHandler::CallbackCmd_P_Position, this, _1));
    sub_ctrl_p_epos_ = create_subscription<DynamixelControlPExtendedPosition>(
        "dynamixel/command/p/extended_position_control", 4, bind(&DynamixelHandler::CallbackCmd_P_ExtendedPosition, this, _1));
}
