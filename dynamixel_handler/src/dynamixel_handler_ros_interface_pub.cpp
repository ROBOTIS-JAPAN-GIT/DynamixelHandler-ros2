#include "dynamixel_handler.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_debug.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_error.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_gain.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_goal.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_limit.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_present.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_status.hpp"
#include "myUtils/make_iterator_convenient.hpp"

// 角度変換
static constexpr double DEG = M_PI/180.0; // degを単位に持つ数字に掛けるとradになる
double round4(double val) { return round(val*10000.0)/10000.0; }

template <typename MsgT>
static void publish_if(const rclcpp::PublisherBase::SharedPtr& pub, const MsgT& msg) {
    if (pub) std::static_pointer_cast<rclcpp::Publisher<MsgT>>(pub)->publish(msg);
}

DynamixelStatus DynamixelHandler::BroadcastState_Status(){
    DynamixelStatus msg;
    for (const auto id : id_set_ ) {
        msg.id_list.push_back(id);
        msg.torque.push_back(tq_mode_[id]==TORQUE_ENABLE);
        msg.error.push_back(has_hardware_error_[id]);
        msg.ping.push_back(ping_err_[id]==0);
        switch(op_mode_[id]) {
            case OPERATING_MODE_PWM:                  msg.mode.push_back(msg.CONTROL_PWM                  ); break;
            case OPERATING_MODE_CURRENT:              msg.mode.push_back(msg.CONTROL_CURRENT              ); break;
            case OPERATING_MODE_VELOCITY:             msg.mode.push_back(msg.CONTROL_VELOCITY             ); break;
            case OPERATING_MODE_POSITION:             msg.mode.push_back(msg.CONTROL_POSITION             ); break;
            case OPERATING_MODE_EXTENDED_POSITION:    msg.mode.push_back(msg.CONTROL_EXTENDED_POSITION    ); break;
            case OPERATING_MODE_CURRENT_BASE_POSITION:msg.mode.push_back(msg.CONTROL_CURRENT_BASE_POSITION); break;
            default:                                  msg.mode.push_back(""                               ); break;
        }
    }
    publish_if(pub_status_, msg);
    return msg;
}

DynamixelPresent DynamixelHandler::BroadcastState_Present(){
    DynamixelPresent msg;
    for (const auto id : id_set_ ) { const auto& value = present_r_[id];
        msg.id_list.push_back(id);
        if ( do_pub_pre_all_ ) {
            msg.pwm_percent.push_back         (round4(value[PRESENT_PWM          ]    ));
            msg.current_ma.push_back          (round4(value[PRESENT_CURRENT      ]    ));
            msg.velocity_deg_s.push_back      (round4(value[PRESENT_VELOCITY     ]/DEG));
            msg.position_deg.push_back        (round4(value[PRESENT_POSITION     ]/DEG));
            msg.vel_trajectory_deg_s.push_back(round4(value[VELOCITY_TRAJECTORY  ]/DEG));
            msg.pos_trajectory_deg.push_back  (round4(value[POSITION_TRAJECTORY  ]/DEG));
            msg.input_voltage_v.push_back     (round4(value[PRESENT_INPUT_VOLTAGE]    ));
            msg.temperature_degc.push_back    (round4(value[PRESENT_TEMPERATURE  ]    ));

        } else for (auto state : present_indice_read_) switch(state) {
            case PRESENT_PWM:          msg.pwm_percent.push_back         (round4(value[state]    )); break;
            case PRESENT_CURRENT:      msg.current_ma.push_back          (round4(value[state]    )); break;
            case PRESENT_VELOCITY:     msg.velocity_deg_s.push_back      (round4(value[state]/DEG)); break;
            case PRESENT_POSITION:     msg.position_deg.push_back        (round4(value[state]/DEG)); break;
            case VELOCITY_TRAJECTORY:  msg.vel_trajectory_deg_s.push_back(round4(value[state]/DEG)); break;
            case POSITION_TRAJECTORY:  msg.pos_trajectory_deg.push_back  (round4(value[state]/DEG)); break;
            case PRESENT_TEMPERATURE:  msg.temperature_degc.push_back    (round4(value[state]    )); break;
            case PRESENT_INPUT_VOLTAGE:msg.input_voltage_v.push_back     (round4(value[state]    )); break;
            default:                                                                                 break;
        }
    }
    publish_if(pub_present_, msg);
    return msg;
}

DynamixelError DynamixelHandler::BroadcastState_Error(){
    DynamixelError msg;
    for (const auto id: id_set_) {
        msg.id_list.push_back(id);
        msg.input_voltage.push_back     (hardware_err_[id][INPUT_VOLTAGE     ]);
        msg.motor_hall_sensor.push_back (hardware_err_[id][MOTOR_HALL_SENSOR ]);
        msg.overheating.push_back       (hardware_err_[id][OVERHEATING       ]);
        msg.motor_encoder.push_back     (hardware_err_[id][MOTOR_ENCODER     ]);
        msg.electronical_shock.push_back(hardware_err_[id][ELECTRONICAL_SHOCK]);
        msg.overload.push_back          (hardware_err_[id][OVERLOAD          ]);
    }
    publish_if(pub_error_, msg);
    return msg;
}

DynamixelLimit DynamixelHandler::BroadcastState_Limit(){
    DynamixelLimit msg;
    for (const auto id : id_set_ ) { const auto limit = limit_r_[id];
        msg.id_list.push_back(id);
        msg.temperature_limit_degc.push_back   (round4(limit[TEMPERATURE_LIMIT ]    ));
        msg.max_voltage_limit_v.push_back      (round4(limit[MAX_VOLTAGE_LIMIT ]    ));
        msg.min_voltage_limit_v.push_back      (round4(limit[MIN_VOLTAGE_LIMIT ]    ));
        msg.pwm_limit_percent.push_back        (round4(limit[PWM_LIMIT         ]    ));
        msg.current_limit_ma.push_back         (round4(limit[CURRENT_LIMIT     ]    ));
        msg.acceleration_limit_deg_ss.push_back(round4(limit[ACCELERATION_LIMIT]/DEG));
        msg.velocity_limit_deg_s.push_back     (round4(limit[VELOCITY_LIMIT    ]/DEG));
        msg.max_position_limit_deg.push_back   (round4(limit[MAX_POSITION_LIMIT]/DEG));
        msg.min_position_limit_deg.push_back   (round4(limit[MIN_POSITION_LIMIT]/DEG));
    }
    publish_if(pub_limit_, msg);
    return msg;
}

DynamixelGain DynamixelHandler::BroadcastState_Gain(){
    DynamixelGain msg;
    for ( const auto id : id_set_ ) { const auto gain = gain_r_[id];
        msg.id_list.push_back(id);
        msg.velocity_i_gain_pulse.push_back     (gain[VELOCITY_I_GAIN     ]);
        msg.velocity_p_gain_pulse.push_back     (gain[VELOCITY_P_GAIN     ]);
        msg.position_d_gain_pulse.push_back     (gain[POSITION_D_GAIN     ]);
        msg.position_i_gain_pulse.push_back     (gain[POSITION_I_GAIN     ]);
        msg.position_p_gain_pulse.push_back     (gain[POSITION_P_GAIN     ]);
        msg.feedforward_2nd_gain_pulse.push_back(gain[FEEDFORWARD_ACC_GAIN]);
        msg.feedforward_1st_gain_pulse.push_back(gain[FEEDFORWARD_VEL_GAIN]);
    }
    publish_if(pub_gain_, msg);
    return msg;
}

DynamixelGoal DynamixelHandler::BroadcastState_Goal(){
    DynamixelGoal msg;
    for ( auto id : id_set_ ) { const auto goal = goal_r_[id];
        msg.id_list.push_back(id);
        msg.pwm_percent.push_back       (round4(goal[GOAL_PWM     ]));
        msg.current_ma.push_back        (round4(goal[GOAL_CURRENT ]));
        msg.velocity_deg_s.push_back    (round4(goal[GOAL_VELOCITY]/DEG));
        msg.profile_vel_deg_s.push_back (round4(goal[PROFILE_VEL  ]/DEG));
        msg.profile_acc_deg_ss.push_back(round4(goal[PROFILE_ACC  ]/DEG));
        msg.position_deg.push_back      (round4(goal[GOAL_POSITION]/DEG));
    }
    publish_if(pub_goal_, msg);
    return msg;
}

DynamixelDebug DynamixelHandler::BroadcastDebug(){
    DynamixelDebug msg;
    for (const auto id : id_set_ ) {
        msg.status.id_list.push_back(id);
        msg.status.torque.push_back(tq_mode_[id]==TORQUE_ENABLE);
        msg.status.error.push_back(has_hardware_error_[id]);
        msg.status.ping.push_back(ping_err_[id]==0);
        switch(op_mode_[id]) {
            case OPERATING_MODE_PWM:                  msg.status.mode.push_back(msg.status.CONTROL_PWM                  ); break;
            case OPERATING_MODE_CURRENT:              msg.status.mode.push_back(msg.status.CONTROL_CURRENT              ); break;
            case OPERATING_MODE_VELOCITY:             msg.status.mode.push_back(msg.status.CONTROL_VELOCITY             ); break;
            case OPERATING_MODE_POSITION:             msg.status.mode.push_back(msg.status.CONTROL_POSITION             ); break;
            case OPERATING_MODE_EXTENDED_POSITION:    msg.status.mode.push_back(msg.status.CONTROL_EXTENDED_POSITION    ); break;
            case OPERATING_MODE_CURRENT_BASE_POSITION:msg.status.mode.push_back(msg.status.CONTROL_CURRENT_BASE_POSITION); break;
            default:                                  msg.status.mode.push_back(""); break;
        }
        msg.current_ma.present.push_back    (round4(present_r_[id][PRESENT_CURRENT ]));
        msg.current_ma.goal.push_back       (round4(   goal_r_[id][GOAL_CURRENT    ]));
        msg.velocity_deg_s.present.push_back(round4(present_r_[id][PRESENT_VELOCITY]/DEG));
        msg.velocity_deg_s.goal.push_back   (round4(   goal_r_[id][GOAL_VELOCITY   ]/DEG));
        msg.position_deg.present.push_back  (round4(present_r_[id][PRESENT_POSITION]/DEG));
        msg.position_deg.goal.push_back     (round4(   goal_r_[id][GOAL_POSITION   ]/DEG));
    }
    publish_if(pub_debug_, msg);
    return msg;
}
