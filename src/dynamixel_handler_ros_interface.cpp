#include "dynamixel_handler.hpp"

using namespace dyn_x;

template <typename T>
bool is_in(const T& val, const vector<T>& vec) { return std::find(vec.begin(), vec.end(), val) != vec.end(); }

string update_info(const vector<uint8_t>& id_list, const string& what_updated) {
    char header[99]; 
    sprintf(header, "[%d] servo(s) %s are updated", (int)id_list.size(), what_updated.c_str());
    return id_list_layout(id_list, string(header));
}

//* ROS関係

void DynamixelHandler::CallBackDxlCommand(const dynamixel_handler::msg::DynamixelCommand& msg) {
    vector<uint8_t> id_list;
    if ( msg.id_list.empty() || msg.id_list[0]==0xFE) for (auto id : id_list_) id_list.push_back(id);
                                                 else for (auto id : msg.id_list) id_list.push_back(id);
    char header[100]; sprintf(header, "Command [%s] \n (id_list=[] or [254] means all IDs)", msg.command.c_str());
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), id_list_layout(id_list, string(header)));
    if (msg.command == "clear_error" || msg.command == "CE")
        for (auto id : id_list) { ClearHardwareError(id); TorqueOn(id);}
    if (msg.command == "torque_on"   || msg.command == "TON") 
        for (auto id : id_list) TorqueOn(id);
    if (msg.command == "torque_off"  || msg.command == "TOFF")
        for (auto id : id_list) TorqueOff(id);
    if (msg.command == "enable") 
        for (auto id : id_list) WriteTorqueEnable(id, true);
    if (msg.command == "disable")
        for (auto id : id_list) WriteTorqueEnable(id, false);
    if (msg.command == "reboot") 
        for (auto id : id_list) dyn_comm_.Reboot(id);
}

void DynamixelHandler::CallBackDxlCmd_Profile(const dynamixel_handler::msg::DynamixelCommandProfile& msg) {
    const bool do_process_vel = msg.id_list.size() == msg.profile_vel_deg_s.size();
    const bool do_process_acc = msg.id_list.size() == msg.profile_acc_deg_ss.size();
    if ( do_process_vel ) {
        vector<uint8_t> store_id_list_vel; 
        for (size_t i=0; i<msg.id_list.size(); i++){
            uint8_t id = msg.id_list[i]; if ( !is_in(id, id_list_ ) ) continue;
            auto& limit = option_limit_[id];
            auto vel = msg.profile_vel_deg_s[i];
            cmd_values_[id][PROFILE_VEL] = clamp( deg2rad(vel), -limit[VELOCITY_LIMIT], limit[VELOCITY_LIMIT] );
            is_cmd_updated_[id] = true;
            list_write_cmd_.insert(PROFILE_VEL);
            store_id_list_vel.push_back(id);
        }
        if (varbose_callback_ ) RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), update_info(store_id_list_vel, "profile_velocity"));
    }
    if ( do_process_acc ){
        vector<uint8_t> store_id_list_acc;
        for (size_t i=0; i<msg.id_list.size(); i++){
            uint8_t id = msg.id_list[i]; if ( !is_in(id, id_list_ ) ) continue;
            auto& limit = option_limit_[id];
            auto acc = msg.profile_acc_deg_ss[i];
            cmd_values_[id][PROFILE_ACC] = clamp( deg2rad(acc), -limit[ACCELERATION_LIMIT], limit[ACCELERATION_LIMIT] );
            is_cmd_updated_[id] = true;
            list_write_cmd_.insert(PROFILE_ACC);
            store_id_list_acc.push_back(id);
        }
        if (varbose_callback_ ) RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), update_info(store_id_list_acc, "profile_acceleration"));
    }
    if ( !do_process_vel && !do_process_acc ) RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Element size all dismatch; skiped callback");
}

void DynamixelHandler::CallBackDxlCmd_X_Position(const dynamixel_handler::msg::DynamixelCommandXControlPosition& msg) {
    // if (varbose_callback_) RCLCPP_INFO("msg generate time: %f", msg.stamp.toSec());  // ↓msg.id_listと同じサイズの奴だけ処理する
    for ( const uint8_t& id : msg.id_list ) if ( is_in(id, id_list_) ) ChangeOperatingMode(id, OPERATING_MODE_POSITION);
    const bool do_process = msg.id_list.size() == msg.position_deg.size();
    if ( do_process ){
        vector<uint8_t> store_id_list;
        for (size_t i = 0; i < msg.id_list.size(); i++) {
            uint8_t id = msg.id_list[i]; if ( !is_in(id, id_list_ ) ) continue;
            auto pos = msg.position_deg[i];
            auto& limit = option_limit_[id];
            cmd_values_[id][GOAL_POSITION] = clamp(deg2rad(pos), limit[MIN_POSITION_LIMIT], limit[MAX_POSITION_LIMIT]);
            is_cmd_updated_[id] = true;
            list_write_cmd_.insert(GOAL_POSITION);
            store_id_list.push_back(id);
        }
        if (varbose_callback_) RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), update_info(store_id_list, "goal_position"));
    }
    if ( !do_process ) RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Element size all dismatch; skiped callback");
}

void DynamixelHandler::CallBackDxlCmd_X_Velocity(const dynamixel_handler::msg::DynamixelCommandXControlVelocity& msg) {
    for ( const uint8_t& id : msg.id_list ) if ( is_in(id, id_list_) ) ChangeOperatingMode(id, OPERATING_MODE_VELOCITY);
    const bool do_process = msg.id_list.size() == msg.velocity_deg_s.size();
    if ( do_process ){
        vector<uint8_t> store_id_list;
        for (size_t i = 0; i < msg.id_list.size(); i++) {
            uint8_t id = msg.id_list[i]; if ( !is_in(id, id_list_ ) ) continue;
            auto vel = msg.velocity_deg_s[i];
            auto& limit = option_limit_[id];
            cmd_values_[id][GOAL_VELOCITY] = clamp(deg2rad(vel), -limit[VELOCITY_LIMIT], limit[VELOCITY_LIMIT]);
            is_cmd_updated_[id] = true;
            list_write_cmd_.insert(GOAL_VELOCITY);
            store_id_list.push_back(id);
        }
        if (varbose_callback_) RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), update_info(store_id_list, "goal_velocity"));
    }
    if ( !do_process ) RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Element size all dismatch; skiped callback");
}

void DynamixelHandler::CallBackDxlCmd_X_Current(const dynamixel_handler::msg::DynamixelCommandXControlCurrent& msg) {
    for ( const uint8_t& id : msg.id_list ) if ( is_in(id, id_list_) ) ChangeOperatingMode(id, OPERATING_MODE_CURRENT);
    const bool do_process = msg.id_list.size() == msg.current_ma.size();
    if ( do_process ){
        vector<uint8_t> store_id_list;
        for (size_t i = 0; i < msg.id_list.size(); i++) {
            uint8_t id = msg.id_list[i]; if ( !is_in(id, id_list_ ) ) continue;
            auto cur = msg.current_ma[i];
            auto& limit = option_limit_[id];
            cmd_values_[id][GOAL_CURRENT] = clamp(cur, -limit[CURRENT_LIMIT], limit[CURRENT_LIMIT]);
            is_cmd_updated_[id] = true;
            list_write_cmd_.insert(GOAL_CURRENT);
            store_id_list.push_back(id);
        }
        if ( varbose_callback_ ) RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), update_info(store_id_list, "goal_current"));
    }
    if ( !do_process ) RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Element size all dismatch; skiped callback");
}

void DynamixelHandler::CallBackDxlCmd_X_CurrentPosition(const dynamixel_handler::msg::DynamixelCommandXControlCurrentPosition& msg) {
    for ( const uint8_t& id : msg.id_list ) if ( is_in(id, id_list_) ) ChangeOperatingMode(id, OPERATING_MODE_CURRENT_BASE_POSITION); 
    const bool do_process_pos = msg.id_list.size() == msg.position_deg.size() || msg.id_list.size() == msg.rotation.size();
    if ( do_process_pos ){
        vector<uint8_t> store_id_list_pos;
        for (size_t i = 0; i < msg.id_list.size(); i++) {
            uint8_t id = msg.id_list[i]; if ( !is_in(id, id_list_ ) ) continue;
            auto pos = msg.id_list.size() == msg.position_deg.size() ? msg.position_deg[i] : 0.0;
            auto rot = msg.id_list.size() == msg.rotation.size()      ? msg.rotation[i]      : 0.0;
            auto& limit = option_limit_[id];
            is_cmd_updated_[id] = true;
            cmd_values_[id][GOAL_POSITION] = clamp( deg2rad(pos + rot * 360.0), -256*2*M_PI, 256*2*M_PI );
            list_write_cmd_.insert(GOAL_POSITION);
            store_id_list_pos.push_back(id);
        }
        if (varbose_callback_ ) RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), update_info(store_id_list_pos, "goal_position"));
    }
    const bool do_process_cur = msg.id_list.size() == msg.current_ma.size();
    if ( do_process_cur ){
        vector<uint8_t> store_id_list_cur;
        for (size_t i = 0; i < msg.id_list.size(); i++) {
            uint8_t id = msg.id_list[i]; if ( !is_in(id, id_list_ ) ) continue;
            auto& limit = option_limit_[id];
            auto cur = msg.current_ma[i];
            is_cmd_updated_[id] = true;
            cmd_values_[id][GOAL_CURRENT] = clamp(cur, -limit[CURRENT_LIMIT], limit[CURRENT_LIMIT]);
            list_write_cmd_.insert(GOAL_CURRENT);
            store_id_list_cur.push_back(id);
        }
        if (varbose_callback_ ) RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), update_info(store_id_list_cur, "goal_current"));
    }
    if ( !do_process_cur && !do_process_pos ) RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Element size all dismatch; skiped callback");
}

void DynamixelHandler::CallBackDxlCmd_X_ExtendedPosition(const dynamixel_handler::msg::DynamixelCommandXControlExtendedPosition& msg) {
    for ( const uint8_t& id : msg.id_list ) if ( is_in(id, id_list_) ) ChangeOperatingMode(id, OPERATING_MODE_EXTENDED_POSITION);
    const bool do_process = msg.id_list.size() == msg.position_deg.size() || msg.id_list.size() == msg.rotation.size();
    if ( do_process ) {
        vector<uint8_t> store_id_list;
        for (size_t i = 0; i < msg.id_list.size(); i++) {
            uint8_t id = msg.id_list[i]; if ( !is_in(id, id_list_ ) ) continue;
            auto pos = msg.id_list.size() == msg.position_deg.size() ? msg.position_deg[i] : 0.0;
            auto rot = msg.id_list.size() == msg.rotation.size()      ? msg.rotation[i]      : 0.0;
            is_cmd_updated_[id] = true;
            cmd_values_[id][GOAL_POSITION] = clamp( deg2rad(pos + rot * 360.0), -256*2*M_PI, 256*2*M_PI );
            list_write_cmd_.insert(GOAL_POSITION);
            store_id_list.push_back(id);
        }
        if (varbose_callback_) RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), update_info(store_id_list, "goal_position"));
    }
    if ( !do_process ) RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Element size all dismatch; skiped callback");
}

void DynamixelHandler::CallBackDxlOpt_Gain(const dynamixel_handler::msg::DynamixelOptionGain& msg) {
    // if (varbose_callback_) RCLCPP_INFO("CallBackDxlOpt_Gain");
    bool is_any = false;
    if (msg.id_list.size() == msg.velocity_i_gain_pulse.size()){ is_any=true;}
    if (msg.id_list.size() == msg.velocity_p_gain_pulse.size()){ is_any=true;}
    if (msg.id_list.size() == msg.position_d_gain_pulse.size()){ is_any=true;}
    if (msg.id_list.size() == msg.position_i_gain_pulse.size()){ is_any=true;}
    if (msg.id_list.size() == msg.position_p_gain_pulse.size()){ is_any=true;}
    if (msg.id_list.size() == msg.feedforward_acc_gain_pulse.size()){ is_any=true;}
    if (msg.id_list.size() == msg.feedforward_vel_gain_pulse.size()){ is_any=true;}
    if (varbose_callback_) {
        //  if (is_any) RCLCPP_INFO(" - %d servo(s) gain are updated", (int)msg.id_list.size());
        //  else                  RCLCPP_ERROR("Element size all dismatch; skiped callback");
    }
}

void DynamixelHandler::CallBackDxlOpt_Limit(const dynamixel_handler::msg::DynamixelOptionLimit& msg) {
    // if (varbose_callback_) RCLCPP_INFO("CallBackDxlOpt_Limit");
    bool is_any = false;

    if (msg.id_list.size() == msg.temperature_limit_deg_c.size()   ){is_any=true;}
    if (msg.id_list.size() == msg.max_voltage_limit_v.size()      ){is_any=true;}
    if (msg.id_list.size() == msg.min_voltage_limit_v.size()      ){is_any=true;}
    if (msg.id_list.size() == msg.pwm_limit_percent.size()        ){is_any=true;}
    if (msg.id_list.size() == msg.current_limit_ma.size()         ){is_any=true;}
    if (msg.id_list.size() == msg.acceleration_limit_deg_ss.size()){is_any=true;}
    if (msg.id_list.size() == msg.velocity_limit_deg_s.size()     ){is_any=true;}
    if (msg.id_list.size() == msg.max_position_limit_deg.size()   ){is_any=true;}
    if (msg.id_list.size() == msg.min_position_limit_deg.size()   ){is_any=true;}

    if (varbose_callback_) {
        //  if (is_any) RCLCPP_INFO(" - %d servo(s) limit are updated", (int)msg.id_list.size());
        //  else                  RCLCPP_ERROR("Element size all dismatch; skiped callback");
    }
}

void DynamixelHandler::CallBackDxlOpt_Mode(const dynamixel_handler::msg::DynamixelOptionMode& msg) {
 
}

double round4(double val) { return round(val*10000.0)/10000.0; }

void DynamixelHandler::BroadcastDxlState(){
    dynamixel_handler::msg::DynamixelState msg;
    rclcpp::Clock ros_clock(RCL_ROS_TIME);
    msg.stamp = ros_clock.now();
    for (const auto& [id, value] : state_values_) {
        msg.id_list.push_back(id);
        for (auto state : list_read_state_) switch(state) {
            case PRESENT_PWM:          msg.pwm_percent.push_back         (round4(value[state]    )); break;
            case PRESENT_CURRENT:      msg.current_ma.push_back          (round4(value[state]    )); break;
            case PRESENT_VELOCITY:     msg.velocity_deg_s.push_back      (round4(value[state]/DEG)); break;
            case PRESENT_POSITION:     msg.position_deg.push_back        (round4(value[state]/DEG)); break;
            case VELOCITY_TRAJECTORY:  msg.vel_trajectory_deg_s.push_back(round4(value[state]/DEG)); break;
            case POSITION_TRAJECTORY:  msg.pos_trajectory_deg.push_back  (round4(value[state]/DEG)); break;
            case PRESENT_TEMPERTURE:   msg.temperature_deg_c.push_back    (round4(value[state]    )); break;
            case PRESENT_INPUT_VOLTAGE:msg.input_voltage_v.push_back     (round4(value[state]    )); break;
        }
    }
    pub_state_->publish(msg);
}

void DynamixelHandler::BroadcastDxlError(){
    dynamixel_handler::msg::DynamixelError msg;
    rclcpp::Clock ros_clock(RCL_ROS_TIME);
    msg.stamp = ros_clock.now();
    for (const auto& [id, error]: hardware_error_) {
        if (error[INPUT_VOLTAGE     ]) msg.input_voltage.push_back     (id);
        if (error[MOTOR_HALL_SENSOR ]) msg.motor_hall_sensor.push_back (id);
        if (error[OVERHEATING       ]) msg.overheating.push_back       (id);
        if (error[MOTOR_ENCODER     ]) msg.motor_encoder.push_back     (id);
        if (error[ELECTRONICAL_SHOCK]) msg.electronical_shock.push_back(id);
        if (error[OVERLOAD          ]) msg.overload.push_back          (id);
    }
    pub_error_->publish(msg);
}

void DynamixelHandler::BroadcastDxlOpt_Limit(){
    dynamixel_handler::msg::DynamixelOptionLimit msg;
    rclcpp::Clock ros_clock(RCL_ROS_TIME);
    msg.stamp = ros_clock.now();
    for (const auto& [id, limit] : option_limit_) {
        msg.id_list.push_back(id);
        msg.temperature_limit_deg_c.push_back   (round4(limit[TEMPERATURE_LIMIT ]));
        msg.max_voltage_limit_v.push_back      (round4(limit[MAX_VOLTAGE_LIMIT ]));
        msg.min_voltage_limit_v.push_back      (round4(limit[MIN_VOLTAGE_LIMIT ]));
        msg.pwm_limit_percent.push_back        (round4(limit[PWM_LIMIT         ]));
        msg.current_limit_ma.push_back         (round4(limit[CURRENT_LIMIT     ]));
        msg.acceleration_limit_deg_ss.push_back(round4(limit[ACCELERATION_LIMIT]/DEG));
        msg.velocity_limit_deg_s.push_back     (round4(limit[VELOCITY_LIMIT    ]/DEG));
        msg.max_position_limit_deg.push_back   (round4(limit[MAX_POSITION_LIMIT]/DEG));
        msg.min_position_limit_deg.push_back   (round4(limit[MIN_POSITION_LIMIT]/DEG));
    }
    pub_opt_limit_->publish(msg);
}

void DynamixelHandler::BroadcastDxlOpt_Gain(){
    dynamixel_handler::msg::DynamixelOptionGain msg;
    rclcpp::Clock ros_clock(RCL_ROS_TIME);
    msg.stamp = ros_clock.now();
    for ( const auto& [id, gain] : option_gain_ ) {
        msg.id_list.push_back(id);
        msg.velocity_i_gain_pulse.push_back     (gain[VELOCITY_I_GAIN     ]);
        msg.velocity_p_gain_pulse.push_back     (gain[VELOCITY_P_GAIN     ]);
        msg.position_d_gain_pulse.push_back     (gain[POSITION_D_GAIN     ]);
        msg.position_i_gain_pulse.push_back     (gain[POSITION_I_GAIN     ]);
        msg.position_p_gain_pulse.push_back     (gain[POSITION_P_GAIN     ]);
        msg.feedforward_acc_gain_pulse.push_back(gain[FEEDFORWARD_ACC_GAIN]);
        msg.feedforward_vel_gain_pulse.push_back(gain[FEEDFORWARD_VEL_GAIN]);
    }
    pub_opt_gain_->publish(msg);
}

void DynamixelHandler::BroadcastDxlOpt_Mode(){
    dynamixel_handler::msg::DynamixelOptionMode msg;
    rclcpp::Clock ros_clock(RCL_ROS_TIME);
    msg.stamp = ros_clock.now();
    for ( const auto& id : id_list_ ) {
        msg.id_list.push_back(id);
        msg.torque_enable.push_back(tq_mode_[id]);
        switch(op_mode_[id]) {
            case OPERATING_MODE_CURRENT:              msg.operating_mode.push_back("current");           break;
            case OPERATING_MODE_VELOCITY:             msg.operating_mode.push_back("velocity");          break;
            case OPERATING_MODE_POSITION:             msg.operating_mode.push_back("position");          break;
            case OPERATING_MODE_EXTENDED_POSITION:    msg.operating_mode.push_back("extended_position"); break;
            case OPERATING_MODE_CURRENT_BASE_POSITION:msg.operating_mode.push_back("current_position");  break;
        }
        switch(dv_mode_[id]) {
            default: msg.drive_mode.push_back("unknown"); break;
        }
    }
    pub_opt_mode_->publish(msg);
}

void DynamixelHandler::BroadcastDxlOpt_Goal(){
    dynamixel_handler::msg::DynamixelOptionGoal msg;
    rclcpp::Clock ros_clock(RCL_ROS_TIME);
    msg.stamp = ros_clock.now();
    for ( const auto& [id, goal] : option_goal_ ) {
        msg.id_list.push_back(id);
        msg.pwm_percent.push_back       (round4(goal[GOAL_PWM         ]));
        msg.current_ma.push_back        (round4(goal[GOAL_CURRENT     ]));
        msg.velocity_deg_s.push_back    (round4(goal[GOAL_VELOCITY    ]/DEG));
        msg.profile_vel_deg_s.push_back (round4(goal[PROFILE_VEL      ]/DEG));
        msg.profile_acc_deg_ss.push_back(round4(goal[PROFILE_ACC      ]/DEG));
        msg.position_deg.push_back      (round4(goal[GOAL_POSITION    ]/DEG));
    }
    pub_opt_goal_->publish(msg);
}