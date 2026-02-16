#include "dynamixel_handler.hpp"
#include "dynamixel_handler_msgs/msg/dxl_commands_all.hpp"
#include "dynamixel_handler_msgs/msg/dxl_commands_p.hpp"
#include "dynamixel_handler_msgs/msg/dxl_commands_pro.hpp"
#include "dynamixel_handler_msgs/msg/dxl_commands_x.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_p_current.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_p_extended_position.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_p_position.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_p_pwm.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_p_velocity.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_pro_current.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_pro_extended_position.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_pro_position.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_pro_velocity.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_x_current.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_x_current_base_position.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_x_extended_position.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_x_position.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_x_pwm.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_x_velocity.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_gain.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_goal.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_limit.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_shortcut.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_status.hpp"
#include "myUtils/formatting_output.hpp"
#include "myUtils/logging_like_ros1.hpp"
#include "myUtils/make_iterator_convenient.hpp"  // enum のインクリメントと， is_in 関数の実装

// 角度変換
static constexpr double DEG = M_PI/180.0; // degを単位に持つ数字に掛けるとradになる
static double deg2rad(double deg){ return deg*DEG; }
static vector<bool> falses(size_t N) { return vector<bool>(N, false); }
static vector<bool> trues (size_t N) { return vector<bool>(N,  true); }

bool DynamixelHandler::check_series(id_t id, series_t series) {
    return series_[id] == series || series_[id] == SERIES_UNKNOWN; 
}

void DynamixelHandler::CallbackCmdsX(std::shared_ptr<DxlCommandsX> msg) {
    if ( verbose_callback_ ) ROS_INFO("=====================================");
    CallbackCmd_Status(msg->status); // Pシリーズ縛りを入れる
    CallbackCmd_X_Pwm(msg->pwm_control);
    CallbackCmd_X_Current(msg->current_control);
    CallbackCmd_X_Velocity(msg->velocity_control);
    CallbackCmd_X_Position(msg->position_control);
    CallbackCmd_X_ExtendedPosition(msg->extended_position_control);
    CallbackCmd_X_CurrentBasePosition(msg->current_base_position_control);
    CallbackCmd_Gain  (msg->gain);  // Xシリーズ縛りを入れる
    CallbackCmd_Limit (msg->limit); // Xシリーズ縛りを入れる
}
void DynamixelHandler::CallbackCmdsP(std::shared_ptr<DxlCommandsP> msg) {
    if ( verbose_callback_ ) ROS_INFO("=====================================");
    CallbackCmd_Status(msg->status);  // Pシリーズ縛りを入れる
    CallbackCmd_P_Pwm(msg->pwm_control);
    CallbackCmd_P_Current(msg->current_control);
    CallbackCmd_P_Velocity(msg->velocity_control);
    CallbackCmd_P_Position(msg->position_control);
    CallbackCmd_P_ExtendedPosition(msg->extended_position_control);
    CallbackCmd_Gain  (msg->gain);  // Pシリーズ縛りを入れる
    CallbackCmd_Limit (msg->limit); // Pシリーズ縛りを入れる
}
void DynamixelHandler::CallbackCmdsPro(std::shared_ptr<DxlCommandsPro> msg) {
    if ( verbose_callback_ ) ROS_INFO("=====================================");
    CallbackCmd_Status(msg->status);  // Pシリーズ縛りを入れる
    CallbackCmd_Pro_Current(msg->current_control);
    CallbackCmd_Pro_Velocity(msg->velocity_control);
    CallbackCmd_Pro_Position(msg->position_control);
    CallbackCmd_Pro_ExtendedPosition(msg->extended_position_control);
    CallbackCmd_Gain  (msg->gain);  // Pシリーズ縛りを入れる
    CallbackCmd_Limit (msg->limit); // Pシリーズ縛りを入れる
}
void DynamixelHandler::CallbackCmdsAll(std::shared_ptr<DxlCommandsAll> msg) {
    if ( verbose_callback_ ) ROS_INFO("=====================================");
    CallbackCmd_Status(msg->status);
    CallbackCmd_Goal  (msg->goal);
    CallbackCmd_Gain  (msg->gain);
    CallbackCmd_Limit (msg->limit);
}

void DynamixelHandler::CallbackShortcut(const DynamixelShortcut& msg) {
    if ( msg.command=="" ) return;
    if (verbose_callback_ ) {
        ROS_INFO_STREAM(id_list_layout(msg.id_list, "Shortcut, ID"));
        ROS_INFO(" command [%s] (ID=[] or [254] means all IDs)", msg.command.c_str());
    }
    auto st = DynamixelStatus();
    const auto& cmd = msg.command; // 略記
    const auto& N = msg.id_list.size();// IDリストのサイズ
    const auto& id_list = ( N==0 || (N==1 && msg.id_list[0]==0xFE) ) // [] or [254] なら全ID
                          ? vector<uint16_t>(id_set_.begin(), id_set_.end()) : msg.id_list;
         if (cmd==msg.CLEAR_ERROR || cmd=="CE"  ) CallbackCmd_Status(st.set__id_list(id_list).set__error (falses(id_list.size())));
    else if (cmd==msg.TORQUE_ON   || cmd=="TON" ) CallbackCmd_Status(st.set__id_list(id_list).set__torque( trues(id_list.size())));
    else if (cmd==msg.TORQUE_OFF  || cmd=="TOFF") CallbackCmd_Status(st.set__id_list(id_list).set__torque(falses(id_list.size())));
    else if (cmd==msg.ADD_ID      || cmd=="ADID") CallbackCmd_Status(st.set__id_list(id_list).set__ping  ( trues(id_list.size())));
    else if (cmd==msg.REMOVE_ID   || cmd=="RMID") CallbackCmd_Status(st.set__id_list(id_list).set__ping  (falses(id_list.size())));
    else if (cmd==msg.RESET_OFFSET ) for(auto id: id_list){ // 開発者用で頻繁には使われないので if(verbose_)は付けない
                                    TorqueOff(id); WriteHomingOffset(id, 0);  ROS_INFO("  - set: offset zero, ID [%d]"   , id);}
    else if (cmd==msg.ENABLE       ) for(auto id: id_list){ // 開発者用で頻繁には使われないので if(verbose_)は付けない
                                    WriteTorqueEnable(id, TORQUE_ENABLE);     ROS_INFO("  - set: torque enable, ID [%d]" , id);}
    else if (cmd==msg.DISABLE      ) for(auto id: id_list){ // 開発者用で頻繁には使われないので if(verbose_)は付けない
                                    WriteTorqueEnable(id, TORQUE_DISABLE);    ROS_INFO("  - set: torque disable, ID [%d]", id);}
    else if (cmd==msg.REBOOT       ) for(auto id: id_list){ // Reboot()はdummyでも送信をしてしまうので，事前に確認する．
                                 if( series_[id] != SERIES_UNKNOWN ) {dyn_comm_.Reboot(id); ROS_INFO("  - reboot: ID [%d]", id);} }
    else ROS_WARN("  Invalid command [%s]", cmd.c_str());
}

void DynamixelHandler::CallbackCmd_Status(const DynamixelStatus& msg) {
    // msg内の各要素のサイズがIDリストと一致しているか確認
    const bool has_torque = msg.id_list.size() == msg.torque.size();
    const bool has_error  = msg.id_list.size() == msg.error.size() ;
    const bool has_ping   = msg.id_list.size() == msg.ping.size()  ;
    const bool has_mode   = msg.id_list.size() == msg.mode.size()  ;
    // msg.id_list内のIDの妥当性を確認, has_ping が true の場合は，id_set_に含まれていないIDも有効とする
    vector<uint16_t> valid_id_list;
    for (auto id : msg.id_list) if ( is_in(id, id_set_) || has_ping ) valid_id_list.push_back(id);
    if ( valid_id_list.empty() ) return; // 有効なIDがない場合は何もしない
    // log出力, 
    if (verbose_callback_ ) {
        ROS_INFO("Status cmd, '%zu' servo(s) are tryed to updated", valid_id_list.size());
        ROS_INFO_STREAM(id_list_layout(valid_id_list, "  ID")); // valid_id_list はここで必要なので，わざわざ独立したvectorとして用意している
        if ( has_torque ) ROS_INFO("  - change torque mode   "); else if ( !msg.torque.empty() ) ROS_WARN("  - skipped: torque (size mismatch)");
        if ( has_error  ) ROS_INFO("  - try clear error      "); else if ( !msg.error .empty() ) ROS_WARN("  - skipped: error  (size mismatch)");
        if ( has_ping   ) ROS_INFO("  - add/remove id (ping) "); else if ( !msg.ping  .empty() ) ROS_WARN("  - skipped: ping   (size mismatch)");
        if ( has_mode   ) ROS_INFO("  - change operating mode"); else if ( !msg.mode  .empty() ) ROS_WARN("  - skipped: mode   (size mismatch)");
    }
    // 各IDに対して，msg内の指令をもとに処理を行う
    for (size_t i=0; i<msg.id_list.size(); i++) if ( auto ID = msg.id_list[i]; is_in(ID, valid_id_list) ) { // 順番がずれるのでわざとこの書き方をしている．
        if ( has_ping   ) msg.ping[i]   ? AddDynamixel(ID) : RemoveDynamixel(ID); // 先にAddDynamixelを行わないと，他の処理ができない.
        if ( has_torque ) msg.torque[i] ? TorqueOn(ID)     : TorqueOff(ID)      ;
        if ( has_error  ) { ClearHardwareError(ID); TorqueOn(ID); }
        if ( has_mode   ) {
                 if (msg.mode[i] == msg.CONTROL_PWM                  ) ChangeOperatingMode(ID, OPERATING_MODE_PWM                  );
            else if (msg.mode[i] == msg.CONTROL_CURRENT              ) ChangeOperatingMode(ID, OPERATING_MODE_CURRENT              );
            else if (msg.mode[i] == msg.CONTROL_VELOCITY             ) ChangeOperatingMode(ID, OPERATING_MODE_VELOCITY             );
            else if (msg.mode[i] == msg.CONTROL_POSITION             ) ChangeOperatingMode(ID, OPERATING_MODE_POSITION             );
            else if (msg.mode[i] == msg.CONTROL_EXTENDED_POSITION    ) ChangeOperatingMode(ID, OPERATING_MODE_EXTENDED_POSITION    );
            else if (msg.mode[i] == msg.CONTROL_CURRENT_BASE_POSITION) 
                                 switch (series_[ID]) { default:       ChangeOperatingMode(ID, OPERATING_MODE_CURRENT_BASE_POSITION); break;
                                                        case SERIES_P: ChangeOperatingMode(ID, OPERATING_MODE_EXTENDED_POSITION    );
                                                                       ROS_WARN("  ID [%d] is P-series, so alternative mode is selected", ID);}
            else ROS_WARN("  Invalid operating mode [%s], please see CallbackCmd_Status.msg definition.", msg.mode[i].c_str());
        }
    } // 各単体関数(ClearHardwareErrorとか)が内部でROS_INFOを出力しているので，ここでは何も出力しない
    if ( verbose_callback_ ) ROS_INFO("==================+==================");
}

void DynamixelHandler::CallbackCmd_X_Pwm(const DynamixelControlXPwm& msg) {
    if ( msg.id_list.empty() ) return; // id_list が空の場合は何もしない
    if (verbose_callback_) ROS_INFO_STREAM(id_list_layout(msg.id_list, "PWM ctrl(X), ID"));
    vector<uint16_t> id_list(msg.id_list);
    for ( auto& ID : id_list ) if ( !check_series(ID, SERIES_X) ) { // SERIES_X か dummy であることを確認
        if(verbose_callback_) ROS_WARN("  ID [%d] is not X series", ID);
        ID = 255; // 不適な series の場合はid=255にして，以降，無視されるようにする 
    }
    for ( auto ID : id_list ) if(ID<255) ChangeOperatingMode(ID, OPERATING_MODE_PWM);
    CallbackCmd_Goal(DynamixelGoal().set__id_list(id_list)
        .set__pwm_percent(msg.pwm_percent)
    );
}

void DynamixelHandler::CallbackCmd_X_Position(const DynamixelControlXPosition& msg) {
    if ( msg.id_list.empty() ) return; // id_list が空の場合は何もしない
    if (verbose_callback_) ROS_INFO_STREAM(id_list_layout(msg.id_list, "Position ctrl(X), ID"));
    vector<uint16_t> id_list(msg.id_list);
    for ( auto& ID : id_list ) if ( !check_series(ID, SERIES_X) ) { // SERIES_X か dummy であることを確認
        if(verbose_callback_) ROS_WARN("  ID [%d] is not X series", ID);
        ID = 255; // 不適な series の場合はid=255にして，以降，無視されるようにする 
    }
    for ( auto ID : id_list ) if(ID<255) ChangeOperatingMode(ID, OPERATING_MODE_POSITION);
    CallbackCmd_Goal(DynamixelGoal().set__id_list(id_list)
        .set__position_deg(msg.position_deg)
        .set__profile_vel_deg_s(msg.profile_vel_deg_s)
        .set__profile_acc_deg_ss(msg.profile_acc_deg_ss)
    );
}

void DynamixelHandler::CallbackCmd_X_Velocity(const DynamixelControlXVelocity& msg) {
    if ( msg.id_list.empty() ) return; // id_list が空の場合は何もしない
    if (verbose_callback_) ROS_INFO_STREAM(id_list_layout(msg.id_list, "Velocity ctrl(X), ID"));
    vector<uint16_t> id_list(msg.id_list);
    for ( auto& ID : id_list ) if ( !check_series(ID, SERIES_X) ) { // SERIES_X か dummy であることを確認
        if(verbose_callback_) ROS_WARN("  ID [%d] is not X series", ID);
        ID = 255; // 不適な series の場合はid=255にして，以降，無視されるようにする 
    }
    for ( auto ID : id_list ) if(ID<255) ChangeOperatingMode(ID, OPERATING_MODE_VELOCITY);
    CallbackCmd_Goal(DynamixelGoal().set__id_list(id_list)
        .set__velocity_deg_s(msg.velocity_deg_s)
        .set__profile_acc_deg_ss(msg.profile_acc_deg_ss)
    );
}

void DynamixelHandler::CallbackCmd_X_Current(const DynamixelControlXCurrent& msg) {
    if ( msg.id_list.empty() ) return; // id_list が空の場合は何もしない
    if (verbose_callback_) ROS_INFO_STREAM(id_list_layout(msg.id_list, "Current ctrl(X), ID"));
    vector<uint16_t> id_list(msg.id_list);
    for ( auto& ID : id_list ) if ( !check_series(ID, SERIES_X) ) { // SERIES_X か dummy であることを確認
        if(verbose_callback_) ROS_WARN("  ID [%d] is not X series", ID);
        ID = 255; // 不適な series の場合はid=255にして，以降，無視されるようにする 
    }
    for ( auto ID : id_list ) if(ID<255) ChangeOperatingMode(ID, OPERATING_MODE_CURRENT);
    CallbackCmd_Goal(DynamixelGoal().set__id_list(id_list)
        .set__current_ma(msg.current_ma)
    );
}

void DynamixelHandler::CallbackCmd_X_CurrentBasePosition(const DynamixelControlXCurrentBasePosition& msg) {
    if ( msg.id_list.empty() ) return; // id_list が空の場合は何もしない
    if (verbose_callback_) ROS_INFO_STREAM(id_list_layout(msg.id_list, "Current-base Position ctrl(X), ID"));
    vector<uint16_t> id_list(msg.id_list);
    for ( auto& ID : id_list ) if ( !check_series(ID, SERIES_X) ) { // SERIES_X か dummy であることを確認
        if(verbose_callback_) ROS_WARN("  ID [%d] is not X series", ID);
        ID = 255; // 不適な series の場合はid=255にして，以降，無視されるようにする 
    }
    vector<double> position_deg(msg.position_deg);
    if ( size_t N_rot=msg.rotation.size(); N_rot==0 || N_rot==id_list.size() ) {
        if (position_deg.size() < N_rot) position_deg.resize(N_rot); // 0埋め拡張
        for ( size_t i=0; i<N_rot; i++ ) position_deg[i] += msg.rotation[i]*360;
    } else { // 0 < N_rot < id_list.size() の場合は不適
        if (verbose_callback_) ROS_WARN("   Field [rotation] is size mismatch");
    }
    for ( auto ID : id_list ) if(ID<255) ChangeOperatingMode(ID, OPERATING_MODE_CURRENT_BASE_POSITION);
    CallbackCmd_Goal(DynamixelGoal().set__id_list(id_list)
        .set__current_ma(msg.current_ma)
        .set__position_deg(position_deg)
        .set__profile_vel_deg_s(msg.profile_vel_deg_s)
        .set__profile_acc_deg_ss(msg.profile_acc_deg_ss)
    );
}

void DynamixelHandler::CallbackCmd_X_ExtendedPosition(const DynamixelControlXExtendedPosition& msg) {
    if ( msg.id_list.empty() ) return; // id_list が空の場合は何もしない
    if (verbose_callback_) ROS_INFO_STREAM(id_list_layout(msg.id_list, "Extended Position ctrl(X), ID"));
    vector<uint16_t> id_list(msg.id_list);
    for ( auto& ID : id_list ) if ( !check_series(ID, SERIES_X) ) { // SERIES_X か dummy であることを確認
        if(verbose_callback_) ROS_WARN("  ID [%d] is not X series", ID);
        ID = 255; // 不適な series の場合はid=255にして，以降，無視されるようにする 
    }
    vector<double> position_deg(msg.position_deg);
    if ( size_t N_rot=msg.rotation.size(); N_rot==0 || N_rot==id_list.size() ) {
        if (position_deg.size() < N_rot) position_deg.resize(N_rot); // 0埋め拡張
        for ( size_t i=0; i<N_rot; i++ ) position_deg[i] += msg.rotation[i]*360;
    } else { // 0 < N_rot < id_list.size() の場合は不適
        if (verbose_callback_) ROS_WARN("   Field [rotation] is size mismatch");
    }
    for ( auto ID : id_list ) if(ID<255) ChangeOperatingMode(ID, OPERATING_MODE_EXTENDED_POSITION);
    CallbackCmd_Goal(DynamixelGoal().set__id_list(id_list)
        .set__position_deg(position_deg)
        .set__profile_vel_deg_s(msg.profile_vel_deg_s)
        .set__profile_acc_deg_ss(msg.profile_acc_deg_ss)
    );
}

void DynamixelHandler::CallbackCmd_P_Pwm(const DynamixelControlPPwm& msg) {
    if ( msg.id_list.empty() ) return; // id_list が空の場合は何もしない
    if (verbose_callback_) ROS_INFO_STREAM(id_list_layout(msg.id_list, "PWM ctrl(P), ID"));
    vector<uint16_t> id_list(msg.id_list);
    for ( auto& ID : id_list ) if ( !check_series(ID, SERIES_P) ) { // SERIES_P か dummy であることを確認
        if(verbose_callback_) ROS_WARN("  ID [%d] is not P series", ID);
        ID = 255; // 不適な series の場合はid=255にして，以降，無視されるようにする 
    }
    for ( auto ID : id_list ) if(ID<255) ChangeOperatingMode(ID, OPERATING_MODE_PWM);
    CallbackCmd_Goal(DynamixelGoal().set__id_list(id_list)
        .set__pwm_percent(msg.pwm_percent)
    );
}

void DynamixelHandler::CallbackCmd_P_Current(const DynamixelControlPCurrent& msg) {
    if ( msg.id_list.empty() ) return; // id_list が空の場合は何もしない
    if (verbose_callback_) ROS_INFO_STREAM(id_list_layout(msg.id_list, "Current ctrl(P), ID"));
    vector<uint16_t> id_list(msg.id_list);
    for ( auto& ID : id_list ) if ( !check_series(ID, SERIES_P) ) { // SERIES_P か dummy であることを確認
        if(verbose_callback_) ROS_WARN("  ID [%d] is not P series", ID);
        ID = 255; // 不適な series の場合はid=255にして，以降，無視されるようにする 
    }
    for ( auto ID : id_list ) if(ID<255) ChangeOperatingMode(ID, OPERATING_MODE_CURRENT);
    CallbackCmd_Goal(DynamixelGoal().set__id_list(id_list)
        .set__current_ma(msg.current_ma)
    );
}

void DynamixelHandler::CallbackCmd_P_Velocity(const DynamixelControlPVelocity& msg) {
    if ( msg.id_list.empty() ) return; // id_list が空の場合は何もしない
    if (verbose_callback_) ROS_INFO_STREAM(id_list_layout(msg.id_list, "Velocity ctrl(P), ID"));
    vector<uint16_t> id_list(msg.id_list);
    for ( auto& ID : id_list ) if ( !check_series(ID, SERIES_P) ) { // SERIES_P か dummy であることを確認
        if(verbose_callback_) ROS_WARN("  ID [%d] is not P series", ID);
        ID = 255; // 不適な series の場合はid=255にして，以降，無視されるようにする 
    }
    for ( auto ID : id_list ) if(ID<255) ChangeOperatingMode(ID, OPERATING_MODE_VELOCITY);
    CallbackCmd_Goal(DynamixelGoal().set__id_list(id_list)
        .set__current_ma(msg.current_ma)
        .set__velocity_deg_s(msg.velocity_deg_s)
        .set__profile_acc_deg_ss(msg.profile_acc_deg_ss)
    );
}

void DynamixelHandler::CallbackCmd_P_Position(const DynamixelControlPPosition& msg) {
    if ( msg.id_list.empty() ) return; // id_list が空の場合は何もしない
    if (verbose_callback_) ROS_INFO_STREAM(id_list_layout(msg.id_list, "Position ctrl(P), ID"));
    vector<uint16_t> id_list(msg.id_list);
    for ( auto& ID : id_list ) if ( !check_series(ID, SERIES_P) ) { // SERIES_P か dummy であることを確認
        if(verbose_callback_) ROS_WARN("  ID [%d] is not P series", ID);
        ID = 255; // 不適な series の場合はid=255にして，以降，無視されるようにする 
    }
    for ( auto ID : id_list ) if(ID<255) ChangeOperatingMode(ID, OPERATING_MODE_POSITION);
    CallbackCmd_Goal(DynamixelGoal().set__id_list(id_list)
        .set__current_ma(msg.current_ma)
        .set__position_deg(msg.position_deg)
        .set__velocity_deg_s(msg.velocity_deg_s)
        .set__profile_acc_deg_ss(msg.profile_acc_deg_ss)
    );
}

void DynamixelHandler::CallbackCmd_P_ExtendedPosition(const DynamixelControlPExtendedPosition& msg) {
    if ( msg.id_list.empty() ) return; // id_list が空の場合は何もしない
    if (verbose_callback_) ROS_INFO_STREAM(id_list_layout(msg.id_list, "Extended Position ctrl(P), ID"));
    vector<uint16_t> id_list(msg.id_list);
    for ( auto& ID : id_list ) if ( !check_series(ID, SERIES_P) ) { // SERIES_P か dummy であることを確認
        if(verbose_callback_) ROS_WARN("  ID [%d] is not P series", ID);
        ID = 255; // 不適な series の場合はid=255にして，以降，無視されるようにする 
    }
    vector<double> position_deg(msg.position_deg);
    if ( size_t N_rot=msg.rotation.size(); N_rot==0 || N_rot==id_list.size() ) {
        if (position_deg.size() < N_rot) position_deg.resize(N_rot); // 0埋め拡張
        for ( size_t i=0; i<N_rot; i++ ) position_deg[i] += msg.rotation[i]*360;
    } else { // 0 < N_rot < id_list.size() の場合は不適
        if (verbose_callback_) ROS_WARN("   Field [rotation] is size mismatch");
    }
    for ( auto ID : id_list ) if(ID<255) ChangeOperatingMode(ID, OPERATING_MODE_EXTENDED_POSITION);
    CallbackCmd_Goal(DynamixelGoal().set__id_list(id_list)
        .set__current_ma(msg.current_ma)
        .set__position_deg(position_deg)
        .set__velocity_deg_s(msg.velocity_deg_s)
        .set__profile_vel_deg_s(msg.profile_vel_deg_s)
        .set__profile_acc_deg_ss(msg.profile_acc_deg_ss)
    );
}



void DynamixelHandler::CallbackCmd_Pro_Current(const DynamixelControlProCurrent& msg) {
    if ( msg.id_list.empty() ) return; // id_list が空の場合は何もしない
    if (verbose_callback_) ROS_INFO_STREAM(id_list_layout(msg.id_list, "Current ctrl(Pro), ID"));
    vector<uint16_t> id_list(msg.id_list);
    for ( auto& ID : id_list ) if ( !check_series(ID, SERIES_PRO) ) { // SERIES_PRO か dummy であることを確認
        if(verbose_callback_) ROS_WARN("  ID [%d] is not Pro series", ID);
        ID = 255; // 不適な series の場合はid=255にして，以降，無視されるようにする 
    }
    for ( auto ID : id_list ) if(ID<255) ChangeOperatingMode(ID, OPERATING_MODE_CURRENT);
    CallbackCmd_Goal(DynamixelGoal().set__id_list(id_list)
        .set__current_ma(msg.current_ma)
    );
}

void DynamixelHandler::CallbackCmd_Pro_Velocity(const DynamixelControlProVelocity& msg) {
    if ( msg.id_list.empty() ) return; // id_list が空の場合は何もしない
    if (verbose_callback_) ROS_INFO_STREAM(id_list_layout(msg.id_list, "Velocity ctrl(Pro), ID"));
    vector<uint16_t> id_list(msg.id_list);
    for ( auto& ID : id_list ) if ( !check_series(ID, SERIES_PRO) ) { // SERIES_PRO か dummy であることを確認
        if(verbose_callback_) ROS_WARN("  ID [%d] is not Pro series", ID);
        ID = 255; // 不適な series の場合はid=255にして，以降，無視されるようにする 
    }
    for ( auto ID : id_list ) if(ID<255) ChangeOperatingMode(ID, OPERATING_MODE_VELOCITY);
    CallbackCmd_Goal(DynamixelGoal().set__id_list(id_list)
        .set__current_ma(msg.current_ma)
        .set__velocity_deg_s(msg.velocity_deg_s)
        .set__profile_acc_deg_ss(msg.acceleration_deg_ss)
    );
}

void DynamixelHandler::CallbackCmd_Pro_Position(const DynamixelControlProPosition& msg) {
    if ( msg.id_list.empty() ) return; // id_list が空の場合は何もしない
    if (verbose_callback_) ROS_INFO_STREAM(id_list_layout(msg.id_list, "Position ctrl(Pro), ID"));
    vector<uint16_t> id_list(msg.id_list);
    for ( auto& ID : id_list ) if ( !check_series(ID, SERIES_PRO) ) { // SERIES_PRO か dummy であることを確認
        if(verbose_callback_) ROS_WARN("  ID [%d] is not Pro series", ID);
        ID = 255; // 不適な series の場合はid=255にして，以降，無視されるようにする 
    }
    for ( auto ID : id_list ) if(ID<255) ChangeOperatingMode(ID, OPERATING_MODE_POSITION);
    CallbackCmd_Goal(DynamixelGoal().set__id_list(id_list)
        .set__current_ma(msg.current_ma)
        .set__position_deg(msg.position_deg)
        .set__velocity_deg_s(msg.velocity_deg_s)
        .set__profile_acc_deg_ss(msg.acceleration_deg_ss)
    );
}

void DynamixelHandler::CallbackCmd_Pro_ExtendedPosition(const DynamixelControlProExtendedPosition& msg) {
    if ( msg.id_list.empty() ) return; // id_list が空の場合は何もしない
    if (verbose_callback_) ROS_INFO_STREAM(id_list_layout(msg.id_list, "Extended Position ctrl(Pro), ID"));
    vector<uint16_t> id_list(msg.id_list);
    for ( auto& ID : id_list ) if ( !check_series(ID, SERIES_PRO) ) { // SERIES_PRO か dummy であることを確認
        if(verbose_callback_) ROS_WARN("  ID [%d] is not Pro series", ID);
        ID = 255; // 不適な series の場合はid=255にして，以降，無視されるようにする 
    }
    vector<double> position_deg(msg.position_deg);
    if ( size_t N_rot=msg.rotation.size(); N_rot==0 || N_rot==id_list.size() ) {
        if (position_deg.size() < N_rot) position_deg.resize(N_rot); // 0埋め拡張
        for ( size_t i=0; i<N_rot; i++ ) position_deg[i] += msg.rotation[i]*360;
    } else { // 0 < N_rot < id_list.size() の場合は不適
        if (verbose_callback_) ROS_WARN("   Field [rotation] is size mismatch");
    }
    for ( auto ID : id_list ) if(ID<255) ChangeOperatingMode(ID, OPERATING_MODE_EXTENDED_POSITION);
    CallbackCmd_Goal(DynamixelGoal().set__id_list(id_list)
        .set__current_ma(msg.current_ma)
        .set__position_deg(position_deg)
        .set__velocity_deg_s(msg.velocity_deg_s)
        .set__profile_vel_deg_s(msg.velocity_deg_s)
        .set__profile_acc_deg_ss(msg.acceleration_deg_ss)
    );
}
constexpr auto warn_s = "(size mismatch)";
constexpr auto warn_n = "(nan/inf value input)";
bool is_valid(const vector<double>& vec) { return std::any_of(vec.begin(), vec.end(), [](auto x){ return std::isfinite(x); }); }

void DynamixelHandler::CallbackCmd_Goal(const DynamixelGoal& msg) { // mutex_goal_を追加し， 排他制御を行う． log出力を排他制御に入れないように注意する．
    // msg.id_list内のIDの妥当性を確認
    vector<uint8_t> valid_id_list;
    for (auto id : msg.id_list) if ( is_in(id, id_set_) ) valid_id_list.push_back(id);
    if ( valid_id_list.empty() ) return; // 妥当なIDがない場合は何もしない, 以降は必ず妥当なIDが存在する
    // 各要素の個数をもとに, それぞれの要素が妥当かどうか確認
    size_t N = msg.id_list.size();
    size_t n_pwm = msg.pwm_percent   .size(), n_pos = msg.position_deg      .size();
    size_t n_cur = msg.current_ma    .size(), n_pv  = msg.profile_vel_deg_s .size();
    size_t n_vel = msg.velocity_deg_s.size(), n_pa  = msg.profile_acc_deg_ss.size();
    bool valid_pwm = N==n_pwm && is_valid(msg.pwm_percent   ), valid_pos = N==n_pos && is_valid(msg.position_deg      );
    bool valid_cur = N==n_cur && is_valid(msg.current_ma    ), valid_pv  = N==n_pv  && is_valid(msg.profile_vel_deg_s ) ;
    bool valid_vel = N==n_vel && is_valid(msg.velocity_deg_s), valid_pa  = N==n_pa  && is_valid(msg.profile_acc_deg_ss) ;
    // log出力
    if ( verbose_callback_ ) {
        ROS_INFO("Goal cmd '%zu' servo(s) are tried to update", valid_id_list.size());
        ROS_INFO_STREAM(id_list_layout(valid_id_list, "  ID")); // valid_id_list はここで必要なので，わざわざ独立したvectorとして用意している
        if ( n_pwm>0 ){ if( valid_pwm) ROS_INFO("  - updated: goal pwm     "); else ROS_WARN("  - skieped: goal pwm      %s", n_pwm!=N?warn_s:warn_n);}
        if ( n_cur>0 ){ if( valid_cur) ROS_INFO("  - updated: goal current "); else ROS_WARN("  - skieped: goal current  %s", n_cur!=N?warn_s:warn_n);}
        if ( n_vel>0 ){ if( valid_vel) ROS_INFO("  - updated: goal velocity"); else ROS_WARN("  - skieped: goal velocity %s", n_vel!=N?warn_s:warn_n);}
        if ( n_pos>0 ){ if( valid_pos) ROS_INFO("  - updated: goal position"); else ROS_WARN("  - skieped: goal position %s", n_pos!=N?warn_s:warn_n);}
        if ( n_pv >0 ){ if( valid_pv ) ROS_INFO("  - updated: profile vel. "); else ROS_WARN("  - skieped: profile vel.  %s", n_pv !=N?warn_s:warn_n);}
        if ( n_pa >0 ){ if( valid_pa ) ROS_INFO("  - updated: profile acc. "); else ROS_WARN("  - skieped: profile acc.  %s", n_pa !=N?warn_s:warn_n);}
    } 
    if ( !valid_pwm && !valid_cur && !valid_vel && !valid_pos && !valid_pv && !valid_pa ) return; // 有効なIDがあるが，有効な要素がない場合は何もしない
    // goal_indice_write_　の更新
    if ( valid_pwm ) goal_indice_write_.insert(GOAL_PWM     ); 
    if ( valid_cur ) goal_indice_write_.insert(GOAL_CURRENT ); 
    if ( valid_vel ) goal_indice_write_.insert(GOAL_VELOCITY);
    if ( valid_pos ) goal_indice_write_.insert(GOAL_POSITION);
    if ( valid_pv  ) goal_indice_write_.insert(PROFILE_VEL  );
    if ( valid_pa  ) goal_indice_write_.insert(PROFILE_ACC  ); 
    // goal_w_ と updated_id_goal_ の更新
    for (size_t i=0; i<msg.id_list.size(); i++) if ( auto ID = msg.id_list[i]; is_in(ID, valid_id_list) ) { // 順番がずれるのでわざとこの書き方をしている．
        updated_id_goal_.insert(ID);
        auto& g = goal_w_[ID];
        auto& l = limit_r_[ID];
        const bool is_x_series = series_[ID] == SERIES_X;
        const bool is_mode_pos = op_mode_[ID] == OPERATING_MODE_POSITION;
        //                                           シンプルに絶対値を制限内に収める
        if ( valid_pwm ) g[GOAL_PWM     ] = clamp( msg.pwm_percent[i]       , -l[PWM_LIMIT     ], l[PWM_LIMIT     ]);
        if ( valid_cur ) g[GOAL_CURRENT ] = clamp( msg.current_ma[i]        , -l[CURRENT_LIMIT ], l[CURRENT_LIMIT ]);
        if ( valid_vel ) g[GOAL_VELOCITY] = clamp( msg.velocity_deg_s[i]*DEG, -l[VELOCITY_LIMIT], l[VELOCITY_LIMIT]);
        //                                           モードによって制限値が変わるので注意して，制限内に収める
        if ( valid_pos ) g[GOAL_POSITION] = clamp( msg.position_deg[i]*DEG, is_mode_pos ? l[MIN_POSITION_LIMIT] : -256*2*M_PI,
                                                                                       is_mode_pos ? l[MAX_POSITION_LIMIT] : +256*2*M_PI );
        //                                           シリーズによって最大値が変わるので注意して，制限内に収める
        if ( valid_pv )  g[PROFILE_VEL  ] = clamp( msg.profile_vel_deg_s[i] *DEG, 0.0, !is_x_series ? l[VELOCITY_LIMIT   ] 
                                                                                : AddrX::profile_velocity.pulse2val(32767, model_[ID])); 
        if ( valid_pa )  g[PROFILE_ACC  ] = clamp( msg.profile_acc_deg_ss[i]*DEG, 0.0, !is_x_series ? l[ACCELERATION_LIMIT] 
                                                                                : AddrX::profile_acceleration.pulse2val(32767, model_[ID])); 
    }
    if (verbose_callback_) ROS_INFO("============+===========+============");
}

void DynamixelHandler::CallbackCmd_Gain(const DynamixelGain& msg) { // mutex_gain_を追加し， 排他制御を行う． log出力を排他制御に入れないように注意する．
    // msg.id_list内のIDの妥当性を確認
    vector<uint8_t> valid_id_list;
    for (auto id : msg.id_list) if ( is_in(id, id_set_) ) valid_id_list.push_back(id);
    if ( valid_id_list.empty() ) return; // 妥当なIDがない場合は何もしない, 以降は必ず妥当なIDが存在する
    // 各要素の個数をもとに, それぞれの要素が妥当かどうか確認
    size_t N = msg.id_list.size();
    size_t n_vi = msg.velocity_i_gain_pulse.size(), n_vp = msg.velocity_p_gain_pulse.size();
    size_t n_pd = msg.position_d_gain_pulse.size(), n_pi = msg.position_i_gain_pulse.size();
    size_t n_pp = msg.position_p_gain_pulse.size();
    size_t n_fa = msg.feedforward_2nd_gain_pulse.size(), n_fv = msg.feedforward_1st_gain_pulse.size();
    bool valid_vi = N==n_vi, valid_vp = N==n_vp;
    bool valid_pd = N==n_pd, valid_pi = N==n_pi;
    bool valid_pp = N==n_pp;
    bool valid_fa = N==n_fa, valid_fv = N==n_fv; // uint16_t なので，nanは存在しない
    // log出力
    if ( verbose_callback_ ) {
        ROS_INFO("Gain cmd, '%zu' servo(s) are tried to update", valid_id_list.size());
        ROS_INFO_STREAM(id_list_layout(valid_id_list, "  ID")); // valid_id_list はここで必要なので，わざわざ独立したvectorとして用意している
        if ( n_vi>0 ){ if(valid_vi) ROS_INFO("  - updated: velocity i gain "); else ROS_WARN("  - skieped: velocity i gain %s", warn_s);}
        if ( n_vp>0 ){ if(valid_vp) ROS_INFO("  - updated: velocity p gain "); else ROS_WARN("  - skieped: velocity p gain %s", warn_s);}
        if ( n_pd>0 ){ if(valid_pd) ROS_INFO("  - updated: position d gain "); else ROS_WARN("  - skieped: position d gain %s", warn_s);}
        if ( n_pi>0 ){ if(valid_pi) ROS_INFO("  - updated: position i gain "); else ROS_WARN("  - skieped: position i gain %s", warn_s);}
        if ( n_pp>0 ){ if(valid_pp) ROS_INFO("  - updated: position p gain "); else ROS_WARN("  - skieped: position p gain %s", warn_s);}
        if ( n_fa>0 ){ if(valid_fa) ROS_INFO("  - updated: feedforward 2nd gain "); else ROS_WARN("  - skieped: feedforward 2nd gain %s", warn_s);}
        if ( n_fv>0 ){ if(valid_fv) ROS_INFO("  - updated: feedforward 1st gain "); else ROS_WARN("  - skieped: feedforward 1st gain %s", warn_s);}
    }
    if ( !valid_vi && !valid_vp && !valid_pd && !valid_pi && !valid_pp && !valid_fa && !valid_fv ) return; // 有効なIDがあるが，有効な要素がない場合は何もしない
    // gain_indice_write_　の更新
    if ( valid_vi ) gain_indice_write_.insert(VELOCITY_I_GAIN);
    if ( valid_vp ) gain_indice_write_.insert(VELOCITY_P_GAIN);
    if ( valid_pd ) gain_indice_write_.insert(POSITION_D_GAIN);
    if ( valid_pi ) gain_indice_write_.insert(POSITION_I_GAIN);
    if ( valid_pp ) gain_indice_write_.insert(POSITION_P_GAIN);
    if ( valid_fa ) gain_indice_write_.insert(FEEDFORWARD_ACC_GAIN);
    if ( valid_fv ) gain_indice_write_.insert(FEEDFORWARD_VEL_GAIN);
    // gain_w_ と updated_id_gain_ の更新
    for (size_t i=0; i<msg.id_list.size(); i++) if ( auto ID = msg.id_list[i]; is_in(msg.id_list[i], valid_id_list) ) { // 順番がずれるのでわざとこの書き方をしている．
        updated_id_gain_.insert(ID);
        if ( valid_vi ) gain_w_[ID][VELOCITY_I_GAIN] = msg.velocity_i_gain_pulse[i];
        if ( valid_vp ) gain_w_[ID][VELOCITY_P_GAIN] = msg.velocity_p_gain_pulse[i];
        if ( valid_pd ) gain_w_[ID][POSITION_D_GAIN] = msg.position_d_gain_pulse[i];
        if ( valid_pi ) gain_w_[ID][POSITION_I_GAIN] = msg.position_i_gain_pulse[i];
        if ( valid_pp ) gain_w_[ID][POSITION_P_GAIN] = msg.position_p_gain_pulse[i];
        if ( valid_fa ) gain_w_[ID][FEEDFORWARD_ACC_GAIN] = msg.feedforward_2nd_gain_pulse[i];
        if ( valid_fv ) gain_w_[ID][FEEDFORWARD_VEL_GAIN] = msg.feedforward_1st_gain_pulse[i];
    } // 値の範囲がまちまちすぎるので，チェックしない．
    if (verbose_callback_) ROS_INFO("========+=========+=========+========");
    
}

void DynamixelHandler::CallbackCmd_Limit(const DynamixelLimit& msg) { // mutex_limit_を追加し， 排他制御を行う． log出力を排他制御に入れないように注意する．
    // msg.id_list内のIDの妥当性を確認
    vector<uint8_t> valid_id_list;
    for (auto id : msg.id_list) if ( is_in(id, id_set_) ) valid_id_list.push_back(id);
    if ( valid_id_list.empty() ) return; // 妥当なIDがない場合は何もしない, 以降は必ず妥当なIDが存在する
    // 各要素の個数をもとに, それぞれの要素が妥当かどうか確認
    size_t N = msg.id_list.size();
    size_t n_temp = msg.temperature_limit_degc   .size();
    size_t n_maxv = msg.max_voltage_limit_v      .size(), n_minv = msg.min_voltage_limit_v      .size();
    size_t n_pwm  = msg.pwm_limit_percent        .size(), n_cur  = msg.current_limit_ma         .size();
    size_t n_acc  = msg.acceleration_limit_deg_ss.size(), n_vel  = msg.velocity_limit_deg_s     .size();
    size_t n_maxp = msg.max_position_limit_deg   .size(), n_minp = msg.min_position_limit_deg   .size();
    bool valid_temp = N==n_temp && is_valid(msg.temperature_limit_degc   );
    bool valid_maxv = N==n_maxv && is_valid(msg.max_voltage_limit_v      ), valid_minv = N==n_minv && is_valid(msg.min_voltage_limit_v      );
    bool valid_pwm  = N==n_pwm  && is_valid(msg.pwm_limit_percent        ), valid_cur  = N==n_cur  && is_valid(msg.current_limit_ma         );
    bool valid_acc  = N==n_acc  && is_valid(msg.acceleration_limit_deg_ss), valid_vel  = N==n_vel  && is_valid(msg.velocity_limit_deg_s     );
    bool valid_maxp = N==n_maxp && is_valid(msg.max_position_limit_deg   ), valid_minp = N==n_minp && is_valid(msg.min_position_limit_deg   );
    // log出力
    if ( verbose_callback_ ) {
        ROS_INFO("Limit cmd, '%zu' servo(s) are tried to update", valid_id_list.size());
        ROS_INFO_STREAM(id_list_layout(valid_id_list, "  ID")); // valid_id_list はここで必要なので，わざわざ独立したvectorとして用意している
        if ( n_temp>0 ){ if( valid_temp) ROS_INFO("  - updated: temperature limit "); else ROS_WARN("  - skieped: temperature limit  %s", n_temp!=N?warn_s:warn_n);}
        if ( n_maxv>0 ){ if( valid_maxv) ROS_INFO("  - updated: max voltage limit "); else ROS_WARN("  - skieped: max voltage limit  %s", n_maxv!=N?warn_s:warn_n);}
        if ( n_minv>0 ){ if( valid_minv) ROS_INFO("  - updated: min voltage limit "); else ROS_WARN("  - skieped: min voltage limit  %s", n_minv!=N?warn_s:warn_n);}
        if ( n_pwm >0 ){ if( valid_pwm ) ROS_INFO("  - updated: pwm limit         "); else ROS_WARN("  - skieped: pwm limit          %s", n_pwm !=N?warn_s:warn_n);}
        if ( n_cur >0 ){ if( valid_cur ) ROS_INFO("  - updated: current limit     "); else ROS_WARN("  - skieped: current limit      %s", n_cur !=N?warn_s:warn_n);}
        if ( n_acc >0 ){ if( valid_acc ) ROS_INFO("  - updated: acceleration limit"); else ROS_WARN("  - skieped: acceleration limit %s", n_acc !=N?warn_s:warn_n);}
        if ( n_vel >0 ){ if( valid_vel ) ROS_INFO("  - updated: velocity limit    "); else ROS_WARN("  - skieped: velocity limit     %s", n_vel !=N?warn_s:warn_n);}
        if ( n_maxp>0 ){ if( valid_maxp) ROS_INFO("  - updated: max position limit"); else ROS_WARN("  - skieped: max position limit %s", n_maxp!=N?warn_s:warn_n);}
        if ( n_minp>0 ){ if( valid_minp) ROS_INFO("  - updated: min position limit"); else ROS_WARN("  - skieped: min position limit %s", n_minp!=N?warn_s:warn_n);}
    }
    if ( !valid_temp && !valid_maxv && !valid_minv && !valid_pwm 
         && !valid_cur && !valid_acc && !valid_vel && !valid_maxp && !valid_minp ) return; // 有効なIDがあるが，有効な要素がない場合は何もしない
    // limit_indice_write_　の更新
    if ( valid_temp ) limit_indice_write_.insert(TEMPERATURE_LIMIT);
    if ( valid_maxv ) limit_indice_write_.insert(MAX_VOLTAGE_LIMIT);
    if ( valid_minv ) limit_indice_write_.insert(MIN_VOLTAGE_LIMIT);
    if ( valid_pwm  ) limit_indice_write_.insert(PWM_LIMIT        );
    if ( valid_cur  ) limit_indice_write_.insert(CURRENT_LIMIT    );
    if ( valid_acc  ) limit_indice_write_.insert(ACCELERATION_LIMIT);
    if ( valid_vel  ) limit_indice_write_.insert(VELOCITY_LIMIT    );
    if ( valid_maxp ) limit_indice_write_.insert(MAX_POSITION_LIMIT);
    if ( valid_minp ) limit_indice_write_.insert(MIN_POSITION_LIMIT);
    // limit_w_ と updated_id_limit_ の更新
    for (size_t i=0; i<msg.id_list.size(); i++) if ( auto ID = msg.id_list[i]; is_in(msg.id_list[i], valid_id_list) ) { // 順番がずれるのでわざとこの書き方をしている．
        updated_id_limit_.insert(ID);
        if ( valid_temp ) limit_w_[ID][TEMPERATURE_LIMIT] = msg.temperature_limit_degc[i];
        if ( valid_maxv ) limit_w_[ID][MAX_VOLTAGE_LIMIT] = msg.max_voltage_limit_v[i];
        if ( valid_minv ) limit_w_[ID][MIN_VOLTAGE_LIMIT] = msg.min_voltage_limit_v[i];
        if ( valid_pwm  ) limit_w_[ID][PWM_LIMIT        ] = msg.pwm_limit_percent[i];
        if ( valid_cur  ) limit_w_[ID][CURRENT_LIMIT    ] = msg.current_limit_ma[i];
        if ( valid_acc  ) limit_w_[ID][ACCELERATION_LIMIT] = deg2rad(msg.acceleration_limit_deg_ss[i]);
        if ( valid_vel  ) limit_w_[ID][VELOCITY_LIMIT    ] = deg2rad(msg.velocity_limit_deg_s[i]     );
        if ( valid_maxp ) limit_w_[ID][MAX_POSITION_LIMIT] = deg2rad(msg.max_position_limit_deg[i]   );
        if ( valid_minp ) limit_w_[ID][MIN_POSITION_LIMIT] = deg2rad(msg.min_position_limit_deg[i]   );
    } // 値の範囲がまちまちすぎるので，チェックしない．
    if (verbose_callback_) ROS_INFO("======+=======+=======+=======+======");
}