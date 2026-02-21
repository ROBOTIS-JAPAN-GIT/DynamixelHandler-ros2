#include "dynamixel_handler.hpp"
#include "myUtils/logging_like_ros1.hpp"
#include "myUtils/make_iterator_convenient.hpp"
// 一定時間待つための関数
static void rsleep(int millisec) { std::this_thread::sleep_for(std::chrono::milliseconds(millisec));}
static constexpr double DEG = M_PI/180.0; // degを単位に持つ数字に掛けるとradになる

//* 基本機能をまとめた関数たち
// 各シリーズのDynamixelを検出する．
bool DynamixelHandler::tryAddDynamixels(const set<id_t>& scan_id_set, uint32_t num_expected, uint32_t times_retry) {
    set<id_t> target_id_set;
    if (dyn_comm_.wait_time_ping_broadcast() < scan_id_set.size() * dyn_comm_.dead_time_retry_ping()){
        ROS_WARN("  Bulk scanning ID: %d ~ %d \x1b[999D\x1b[1A", *scan_id_set.begin(), *scan_id_set.rbegin());
        for (const auto id : dyn_comm_.Ping_broadcast()) if (is_in(id, scan_id_set)) target_id_set.insert(id);
    }
    if (target_id_set.empty()) target_id_set = scan_id_set;
    for (const auto id : target_id_set) {
        ROS_INFO("  Scanning ID: %d           \x1b[999D\x1b[1A", id);
        AddDynamixel(id);
        if ( !rclcpp::ok() ) return false;
    }
    auto num_found = id_set_.size();
    // 再帰から脱する条件
    if ( times_retry <= 0 ) return false;
    if ( num_found != 0 && num_found >= num_expected ) return true;
    // 再帰処理
    if ( num_found < num_expected ) ROS_WARN("  '%ld' dynamixels are not found yet", num_expected-num_found );
    if ( num_expected == 0 )        ROS_WARN("  No dynamixels are found yet" );
    ROS_WARN("   > %d times retry left ( %ld/%s servos )", times_retry, num_found, num_expected==0?"?":std::to_string(num_expected).c_str());
    rsleep(100);
    return tryAddDynamixels(scan_id_set, num_expected, times_retry-1);
}

bool DynamixelHandler::DummyUpDynamixel(id_t id){
    if ( is_in(id, id_set_) ) return false; // すでに登録されている場合は失敗
    ROS_INFO("   *    Dummy   servo ID [%d] is added", id);
    model_[id] = 0;
    series_[id] = SERIES_UNKNOWN;
    id_set_.insert(id);
    // limitのみgoal値の制限に用いられるので，動作に必要なもののみ仮の値を入れておく
    limit_w_[id][PWM_LIMIT         ] = 100; // 最大
    limit_w_[id][CURRENT_LIMIT     ] = 20000; // 適当に20A
    limit_w_[id][ACCELERATION_LIMIT] = 100000*DEG; // 適当に大きな値
    limit_w_[id][VELOCITY_LIMIT    ] = 100000*DEG; // 適当に大きな値
    limit_w_[id][MAX_POSITION_LIMIT] =  180*DEG; // 最大 
    limit_w_[id][MIN_POSITION_LIMIT] = -180*DEG; // 最小 
    if ( do_clean_hwerr_ ) ClearHardwareError(id); // 現在の状態を変えない
    if ( do_torque_on_ )   TorqueOn(id);           // 現在の状態を変えない
    return true;
}

bool DynamixelHandler::AddDynamixel(id_t id){
    if ( is_in(id, id_set_) ) return true;
    if ( !dyn_comm_.tryPing(id) ) return false;

    auto dyn_model = dyn_comm_.tryRead(AddrCommon::model_number, id);
    switch ( dynamixel_series(dyn_model) ) { 
        case SERIES_X:   ROS_INFO("  *  X  series servo ID [%d] is %s", id, (use_["x"] ? "found" : "ignored"));
            if ( !use_["x"] ) return false;
            model_[id] = dyn_model;
            series_[id] = SERIES_X;
            id_set_.insert(id); break;
        case SERIES_P:   ROS_INFO("  *  P  series servo ID [%d] is %s", id, (use_["p"] ? "found" : "ignored"));
            if ( !use_["p"] ) return false;
            model_[id] = dyn_model;
            series_[id] = SERIES_P;
            id_set_.insert(id); break;
        case SERIES_PRO: ROS_INFO("  * PRO series servo ID [%d] is %s", id, (use_["pro"] ? "found" : "ignored"));
            if ( !use_["pro"] ) return false;
            model_[id] = dyn_model;
            series_[id] = SERIES_PRO;
            id_set_.insert(id); break;
        default: ROS_WARN("  * No supported model [%d] servo ID [%d] is found, ignored", (int)dyn_model, id);
            return false;
    }
    if ( use_fast_read_ && !has_current_sensor(dyn_model) ) {
        ROS_WARN("   no current sensor is found, fast_read is disabled");
        use_fast_read_ = false; // 電流センサを持たないサーボの場合はfast_readを無効化する，エラーを起こしてfast_readに反応しなくなるので．
    }

    WriteBusWatchdog (id, 0.0/*ms*/); // 最初にBusWatchdogを無効化することで，全てのGoal値の書き込みを許可する
    WriteProfileAcc(id, default_["profile_acc_deg_ss"]*DEG ); 
    WriteProfileVel(id, default_["profile_vel_deg_s"]*DEG );

    set<uint8_t> tmp = {id};
    static constexpr tuple<double, uint8_t> complete = {1.0-1e-6, 1};
    while (SyncReadPresent( present_indice_read_, tmp)<complete) {if(!rclcpp::ok()) ROS_STOP("Failed to initial read"); rsleep(50); ROS_INFO_T(5000, "    Reading  present values...");}
    while (SyncReadGoal   ( goal_indice_read_   , tmp)<complete) {if(!rclcpp::ok()) ROS_STOP("Failed to initial read"); rsleep(50); ROS_INFO_T(5000, "    Reading   goal   values...");} 
    while (SyncReadGain   ( gain_indice_read_   , tmp)<complete) {if(!rclcpp::ok()) ROS_STOP("Failed to initial read"); rsleep(50); ROS_INFO_T(5000, "    Reading   gain   values...");} 
    while (SyncReadLimit  ( limit_indice_read_  , tmp)<complete) {if(!rclcpp::ok()) ROS_STOP("Failed to initial read"); rsleep(50); ROS_INFO_T(5000, "    Reading  limit   values...");} 
    while (SyncReadHardwareErrors(tmp)                <complete) {if(!rclcpp::ok()) ROS_STOP("Failed to initial read"); rsleep(50); ROS_INFO_T(5000, "    Reading hardware errors...");}

    tq_mode_[id] = ReadTorqueEnable(id) ? TORQUE_ENABLE : TORQUE_DISABLE;
    op_mode_[id] = ReadOperatingMode(id);
    dv_mode_[id] = ReadDriveMode(id);
    limit_w_[id] = limit_r_[id];
    gain_w_[id] = gain_r_[id];
    goal_w_[id] = goal_r_[id];

    if ( abs(default_["profile_acc_deg_ss"] - goal_r_[id][PROFILE_ACC]/DEG) > 3 ) 
        ROS_WARN("   profile acc. '%2.1f' could not set (now '%2.1f')", default_["profile_acc_deg_ss"], goal_r_[id][PROFILE_ACC]/DEG);
    if ( abs(default_["profile_vel_deg_s"] - goal_r_[id][PROFILE_VEL]/DEG) > 1 ) 
        ROS_WARN("   profile vel. '%2.1f' could not set (now '%2.1f')", default_["profile_vel_deg_s"], goal_r_[id][PROFILE_VEL]/DEG);

    WriteReturnDelayTime(id, default_["return_delay_time_us"]);
    if ( abs(ReadReturnDelayTime(id) - default_["return_delay_time_us"]) > 0.1 ) 
        ROS_WARN("   return delay time '%2.1f' could not set (now '%2.1f')", default_["return_delay_time_us"], ReadReturnDelayTime(id));

    if ( do_clean_hwerr_ ) ClearHardwareError(id); // 現在の状態を変えない
    if ( do_torque_on_ )   TorqueOn(id);           // 現在の状態を変えない
    return true;
}

bool DynamixelHandler::RemoveDynamixel(id_t id){
    if ( !is_in(id, id_set_) ) return true;
    id_set_.erase(id);
    ROS_INFO("   ID [%d] is removed", id);
    return true;
}

// 回転数が消えることを考慮して，モータをリブートする．
bool DynamixelHandler::ClearHardwareError(id_t id){
    if ( !is_in(id, id_set_) ) return false;
    if ( !has_hardware_error_[id] ) return true; // エラーがない場合は何もしない

    const auto now_pos = ReadPresentPosition(id); // 失敗すると0が返って危ないので成功した場合だけリブート処理を行う
    const bool pos_success = !dyn_comm_.timeout_last_read() && !dyn_comm_.comm_error_last_read();
    const auto now_offset = ReadHomingOffset(id); // 失敗すると0が返って危ないので成功した場合だけリブート処理を行う
    const bool offset_success = !dyn_comm_.timeout_last_read() && !dyn_comm_.comm_error_last_read();
    if ( pos_success && offset_success ) {
        int now_rot = (now_pos-now_offset+M_PI) / (2*M_PI);
        if (now_pos < -M_PI) now_rot--;
        const double offset = now_offset+now_rot*(2*M_PI);
        /*リブート処理*/dyn_comm_.Reboot(id); //** RAMのデータが消えるが，この処理の後は電源喪失と同じ扱いなので，ここでは気にしない．
        // homing offsetが書き込めるまで待機する．
        while ( !WriteHomingOffset(id, offset) && rclcpp::ok() ) rsleep(10);
        tq_mode_[id] = TORQUE_DISABLE;
    }
    // 結果を確認
    bool is_clear = (ReadHardwareError(id) == 0b00000000);
    if (is_clear) ROS_INFO ("   ID [%d] is cleared error", id);
    else          ROS_ERROR("   ID [%d] failed to clear error", id);
    return is_clear;
}

// モータの動作モードを変更する．連続で変更するときは1秒のインターバルを入れる
bool DynamixelHandler::ChangeOperatingMode(id_t id, DynamixelOperatingMode mode){
    if ( !is_in(id, id_set_) ) return false;
    if ( op_mode_[id] == mode ) return true; // 既に同じモードの場合は何もしない
    if ( series_[id] == SERIES_UNKNOWN ) { op_mode_[id] = mode; return true;} // ダミーの場合は即時反映
    if ( get_clock()->now().seconds() - when_op_mode_updated_[id] < 1.0 ) rsleep(1000); // 1秒以内に変更した場合は1秒待つ
    // 変更前のトルク状態を確認
    const bool prev_torque = ReadTorqueEnable(id); // read失敗しても0が返ってくるので問題ない
    WriteTorqueEnable(id, false);
    /*モード変更*/WriteOperatingMode(id, mode);  //**RAMのデータが消えるので注意, これは電源喪失とは異なるのでRAMデータの回復を入れる
    // goal_w_を全部書き込んで，本体とこのプログラムの同期行う．
    if ( op_mode_[id] != OPERATING_MODE_PWM      )  WriteGoalPWM     (id, goal_w_[id][GOAL_PWM     ]);
    if ( op_mode_[id] != OPERATING_MODE_CURRENT  )  WriteGoalCurrent (id, goal_w_[id][GOAL_CURRENT ]);
    if ( op_mode_[id] != OPERATING_MODE_VELOCITY )  WriteGoalVelocity(id, goal_w_[id][GOAL_VELOCITY]);
    WriteProfileAcc  (id, goal_w_[id][PROFILE_ACC  ]);
    WriteProfileVel  (id, goal_w_[id][PROFILE_VEL  ]);
    WriteGoalPosition(id, goal_w_[id][GOAL_POSITION]);
    // WriteGains(id, gain_r_[id]);　// ** Gain値のデフォルトも変わる．面倒な．．．
    WriteTorqueEnable(id, prev_torque );
    // 結果を確認
    bool is_changed = (ReadOperatingMode(id) == mode);
    if ( is_changed ) {
        op_mode_[id] = mode;
        when_op_mode_updated_[id] = get_clock()->now().seconds();
        ROS_INFO ("   ID [%d] is changed operating mode [%d]", id, mode);
    } else {
        ROS_ERROR("   ID [%d] failed to change operating mode", id); 
    }
    return is_changed;
}

// モータを停止させてからトルクを入れる．
bool DynamixelHandler::TorqueOn(id_t id){
    if ( !is_in(id, id_set_) ) return false;
    if ( tq_mode_[id] == TORQUE_ENABLE ) return true; // 既にトルクが入っている場合は何もしない
    if ( series_[id] == SERIES_UNKNOWN ) { tq_mode_[id] = TORQUE_ENABLE; return true;} // ダミーの場合は即時反映
    // dynamixel内のgoal値とこのプログラム内のgoal_w_を一致させる．
    const auto now_pos = ReadPresentPosition(id); // 失敗すると0が返って危ないので確認する
    if ( !( dyn_comm_.timeout_last_read() || dyn_comm_.comm_error_last_read() )){
        // 急に動き出さないように，以下のgoal_w_を設定する
        goal_w_[id][GOAL_POSITION] = now_pos; // トルクがオフならDynamixel本体のgoal_positionはpresent_positionと一致している．
        if (op_mode_[id]==OPERATING_MODE_VELOCITY) goal_w_[id][GOAL_VELOCITY] = 0.0;
        if (op_mode_[id]==OPERATING_MODE_CURRENT ) goal_w_[id][GOAL_CURRENT ] = 0.0;
        if (op_mode_[id]==OPERATING_MODE_PWM     ) goal_w_[id][GOAL_PWM     ] = 0.0;
        // goal_w_を全部書き込んで，本体とこのプログラムの同期行う．
        WriteGoalPWM     (id, goal_w_[id][GOAL_PWM     ]);
        WriteGoalCurrent (id, goal_w_[id][GOAL_CURRENT ]);
        WriteGoalVelocity(id, goal_w_[id][GOAL_VELOCITY]);
        WriteProfileAcc  (id, goal_w_[id][PROFILE_ACC  ]);
        WriteProfileVel  (id, goal_w_[id][PROFILE_VEL  ]);
        WriteGoalPosition(id, goal_w_[id][GOAL_POSITION]);
        // WriteGains(id, gain_r_[id]); 　// その他電源喪失時に消えるデータを念のため書き込む
        /*トルクを入れる*/WriteTorqueEnable(id, true);
    }
    // 結果を確認
    tq_mode_[id] = ReadTorqueEnable(id) ? TORQUE_ENABLE : TORQUE_DISABLE;
    if ( tq_mode_[id] != TORQUE_ENABLE ) ROS_ERROR("   ID [%d] failed to enable torque", id);
                                    else ROS_INFO( "   ID [%d] is enabled torque"      , id);
    return tq_mode_[id];
}

// トルクを切る
bool DynamixelHandler::TorqueOff(id_t id){
    if ( !is_in(id, id_set_) ) return false;
    if ( tq_mode_[id] == TORQUE_DISABLE ) return true; // 既にトルクが切られている場合は何もしない
    if ( series_[id] == SERIES_UNKNOWN ) { tq_mode_[id] = TORQUE_DISABLE; return true;} // ダミーの場合は即時反映
    // トルクを切る
    WriteTorqueEnable(id, false);
    // 結果を確認
    tq_mode_[id] = ReadTorqueEnable(id) ? TORQUE_ENABLE : TORQUE_DISABLE;
    if ( tq_mode_[id] != TORQUE_DISABLE ) ROS_ERROR("   ID [%d] failed to disable torque", id);
                                     else ROS_INFO( "   ID [%d] is disabled torque"      , id); 
    return tq_mode_[id];
}

bool DynamixelHandler::UnifyBaudrate(uint64_t baudrate) {
    constexpr static uint8_t BROADCAST_ID = 0xFE;
    const static map<uint64_t, DynamixelBaudrateIndex> baudrate_map = {
        {9600,    BAUDRATE_INDEX_9600},
        {57600,   BAUDRATE_INDEX_57600},
        {115200,  BAUDRATE_INDEX_115200},
        {1000000, BAUDRATE_INDEX_1M},
        {2000000, BAUDRATE_INDEX_2M},
        {3000000, BAUDRATE_INDEX_3M},
        {4000000, BAUDRATE_INDEX_4M},
        // {4500000, BAUDRATE_INDEX_4M5},
        // {6000000, BAUDRATE_INDEX_6M},
        // {10500000,BAUDRATE_INDEX_10M5}
    };
    // check baudrate
    if ( !baudrate_map.count(baudrate) ) {
        ROS_WARN("  === Valid baudrate list ===" );
        for ( const auto& [br, _] : baudrate_map ) ROS_WARN("    - %ld", br);
        ROS_STOP("  Invalid baudrate %ld for Dynamixel", baudrate);
    } // もうここはしゃーない．
    /// make id_list
    for ( const auto& [br, _] : baudrate_map ) {
        if ( br == baudrate ) continue;
        ROS_INFO("  Try to change baudrate %8ld to %ld", br, baudrate);
        dyn_comm_.set_baudrate(br);
        if ( !dyn_comm_.OpenPort() ) ROS_ERROR("  Failed to open port at baudrate %ld", br);
        else  dyn_comm_.Write(AddrX::baudrate, BROADCAST_ID, baudrate_map.at(baudrate)); // baudrateのアドレスはX, P, Proで共通なので... 
        rsleep(16);
    }
    dyn_comm_.set_baudrate(baudrate);
    return dyn_comm_.OpenPort();
}

//* 基本機能たち Read
uint8_t DynamixelHandler::ReadHardwareError(id_t id){ switch ( series_[id] ) {
    case SERIES_X:    return dyn_comm_.tryRead(AddrX::hardware_error_status, id);
    case SERIES_P:    return dyn_comm_.tryRead(AddrP::hardware_error_status, id);
    case SERIES_PRO:  return dyn_comm_.tryRead(AddrPro::hardware_error_status, id);
    default:          return 0b00000000;
} }

bool DynamixelHandler::ReadTorqueEnable(id_t id){ switch ( series_[id] ) {
    case SERIES_X:    return dyn_comm_.tryRead(AddrX::torque_enable, id) == TORQUE_ENABLE;
    case SERIES_P:    return dyn_comm_.tryRead(AddrP::torque_enable, id) == TORQUE_ENABLE;
    case SERIES_PRO:  return dyn_comm_.tryRead(AddrPro::torque_enable, id) == TORQUE_ENABLE;
    default:          return false;
} }

double DynamixelHandler::ReadPresentPWM(id_t id){ switch ( series_[id] ) {
    case SERIES_X:    return AddrX::present_pwm.pulse2val(dyn_comm_.tryRead(AddrX::present_pwm   , id), model_[id]);
    case SERIES_P:    return AddrP::present_pwm.pulse2val(dyn_comm_.tryRead(AddrP::present_pwm   , id), model_[id]);
    case SERIES_PRO:  ROS_WARN("   = PRO series don't support 'present_pwm'"); return AddrPro::present_pwm .pulse2val(0.0, model_[id]);
    default:          return 0.0;
} }

double DynamixelHandler::ReadPresentCurrent(id_t id){ switch ( series_[id] ) {
    case SERIES_X:    return AddrX::present_current  .pulse2val(dyn_comm_.tryRead(AddrX::present_current   , id), model_[id]);
    case SERIES_P:    return AddrP::present_current  .pulse2val(dyn_comm_.tryRead(AddrP::present_current   , id), model_[id]);
    case SERIES_PRO:  return AddrPro::present_current.pulse2val(dyn_comm_.tryRead(AddrPro::present_current , id), model_[id]);
    default:          return 0.0;
} }

double DynamixelHandler::ReadPresentVelocity(id_t id){ switch ( series_[id] ) {
    case SERIES_X:    return AddrX::present_velocity  .pulse2val(dyn_comm_.tryRead(AddrX::present_velocity   , id), model_[id]);
    case SERIES_P:    return AddrP::present_velocity  .pulse2val(dyn_comm_.tryRead(AddrP::present_velocity   , id), model_[id]);
    case SERIES_PRO:  return AddrPro::present_velocity.pulse2val(dyn_comm_.tryRead(AddrPro::present_velocity , id), model_[id]);
    default:          return 0.0;
} }

double DynamixelHandler::ReadPresentPosition(id_t id){ switch ( series_[id] ) {
    case SERIES_X:    return AddrX::present_position  .pulse2val(dyn_comm_.tryRead(AddrX::present_position   , id), model_[id]);
    case SERIES_P:    return AddrP::present_position  .pulse2val(dyn_comm_.tryRead(AddrP::present_position   , id), model_[id]);
    case SERIES_PRO:  return AddrPro::present_position.pulse2val(dyn_comm_.tryRead(AddrPro::present_position , id), model_[id]);
    default:          return 0.0;
} }

double DynamixelHandler::ReadGoalPWM(id_t id){ switch ( series_[id] ) {
    case SERIES_X:    return AddrX::goal_pwm.pulse2val(dyn_comm_.tryRead(AddrX::goal_pwm   , id), model_[id]);
    case SERIES_P:    return AddrP::goal_pwm.pulse2val(dyn_comm_.tryRead(AddrP::goal_pwm   , id), model_[id]);
    case SERIES_PRO:  ROS_WARN("   = PRO series don't support 'goal_pwm'"); return AddrPro::goal_pwm .pulse2val(0.0, model_[id]);
    default:          return 0.0;
} }

double DynamixelHandler::ReadGoalCurrent(id_t id){ switch ( series_[id] ) {
    case SERIES_X:    return AddrX::goal_current  .pulse2val(dyn_comm_.tryRead(AddrX::goal_current   , id), model_[id]);
    case SERIES_P:    return AddrP::goal_current  .pulse2val(dyn_comm_.tryRead(AddrP::goal_current   , id), model_[id]);
    case SERIES_PRO:  return AddrPro::goal_current.pulse2val(dyn_comm_.tryRead(AddrPro::goal_current , id), model_[id]);
    default:          return 0.0;
} }

double DynamixelHandler::ReadGoalVelocity(id_t id){ switch ( series_[id] ) {
    case SERIES_X:    return AddrX::goal_velocity  .pulse2val(dyn_comm_.tryRead(AddrX::goal_velocity   , id), model_[id]);
    case SERIES_P:    return AddrP::goal_velocity  .pulse2val(dyn_comm_.tryRead(AddrP::goal_velocity   , id), model_[id]);
    case SERIES_PRO:  return AddrPro::goal_velocity.pulse2val(dyn_comm_.tryRead(AddrPro::goal_velocity , id), model_[id]);
    default:          return 0.0;
} }

double DynamixelHandler::ReadGoalPosition(id_t id){ switch ( series_[id] ) {
    case SERIES_X:    return AddrX::goal_position  .pulse2val(dyn_comm_.tryRead(AddrX::goal_position   , id), model_[id]);
    case SERIES_P:    return AddrP::goal_position  .pulse2val(dyn_comm_.tryRead(AddrP::goal_position   , id), model_[id]);
    case SERIES_PRO:  return AddrPro::goal_position.pulse2val(dyn_comm_.tryRead(AddrPro::goal_position , id), model_[id]);
    default:          return 0.0;
} }

double DynamixelHandler::ReadProfileAcc(id_t id){ switch ( series_[id] ) {
    case SERIES_X:    return AddrX::profile_acceleration  .pulse2val(dyn_comm_.tryRead(AddrX::profile_acceleration   , id), model_[id]);
    case SERIES_P:    return AddrP::profile_acceleration  .pulse2val(dyn_comm_.tryRead(AddrP::profile_acceleration   , id), model_[id]);
    case SERIES_PRO:  return AddrPro::profile_acceleration.pulse2val(dyn_comm_.tryRead(AddrPro::profile_acceleration , id), model_[id]);
    default:          return 0.0;
} }

double DynamixelHandler::ReadProfileVel(id_t id){ switch ( series_[id] ) {
    case SERIES_X:    return AddrX::profile_velocity  .pulse2val(dyn_comm_.tryRead(AddrX::profile_velocity   , id), model_[id]);
    case SERIES_P:    return AddrP::profile_velocity  .pulse2val(dyn_comm_.tryRead(AddrP::profile_velocity   , id), model_[id]);
    case SERIES_PRO:  return AddrPro::profile_velocity.pulse2val(dyn_comm_.tryRead(AddrPro::profile_velocity , id), model_[id]);
    default:          return 0.0;
} }

double DynamixelHandler::ReadHomingOffset(id_t id){ switch ( series_[id] ) {
    case SERIES_X:    return AddrX::homing_offset  .pulse2val(dyn_comm_.tryRead(AddrX::homing_offset   , id), model_[id]);
    case SERIES_P:    return AddrP::homing_offset  .pulse2val(dyn_comm_.tryRead(AddrP::homing_offset   , id), model_[id]);
    case SERIES_PRO:  return AddrPro::homing_offset.pulse2val(dyn_comm_.tryRead(AddrPro::homing_offset , id), model_[id]);
    default:          return 0.0;
} }

double DynamixelHandler::ReadBusWatchdog(id_t id){ switch ( series_[id] ) {
    case SERIES_X:    return AddrX::bus_watchdog.pulse2val(dyn_comm_.tryRead(AddrX::bus_watchdog   , id), model_[id]);
    case SERIES_P:    return AddrP::bus_watchdog.pulse2val(dyn_comm_.tryRead(AddrP::bus_watchdog   , id), model_[id]);
    case SERIES_PRO:  ROS_WARN("   = PRO series don't support 'bus_watchdog'"); return 0.0;
    default:          return 0.0;
} }

uint8_t DynamixelHandler::ReadOperatingMode(id_t id){ switch ( series_[id] ) {
    case SERIES_X:   return dyn_comm_.tryRead(AddrX::operating_mode, id);
    case SERIES_P:   return dyn_comm_.tryRead(AddrP::operating_mode, id);
    case SERIES_PRO: return dyn_comm_.tryRead(AddrPro::operating_mode, id);
    default: return 0;
} }

uint8_t DynamixelHandler::ReadDriveMode(id_t id){ switch ( series_[id] ) {
    case SERIES_X:   return dyn_comm_.tryRead(AddrX::drive_mode, id);
    case SERIES_P:   return dyn_comm_.tryRead(AddrP::drive_mode, id);
    case SERIES_PRO: ROS_WARN("   = PRO series don't support 'drive_mode'"); return 0;
    default: return 0; 
} }
double DynamixelHandler::ReadReturnDelayTime(id_t id){ switch ( series_[id] ) {
    case SERIES_X:   return AddrX::return_delay_time.pulse2val(dyn_comm_.tryRead(AddrX::return_delay_time, id), model_[id]);
    case SERIES_P:   return AddrP::return_delay_time.pulse2val(dyn_comm_.tryRead(AddrP::return_delay_time, id), model_[id]);
    case SERIES_PRO: return AddrPro::return_delay_time.pulse2val(dyn_comm_.tryRead(AddrPro::return_delay_time, id), model_[id]);
    default: return 0.0;
} }

//* 基本機能たち Write

bool DynamixelHandler::WriteTorqueEnable(id_t id, bool enable){ switch ( series_[id] ) {
    case SERIES_X: return dyn_comm_.tryWrite(AddrX::torque_enable, id, enable ? TORQUE_ENABLE : TORQUE_DISABLE);
    case SERIES_P: return dyn_comm_.tryWrite(AddrP::torque_enable, id, enable ? TORQUE_ENABLE : TORQUE_DISABLE);
    case SERIES_PRO: return dyn_comm_.tryWrite(AddrPro::torque_enable, id, enable ? TORQUE_ENABLE : TORQUE_DISABLE);
    default: return false;
} }

bool DynamixelHandler::WriteGoalPosition(id_t id, double pos){ switch ( series_[id] ) {
    case SERIES_X: return dyn_comm_.tryWrite(AddrX::goal_position, id, AddrX::goal_position.val2pulse(pos, model_[id]));
    case SERIES_P: return dyn_comm_.tryWrite(AddrP::goal_position, id, AddrP::goal_position.val2pulse(pos, model_[id]));
    case SERIES_PRO: return dyn_comm_.tryWrite(AddrPro::goal_position, id, AddrPro::goal_position.val2pulse(pos, model_[id]));
    default: return false;
} }

bool DynamixelHandler::WriteGoalPWM(id_t id, double pwm){ switch ( series_[id] ) {
    case SERIES_X: return dyn_comm_.tryWrite(AddrX::goal_pwm, id, AddrX::goal_pwm.val2pulse(pwm, model_[id]));
    case SERIES_P: return dyn_comm_.tryWrite(AddrP::goal_pwm, id, AddrP::goal_pwm.val2pulse(pwm, model_[id]));
    case SERIES_PRO: ROS_WARN("   = PRO series don't support 'goal_pwm'"); return false;
    default: return false;
} }

bool DynamixelHandler::WriteGoalCurrent(id_t id, double cur){ switch ( series_[id] ) {
    case SERIES_X: return dyn_comm_.tryWrite(AddrX::goal_current, id, AddrX::goal_current.val2pulse(cur, model_[id]));
    case SERIES_P: return dyn_comm_.tryWrite(AddrP::goal_current, id, AddrP::goal_current.val2pulse(cur, model_[id]));
    case SERIES_PRO: return dyn_comm_.tryWrite(AddrPro::goal_current, id, AddrPro::goal_current.val2pulse(cur, model_[id]));
    default: return false;
} }

bool DynamixelHandler::WriteGoalVelocity(id_t id, double vel){ switch ( series_[id] ) {
    case SERIES_X: return dyn_comm_.tryWrite(AddrX::goal_velocity, id, AddrX::goal_velocity.val2pulse(vel, model_[id]));
    case SERIES_P: return dyn_comm_.tryWrite(AddrP::goal_velocity, id, AddrP::goal_velocity.val2pulse(vel, model_[id]));
    case SERIES_PRO: return dyn_comm_.tryWrite(AddrPro::goal_velocity, id, AddrPro::goal_velocity.val2pulse(vel, model_[id]));
    default: return false;
} }

bool DynamixelHandler::WriteProfileAcc(id_t id, double acc){ switch ( series_[id] ) {
    case SERIES_X: return dyn_comm_.tryWrite(AddrX::profile_acceleration, id, AddrX::profile_acceleration.val2pulse(acc, model_[id]));
    case SERIES_P: return dyn_comm_.tryWrite(AddrP::profile_acceleration, id, AddrP::profile_acceleration.val2pulse(acc, model_[id]));
    case SERIES_PRO: return dyn_comm_.tryWrite(AddrPro::profile_acceleration, id, AddrPro::profile_acceleration.val2pulse(acc, model_[id]));
    default: return false;
} }

bool DynamixelHandler::WriteProfileVel(id_t id, double vel){ switch ( series_[id] ) {
    case SERIES_X: return dyn_comm_.tryWrite(AddrX::profile_velocity, id, AddrX::profile_velocity.val2pulse(vel, model_[id]));
    case SERIES_P: return dyn_comm_.tryWrite(AddrP::profile_velocity, id, AddrP::profile_velocity.val2pulse(vel, model_[id]));
    case SERIES_PRO: return dyn_comm_.tryWrite(AddrPro::profile_velocity, id, AddrPro::profile_velocity.val2pulse(vel, model_[id]));
    default: return false;
} }

bool DynamixelHandler::WriteHomingOffset(id_t id, double offset){ switch ( series_[id] ) {
    case SERIES_X: return dyn_comm_.tryWrite(AddrX::homing_offset, id, AddrX::homing_offset.val2pulse(offset, model_[id]));
    case SERIES_P: return dyn_comm_.tryWrite(AddrP::homing_offset, id, AddrP::homing_offset.val2pulse(offset, model_[id]));
    case SERIES_PRO: return dyn_comm_.tryWrite(AddrPro::homing_offset, id, AddrPro::homing_offset.val2pulse(offset, model_[id]));
    default: return false;
} }

bool DynamixelHandler::WriteBusWatchdog(id_t id, double time){ switch ( series_[id] ) {
    case SERIES_X: return dyn_comm_.tryWrite(AddrX::bus_watchdog, id, AddrX::bus_watchdog.val2pulse(time, model_[id]));
    case SERIES_P: return dyn_comm_.tryWrite(AddrP::bus_watchdog, id, AddrP::bus_watchdog.val2pulse(time, model_[id]));
    case SERIES_PRO: ROS_WARN("   = PRO series don't support 'bus_watchdog'"); return false;
    default: return false;
} }

bool DynamixelHandler::WriteGains(id_t id, array<uint16_t, _num_gain> gains){
    bool is_success = true;
    switch ( series_[id] ) {
        case SERIES_X: is_success &= dyn_comm_.tryWrite(AddrX::velocity_i_gain, id, gains[VELOCITY_I_GAIN]);
                       is_success &= dyn_comm_.tryWrite(AddrX::velocity_p_gain, id, gains[VELOCITY_P_GAIN]);
                       is_success &= dyn_comm_.tryWrite(AddrX::position_d_gain, id, gains[POSITION_D_GAIN]);
                       is_success &= dyn_comm_.tryWrite(AddrX::position_i_gain, id, gains[POSITION_I_GAIN]);
                       is_success &= dyn_comm_.tryWrite(AddrX::position_p_gain, id, gains[POSITION_P_GAIN]);
                       is_success &= dyn_comm_.tryWrite(AddrX::feedforward_2nd_gain, id, gains[FEEDFORWARD_ACC_GAIN]);
                       is_success &= dyn_comm_.tryWrite(AddrX::feedforward_1st_gain, id, gains[FEEDFORWARD_VEL_GAIN]); break;
        case SERIES_P: is_success &= dyn_comm_.tryWrite(AddrP::velocity_i_gain, id, gains[VELOCITY_I_GAIN]);
                       is_success &= dyn_comm_.tryWrite(AddrP::velocity_p_gain, id, gains[VELOCITY_P_GAIN]);
                       is_success &= dyn_comm_.tryWrite(AddrP::position_d_gain, id, gains[POSITION_D_GAIN]);
                       is_success &= dyn_comm_.tryWrite(AddrP::position_i_gain, id, gains[POSITION_I_GAIN]);
                       is_success &= dyn_comm_.tryWrite(AddrP::position_p_gain, id, gains[POSITION_P_GAIN]);
                       is_success &= dyn_comm_.tryWrite(AddrP::feedforward_2nd_gain, id, gains[FEEDFORWARD_ACC_GAIN]);
                       is_success &= dyn_comm_.tryWrite(AddrP::feedforward_1st_gain, id, gains[FEEDFORWARD_VEL_GAIN]); break;
        case SERIES_PRO: is_success &= dyn_comm_.tryWrite(AddrPro::velocity_i_gain, id, gains[VELOCITY_I_GAIN]);
                         is_success &= dyn_comm_.tryWrite(AddrPro::velocity_p_gain, id, gains[VELOCITY_P_GAIN]);
                         is_success &= dyn_comm_.tryWrite(AddrPro::position_p_gain , id, gains[POSITION_P_GAIN]); break;
        default:         return false;
    }
    return is_success;
}

bool DynamixelHandler::WriteOperatingMode(id_t id, uint8_t mode){ switch ( series_[id] ) {
    case SERIES_X:    return dyn_comm_.tryWrite(AddrX::operating_mode, id, mode);
    case SERIES_P:    return dyn_comm_.tryWrite(AddrP::operating_mode, id, mode);
    case SERIES_PRO:  return dyn_comm_.tryWrite(AddrPro::operating_mode, id, mode);
    default:         return false;
} }

bool DynamixelHandler::WriteReturnDelayTime(id_t id, double time){ switch ( series_[id] ) {
    case SERIES_X:    return dyn_comm_.tryWrite(AddrX::return_delay_time, id, AddrX::return_delay_time.val2pulse(time, model_[id]));
    case SERIES_P:    return dyn_comm_.tryWrite(AddrP::return_delay_time, id, AddrP::return_delay_time.val2pulse(time, model_[id]));
    case SERIES_PRO:  return dyn_comm_.tryWrite(AddrPro::return_delay_time, id, AddrPro::return_delay_time.val2pulse(time, model_[id]));
    default:          return false;
} }
    
