#include "rclcpp/rclcpp.hpp"
#include "myUtils/logging_like_ros1.hpp"
#include "myUtils/make_iterator_convenient.hpp"
#include "dynamixel_handler.hpp"
#include "dynamixel_handler_msgs/msg/dxl_states.hpp"
#include "optional_function/imu_dxmio.hpp"
#include "optional_function/imu_opencr.hpp"
#include "optional_function/external_port.hpp"

using namespace std::string_literals;

DynamixelHandler::DynamixelHandler() : Node("dynamixel_handler", rclcpp::NodeOptions()
                                                                  .allow_undeclared_parameters(true)
                                                                  .automatically_declare_parameters_from_overrides(true)) {
    ROS_INFO( "Initializing DynamixelHandler .....");
    cbg_serial_  = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cbg_command_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // 開発用
    bool is_debug; get_parameter_or("debug", is_debug, false);
    // ダミーモータのIDリストを取得, 0未満，255以上のIDを排除する
    vector<int64_t> dummy_id_list; this->get_parameter("init/dummy_servo_list", dummy_id_list); //int64_t で受けないといけない．
    dummy_id_list.erase(std::remove_if(dummy_id_list.begin(), dummy_id_list.end(), [](int64_t id){ return id<0 || id>=255; }), dummy_id_list.end());
    // Serial通信の設定
    int baudrate      ; this->get_parameter_or("baudrate"     , baudrate     ,           57600);
    int latency_timer ; this->get_parameter_or("latency_timer", latency_timer,              16);
    string device_name; this->get_parameter_or("device_name"  , device_name  , "/dev/ttyUSB0"s);
    dyn_comm_ = DynamixelCommunicator(device_name.c_str(), baudrate, latency_timer);
    // serial通信のverbose設定
    bool serial_verbose; get_parameter_or("dyn_comm/verbose", serial_verbose, false);
    dyn_comm_.set_verbose(serial_verbose);
    // serial通信の開始
    if ( !dyn_comm_.OpenPort() ) { fflush(stdout); // printfのバッファを吐き出す． これがないと printfの表示が遅延する
        ROS_ERROR(" Failed to open USB device [%s]", dyn_comm_.port_name().c_str());
        if ( !is_debug && dummy_id_list.empty() ) ROS_STOP("Initialization failed (device open)");
    } else { fflush(stdout);
        ROS_INFO(" Succeeded to open device");
        ROS_INFO("  ------------ name '%s'", device_name.c_str());
        ROS_INFO("  -------- baudrate '%d'", baudrate);
        ROS_INFO("  --- latency_timer '%d'", latency_timer);
        bool auto_unify_baudrate; this->get_parameter_or("init/baudrate_auto_set", auto_unify_baudrate, false);
        if ( auto_unify_baudrate ) {
            ROS_INFO(" Unifying all Dynamixels' baudrate");
            UnifyBaudrate(static_cast<uint64_t>(baudrate));
        }
    }

    // serial通信のretry設定
    int num_try      ; get_parameter_or("dyn_comm/retry_num"   , num_try      ,  5);
    int msec_interval; get_parameter_or("dyn_comm/inerval_msec", msec_interval, 10);
    dyn_comm_.set_retry_config(num_try, msec_interval); fflush(stdout);

    // main loop の設定
    this->get_parameter_or("loop_rate", loop_rate_, 50u);
    this->get_parameter_or("verbose_ratio", ratio_mainloop_ , 100u);
    this->get_parameter_or("pub_outdated_present_value", do_pub_pre_all_, true);
    this->get_parameter_or("pub_ratio/present.pwm"                 , pub_ratio_present_[PRESENT_PWM          ],  0u);
    this->get_parameter_or("pub_ratio/present.current"             , pub_ratio_present_[PRESENT_CURRENT      ],  1u);
    this->get_parameter_or("pub_ratio/present.velocity"            , pub_ratio_present_[PRESENT_VELOCITY     ],  1u);
    this->get_parameter_or("pub_ratio/present.position"            , pub_ratio_present_[PRESENT_POSITION     ],  1u);
    this->get_parameter_or("pub_ratio/present.velocity_trajectory" , pub_ratio_present_[VELOCITY_TRAJECTORY  ],  0u);
    this->get_parameter_or("pub_ratio/present.position_trajectory" , pub_ratio_present_[POSITION_TRAJECTORY  ],  0u);
    this->get_parameter_or("pub_ratio/present.input_voltage"       , pub_ratio_present_[PRESENT_INPUT_VOLTAGE], 10u);
    this->get_parameter_or("pub_ratio/present.temperature"         , pub_ratio_present_[PRESENT_TEMPERATURE  ], 10u);
    this->get_parameter_or("pub_ratio/status" , pub_ratio_["status"], 50u);
    this->get_parameter_or("pub_ratio/goal"   , pub_ratio_["goal"] ,   0u);
    this->get_parameter_or("pub_ratio/gain"   , pub_ratio_["gain"] ,   0u);
    this->get_parameter_or("pub_ratio/limit"  , pub_ratio_["limit"],   0u);
    this->get_parameter_or("pub_ratio/error"  , pub_ratio_["error"], 100u);
    this->get_parameter_or("pub_ratio/extra.dynamic_items", pub_ratio_["extra_dynamic"], 0u);
    this->get_parameter_or("pub_ratio/extra.static_items" , pub_ratio_["extra_static"] , 0u);
    this->get_parameter_or("max_log_width"    , width_log_, 7u);
    this->get_parameter_or("method/split_write"    , use_split_write_    , false);
    this->get_parameter_or("method/split_read"     , use_split_read_     , false);
    this->get_parameter_or("method/fast_read"      , use_fast_read_      , true);
    // 初期化・終了時
    this->get_parameter_or("init/hardware_error_auto_clean", do_clean_hwerr_, true);
    this->get_parameter_or("init/torque_auto_enable"       , do_torque_on_  , true);
    this->get_parameter_or("term/torque_auto_disable"      , do_torque_off_ , true);
    this->get_parameter_or("term/servo_auto_stop"          , do_stop_end_   , true);
    this->get_parameter_or("default/profile_vel", default_["profile_vel_deg_s"], 100.0);
    this->get_parameter_or("default/profile_acc", default_["profile_acc_deg_ss"], 600.0);
    this->get_parameter_or("default/return_delay_time", default_["return_delay_time_us"],   0.0);
    this->get_parameter_or("default/bus_watchdog"     , default_["bus_watchdog_ms"     ], 500.0);
    // id_set_の作成に関連するもの
    this->get_parameter_or("init/used_servo_series.X", use_["x"], true);
    this->get_parameter_or("init/used_servo_series.P", use_["p"], false);
    this->get_parameter_or("init/used_servo_series.Pro", use_["pro"], false);
    int num_expected; this->get_parameter_or("init/expected_servo_num"     , num_expected, 0);
    int times_retry ; this->get_parameter_or("init/servo_auto_search.retry_times", times_retry , 5);
    int id_min      ; this->get_parameter_or("init/servo_auto_search.min_id"     , id_min      , 1);
    int id_max      ; this->get_parameter_or("init/servo_auto_search.max_id"     , id_max      , 35);
    set<id_t> scan_id_set; for (int id=id_min; id<=id_max; id++) scan_id_set.insert(static_cast<id_t>(id));
    // id_set_の作成
    if ( num_expected>0 ) ROS_INFO(" '%d' servo(s) are expected", num_expected);
    else                 {ROS_WARN(" Expected servo number is not set."); ROS_WARN(" > Any number of Dynamixel is allowed");}
    ROS_INFO(" Auto scanning Dynamixel (id range '%d' to '%d') ...", id_min, id_max);
    ROS_INFO(" > series: X [%suse], P [%suse], PRO [%suse]", use_["x"]?"":"no ", use_["p"]?"":"no ", use_["pro"]?"":"no ");
    /* *********************** dynamixelを探索し，初期化する ***********************************/
    /* */for (const auto id : dummy_id_list) DummyUpDynamixel(id); // 最初にダミーモータを登録しておく．これにより，実際のモータの探索の際に，ダミーモータのIDをスキャン対象から外すことができる．
    /* */tryAddDynamixels(scan_id_set, num_expected, times_retry);
    /* ***********************************************************************************/
    if( id_set_.size()==0 ) { // 見つからなかった場合は初期化失敗で終了
        ROS_ERROR(" Dynamixel is not found in USB device [%s]", dyn_comm_.port_name().c_str());
        if ( !is_debug ) ROS_STOP("Initialization failed (no dynamixel found)");
    }
    if( num_expected>0 && num_expected!=(int)id_set_.size() ) { // 期待数が設定されているときに、見つかった数が期待数と異なる場合は初期化失敗で終了
        ROS_ERROR(" Number of Dynamixel is not matched.");
        ROS_ERROR(" Expected '%d', but found '%ld'. please check & retry", num_expected, id_set_.size());
        if ( !is_debug ) ROS_STOP("Initialization failed (number of dynamixel is not matched)");
    }
    ROS_INFO("  ... Finish scanning Dynamixel");
    // ループ中のverbose設定
    this->get_parameter_or("verbose/callback"           , verbose_callback_, false);
    this->get_parameter_or("verbose/write_status"       , verbose_["w_status"], false);
    this->get_parameter_or("verbose/write_goal"         , verbose_["w_goal"  ], false);
    this->get_parameter_or("verbose/write_gain"         , verbose_["w_gain"  ], false);
    this->get_parameter_or("verbose/write_limit"        , verbose_["w_limit" ], false);
    this->get_parameter_or("verbose/write_extra"        , verbose_["w_extra" ], false);
    this->get_parameter_or("verbose/read_status.raw"    , verbose_["r_status"    ], false);
    this->get_parameter_or("verbose/read_status.err"    , verbose_["r_status_err"], false);
    this->get_parameter_or("verbose/read_present.raw"   , verbose_["r_present"    ], false);
    this->get_parameter_or("verbose/read_present.err"   , verbose_["r_present_err"], false);
    this->get_parameter_or("verbose/read_goal.raw"      , verbose_["r_goal"    ], false);
    this->get_parameter_or("verbose/read_goal.err"      , verbose_["r_goal_err"], false);
    this->get_parameter_or("verbose/read_gain.raw"      , verbose_["r_gain"    ], false);
    this->get_parameter_or("verbose/read_gain.err"      , verbose_["r_gain_err"], false);
    this->get_parameter_or("verbose/read_limit.raw"     , verbose_["r_limit"    ], false);
    this->get_parameter_or("verbose/read_limit.err"     , verbose_["r_limit_err"], false);
    this->get_parameter_or("verbose/read_extra.raw"     , verbose_["r_extra"    ], false);
    this->get_parameter_or("verbose/read_extra.err"     , verbose_["r_extra_err"], false);
    this->get_parameter_or("verbose/read_hardware_error", verbose_["r_hwerr" ], false);
    this->get_parameter_or("no_response_id_auto_remove_count", auto_remove_count_   , 0u);
    this->get_parameter_or("mode_change_interval_sec", opmode_change_interval_, 1.0);
    this->get_parameter_or("clear_error_interval_sec", hwerr_clear_interval_, 1.0);

    bool no_use_command_line; get_parameter_or("no_use_command_line", no_use_command_line, false);

                                SetupRosInterfaces_byProgram();
    if ( !no_use_command_line ) SetupRosInterfaces_byCLI();

    BroadcastState_Status();
    BroadcastState_Limit();
    BroadcastState_Gain();
    BroadcastState_Goal();
    BroadcastState_Error();
    BroadcastState_Extra();

    bool use_ex_port; get_parameter_or("option/external_port.use", use_ex_port, false);
    if ( use_ex_port ) external_port_ = std::make_unique<ExternalPort>(*this);

    bool use_imu_dxmio; get_parameter_or("option/imu_dxmio.use", use_imu_dxmio, false);
    if ( use_imu_dxmio ) imu_dxmio_ = std::make_unique<ImuDXMIO>(*this);

    bool use_imu_opencr; get_parameter_or("option/imu_opencr.use", use_imu_opencr, false);
    if ( use_imu_opencr ) imu_opencr_ = std::make_unique<ImuOpenCR>(*this);

    ROS_INFO( "..... DynamixelHandler is initialized");
}

using std::chrono::system_clock;
using std::chrono::duration_cast;
using std::chrono::microseconds;
using std::tie;
using std::ignore;

enum state { STATUS, PRESENT, GOAL, GAIN, LIMIT, ERROR, EXTRA_D, EXTRA_S , _num_state };

void DynamixelHandler::MainLoop(){
    static int cnt = -1; cnt++;
    static float t_read=0, t_total=0;
    static double n_present_suc_p=1, n_present_suc_f=1, n_present_read=1, n_any_read=1;

/* 処理時間時間の計測 */ auto s_total = system_clock::now();
    //* Additional function
    if (external_port_) external_port_->MainProcess();
    if (imu_dxmio_    ) imu_dxmio_->MainProcess();
    if (imu_opencr_   ) imu_opencr_->MainProcess();

    //* topicをSubscribe & Dynamixelへ目標角をWrite
    { // write 区間はまとめて lock する
        std::lock_guard<std::recursive_mutex> lock(mutex_state_);
        if ( auto_remove_count_ ) for (auto [id, cnt_err]: ping_err_) 
            cnt_err>auto_remove_count_ && (id_edit_[id] = 0xFF);
        if ( !id_edit_.empty() )  for (auto [id, id_sub]: exchange(id_edit_, {}) )
            id_sub==0xFF ? (is_in(id, id_set_) ? RemoveDynamixel(id) : AddDynamixel(id)) : ChangeDynamixel(id, id_sub);
        for ( auto id : id_set_ ) { // mode変更の失敗時のみ，goal更新しないように修正
            if (op_mode_w_.count(id)){ ChangeOperatingMode(id, op_mode_w_[id]) ? op_mode_w_.erase(id) : updated_id_goal_.erase(id);} 
            if ( hw_err_w_.count(id)){ ClearHardwareError (id,  hw_err_w_[id]);   hw_err_w_.erase(id);} // hw_err_w_[id]=複数回転をoffsetで復元するか
            if (tq_mode_w_.count(id)){ tq_mode_w_[id]?TorqueOn(id):TorqueOff(id);tq_mode_w_.erase(id);}
        }
        BulkWriteExtra_ram(ex_ram_indice_write_, updated_id_ex_ram_);
        ex_ram_indice_write_.clear(); updated_id_ex_ram_.clear();
        BulkWriteExtra_rom(ex_rom_indice_write_, updated_id_ex_rom_);
        ex_rom_indice_write_.clear(); updated_id_ex_rom_.clear();
        SyncWriteGoal(goal_indice_write_, updated_id_goal_);
        goal_indice_write_.clear() ; updated_id_goal_.clear();
        SyncWriteGain(gain_indice_write_, updated_id_gain_);
        gain_indice_write_.clear() ; updated_id_gain_.clear();
        SyncWriteLimit(limit_indice_write_, updated_id_limit_);
        limit_indice_write_.clear(); updated_id_limit_.clear();
    }

    //* present value について read する情報を決定
    static const auto& r = pub_ratio_present_; //長いので省略
    present_indice_read_.clear();
    if (r[PRESENT_PWM          ] && cnt % r[PRESENT_PWM          ] == 0) present_indice_read_.insert(PRESENT_PWM          );
    if (r[PRESENT_CURRENT      ] && cnt % r[PRESENT_CURRENT      ] == 0) present_indice_read_.insert(PRESENT_CURRENT      );
    if (r[PRESENT_VELOCITY     ] && cnt % r[PRESENT_VELOCITY     ] == 0) present_indice_read_.insert(PRESENT_VELOCITY     );
    if (r[PRESENT_POSITION     ] && cnt % r[PRESENT_POSITION     ] == 0) present_indice_read_.insert(PRESENT_POSITION     );
    if (r[VELOCITY_TRAJECTORY  ] && cnt % r[VELOCITY_TRAJECTORY  ] == 0) present_indice_read_.insert(VELOCITY_TRAJECTORY  );
    if (r[POSITION_TRAJECTORY  ] && cnt % r[POSITION_TRAJECTORY  ] == 0) present_indice_read_.insert(POSITION_TRAJECTORY  );
    if (r[PRESENT_INPUT_VOLTAGE] && cnt % r[PRESENT_INPUT_VOLTAGE] == 0) present_indice_read_.insert(PRESENT_INPUT_VOLTAGE);
    if (r[PRESENT_TEMPERATURE  ] && cnt % r[PRESENT_TEMPERATURE  ] == 0) present_indice_read_.insert(PRESENT_TEMPERATURE  );

/* 処理時間時間の計測 */ auto s_read = system_clock::now();
    //* Dynamixelから状態Read & topicをPublish
    auto msg = DxlStates().set__stamp(this->get_clock()->now());
    array<double, _num_state> success_rate{}; // 0初期化する．
    if ( pub_ratio_["status"] && cnt % pub_ratio_["status"] == 0 ) { /*statusは関連は複数個所で順次更新されるので特別扱い*/
        success_rate[STATUS] = CheckDynamixels() ? 1.0 : 0.1;
    }
    set<uint8_t> target_id_set; for (auto id : id_set_) if ( ping_err_[id]==0 ) target_id_set.insert(id);
    if ( !present_indice_read_.empty() ){ n_present_read++;
        tie(success_rate[PRESENT], ignore) = SyncReadPresent(present_indice_read_, target_id_set);
        n_present_suc_p += !target_id_set.empty()  && success_rate[PRESENT] > 0.0;
        n_present_suc_f +=  target_id_set==id_set_ && success_rate[PRESENT] > 1.0-1e-6;
    }
    if ( pub_ratio_["goal" ] && cnt % pub_ratio_["goal" ] == 0 )
        tie(success_rate[GOAL], ignore) = SyncReadGoal(goal_indice_read_, target_id_set);
    if ( pub_ratio_["gain" ] && cnt % pub_ratio_["gain" ] == 0 )
        tie(success_rate[GAIN], ignore) = SyncReadGain(gain_indice_read_, target_id_set);
    if ( pub_ratio_["limit"] && cnt % pub_ratio_["limit"] == 0 )
        tie(success_rate[LIMIT], ignore) = SyncReadLimit(limit_indice_read_, target_id_set);
    if ( pub_ratio_["error"] && cnt % pub_ratio_["error"] == 0 )
        tie(success_rate[ERROR], ignore) = SyncReadHardwareErrors(target_id_set);
    if ( pub_ratio_["extra_dynamic"] && cnt % pub_ratio_["extra_dynamic"] == 0 )
        tie(success_rate[EXTRA_D], ignore) = BulkReadExtra_rapid(target_id_set);
    if ( pub_ratio_["extra_static" ] && cnt % pub_ratio_["extra_static" ] == 0 )
        tie(success_rate[EXTRA_S], ignore) = BulkReadExtra_slow(target_id_set);
    bool is_any_read = std::any_of( success_rate.begin(), success_rate.end(), [](auto& x){ return x > 0.0; });
    if ( success_rate[STATUS ]>0.0 ) msg.status  = BroadcastState_Status();
    if ( success_rate[PRESENT]>0.0 ) msg.present = BroadcastState_Present();
    if ( success_rate[GOAL   ]>0.0 ) msg.goal    = BroadcastState_Goal();
    if ( success_rate[GAIN   ]>0.0 ) msg.gain    = BroadcastState_Gain();
    if ( success_rate[LIMIT  ]>0.0 ) msg.limit   = BroadcastState_Limit();
    if ( success_rate[ERROR  ]>0.0 ) msg.error   = BroadcastState_Error();
    if ( success_rate[EXTRA_D]+success_rate[EXTRA_S] > 0.0 ) msg.extra = BroadcastState_Extra();
    if ( is_any_read ) { n_any_read++;
        BroadcastDebug(msg);
        BroadcastStates(msg);
    }
/* 処理時間時間の計測 */ t_read  += duration_cast<microseconds>(system_clock::now()-s_read ).count() / 1000.0;
/* 処理時間時間の計測 */ t_total += duration_cast<microseconds>(system_clock::now()-s_total).count() / 1000.0;

    //* デバック
    if ( ratio_mainloop_ && cnt % ratio_mainloop_ == 0) {
        float partial_suc = 100*n_present_suc_p/n_present_read;
        float full_suc    = 100*n_present_suc_f/n_present_read;
        char msg[100]; sprintf(msg, "time=%2.2fms/loop(%2.2fms/read), success=%3.0f%%(full=%3.0f%%)",
                               t_total/ratio_mainloop_, t_read/n_any_read, partial_suc, full_suc);
        if (full_suc < 80) ROS_ERROR("%s", msg); else if (partial_suc < 99) ROS_WARN("%s", msg); else ROS_INFO("%s", msg);
        t_total = 0.0; /* mainloopで行われてる処理の計測時間を初期化 */
    }
    if ( cnt % max({(int)loop_rate_, (int)ratio_mainloop_, 10}) == 0)
        t_read = n_present_suc_p = n_present_suc_f = n_present_read = n_any_read = 0.00001; /* present value の read の周期で行われてる処理の初期化 */
}

DynamixelHandler::~DynamixelHandler(){
    ROS_INFO( "Terminating DynamixelHandler .....");
    StopDynamixels(id_set_);
    ROS_INFO( "..... DynamixelHandler is terminated");
}
