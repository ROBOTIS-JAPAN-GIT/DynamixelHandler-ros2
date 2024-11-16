#include "dynamixel_handler.hpp"
#include "rclcpp/rclcpp.hpp"
#include "myUtils/logging_like_ros1.hpp"
#include "myUtils/make_iterator_convenient.hpp"

using std::bind;
using std::placeholders::_1;

DynamixelHandler::DynamixelHandler() : Node("dynamixel_handler", rclcpp::NodeOptions()
                                                                  .allow_undeclared_parameters(true)
                                                                  .automatically_declare_parameters_from_overrides(true)) {
    ROS_INFO( "Initializing DynamixelHandler .....");

    bool is_debug; get_parameter_or("debug", is_debug, false);

    // 通信の開始
    int baudrate      ; this->get_parameter_or("baudrate"     , baudrate     , 57600                 );
    int latency_timer ; this->get_parameter_or("latency_timer", latency_timer,    16                 );
    string device_name; this->get_parameter_or("device_name"  , device_name  , string("/dev/ttyUSB0"));

    dyn_comm_ = DynamixelCommunicator(device_name.c_str(), baudrate, latency_timer);
    if ( !dyn_comm_.OpenPort() ) { fflush(stdout); // printfのバッファを吐き出す． これがないと printfの表示が遅延する
        ROS_ERROR("Failed to open USB device [%s]", dyn_comm_.port_name().c_str()); 
        if ( !is_debug ) throw std::runtime_error("Initialization failed (device open)");
    } 
    // serial通信のverbose設定
    bool serial_verbose; get_parameter_or("dyn_comm/verbose", serial_verbose, false);
    dyn_comm_.set_verbose(serial_verbose); fflush(stdout);

    // serial通信のretry設定
    int num_try      ; get_parameter_or("dyn_comm/retry_num"   , num_try      ,  5);
    int msec_interval; get_parameter_or("dyn_comm/inerval_msec", msec_interval, 10);
    dyn_comm_.set_retry_config(num_try, msec_interval); fflush(stdout);

    // main loop の設定
    this->get_parameter_or("loop_rate", loop_rate_, 50u);
    this->get_parameter_or("verbose_ratio", ratio_mainloop_ , 100u);
    this->get_parameter_or("pub_ratio/present.pwm"                 , pub_ratio_present_[PRESENT_PWM          ],  0u);
    this->get_parameter_or("pub_ratio/present.current"             , pub_ratio_present_[PRESENT_CURRENT      ],  1u);
    this->get_parameter_or("pub_ratio/present.velocity"            , pub_ratio_present_[PRESENT_VELOCITY     ],  1u);
    this->get_parameter_or("pub_ratio/present.position"            , pub_ratio_present_[PRESENT_POSITION     ],  1u);
    this->get_parameter_or("pub_ratio/present.velocity_trajectory" , pub_ratio_present_[VELOCITY_TRAJECTORY  ],  0u);
    this->get_parameter_or("pub_ratio/present.position_trajectory" , pub_ratio_present_[POSITION_TRAJECTORY  ],  0u);
    this->get_parameter_or("pub_ratio/present.input_voltage"       , pub_ratio_present_[PRESENT_INPUT_VOLTAGE], 10u);
    this->get_parameter_or("pub_ratio/present.temperature"         , pub_ratio_present_[PRESENT_TEMPERATURE   ], 10u);
    this->get_parameter_or("pub_ratio/status" , pub_ratio_["status"], 50u);
    this->get_parameter_or("pub_ratio/goal"   , pub_ratio_["goal"] ,   0u);
    this->get_parameter_or("pub_ratio/gain"   , pub_ratio_["gain"] ,   0u);
    this->get_parameter_or("pub_ratio/limit"  , pub_ratio_["limit"],   0u);
    this->get_parameter_or("pub_ratio/error"  , pub_ratio_["error"], 100u);
    this->get_parameter_or("max_log_width"     , width_log_      ,   7u);
    this->get_parameter_or("use/split_write"    , use_split_write_    , false);
    this->get_parameter_or("use/split_read"     , use_split_read_     , false);
    this->get_parameter_or("use/fast_read"      , use_fast_read_      , true);
    this->get_parameter_or("verbose/callback"           , verbose_callback_, false);
    this->get_parameter_or("verbose/write_goal"         , verbose_["w_goal"  ], false);
    this->get_parameter_or("verbose/write_gain"         , verbose_["w_gain"  ], false);
    this->get_parameter_or("verbose/write_limit"        , verbose_["w_limit" ], false);
    this->get_parameter_or("verbose/read_status/raw"    , verbose_["r_status"    ], false);
    this->get_parameter_or("verbose/read_status/err"    , verbose_["r_status_err"], false);
    this->get_parameter_or("verbose/read_present/raw"   , verbose_["r_present"    ], false);
    this->get_parameter_or("verbose/read_present/err"   , verbose_["r_present_err"], false);
    this->get_parameter_or("verbose/read_goal/raw"      , verbose_["r_goal"    ], false);
    this->get_parameter_or("verbose/read_goal/err"      , verbose_["r_goal_err"], false);
    this->get_parameter_or("verbose/read_gain/raw"      , verbose_["r_gain"    ], false);
    this->get_parameter_or("verbose/read_gain/err"      , verbose_["r_gain_err"], false);
    this->get_parameter_or("verbose/read_limit/raw"     , verbose_["r_limit"    ], false);
    this->get_parameter_or("verbose/read_limit/err"     , verbose_["r_limit_err"], false);
    this->get_parameter_or("verbose/read_hardware_error", verbose_["r_hwerr" ], false);
    this->get_parameter_or("middle/no_response_id_auto_remove_count", auto_remove_count_   , 0u);

    // id_listの作成
    this->get_parameter_or("default/profile_vel", default_profile_vel_deg_s_, 100.0);
    this->get_parameter_or("default/profile_acc", default_profile_acc_deg_ss_, 600.0);
    int num_expected; this->get_parameter_or("init/expected_servo_num"     , num_expected, 0);
    int times_retry ; this->get_parameter_or("init/auto_search_retry_times", times_retry , 5);
    int id_min      ; this->get_parameter_or("init/auto_search_min_id"     , id_min      , 1);
    int id_max      ; this->get_parameter_or("init/auto_search_max_id"     , id_max      , 35);
                      this->get_parameter_or("init/hardware_error_auto_clean", do_clean_hwerr_, true);
                      this->get_parameter_or("init/torque_auto_enable"       , do_torque_on_  , true);
    if ( num_expected>0 ) ROS_INFO("Expected number of Dynamixel is [%d]", num_expected);
    else                  ROS_WARN("\nExpected number of Dynamixel is not set. Free number of Dynamixel is allowed");
    ROS_INFO(" Auto scanning Dynamixel (id range [%d] to [%d]) ...", id_min, id_max);
    /* *********************** dynamixelを探索し，初期化する ***********************************/
    /* */auto num_found = ScanDynamixels(id_min, id_max, num_expected, times_retry);
    /* ***********************************************************************************/
    if( num_found==0 ) { // 見つからなかった場合は初期化失敗で終了
        ROS_ERROR("Dynamixel is not found in USB device [%s]", dyn_comm_.port_name().c_str());
        if ( !is_debug ) throw std::runtime_error("Initialization failed (no dynamixel found)");
    }
    if( num_expected>0 && num_expected!=num_found ) { // 期待数が設定されているときに、見つかった数が期待数と異なる場合は初期化失敗で終了
        ROS_ERROR("Number of Dynamixel is not matched. Expected [%d], but found [%d]. please check & retry", num_expected, num_found);
        if ( !is_debug ) throw std::runtime_error("Initialization failed (number of dynamixel is not matched)");
    }
    ROS_INFO("  ... Finish scanning Dynamixel");

    auto callback_group_subscriber = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_subscriber;
    // Subscriber / Publisherの設定
    if ( num_[SERIES_X] > 0 ) {
        sub_dxl_x_cmds_  = create_subscription<DxlCommandsX>("dynamixel/commands/x", 10, bind(&DynamixelHandler::CallbackCmdsX, this, _1));
        sub_ctrl_x_pwm_  = create_subscription<DynamixelControlXPwm>                 ("dynamixel/command/x/pwm_control",                   4, bind(&DynamixelHandler::CallbackCmd_X_Pwm, this, _1));
        sub_ctrl_x_cur_  = create_subscription<DynamixelControlXCurrent>             ("dynamixel/command/x/current_control",               4, bind(&DynamixelHandler::CallbackCmd_X_Current, this, _1));
        sub_ctrl_x_vel_  = create_subscription<DynamixelControlXVelocity>            ("dynamixel/command/x/velocity_control",              4, bind(&DynamixelHandler::CallbackCmd_X_Velocity, this, _1));
        sub_ctrl_x_pos_  = create_subscription<DynamixelControlXPosition>            ("dynamixel/command/x/position_control",              4, bind(&DynamixelHandler::CallbackCmd_X_Position, this, _1));
        sub_ctrl_x_epos_ = create_subscription<DynamixelControlXExtendedPosition>    ("dynamixel/command/x/extended_position_control",     4, bind(&DynamixelHandler::CallbackCmd_X_ExtendedPosition, this, _1));
        sub_ctrl_x_cpos_ = create_subscription<DynamixelControlXCurrentBasePosition> ("dynamixel/command/x/current_base_position_control", 4, bind(&DynamixelHandler::CallbackCmd_X_CurrentBasePosition, this, _1));
    }
    if ( num_[SERIES_P] > 0) {
        sub_dxl_p_cmds_  = create_subscription<DxlCommandsP>("dynamixel/commands/p", 10, bind(&DynamixelHandler::CallbackCmdsP, this, _1));
        sub_ctrl_p_pwm_  = create_subscription<DynamixelControlPPwm>             ("dynamixel/command/p/pwm_control",               4, bind(&DynamixelHandler::CallbackCmd_P_Pwm, this, _1));
        sub_ctrl_p_cur_  = create_subscription<DynamixelControlPCurrent>         ("dynamixel/command/p/current_control",           4, bind(&DynamixelHandler::CallbackCmd_P_Current, this, _1));
        sub_ctrl_p_vel_  = create_subscription<DynamixelControlPVelocity>        ("dynamixel/command/p/velocity_control",          4, bind(&DynamixelHandler::CallbackCmd_P_Velocity, this, _1));
        sub_ctrl_p_pos_  = create_subscription<DynamixelControlPPosition>        ("dynamixel/command/p/position_control",          4, bind(&DynamixelHandler::CallbackCmd_P_Position, this, _1));
        sub_ctrl_p_epos_ = create_subscription<DynamixelControlPExtendedPosition>("dynamixel/command/p/extended_position_control", 4, bind(&DynamixelHandler::CallbackCmd_P_ExtendedPosition, this, _1));
    }
    sub_common_ = create_subscription<DynamixelCommonCmd>("dynamixel/command/common", 4, bind(&DynamixelHandler::CallbackCmd_Common, this, _1));
    sub_status_ = create_subscription<DynamixelStatus>   ("dynamixel/command/status", 4, bind(&DynamixelHandler::CallbackCmd_Status, this, _1));
    sub_goal_   = create_subscription<DynamixelGoal>     ("dynamixel/command/goal",   4, bind(&DynamixelHandler::CallbackCmd_Goal, this, _1));
    sub_gain_   = create_subscription<DynamixelGain>     ("dynamixel/command/gain",   4, bind(&DynamixelHandler::CallbackCmd_Gain, this, _1));
    sub_limit_  = create_subscription<DynamixelLimit>    ("dynamixel/command/limit",  4, bind(&DynamixelHandler::CallbackCmd_Limit, this, _1));

    pub_dxl_states_ = create_publisher<DxlStates>   ("dynamixel/states", 4);
    pub_status_ = create_publisher<DynamixelStatus> ("dynamixel/state/status"  , 4);
    pub_present_= create_publisher<DynamixelPresent>("dynamixel/state/present", 4);
    if ( pub_ratio_["goal"]  ) pub_goal_   = create_publisher<DynamixelGoal>   ("dynamixel/state/goal" , 4);
    if ( pub_ratio_["gain"]  ) pub_gain_   = create_publisher<DynamixelGain>   ("dynamixel/state/gain" , 4);
    if ( pub_ratio_["limit"] ) pub_limit_  = create_publisher<DynamixelLimit>  ("dynamixel/state/limit", 4);
    if ( pub_ratio_["error"] ) pub_error_  = create_publisher<DynamixelError>  ("dynamixel/state/error"  , 4);
    pub_debug_ = create_publisher<DynamixelDebug>("dynamixel/debug", 4);

    BroadcastState_Status();
    BroadcastState_Limit();
    BroadcastState_Gain();  
    BroadcastState_Goal();   
    BroadcastState_Error(); 

    ROS_INFO( "..... DynamixelHandler is initialized");
}

using std::chrono::system_clock;
using std::chrono::duration_cast;
using std::chrono::microseconds;

enum state { STATUS, PRESENT, GOAL, GAIN, LIMIT, ERROR, _num_state };

void DynamixelHandler::MainLoop(){
    static int cnt = -1; cnt++;
    static float rtime=0, wtime=0, num_pre_suc_p=1, num_pre_suc_f=1, num_pre_read=1, num_read=1;

/* 処理時間時間の計測 */ auto wstart = system_clock::now();
    //* topicをSubscribe & Dynamixelへ目標角をWrite
    SyncWriteGoal(list_write_goal_);
    list_write_goal_.clear();
    SyncWriteGain(list_write_gain_);
    list_write_gain_.clear();
    SyncWriteLimit(list_write_limit_);
    list_write_limit_.clear();
/* 処理時間時間の計測 */ wtime += duration_cast<microseconds>(system_clock::now()-wstart).count() / 1000.0;

    //* present value について read する情報を決定
    static const auto& r = pub_ratio_present_; //長いので省略
    list_read_present_.clear();
    if (r[PRESENT_PWM          ] && cnt % r[PRESENT_PWM          ] == 0) list_read_present_.insert(PRESENT_PWM          );
    if (r[PRESENT_CURRENT      ] && cnt % r[PRESENT_CURRENT      ] == 0) list_read_present_.insert(PRESENT_CURRENT      );
    if (r[PRESENT_VELOCITY     ] && cnt % r[PRESENT_VELOCITY     ] == 0) list_read_present_.insert(PRESENT_VELOCITY     );
    if (r[PRESENT_POSITION     ] && cnt % r[PRESENT_POSITION     ] == 0) list_read_present_.insert(PRESENT_POSITION     );
    if (r[VELOCITY_TRAJECTORY  ] && cnt % r[VELOCITY_TRAJECTORY  ] == 0) list_read_present_.insert(VELOCITY_TRAJECTORY  );
    if (r[POSITION_TRAJECTORY  ] && cnt % r[POSITION_TRAJECTORY  ] == 0) list_read_present_.insert(POSITION_TRAJECTORY  );
    if (r[PRESENT_INPUT_VOLTAGE] && cnt % r[PRESENT_INPUT_VOLTAGE] == 0) list_read_present_.insert(PRESENT_INPUT_VOLTAGE);
    if (r[PRESENT_TEMPERATURE  ] && cnt % r[PRESENT_TEMPERATURE  ] == 0) list_read_present_.insert(PRESENT_TEMPERATURE   );

/* 処理時間時間の計測 */ auto rstart = system_clock::now();
    //* Dynamixelから状態Read & topicをPublish
    auto msg = DxlStates().set__stamp(this->get_clock()->now());
    std::array<double, _num_state> success_rate{}; // 0初期化する．
    set<uint8_t> target_id_set; for (auto id : id_set_) if ( ping_err_[id]==0 ) target_id_set.insert(id);
    if ( pub_ratio_["status"] && cnt % pub_ratio_["status"] == 0 ) {
        CheckDynamixels(); // Statusに該当するもろもろをチェック
        success_rate[STATUS] = 1.0;
        for (auto id: id_set_) if ( auto_remove_count_ ) 
            if ( ping_err_[id] > auto_remove_count_) RemoveDynamixel(id);
        if (success_rate[STATUS]) msg.status = BroadcastState_Status();
    }
    if ( !list_read_present_.empty() ){
        success_rate[PRESENT] = SyncReadPresent(list_read_present_, target_id_set);
        num_pre_read++;
        num_pre_suc_p += success_rate[PRESENT] > 0.0;
        num_pre_suc_f += success_rate[PRESENT] > 1.0-1e-6;
        if ( success_rate[PRESENT]>0.0 ) msg.present = BroadcastState_Present();
    }
    if ( pub_ratio_["goal"] && cnt % pub_ratio_["goal"] == 0 ) {
        success_rate[GOAL] = SyncReadGoal(list_read_goal_, target_id_set);
        if ( success_rate[GOAL   ]>0.0 ) msg.goal = BroadcastState_Goal();
    }
    if ( pub_ratio_["gain"] && cnt % pub_ratio_["gain"] == 0 ) {
        success_rate[GAIN] = SyncReadGain(list_read_gain_, target_id_set);
        if ( success_rate[GAIN   ]>0.0 ) msg.gain = BroadcastState_Gain();
    }
    if ( pub_ratio_["limit"] && cnt % pub_ratio_["limit"] == 0 ) {
        success_rate[LIMIT] = SyncReadLimit(list_read_limit_, target_id_set);
        if ( success_rate[LIMIT  ]>0.0 ) msg.limit = BroadcastState_Limit();
    }
    if ( pub_ratio_["error"] && cnt % pub_ratio_["error"] == 0 ) {
        success_rate[ERROR] = SyncReadHardwareErrors(target_id_set);
        if ( success_rate[ERROR  ]>0.0 ) msg.error = BroadcastState_Error();
    }
    bool is_any_read = std::any_of( success_rate.begin(), success_rate.end(), [](auto& x){ return x > 0.0; });
    if ( is_any_read ) {
        num_read++;
        BroadcastDebug();
        pub_dxl_states_->publish(msg);
    }
/* 処理時間時間の計測 */ rtime += duration_cast<microseconds>(system_clock::now()-rstart).count() / 1000.0;

    //* デバック
    if ( ratio_mainloop_ !=0 )
    if ( cnt % ratio_mainloop_ == 0) {
        float partial_suc = 100*num_pre_suc_p/num_pre_read; 
        float full_suc = 100*num_pre_suc_f/num_pre_read;
        char msg[100]; sprintf(msg, "Loop [%d]: write=%.2fms read=%.2fms(p/f=%3.0f%%/%3.0f%%)",
                               cnt, wtime/ratio_mainloop_, rtime/num_read, partial_suc, full_suc);
        if (full_suc < 80) ROS_ERROR("%s", msg); else if (partial_suc < 99) ROS_WARN("%s", msg); else ROS_INFO("%s", msg);
        wtime = 0.0; /* mainloopで行われてる処理の計測時間を初期化 */
    }
    if ( cnt % max({(int)loop_rate_, (int)ratio_mainloop_, 10}) == 0)
        rtime = num_pre_suc_p = num_pre_suc_f = num_pre_read = num_read = 0.00001; /* present value の read の周期で行われてる処理の初期化 */ 
}

DynamixelHandler::~DynamixelHandler(){
    ROS_INFO( "Terminating DynamixelHandler ...");
    bool do_torque_off; get_parameter_or("term/torque_auto_disable", do_torque_off, true);
    if ( do_torque_off ) for ( auto id : id_set_ ) TorqueOff(id);
    StopDynamixels();
    ROS_INFO( "  ... DynamixelHandler is terminated");
}

#include <chrono>
using namespace std::chrono_literals;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
	/*Initialization*/
    auto node = std::make_shared<DynamixelHandler>();
    /*Mainloop*/
    auto timer_ = node.get()->create_wall_timer(
          1.0s/(node->loop_rate_)
        , bind(&DynamixelHandler::MainLoop, node.get())
    ); // 変数に保存する必要あり
    /*Interruption*/
    auto executor = rclcpp::executors::MultiThreadedExecutor::make_unique();
    executor->add_node(node);
    executor->spin();
    /*Termination*/
    node.reset(); // rclcpp::shutdown() の前に呼ぶ必要あり
    rclcpp::shutdown();
    return 0;
}