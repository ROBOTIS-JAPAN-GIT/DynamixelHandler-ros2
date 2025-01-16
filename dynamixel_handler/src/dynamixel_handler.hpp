#ifndef DYNAMIXEL_HANDLER_H
#define DYNAMIXEL_HANDLER_H

#include "rclcpp/rclcpp.hpp"

#include "dynamixel_communicator.h"

#include "dynamixel_handler_msgs/msg/dxl_states.hpp"
#include "dynamixel_handler_msgs/msg/dxl_commands_x.hpp"
#include "dynamixel_handler_msgs/msg/dxl_commands_p.hpp"
#include "dynamixel_handler_msgs/msg/dxl_commands_all.hpp"

#include "dynamixel_handler_msgs/msg/dynamixel_status.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_present.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_goal.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_gain.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_error.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_limit.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_extra.hpp"

#include "dynamixel_handler_msgs/msg/dynamixel_control_x_pwm.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_x_current.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_x_velocity.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_x_position.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_x_extended_position.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_x_current_base_position.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_p_pwm.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_p_position.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_p_velocity.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_p_current.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_control_p_extended_position.hpp"

#include "dynamixel_handler_msgs/msg/dynamixel_debug.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_shortcut.hpp"

using namespace dynamixel_handler_msgs::msg;

#include <string>
using std::string;
#include <map>
using std::map;
#include <unordered_map>
using std::unordered_map;
#include <vector>
using std::vector;
#include <array>
using std::array;
#include <set>
using std::set;
#include <unordered_set>
using std::unordered_set;
#include <algorithm>
using std::minmax_element;
using std::clamp;
using std::min;
using std::max;
#include <tuple>
using std::tuple;

/**
 * DynamixelをROSで動かすためのクラス．本pkgのメインクラス． 
 * 検出したDynamixelに関して，idのみベクトルとして保持し，
 * それ以外の情報はすべて，idをキーとしたmapに保持している．
*/
class DynamixelHandler : public rclcpp::Node {
    public:
        //* ROS 初期設定とメインループ
        DynamixelHandler();  // コンストラクタ, 初期設定を行う
        ~DynamixelHandler(); // デストラクタ,  終了処理を行う
        void MainLoop();     // メインループ

        //* ROS publishを担う関数と subscliber callback関数
        DynamixelDebug BroadcastDebug();
        DynamixelStatus BroadcastState_Status();
        DynamixelPresent BroadcastState_Present();
        DynamixelGoal BroadcastState_Goal(); 
        DynamixelGain BroadcastState_Gain();
        DynamixelLimit BroadcastState_Limit();
        DynamixelError BroadcastState_Error();
        // void BroadcastStateExtra();  // todo
        void CallbackShortcut                (const DynamixelShortcut& msg);
        void CallbackCmd_X_Pwm                 (const DynamixelControlXPwm& msg);
        void CallbackCmd_X_Current             (const DynamixelControlXCurrent& msg);
        void CallbackCmd_X_Velocity            (const DynamixelControlXVelocity& msg);
        void CallbackCmd_X_Position            (const DynamixelControlXPosition& msg);
        void CallbackCmd_X_ExtendedPosition    (const DynamixelControlXExtendedPosition& msg);
        void CallbackCmd_X_CurrentBasePosition (const DynamixelControlXCurrentBasePosition& msg);
        void CallbackCmd_P_Pwm             (const DynamixelControlPPwm& msg);
        void CallbackCmd_P_Current         (const DynamixelControlPCurrent& msg);
        void CallbackCmd_P_Velocity        (const DynamixelControlPVelocity& msg);
        void CallbackCmd_P_Position        (const DynamixelControlPPosition& msg);
        void CallbackCmd_P_ExtendedPosition(const DynamixelControlPExtendedPosition& msg);
        void CallbackCmd_Status (const DynamixelStatus& msg); 
        void CallbackCmd_Goal   (const DynamixelGoal& msg); 
        void CallbackCmd_Gain   (const DynamixelGain& msg); 
        void CallbackCmd_Limit  (const DynamixelLimit& msg);
        // void CallbackExtra (const DynamixelExtra& msg);  // todo
        void CallbackCmdsX               (const DxlCommandsX::SharedPtr msg);
        void CallbackCmdsP               (const DxlCommandsP::SharedPtr msg);
        void CallbackCmdsAll             (const DxlCommandsAll::SharedPtr msg);

        //* ROS publisher subscriber instance
        rclcpp::Publisher<DynamixelStatus>::SharedPtr  pub_status_;
        rclcpp::Publisher<DynamixelPresent>::SharedPtr pub_present_;
        rclcpp::Publisher<DynamixelGoal>::SharedPtr    pub_goal_;
        rclcpp::Publisher<DynamixelGain>::SharedPtr    pub_gain_;
        rclcpp::Publisher<DynamixelLimit>::SharedPtr   pub_limit_;
        rclcpp::Publisher<DynamixelError>::SharedPtr   pub_error_;
        rclcpp::Publisher<DxlStates>::SharedPtr  pub_dxl_states_;
        rclcpp::Publisher<DynamixelDebug>::SharedPtr   pub_debug_;
        rclcpp::Subscription<DynamixelShortcut>::SharedPtr sub_shortcut_;
        rclcpp::Subscription<DynamixelControlXPwm>::SharedPtr                 sub_ctrl_x_pwm_;
        rclcpp::Subscription<DynamixelControlXCurrent>::SharedPtr             sub_ctrl_x_cur_;
        rclcpp::Subscription<DynamixelControlXVelocity>::SharedPtr            sub_ctrl_x_vel_;
        rclcpp::Subscription<DynamixelControlXPosition>::SharedPtr            sub_ctrl_x_pos_;
        rclcpp::Subscription<DynamixelControlXExtendedPosition>::SharedPtr    sub_ctrl_x_epos_;
        rclcpp::Subscription<DynamixelControlXCurrentBasePosition>::SharedPtr sub_ctrl_x_cpos_;
        rclcpp::Subscription<DynamixelControlPPwm>::SharedPtr              sub_ctrl_p_pwm_;
        rclcpp::Subscription<DynamixelControlPCurrent>::SharedPtr          sub_ctrl_p_cur_;
        rclcpp::Subscription<DynamixelControlPVelocity>::SharedPtr         sub_ctrl_p_vel_;
        rclcpp::Subscription<DynamixelControlPPosition>::SharedPtr         sub_ctrl_p_pos_;
        rclcpp::Subscription<DynamixelControlPExtendedPosition>::SharedPtr sub_ctrl_p_epos_;
        rclcpp::Subscription<DynamixelStatus>::SharedPtr  sub_status_;
        rclcpp::Subscription<DynamixelGoal>::SharedPtr    sub_goal_;
        rclcpp::Subscription<DynamixelGain>::SharedPtr    sub_gain_;
        rclcpp::Subscription<DynamixelLimit>::SharedPtr   sub_limit_;
        rclcpp::Subscription<DxlCommandsX>::SharedPtr sub_dxl_x_cmds_;
        rclcpp::Subscription<DxlCommandsP>::SharedPtr sub_dxl_p_cmds_;
        rclcpp::Subscription<DxlCommandsAll>::SharedPtr sub_dxl_all_cmds_;

        //* 各種のフラグとパラメータ
        unsigned int  loop_rate_ = 50;
        unsigned int  ratio_mainloop_   = 100; // 0の時は初回のみ
        unsigned int  auto_remove_count_ = 0;
        unsigned int  width_log_ = 7;
        bool use_split_write_     = false;
        bool use_split_read_      = false;
        bool use_fast_read_       = false;
        map<string, bool> verbose_; // 各種のverboseフラグ
                    bool  verbose_callback_ = false;
        double default_profile_vel_deg_s_ = 0.0;
        double default_profile_acc_deg_ss_ = 0.0;
        double default_return_delay_time_us_ = 0.0;
        bool do_clean_hwerr_ = false;
        bool do_torque_on_   = false;
        bool do_pub_pre_all_ = true;
        bool do_torque_off_  = true;
        bool do_stop_end_    = true;

        //* Dynamixelとの通信
        DynamixelCommunicator dyn_comm_;

        //* Dynamixelを扱うための変数群 
        enum GoalIndex { //　goal_w_のIndex, サーボに毎周期で書き込むことができる値
            GOAL_PWM     ,
            GOAL_CURRENT ,
            GOAL_VELOCITY,
            PROFILE_ACC  ,
            PROFILE_VEL  ,
            GOAL_POSITION,
            /*Indexの最大値*/_num_goal     
        };
        enum PresentIndex { // present_r_のIndex, サーボから毎周期で読み込むことができる値
            PRESENT_PWM          ,
            PRESENT_CURRENT      ,
            PRESENT_VELOCITY     ,
            PRESENT_POSITION     ,
            VELOCITY_TRAJECTORY  ,
            POSITION_TRAJECTORY  ,
            PRESENT_INPUT_VOLTAGE,
            PRESENT_TEMPERATURE  ,
            /*Indexの最大値*/_num_present
        };
        enum HWErrIndex { // hardware_error_のIndex, サーボが起こしたハードウェアエラー
            INPUT_VOLTAGE    ,
            MOTOR_HALL_SENSOR,
            OVERHEATING      ,
            MOTOR_ENCODER    ,
            ELECTRONICAL_SHOCK ,
            OVERLOAD         ,
            /*Indexの最大値*/_num_hw_err
        };
        enum LimitIndex { // limit_w/r_のIndex, 各種の制限値
            NONE = -1, // Indexには使わない特殊値
            TEMPERATURE_LIMIT ,
            MAX_VOLTAGE_LIMIT ,
            MIN_VOLTAGE_LIMIT ,
            PWM_LIMIT         ,
            CURRENT_LIMIT     ,
            ACCELERATION_LIMIT,
            VELOCITY_LIMIT    ,
            MAX_POSITION_LIMIT,
            MIN_POSITION_LIMIT,
            /*Indexの最大値*/_num_limit
        };
        enum GainIndex { // gain_w/r_のIndex, 各種のゲイン値
            VELOCITY_I_GAIN     ,
            VELOCITY_P_GAIN     ,
            POSITION_D_GAIN     ,
            POSITION_I_GAIN     ,
            POSITION_P_GAIN     ,
            FEEDFORWARD_ACC_GAIN,
            FEEDFORWARD_VEL_GAIN, 
            /*Indexの最大値*/_num_gain           
        };
        // 連結したサーボの基本情報
        using id_t     = uint8_t;
        using model_t  = uint16_t; // DynamixelModelNumber 型を使うと, read(model_number) の結果をそのまま使えないので，uint16_t にしている
        using series_t = DynamixelSeries;
        using torque_t = DynamixelTorquePermission;
        static inline set<id_t> id_set_; // chained dynamixel id list // 順序を保持する必要があうのでset
        // 連結しているサーボの個々の状態を保持するunordered_map
        static inline unordered_map<id_t, model_t > model_;  // 各dynamixelの id と model のマップ
        static inline unordered_map<id_t, series_t> series_; // 各dynamixelの id と series のマップ
        static inline unordered_map<id_t, uint64_t> ping_err_; // 各dynamixelの id と 連続でpingに応答しなかった回数のマップ
        static inline unordered_map<id_t, torque_t> tq_mode_;  // 各dynamixelの id と トルクON/OFF のマップ
        static inline unordered_map<id_t, uint8_t> op_mode_; // 各dynamixelの id と 制御モード のマップ
        static inline unordered_map<id_t, uint8_t> dv_mode_; // 各dynamixelの id と ドライブモード のマップ
        static inline unordered_map<id_t, array<bool,   _num_hw_err >> hardware_err_; // 各dynamixelの id と サーボが起こしたハードウェアエラーのマップ, 中身の並びはHWErrIndexに対応する
        static inline unordered_map<id_t, array<double, _num_present>> present_r_; // 各dynamixelの id と サーボから読み込んだ状態のマップ
        static inline unordered_map<id_t, array<double, _num_goal   >> goal_w_;    // 各dynamixelの id と サーボへ書き込む目標状態のマップ
        static inline unordered_map<id_t, array<double ,_num_goal   >> goal_r_;    // 各dynamixelの id と サーボから読み込んだ目標状態のマップ
        static inline unordered_map<id_t, array<uint16_t,_num_gain  >> gain_w_;    // 各dynamixelの id と サーボへ書き込むゲインのマップ
        static inline unordered_map<id_t, array<uint16_t,_num_gain  >> gain_r_;    // 各dynamixelの id と サーボから読み込んだゲインのマップ
        static inline unordered_map<id_t, array<double, _num_limit  >> limit_w_;   // 各dynamixelの id と サーボへ書き込む制限値のマップ
        static inline unordered_map<id_t, array<double, _num_limit  >> limit_r_;   // 各dynamixelの id と サーボから読み込んだ制限値のマップ

        // 上記の変数を適切に使うための補助的なフラグ
        static inline unordered_map<id_t, double> when_op_mode_updated_; // 各dynamixelの id と op_mode_ が更新された時刻のマップ
        static inline unordered_set<id_t> updated_id_goal_;    // topicのcallbackによって，goal_w_が更新されたidの集合
        static inline unordered_set<id_t> updated_id_gain_;    // topicのcallbackによって，limit_w_が更新されたidの集合
        static inline unordered_set<id_t> updated_id_limit_;   // topicのcallbackによって，limit_w_が更新されたidの集合
        static inline unordered_map<id_t, bool> has_hardware_error_;    // ハードウェアエラーを起こしているかどうか
        // 各周期で実行するserial通信の内容を決めるためのset, 順序が必要なのでset
        static inline set<GoalIndex   > goal_indice_write_;
        static inline set<GainIndex   > gain_indice_write_;
        static inline set<LimitIndex  > limit_indice_write_;
        static inline set<PresentIndex> present_indice_read_ = {PRESENT_POSITION, PRESENT_VELOCITY, PRESENT_CURRENT, PRESENT_PWM, VELOCITY_TRAJECTORY, POSITION_TRAJECTORY, PRESENT_INPUT_VOLTAGE, PRESENT_TEMPERATURE};
        static inline set<GoalIndex   > goal_indice_read_    = {GOAL_POSITION, GOAL_VELOCITY, GOAL_CURRENT, GOAL_PWM, PROFILE_ACC, PROFILE_VEL};
        static inline set<GainIndex   > gain_indice_read_    = {VELOCITY_I_GAIN, VELOCITY_P_GAIN, POSITION_D_GAIN, POSITION_I_GAIN, POSITION_P_GAIN, FEEDFORWARD_ACC_GAIN, FEEDFORWARD_VEL_GAIN};
        static inline set<LimitIndex  > limit_indice_read_   = {TEMPERATURE_LIMIT, MAX_VOLTAGE_LIMIT, MIN_VOLTAGE_LIMIT, PWM_LIMIT, CURRENT_LIMIT, ACCELERATION_LIMIT, VELOCITY_LIMIT, MAX_POSITION_LIMIT, MIN_POSITION_LIMIT};
        // read & publish の周期を決めるためのmap
        static inline unordered_map<string, unsigned int>       pub_ratio_; // present value以外の周期
        static inline array<unsigned int, _num_present> pub_ratio_present_;

        //* 単体通信を組み合わせた上位機能
        bool ScanDynamixels(id_t id_min, id_t id_max, uint32_t num_expected, uint32_t time_retry_ms);
        bool DummyUpDynamixel(id_t servo_id);
        bool RemoveDynamixel(id_t servo_id);
        bool AddDynamixel(id_t servo_id);
        bool ClearHardwareError(id_t servo_id);
        bool ChangeOperatingMode(id_t servo_id, DynamixelOperatingMode mode);
        bool TorqueOn(id_t servo_id);
        bool TorqueOff(id_t servo_id);
        bool UnifyBaudrate(uint64_t baudrate);
        //* Dynamixel単体との通信による下位機能
        uint8_t ReadHardwareError(id_t servo_id);
        bool ReadTorqueEnable(id_t servo_id);
        double  ReadPresentPWM(id_t servo_id);
        double  ReadPresentCurrent(id_t servo_id);
        double  ReadPresentVelocity(id_t servo_id);
        double  ReadPresentPosition(id_t servo_id);
        double  ReadGoalPWM(id_t servo_id);
        double  ReadGoalCurrent(id_t servo_id);
        double  ReadGoalVelocity(id_t servo_id);
        double  ReadGoalPosition(id_t servo_id);
        double  ReadProfileAcc(id_t servo_id);
        double  ReadProfileVel(id_t servo_id);
        double  ReadHomingOffset(id_t servo_id);
        double  ReadBusWatchdog(id_t servo_id);
        double  ReadReturnDelayTime(id_t servo_id);
        uint8_t ReadOperatingMode(id_t servo_id);
        uint8_t ReadDriveMode(id_t servo_id);
        bool WriteTorqueEnable(id_t servo_id, bool enable);
        bool WriteGoalPWM(id_t servo_id, double pwm);
        bool WriteGoalCurrent(id_t servo_id, double current);
        bool WriteGoalVelocity(id_t servo_id, double velocity);
        bool WriteGoalPosition(id_t servo_id, double position);
        bool WriteProfileAcc(id_t servo_id, double acceleration);
        bool WriteProfileVel(id_t servo_id, double velocity);
        bool WriteHomingOffset(id_t servo_id, double offset);
        bool WriteOperatingMode(id_t servo_id, uint8_t mode);
        bool WriteReturnDelayTime(id_t servo_id, double time);
        bool WriteBusWatchdog(id_t servo_id, double time);
        bool WriteGains(id_t servo_id, array<uint16_t, _num_gain> gains);
        //* 連結しているDynamixelに一括で読み書きするloopで使用する機能
        template <typename Container> vector<id_t> id_filter(const Container& id_set, series_t series);
        void SyncWrite_log(const vector<DynamixelAddress>& addr_list, const map<id_t, vector<int64_t>>& id_data_map, bool verbose);
        void SyncWrite_log(const        DynamixelAddress&  addr,      const vector<id_t>& id_list, const vector<int64_t>& data_list, bool verbose);
        map<id_t, vector<int64_t>> SyncRead_log(const vector<DynamixelAddress>& addr_list, const vector<id_t>& id_list, bool verbose, bool verbose_err);
        map<id_t,        int64_t > SyncRead_log(const        DynamixelAddress&  addr,      const vector<id_t>& id_list, bool verbose, bool verbose_err);
        template <typename Addr=AddrCommon> void SyncWriteGoal (set<GoalIndex>   goal_indice_write, const unordered_set<id_t>&  updated_id_goal);
        template <typename Addr=AddrCommon> void SyncWriteGain (set<GainIndex>   gain_indice_write, const unordered_set<id_t>&  updated_id_gain);
        template <typename Addr=AddrCommon> void SyncWriteLimit(set<LimitIndex> limit_indice_write, const unordered_set<id_t>& updated_id_limit);
        template <typename Addr=AddrCommon> tuple<double, uint8_t> SyncReadPresent(set<PresentIndex> present_indice_read, const set<id_t>& id_set=id_set_);
        template <typename Addr=AddrCommon> tuple<double, uint8_t> SyncReadGoal   (set<GoalIndex>       goal_indice_read, const set<id_t>& id_set=id_set_);
        template <typename Addr=AddrCommon> tuple<double, uint8_t> SyncReadGain   (set<GainIndex>       gain_indice_read, const set<id_t>& id_set=id_set_); 
        template <typename Addr=AddrCommon> tuple<double, uint8_t> SyncReadLimit  (set<LimitIndex>     limit_indice_read, const set<id_t>& id_set=id_set_);
        template <typename Addr=AddrCommon> tuple<double, uint8_t> SyncReadHardwareErrors(const set<id_t>& id_set=id_set_);
        template <typename Addr=AddrCommon> void StopDynamixels (const set<id_t>& id_set=id_set_);
        template <typename Addr=AddrCommon> void CheckDynamixels(const set<id_t>& id_set=id_set_);

        class ExternalPort; // XH540とPシリーズに搭載されている外部ポートを使用するためのクラス，実際の宣言と定義は別ファイル
        std::unique_ptr<ExternalPort> external_port_; // 
};

#endif /* DYNAMIXEL_HANDLER_H */
