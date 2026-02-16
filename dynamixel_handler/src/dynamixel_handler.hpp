#ifndef DYNAMIXEL_HANDLER_H
#define DYNAMIXEL_HANDLER_H

#include "rclcpp/rclcpp.hpp"

#include "dynamixel_communicator.h"
#include <memory>

namespace dynamixel_handler_msgs {
    namespace msg {
        #define DYNAMIXEL_MSG_FWD(msg_name) \
        template<class ContainerAllocator> struct msg_name##_; \
        using msg_name = msg_name##_<std::allocator<void>>

        DYNAMIXEL_MSG_FWD(DxlStates);
        DYNAMIXEL_MSG_FWD(DxlCommandsX);
        DYNAMIXEL_MSG_FWD(DxlCommandsP);
        DYNAMIXEL_MSG_FWD(DxlCommandsPro);
        DYNAMIXEL_MSG_FWD(DxlCommandsAll);
        DYNAMIXEL_MSG_FWD(DynamixelStatus);
        DYNAMIXEL_MSG_FWD(DynamixelPresent);
        DYNAMIXEL_MSG_FWD(DynamixelGoal);
        DYNAMIXEL_MSG_FWD(DynamixelGain);
        DYNAMIXEL_MSG_FWD(DynamixelError);
        DYNAMIXEL_MSG_FWD(DynamixelLimit);
        DYNAMIXEL_MSG_FWD(DynamixelExtra);
        DYNAMIXEL_MSG_FWD(DynamixelControlXPwm);
        DYNAMIXEL_MSG_FWD(DynamixelControlXCurrent);
        DYNAMIXEL_MSG_FWD(DynamixelControlXVelocity);
        DYNAMIXEL_MSG_FWD(DynamixelControlXPosition);
        DYNAMIXEL_MSG_FWD(DynamixelControlXExtendedPosition);
        DYNAMIXEL_MSG_FWD(DynamixelControlXCurrentBasePosition);
        DYNAMIXEL_MSG_FWD(DynamixelControlPPwm);
        DYNAMIXEL_MSG_FWD(DynamixelControlPPosition);
        DYNAMIXEL_MSG_FWD(DynamixelControlPVelocity);
        DYNAMIXEL_MSG_FWD(DynamixelControlPCurrent);
        DYNAMIXEL_MSG_FWD(DynamixelControlPExtendedPosition);
        DYNAMIXEL_MSG_FWD(DynamixelControlProPosition);
        DYNAMIXEL_MSG_FWD(DynamixelControlProVelocity);
        DYNAMIXEL_MSG_FWD(DynamixelControlProCurrent);
        DYNAMIXEL_MSG_FWD(DynamixelControlProExtendedPosition);
        DYNAMIXEL_MSG_FWD(DynamixelDebug);
        DYNAMIXEL_MSG_FWD(DynamixelShortcut);

        #undef DYNAMIXEL_MSG_FWD
    }  // namespace msg
}  // namespace dynamixel_handler_msgs

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
        void CallbackCmd_Pro_Current         (const DynamixelControlProCurrent& msg);
        void CallbackCmd_Pro_Velocity        (const DynamixelControlProVelocity& msg);
        void CallbackCmd_Pro_Position        (const DynamixelControlProPosition& msg);
        void CallbackCmd_Pro_ExtendedPosition(const DynamixelControlProExtendedPosition& msg);
        void CallbackCmd_Status (const DynamixelStatus& msg); 
        void CallbackCmd_Goal   (const DynamixelGoal& msg); 
        void CallbackCmd_Gain   (const DynamixelGain& msg); 
        void CallbackCmd_Limit  (const DynamixelLimit& msg);
        // void CallbackExtra (const DynamixelExtra& msg);  // todo
        void CallbackCmdsX  (std::shared_ptr<DxlCommandsX> msg);
        void CallbackCmdsP  (std::shared_ptr<DxlCommandsP> msg);
        void CallbackCmdsPro(std::shared_ptr<DxlCommandsPro> msg);
        void CallbackCmdsAll(std::shared_ptr<DxlCommandsAll> msg);
        void SetupRosInterfaces_byProgram();
        void SetupRosInterfaces_byCLI();

        //* ROS publisher subscriber instance
        rclcpp::PublisherBase::SharedPtr  pub_status_;
        rclcpp::PublisherBase::SharedPtr  pub_present_;
        rclcpp::PublisherBase::SharedPtr  pub_goal_;
        rclcpp::PublisherBase::SharedPtr  pub_gain_;
        rclcpp::PublisherBase::SharedPtr  pub_limit_;
        rclcpp::PublisherBase::SharedPtr  pub_error_;
        rclcpp::PublisherBase::SharedPtr  pub_dxl_states_;
        rclcpp::PublisherBase::SharedPtr  pub_debug_;
        rclcpp::SubscriptionBase::SharedPtr sub_shortcut_;
        rclcpp::SubscriptionBase::SharedPtr sub_ctrl_x_pwm_;
        rclcpp::SubscriptionBase::SharedPtr sub_ctrl_x_cur_;
        rclcpp::SubscriptionBase::SharedPtr sub_ctrl_x_vel_;
        rclcpp::SubscriptionBase::SharedPtr sub_ctrl_x_pos_;
        rclcpp::SubscriptionBase::SharedPtr sub_ctrl_x_epos_;
        rclcpp::SubscriptionBase::SharedPtr sub_ctrl_x_cpos_;
        rclcpp::SubscriptionBase::SharedPtr sub_ctrl_p_pwm_;
        rclcpp::SubscriptionBase::SharedPtr sub_ctrl_p_cur_;
        rclcpp::SubscriptionBase::SharedPtr sub_ctrl_p_vel_;
        rclcpp::SubscriptionBase::SharedPtr sub_ctrl_p_pos_;
        rclcpp::SubscriptionBase::SharedPtr sub_ctrl_p_epos_;
        rclcpp::SubscriptionBase::SharedPtr sub_ctrl_pro_cur_;
        rclcpp::SubscriptionBase::SharedPtr sub_ctrl_pro_vel_;
        rclcpp::SubscriptionBase::SharedPtr sub_ctrl_pro_pos_;
        rclcpp::SubscriptionBase::SharedPtr sub_ctrl_pro_epos_;
        rclcpp::SubscriptionBase::SharedPtr sub_status_;
        rclcpp::SubscriptionBase::SharedPtr sub_goal_;
        rclcpp::SubscriptionBase::SharedPtr sub_gain_;
        rclcpp::SubscriptionBase::SharedPtr sub_limit_;
        rclcpp::SubscriptionBase::SharedPtr sub_dxl_x_cmds_;
        rclcpp::SubscriptionBase::SharedPtr sub_dxl_p_cmds_;
        rclcpp::SubscriptionBase::SharedPtr sub_dxl_pro_cmds_;
        rclcpp::SubscriptionBase::SharedPtr sub_dxl_all_cmds_;

        //* 各種のフラグとパラメータ
        unsigned int  loop_rate_ = 50;
        unsigned int  ratio_mainloop_   = 100; // 0の時は初回のみ
        unsigned int  auto_remove_count_ = 0;
        unsigned int  width_log_ = 7;
        map<string, bool> use_;
        map<string, bool> verbose_; // 各種のverboseフラグ
                    bool  verbose_callback_ = false;
        map<string, double> default_;
        bool do_clean_hwerr_ = false;
        bool do_torque_on_   = false;
        bool do_pub_pre_all_ = true;
        bool do_torque_off_  = true;
        bool do_stop_end_    = true;
        bool use_split_write_     = false;
        bool use_split_read_      = false;
        bool use_fast_read_       = false;

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

        //* 実装の上でどうしても必要になったUtility関数群
        template <typename Container> 
        vector<id_t> id_filter(const Container& id_set, series_t series);
        bool check_series(id_t id, series_t series);
        
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
        class ImuOpenCR;
        std::unique_ptr<ImuOpenCR> imu_opencr_; //
};

#endif /* DYNAMIXEL_HANDLER_H */
