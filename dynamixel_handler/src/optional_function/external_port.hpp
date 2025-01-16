#ifndef DYNAMIXEL_EXTERNAL_PORT_H
#define DYNAMIXEL_EXTERNAL_PORT_H

#include "rclcpp/rclcpp.hpp"

#include "dynamixel_handler_msgs/msg/dxl_external_port.hpp"
using namespace dynamixel_handler_msgs::msg;

#include "../dynamixel_handler.hpp"

using std::bind;
using std::placeholders::_1;

#include <string>
using std::string;
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
#include <tuple>
using std::tuple;

/**
 * Dynanixel XH540 や P シリーズが持つ外部ポートを扱うためのクラス
 * Dynaimxelの基本動作に対する拡張機能という位置づけとしたため，このように分離して実装する．
*/
class DynamixelHandler::ExternalPort {
    public:
        //* ROS 初期設定とメインループ 
        ExternalPort(DynamixelHandler& parent); // コンストラクタ, 初期設定を行う
        ~ExternalPort(); // デストラクタ,  終了処理を行う
        void MainProccess(); // 本体のdynamixel_handlerのメインループで実行したい処理．

        // DynamixelHandlerのインスタンスを保持するための変数
        DynamixelHandler& parent_;

        //* ROS publishを担う関数と subscliber callback関数
        void BroadcastExternalPort();
        rclcpp::Publisher<DxlExternalPort>::SharedPtr pub_ex_port_;

        //* ROS publisher subscriber instance
        void CallbackExternalPort(const DxlExternalPort::SharedPtr msg);
        rclcpp::Subscription<DxlExternalPort>::SharedPtr sub_ex_port_;

        unsigned int pub_ratio_mode_ = 0; // 何回に1回publishするか
        unsigned int pub_ratio_data_ = 0; // 何回に1回publishするか
        bool verbose_callback_ = false; // callback関数のverbose設定
        bool verbose_write_    = false; // serial通信関係のverbose設定
        bool verbose_read_     = false; // serial通信関係のverbose設定
        bool verbose_read_err_ = false; // serial通信関係のverbose設定
  
        template<typename T> struct ex_port_t {
            T mode;
            T data;
        };
        static inline unordered_map<DynamixelSeries, vector<uint8_t>> ex_port_indice_ = {
            {SERIES_X, {1,2,3}}, 
            {SERIES_P, {1,2,3,4}}
        };

        // 連結しているサーボの個々の状態を保持するmap
        static inline map<uint8_t, map<uint8_t, ex_port_t<uint16_t>>> export_w_; // 各dynamixelの id と サーボへ書き込むExternalPortのマップ, ex_port は port番号と mode, data の組のマップとして扱う
        static inline map<uint8_t, map<uint8_t, ex_port_t<uint16_t>>> export_r_; // 各dynamixelの id と サーボへ書き込むExternalPortのマップ, ex_port は port番号と mode, data の組のマップとして扱う

        // 上記の変数を適切に使うための補助的なフラグ
        static inline ex_port_t<unordered_set<uint8_t>> updated_id_export_;    // topicのcallbackによって，ex_port_w_が更新されたサーボidの集合
        static inline           unordered_set<uint8_t>  touched_id_export_;    // topicのcallbackによって，一度でも指令が送られたサーボidの集合
        //* Dynamixel単体との通信による下位機能
        uint16_t ReadExternalPortMode(uint8_t servo_id, uint8_t port);
        uint16_t ReadExternalPortData(uint8_t servo_id, uint8_t port);
        bool WriteExternalPortMode(uint8_t servo_id, uint8_t port, uint16_t mode);
        bool WriteExternalPortData(uint8_t servo_id, uint8_t port, uint16_t data);
        //* 連結しているDynamixelに一括で読み書きするloopで使用する機能
        template <typename Addr=AddrCommon> void SyncWriteExternalPortMode(unordered_set<uint8_t> updated_id_mode);
        template <typename Addr=AddrCommon> void SyncWriteExternalPortData(unordered_set<uint8_t> updated_id_data);
        template <typename Addr=AddrCommon> double SyncReadExternalPortMode(set<uint8_t> id_set);
        template <typename Addr=AddrCommon> double SyncReadExternalPortData(set<uint8_t> id_set);

};

#endif /* DYNAMIXEL_EXTERNAL_PORT_H */
