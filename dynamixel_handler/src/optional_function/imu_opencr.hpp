#ifndef DYNAMIXEL_IMU_OPEN_CR_H
#define DYNAMIXEL_IMU_OPEN_CR_H

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/empty.hpp"
using namespace std_msgs::msg;
using namespace sensor_msgs::msg;
using namespace std::chrono_literals;

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
 * Robotis OpenCR に搭載されたIMUを扱うためのクラス
*/
class DynamixelHandler::ImuOpenCR {
	public:
		//* ROS 初期設定とメインループ
		ImuOpenCR(DynamixelHandler& parent); // コンストラクタ, 初期設定を行う
		~ImuOpenCR(); // デストラクタ,  終了処理を行う
		void MainProcess(); // 本体のdynamixel_handlerのメインループで実行したい処理．

		// DynamixelHandlerのインスタンスを保持するための変数
		DynamixelHandler& parent_;


		//* ROS publishを担う関数と subscliber callback関数
		void BroadcastImuData();
		rclcpp::Publisher<Imu>::SharedPtr pub_imu_;

		//* ROS publisher subscriber instance
		void CallbackCalibGyro(const Empty::SharedPtr msg);
		rclcpp::Subscription<Empty>::SharedPtr sub_calib_;

		unsigned int id_imu_ = 40;
		string frame_id_ = "base_link";

		unsigned int pub_ratio_ = 0; // 何回に1回publishするか
		bool verbose_callback_ = false; // callback関数のverbose設定
		bool verbose_write_    = false; // serial通信関係のverbose設定
		bool verbose_read_     = false; // serial通信関係のverbose設定
		bool verbose_read_err_ = false; // serial通信関係のverbose設定

		static inline vector<double> angular_velocity_;
		static inline vector<double> linear_acceleration_;
		static inline vector<double> orientation_;

		bool ReadImuData(uint8_t imu_id);
		bool WriteCalibGyro(uint8_t imu_id);
};

#endif /* DYNAMIXEL_IMU_OPEN_CR_H */
