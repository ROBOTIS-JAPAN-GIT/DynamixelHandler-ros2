#ifndef DYNAMIXEL_IMU_DXMIO_H
#define DYNAMIXEL_IMU_DXMIO_H

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/imu.hpp"
using namespace sensor_msgs::msg;

#include "dynamixel_handler_msgs/msg/dxmio_imu.hpp"
using namespace dynamixel_handler_msgs::msg;

#include "../dynamixel_handler.hpp"

#include <string>
using std::string;
#include <vector>
using std::vector;
#include <array>
using std::array;

class DynamixelHandler::ImuDXMIO {
	public:
		ImuDXMIO(DynamixelHandler& parent);
		~ImuDXMIO();
		void MainProcess();

		DynamixelHandler& parent_;

		void BroadcastImuData();
		rclcpp::Publisher<DxmioImu>::SharedPtr pub_imu_;

		uint8_t id_imu_ = 200;
		string frame_id_ = "base_link";
		bool is_dxmio_ready_ = false;
		array<double, 4> quat_adjust_ = {0.0, 0.0, 0.0, 1.0};
		bool flip_z_axis_ = false;

		unsigned int pub_ratio_data_ = 0;
		unsigned int pub_ratio_extra_ = 0;
		bool verbose_read_     = false;
		bool verbose_read_err_ = false;

		static inline array<double, 3> gyro_        = {0.0, 0.0, 0.0};
		static inline array<double, 3> acc_         = {0.0, 0.0, 0.0};
		static inline array<double, 4> quat_        = {0.0, 0.0, 0.0, 1.0};
		static inline array<double, 3> gravity_dir_ = {0.0, 0.0, 0.0};
		static inline double gravity_norm_ = 0.0;
		static inline double temperature_degc_ = 0.0;
		static inline bool terminator_ = false;
		static inline builtin_interfaces::msg::Time stamp_imu_{};

		bool ReadImuData(uint8_t imu_id);
		bool ReadImuExtra(uint8_t imu_id);
};

#endif /* DYNAMIXEL_IMU_DXMIO_H */
