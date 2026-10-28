#ifndef DYNAMIXEL_IMU_DXMIO_H
#define DYNAMIXEL_IMU_DXMIO_H

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/imu.hpp"
using namespace sensor_msgs::msg;

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
		rclcpp::Publisher<Imu>::SharedPtr pub_imu_;

		uint8_t id_imu_ = 200;
		string frame_id_ = "base_link";
		bool is_dxmio_ready_ = false;
		array<double, 4> quat_adjust_ = {0.0, 0.0, 0.0, 1.0};
		bool flip_z_axis_ = false;

		unsigned int pub_ratio_ = 0;
		bool verbose_read_     = false;
		bool verbose_read_err_ = false;

		static inline array<double, 3> angular_velocity_    = {0.0, 0.0, 0.0};
		static inline array<double, 3> linear_acceleration_ = {0.0, 0.0, 0.0};
		static inline array<double, 4> orientation_         = {0.0, 0.0, 0.0, 1.0};

		bool ReadImuData(uint8_t imu_id);
};

#endif /* DYNAMIXEL_IMU_DXMIO_H */
