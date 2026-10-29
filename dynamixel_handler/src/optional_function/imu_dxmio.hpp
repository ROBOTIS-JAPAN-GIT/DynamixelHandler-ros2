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
#include <unordered_map>
using std::unordered_map;
#include <unordered_set>
using std::unordered_set;

using std::bind;
using std::placeholders::_1;

class DynamixelHandler::ImuDXMIO {
	public:
		ImuDXMIO(DynamixelHandler& parent);
		~ImuDXMIO();
		void MainProcess();

		DynamixelHandler& parent_;

		void BroadcastImuData();
		rclcpp::Publisher<DxmioImu>::SharedPtr pub_imu_;
		void CallbackImu(const DxmioImu::SharedPtr msg);
		rclcpp::Subscription<DxmioImu>::SharedPtr sub_imu_;

		using id_t = uint8_t;
		using vec3d = array<double, 3>;
		vector<id_t> id_imu_list_;
		bool is_dxmio_ready_ = false;
		unordered_map<id_t, string> frame_id_;
		unordered_map<id_t, bool> flip_z_axis_;
		unordered_map<id_t, vec3d> rpy_adjust_; // [roll, pitch, yaw] [deg]
		unordered_map<id_t, bool> terminator_w_;
		unordered_set<id_t> updated_id_terminator_;

		unsigned int pub_ratio_motion_ = 0;
		unsigned int pub_ratio_extra_ = 0;
		bool verbose_callback_ = false;
		bool verbose_write_    = false;
		bool verbose_read_     = false;
		bool verbose_read_err_ = false;

		// Measurements after coordinate adjustment.
		static inline unordered_map<id_t, vec3d> gyro_, acc_, gravity_dir_;
		static inline unordered_map<id_t, array<double, 4>> quat_;
		static inline unordered_map<id_t, double> gravity_norm_, temperature_degc_;
		static inline unordered_map<id_t, bool> terminator_r_;
		static inline unordered_map<id_t, builtin_interfaces::msg::Time> stamp_imu_;

		bool ReadImuMotion(const vector<id_t>& id_list);
		bool ReadImuExtra(const vector<id_t>& id_list);
		bool SyncWriteTerminator(const unordered_set<id_t>& id_set);
		bool SyncReadTerminator(const unordered_set<id_t>& id_set);
};

#endif /* DYNAMIXEL_IMU_DXMIO_H */
