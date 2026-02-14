#include "imu_opencr.hpp"

#include "myUtils/formatting_output.hpp"
#include "myUtils/make_iterator_convenient.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

using namespace std::string_literals;

#define ROS_INFO(...)  RCLCPP_INFO(parent_.get_logger(), __VA_ARGS__)
#define ROS_WARN(...)  RCLCPP_WARN(parent_.get_logger(), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(parent_.get_logger(), __VA_ARGS__)
#define ROS_INFO_STREAM(...)  RCLCPP_INFO_STREAM(parent_.get_logger(), __VA_ARGS__)
#define ROS_WARN_STREAM(...)  RCLCPP_WARN_STREAM(parent_.get_logger(), __VA_ARGS__)
#define ROS_ERROR_STREAM(...) RCLCPP_ERROR_STREAM(parent_.get_logger(), __VA_ARGS__)
#define ROS_STOP(...) \
    {{ RCLCPP_FATAL(parent_.get_logger(), __VA_ARGS__); rclcpp::shutdown(); std::exit(EXIT_FAILURE); }}

DynamixelAddress addr_calib(64, DynamixelDataType::TYPE_UINT8);
DynamixelAddress addr_gx(76, DynamixelDataType::TYPE_INT16);
DynamixelAddress addr_gy(78, DynamixelDataType::TYPE_INT16);
DynamixelAddress addr_gz(80, DynamixelDataType::TYPE_INT16);
DynamixelAddress addr_ax(82, DynamixelDataType::TYPE_INT16);
DynamixelAddress addr_ay(84, DynamixelDataType::TYPE_INT16);
DynamixelAddress addr_az(86, DynamixelDataType::TYPE_INT16);
DynamixelAddress addr_quat_x(88, DynamixelDataType::TYPE_INT32);
DynamixelAddress addr_quat_y(92, DynamixelDataType::TYPE_INT32);
DynamixelAddress addr_quat_z(96, DynamixelDataType::TYPE_INT32);
DynamixelAddress addr_quat_w(100, DynamixelDataType::TYPE_INT32);
static constexpr double res_acc = 2.0 / 32768.0 * 9.8;      // 2g [m/s^2]
static constexpr double res_gyro = 2000.0 / 32768.0 * 3.14159/180;  // 2000dps [rad/s]

#include <cstring>
float int32_bits_to_float(int32_t data) {
	uint32_t bits = static_cast<uint32_t>(data);
	float result = 0.0f;
	std::memcpy(&result, &bits, sizeof(float));
	return result;
}

DynamixelHandler::ImuOpenCR::ImuOpenCR(DynamixelHandler& parent) : parent_(parent) {
	ROS_INFO( " < Initializing IMU on OpenCR .............. > ");
	
	id_imu_ = static_cast<uint8_t>(parent_.get_parameter_or("option/imu_opencr.opencr_id", 40));
	model_number_ = parent_.get_parameter_or("option/imu_opencr.model_number", int64_t(0));
	parent_.get_parameter_or("option/imu_opencr.frame_id", frame_id_, "base_link"s);
	parent_.get_parameter_or("option/imu_opencr.pub_ratio", pub_ratio_, 10u);
	parent_.get_parameter_or("option/imu_opencr.verbose/callback", verbose_callback_, false);
	parent_.get_parameter_or("option/imu_opencr.verbose/write"   , verbose_write_   , false);
	parent_.get_parameter_or("option/imu_opencr.verbose/read.raw", verbose_read_    , false);
	parent_.get_parameter_or("option/imu_opencr.verbose/read.err", verbose_read_err_, false);
	static auto& dyn_comm_ = parent_.dyn_comm_;

	if ( !dyn_comm_.tryPing(id_imu_) ) {
		ROS_INFO( "  * OpenCR IMU ID [%d] is not found", id_imu_);
		ROS_WARN( " < ... IMU on OpenCR is failed to initialize > ");
		return;
	}
	auto oepncr_num = dyn_comm_.tryRead(AddrCommon::model_number, id_imu_);
	if ( oepncr_num != model_number_ ) {
		ROS_INFO("  * OpenCR IMU ID [%d] model_number [%d] is ignored (expected [%d])", id_imu_, (int)oepncr_num, (int)model_number_);
		ROS_WARN( " < ... IMU on OpenCR is failed to initialize > ");
		return;
	}
	ROS_INFO("  * OpenCR IMU ID [%d] model_number [%d] is found", id_imu_, (int)model_number_);
	
	pub_imu_ = parent_.create_publisher<Imu>("dynamixel/imu/raw", 4);
	sub_calib_ = parent_.create_generic_subscription(
		"dynamixel/imu/calibration_gyro", "std_msgs/msg/Empty", rclcpp::QoS(10),
		[this](std::shared_ptr<rclcpp::SerializedMessage> serialized_msg) {
			Empty msg;
			rclcpp::Serialization<Empty> serializer;
			serializer.deserialize_message(serialized_msg.get(), &msg);
			CallbackCalibGyro(std::make_shared<Empty>(msg));
		}
	);

	is_opencr_ready_ = true;
	ROS_INFO( " < ............ IMU on OpenCR is initialized > ");
}

DynamixelHandler::ImuOpenCR::~ImuOpenCR(){ // デストラクタ,  終了処理を行う

}

void DynamixelHandler::ImuOpenCR::MainProcess() {
	if (!is_opencr_ready_) return;
	static int cnt = -1; cnt++;

	double success_rate = 0;
	if ( pub_ratio_ && cnt % pub_ratio_ == 0 )
		success_rate += ReadImuData(id_imu_);
	if ( success_rate > 0.0 )
		BroadcastImuData();
}

bool DynamixelHandler::ImuOpenCR::WriteCalibGyro(uint8_t imu_id) {
	static auto& dyn_comm_ = parent_.dyn_comm_;
	if (verbose_write_) ROS_INFO("   Sending calibration command to IMU on OpenCR (id=%d) ... ", imu_id);
 	return dyn_comm_.tryWrite(addr_calib, imu_id, 1);
}

bool DynamixelHandler::ImuOpenCR::ReadImuData(uint8_t imu_id) {
	static auto& dyn_comm_ = parent_.dyn_comm_;
	static const vector<DynamixelAddress> addr_imu_list = {
		addr_gx, addr_gy, addr_gz,
		addr_ax, addr_ay, addr_az,
		addr_quat_x, addr_quat_y, addr_quat_z, addr_quat_w
	};

	auto result = dyn_comm_.Read(addr_imu_list, imu_id);
	const bool is_timeout   = dyn_comm_.timeout_last_read();
	const bool has_comm_err = dyn_comm_.comm_error_last_read();

	//* 通信エラーの処理
	if ( is_timeout || has_comm_err ) {
		if (verbose_read_err_) ROS_WARN("OpenCR (id=[%d]) failed to read %s", imu_id, is_timeout ? " (time out)" : " (some kind packet error)");
		return false;
	}

	//* resultの内容を確認
	if (verbose_read_) {
		map<uint8_t, vector<int64_t>> id_data_map;
		id_data_map[imu_id] = result;
		char header[99]; sprintf(header, "OpenCR (id=[%d]) id read", imu_id);
		auto ss = control_table_layout(2, id_data_map, addr_imu_list, string(header));
		ROS_INFO_STREAM(ss);
	};

	//* 読み取ったデータを反映
	angular_velocity_ = {
		(double)result[0] * res_gyro, 
		(double)result[1] * res_gyro,
		(double)result[2] * res_gyro
	};
	linear_acceleration_ = {
		(double)result[3] * res_acc,
		(double)result[4] * res_acc,
		(double)result[5] * res_acc
	};
	orientation_ = {
		int32_bits_to_float(static_cast<int32_t>(result[6])),
		int32_bits_to_float(static_cast<int32_t>(result[7])),
		int32_bits_to_float(static_cast<int32_t>(result[8])),
		int32_bits_to_float(static_cast<int32_t>(result[9]))
	};
	return true;
}

void DynamixelHandler::ImuOpenCR::BroadcastImuData() {
	Imu msg_imu;
	msg_imu.header.frame_id = frame_id_;
	msg_imu.header.stamp = parent_.get_clock()->now();
	msg_imu.angular_velocity.x = angular_velocity_[0];
	msg_imu.angular_velocity.y = angular_velocity_[1];
	msg_imu.angular_velocity.z = angular_velocity_[2];
	msg_imu.linear_acceleration.x = linear_acceleration_[0];
	msg_imu.linear_acceleration.y = linear_acceleration_[1];
	msg_imu.linear_acceleration.z = linear_acceleration_[2];
	msg_imu.orientation.x = orientation_[0];
	msg_imu.orientation.y = orientation_[1];
	msg_imu.orientation.z = orientation_[2];
	msg_imu.orientation.w = orientation_[3];
	pub_imu_->publish(msg_imu);
}

void DynamixelHandler::ImuOpenCR::CallbackCalibGyro(const std_msgs::msg::Empty::SharedPtr msg) {
	(void)msg; // warｎing 抑制
	if ( !WriteCalibGyro(id_imu_) )
		ROS_ERROR("   Failed to send calibration command to IMU on OpenCR (id=%d)", id_imu_);

	if ( verbose_callback_ ) ROS_INFO("   Calibrating Gyro ............");
	rclcpp::sleep_for(5000ms);
	if ( verbose_callback_ ) ROS_INFO("   ... Finished Calibration Gyro");
}
