#include "imu_opencr.hpp"

#include "myUtils/formatting_output.hpp"
#include "myUtils/make_iterator_convenient.hpp"

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
static constexpr double res_acc = 8.0 / 32768.0;      // 8g
static constexpr double res_gyro = 2000.0 / 32768.0;  // 2000dps

DynamixelHandler::ImuOpenCR::ImuOpenCR(DynamixelHandler& parent) : parent_(parent) {
	ROS_INFO( " < Initializing IMU on OpenCR ...   > ");
	
	parent_.get_parameter_or("option/imu_opencr.opencr_id", id_imu_, 40u);
	parent_.get_parameter_or("option/imu_opencr.frame_id", frame_id_, "base_link"s);
	parent_.get_parameter_or("option/imu_opencr.pub_ratio", pub_ratio_, 10u);
	parent_.get_parameter_or("option/imu_opencr.verbose/callback", verbose_callback_, false);
	parent_.get_parameter_or("option/imu_opencr.verbose/write"   , verbose_write_   , false);
	parent_.get_parameter_or("option/imu_opencr.verbose/read/raw", verbose_read_    , false);
	parent_.get_parameter_or("option/imu_opencr.verbose/read/err", verbose_read_err_, false);

	pub_imu_   = parent_.create_publisher<Imu>("dynamixel/imu/raw", 4);
	sub_calib_ = parent_.create_subscription<Empty>("dynamixel/imu/calibration_gyro", 10, bind(&DynamixelHandler::ImuOpenCR::CallbackCalibGyro, this, _1));

	ROS_INFO( " < ... IMU on OpenCR is initialized > ");
}

DynamixelHandler::ImuOpenCR::~ImuOpenCR(){ // デストラクタ,  終了処理を行う

}

void DynamixelHandler::ImuOpenCR::MainProcess() {
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
 	return dyn_comm_.tryWrite(addr_calib, id_imu_, 1);
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
	int32_t quat[4] = {
		(int32_t)result[6],
		(int32_t)result[7],
		(int32_t)result[8],
		(int32_t)result[9]
	};
	float* quatF = (float*)quat;
	orientation_ = { quatF[0], quatF[1], quatF[2], quatF[3] };
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
	if ( !WriteCalibGyro(id_imu_) ) {
		ROS_ERROR("   Failed to send calibration command to IMU on OpenCR (id=%d)", id_imu_);
	}

	if ( verbose_callback_ ) ROS_INFO("   Calibrating Gyro (id=%d) ... ", id_imu_);
	rclcpp::sleep_for(5000ms);
	if ( verbose_callback_ ) ROS_INFO("   ... Finished Calibration Gyro");
}
