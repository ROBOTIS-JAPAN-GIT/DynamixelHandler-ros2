#include "imu_dxmio.hpp"

#include "myUtils/formatting_output.hpp"
#include "myUtils/make_iterator_convenient.hpp"
#include <cmath>

using namespace std::string_literals;

#define ROS_INFO(...)  RCLCPP_INFO(parent_.get_logger(), __VA_ARGS__)
#define ROS_WARN(...)  RCLCPP_WARN(parent_.get_logger(), __VA_ARGS__)
#define ROS_INFO_STREAM(...)  RCLCPP_INFO_STREAM(parent_.get_logger(), __VA_ARGS__)

// E097B_SMPL1.2/tools/dxl/cui/dxmio3_dxl.c: model 0x4024, packed float32 fields.
using DXMIOAddress = DynamixelAddress;
static DXMIOAddress addr_write_nvm(9, DynamixelDataType::TYPE_UINT8);
static DXMIOAddress addr_terminator(11, DynamixelDataType::TYPE_UINT8);
static DXMIOAddress addr_gx(36, DynamixelDataType::TYPE_UINT32);
static DXMIOAddress addr_gy(40, DynamixelDataType::TYPE_UINT32);
static DXMIOAddress addr_gz(44, DynamixelDataType::TYPE_UINT32);
static DXMIOAddress addr_ax(12, DynamixelDataType::TYPE_UINT32);
static DXMIOAddress addr_ay(16, DynamixelDataType::TYPE_UINT32);
static DXMIOAddress addr_az(20, DynamixelDataType::TYPE_UINT32);
static DXMIOAddress addr_quat_w(66, DynamixelDataType::TYPE_UINT32);
static DXMIOAddress addr_quat_x(54, DynamixelDataType::TYPE_UINT32);
static DXMIOAddress addr_quat_y(58, DynamixelDataType::TYPE_UINT32);
static DXMIOAddress addr_quat_z(62, DynamixelDataType::TYPE_UINT32);
static DXMIOAddress addr_gravity_x(86, DynamixelDataType::TYPE_UINT32);
static DXMIOAddress addr_gravity_y(90, DynamixelDataType::TYPE_UINT32);
static DXMIOAddress addr_gravity_z(94, DynamixelDataType::TYPE_UINT32);
static DXMIOAddress addr_temperature(98, DynamixelDataType::TYPE_INT16);

#include <cstring>
static float int32_bits_to_float(int32_t data) {
	uint32_t bits = static_cast<uint32_t>(data);
	float result = 0.0f;
	std::memcpy(&result, &bits, sizeof(float));
	return result;
}

static array<double, 4> normalize_quat(const array<double, 4>& q) {
	const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	if ( norm < 1.0e-12 ) return {0.0, 0.0, 0.0, 1.0};
	return {q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm};
}

static array<double, 4> multiply_quat(const array<double, 4>& q1, const array<double, 4>& q2) {
	return {
		q1[3] * q2[0] + q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1],
		q1[3] * q2[1] - q1[0] * q2[2] + q1[1] * q2[3] + q1[2] * q2[0],
		q1[3] * q2[2] + q1[0] * q2[1] - q1[1] * q2[0] + q1[2] * q2[3],
		q1[3] * q2[3] - q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2]
	};
}

static array<double, 3> rotate_vector_by_quat(const array<double, 3>& v, const array<double, 4>& q) {
	const array<double, 4> q_vec = {v[0], v[1], v[2], 0.0};
	const array<double, 4> q_conj = {-q[0], -q[1], -q[2], q[3]};
	const array<double, 4> q_rot = multiply_quat(multiply_quat(q, q_vec), q_conj);
	return {q_rot[0], q_rot[1], q_rot[2]};
}

static array<double, 4> convert_rpy_to_quat(double roll_deg, double pitch_deg, double yaw_deg) {
	static constexpr double DEG2RAD = 3.14159265358979323846 / 180.0;
	const double roll = roll_deg * DEG2RAD;
	const double pitch = pitch_deg * DEG2RAD;
	const double yaw = yaw_deg * DEG2RAD;

	const double cr = std::cos(roll * 0.5);
	const double sr = std::sin(roll * 0.5);
	const double cp = std::cos(pitch * 0.5);
	const double sp = std::sin(pitch * 0.5);
	const double cy = std::cos(yaw * 0.5);
	const double sy = std::sin(yaw * 0.5);

	return normalize_quat({
		sr * cp * cy - cr * sp * sy,
		cr * sp * cy + sr * cp * sy,
		cr * cp * sy - sr * sp * cy,
		cr * cp * cy + sr * sp * sy
	});
}

DynamixelHandler::ImuDXMIO::ImuDXMIO(DynamixelHandler& parent) : parent_(parent) {
	ROS_INFO( " < Initializing IMU on DXMIO ............... > ");

	int64_t model_number = 0;
	bool terminator = true;
	vector<double> rpy_adjust_deg = {0.0, 0.0, 0.0};
	parent_.get_parameter_or("option/imu_dxmio.dxmio_id", id_imu_, uint8_t{200});
	parent_.get_parameter_or("option/imu_dxmio.model_number", model_number, int64_t(0x4024));
	parent_.get_parameter_or("option/imu_dxmio.terminator", terminator, true);
	parent_.get_parameter_or("option/imu_dxmio.frame_id", frame_id_, "base_link"s);
	parent_.get_parameter_or("option/imu_dxmio.pub_ratio/data", pub_ratio_data_, 10u);
	parent_.get_parameter_or("option/imu_dxmio.pub_ratio/extra", pub_ratio_extra_, 100u);
	parent_.get_parameter_or("option/imu_dxmio.verbose/read.raw", verbose_read_    , false);
	parent_.get_parameter_or("option/imu_dxmio.verbose/read.err", verbose_read_err_, false);
	parent_.get_parameter_or("option/imu_dxmio.adjust/rpy_deg", rpy_adjust_deg, vector<double>{0.0, 0.0, 0.0});
	parent_.get_parameter_or("option/imu_dxmio.adjust/flip_z_axis", flip_z_axis_, false);
	if ( rpy_adjust_deg.size() != 3 ) rpy_adjust_deg = {0.0, 0.0, 0.0};

	quat_adjust_ = convert_rpy_to_quat(rpy_adjust_deg[0], rpy_adjust_deg[1], rpy_adjust_deg[2]);

	static auto& dyn_comm_ = parent_.dyn_comm_;

	if ( is_in(id_imu_, parent_.id_set_) ){
		ROS_WARN( "  * IMU ID [%d] is duplicated with Dynamixel ID", id_imu_);
		ROS_WARN( " < ... IMU on DXMIO is failed to initialize > ");
		return;
	}
	if ( !dyn_comm_.tryPing(id_imu_) ) {
		ROS_WARN( "  * IMU ID [%d] is not found", id_imu_);
		ROS_WARN( " < ... IMU on DXMIO is failed to initialize > ");
		return;
	}
	auto model_read = dyn_comm_.tryRead(AddrCommon::model_number, id_imu_);
	if ( dyn_comm_.timeout_last_read() || dyn_comm_.comm_error_last_read() ) {
		ROS_WARN("  * IMU ID [%d] failed to read model_number", id_imu_);
		ROS_WARN( " < ... IMU on DXMIO is failed to initialize > ");
		return;
	}
	if ( model_read != model_number ) {
		ROS_WARN("  * IMU ID [%d] is mismatched in model_number (read [%d], expected [%d])",
				 id_imu_, (int)model_read, (int)model_number);
		ROS_WARN( " < ... IMU on DXMIO is failed to initialize > ");
		return;
	}
	ROS_INFO("  * IMU ID [%d] model_number [%d] is found", id_imu_, (int)model_number);
	// Save the node-level baudrate setting together with the terminator setting.
	if ( !dyn_comm_.tryWrite(addr_terminator, id_imu_, terminator) || !dyn_comm_.tryWrite(addr_write_nvm, id_imu_, 1) ) {
		ROS_WARN("  * IMU ID [%d] failed to write/save config", id_imu_);
		return;
	}
	while ( !dyn_comm_.tryPing(id_imu_) ) { if ( !rclcpp::ok() ) return; }
	auto terminator_read = dyn_comm_.tryRead(addr_terminator, id_imu_);
	if ( dyn_comm_.timeout_last_read() || dyn_comm_.comm_error_last_read() || terminator_read != terminator ) {
		ROS_WARN("  * IMU ID [%d] failed to verify config", id_imu_);
		return;
	}
	terminator_ = terminator_read != 0;
	ROS_INFO("    - coord. offset [roll, pitch, yaw] = [%.1f, %.1f, %.1f] deg", rpy_adjust_deg[0], rpy_adjust_deg[1], rpy_adjust_deg[2]);
	ROS_INFO("    - '%s-handed' coordinate system", flip_z_axis_ ? "right" : "left");

	pub_imu_ = parent_.create_publisher<DxmioImu>("dynamixel/dxmio_imu/read", 4);

	is_dxmio_ready_ = true;
	ROS_INFO( " < ............. IMU on DXMIO is initialized > ");
}

DynamixelHandler::ImuDXMIO::~ImuDXMIO(){

}

void DynamixelHandler::ImuDXMIO::MainProcess() {
	if (!is_dxmio_ready_) return;
	static int cnt = -1; cnt++;

	double success_rate = 0;
	if ( pub_ratio_data_ && cnt % pub_ratio_data_ == 0 )
		success_rate += ReadImuData(id_imu_);
	if ( pub_ratio_extra_ && cnt % pub_ratio_extra_ == 0 )
		success_rate += ReadImuExtra(id_imu_);
	if ( success_rate > 0.0 )
		BroadcastImuData();
}

bool DynamixelHandler::ImuDXMIO::ReadImuData(uint8_t imu_id) {
	static auto& dyn_comm_ = parent_.dyn_comm_;
	static const vector<DXMIOAddress> addr_imu_list = {
		addr_gx, addr_gy, addr_gz,
		addr_ax, addr_ay, addr_az,
		addr_quat_w, addr_quat_x, addr_quat_y, addr_quat_z
	};

	auto id_data_map = dyn_comm_.SyncRead(addr_imu_list, {imu_id});
	const bool is_timeout   = dyn_comm_.timeout_last_read();
	const bool has_comm_err = dyn_comm_.comm_error_last_read();

	if ( is_timeout || has_comm_err ) {
		if (verbose_read_err_) ROS_WARN("DXMIO (id=[%d]) failed to read %s", imu_id, is_timeout ? " (time out)" : " (some kind packet error)");
		return false;
	}

	if (verbose_read_) {
		ROS_INFO_STREAM( "DXMIO IMU are read"
			<< control_table_layout(parent_.width_log_, id_data_map, addr_imu_list) );
	}

	const auto& result = id_data_map.at(imu_id);
	// Convert axis convention before applying the mounting rotation.
	const double sign = flip_z_axis_ ? -1.0 : 1.0;
	const array<double, 3> gyro = {
		int32_bits_to_float(static_cast<int32_t>(result[0])) * sign,
		int32_bits_to_float(static_cast<int32_t>(result[1])) * sign,
		int32_bits_to_float(static_cast<int32_t>(result[2]))
	};
	const array<double, 3> acc = {
		int32_bits_to_float(static_cast<int32_t>(result[3])),
		int32_bits_to_float(static_cast<int32_t>(result[4])),
		int32_bits_to_float(static_cast<int32_t>(result[5])) * sign
	};
	gyro_ = rotate_vector_by_quat(gyro, quat_adjust_);
	acc_ = rotate_vector_by_quat(acc, quat_adjust_);

	// DXMIO quaternion is read in [w, x, y, z] order and converted to ROS [x, y, z, w].
	const double quat_w = int32_bits_to_float(static_cast<int32_t>(result[6]));
	const double quat_x = int32_bits_to_float(static_cast<int32_t>(result[7])) * sign;
	const double quat_y = int32_bits_to_float(static_cast<int32_t>(result[8])) * sign;
	const double quat_z = int32_bits_to_float(static_cast<int32_t>(result[9]));
	const array<double, 4> quat = normalize_quat({quat_x, quat_y, quat_z, quat_w});
	quat_ = normalize_quat(multiply_quat(quat, quat_adjust_));
	// Update direction from quaternion; extra reads only update its magnitude.
	const array<double, 4> quat_conj = {-quat[0], -quat[1], -quat[2], quat[3]};
	const auto gravity_dir = rotate_vector_by_quat({0.0, 0.0, sign}, quat_conj);
	gravity_dir_ = rotate_vector_by_quat(gravity_dir, quat_adjust_);

	stamp_imu_ = parent_.get_clock()->now();
	return true;
}

bool DynamixelHandler::ImuDXMIO::ReadImuExtra(uint8_t imu_id) {
	static auto& dyn_comm_ = parent_.dyn_comm_;
	static const vector<DXMIOAddress> addr_imu_list = {
		addr_gravity_x, addr_gravity_y, addr_gravity_z, addr_temperature
	};

	auto id_data_map = dyn_comm_.SyncRead(addr_imu_list, {imu_id});
	const bool is_timeout   = dyn_comm_.timeout_last_read();
	const bool has_comm_err = dyn_comm_.comm_error_last_read();

	if ( is_timeout || has_comm_err ) {
		if (verbose_read_err_) ROS_WARN("DXMIO (id=[%d]) failed to read %s", imu_id, is_timeout ? " (time out)" : " (some kind packet error)");
		return false;
	}

	if (verbose_read_) {
		ROS_INFO_STREAM( "DXMIO IMU extra are read"
			<< control_table_layout(parent_.width_log_, id_data_map, addr_imu_list) );
	}

	const auto& result = id_data_map.at(imu_id);
	const array<double, 3> gravity = {
		int32_bits_to_float(static_cast<int32_t>(result[0])),
		int32_bits_to_float(static_cast<int32_t>(result[1])),
		int32_bits_to_float(static_cast<int32_t>(result[2]))
	};
	gravity_norm_ = std::sqrt(gravity[0] * gravity[0] + gravity[1] * gravity[1] + gravity[2] * gravity[2]);
	// BNO085 raw gyro temperature: 0.5 degC/LSB, zero at 23 degC (as in E097C smpl19).
	temperature_degc_ = 23.0 + 0.5 * result[3];

	return true;
}

void DynamixelHandler::ImuDXMIO::BroadcastImuData() {
	DxmioImu msg;
	msg.stamp = parent_.get_clock()->now();
	msg.id_list.push_back(id_imu_);
	Imu msg_imu;
	msg_imu.header.frame_id = frame_id_;
	msg_imu.header.stamp = stamp_imu_;
	msg_imu.angular_velocity.x = gyro_[0];
	msg_imu.angular_velocity.y = gyro_[1];
	msg_imu.angular_velocity.z = gyro_[2];
	msg_imu.linear_acceleration.x = acc_[0];
	msg_imu.linear_acceleration.y = acc_[1];
	msg_imu.linear_acceleration.z = acc_[2];
	msg_imu.orientation.x = quat_[0];
	msg_imu.orientation.y = quat_[1];
	msg_imu.orientation.z = quat_[2];
	msg_imu.orientation.w = quat_[3];
	msg.data.push_back(msg_imu);
	geometry_msgs::msg::Vector3 msg_gravity;
	msg_gravity.x = gravity_dir_[0] * gravity_norm_;
	msg_gravity.y = gravity_dir_[1] * gravity_norm_;
	msg_gravity.z = gravity_dir_[2] * gravity_norm_;
	msg.gravity.push_back(msg_gravity);
	msg.temperature_degc.push_back(temperature_degc_);
	msg.terminator.push_back(terminator_);
	if ( pub_imu_ && rclcpp::ok() ) pub_imu_->publish(msg);
}
