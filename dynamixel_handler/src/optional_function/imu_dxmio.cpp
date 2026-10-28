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
	const array<double, 4> q_rotated = multiply_quat(multiply_quat(q, q_vec), q_conj);
	return {q_rotated[0], q_rotated[1], q_rotated[2]};
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
	vector<double> rpy_adjust_deg = {0.0, 0.0, 0.0};
	parent_.get_parameter_or("option/imu_dxmio.dxmio_id", id_imu_, uint8_t{200});
	parent_.get_parameter_or("option/imu_dxmio.model_number", model_number, int64_t(0x4024));
	parent_.get_parameter_or("option/imu_dxmio.frame_id", frame_id_, "base_link"s);
	parent_.get_parameter_or("option/imu_dxmio.pub_ratio", pub_ratio_, 10u);
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
	auto dxmio_num = dyn_comm_.tryRead(AddrCommon::model_number, id_imu_);
	if ( dyn_comm_.timeout_last_read() || dyn_comm_.comm_error_last_read() ) {
		ROS_WARN("  * IMU ID [%d] failed to read model_number", id_imu_);
		ROS_WARN( " < ... IMU on DXMIO is failed to initialize > ");
		return;
	}
	if ( dxmio_num != model_number ) {
		ROS_WARN("  * IMU ID [%d] is mismatched in model_number (read [%d], expected [%d])",
				 id_imu_, (int)dxmio_num, (int)model_number);
		ROS_WARN( " < ... IMU on DXMIO is failed to initialize > ");
		return;
	}
	ROS_INFO("  * IMU ID [%d] model_number [%d] is found", id_imu_, (int)model_number);
	ROS_INFO("    - coord. offset [roll, pitch, yaw] = [%.1f, %.1f, %.1f] deg", rpy_adjust_deg[0], rpy_adjust_deg[1], rpy_adjust_deg[2]);
	ROS_INFO("    - '%s-handed' coordinate system", flip_z_axis_ ? "right" : "left");

	pub_imu_ = parent_.create_publisher<Imu>("dynamixel/imu/raw", 4);

	is_dxmio_ready_ = true;
	ROS_INFO( " < ............. IMU on DXMIO is initialized > ");
}

DynamixelHandler::ImuDXMIO::~ImuDXMIO(){

}

void DynamixelHandler::ImuDXMIO::MainProcess() {
	if (!is_dxmio_ready_) return;
	static int cnt = -1; cnt++;

	double success_rate = 0;
	if ( pub_ratio_ && cnt % pub_ratio_ == 0 )
		success_rate += ReadImuData(id_imu_);
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

	auto result = dyn_comm_.Read(addr_imu_list, imu_id);
	const bool is_timeout   = dyn_comm_.timeout_last_read();
	const bool has_comm_err = dyn_comm_.comm_error_last_read();

	if ( is_timeout || has_comm_err ) {
		if (verbose_read_err_) ROS_WARN("DXMIO (id=[%d]) failed to read %s", imu_id, is_timeout ? " (time out)" : " (some kind packet error)");
		return false;
	}

	if (verbose_read_) {
		const map<uint8_t, vector<int64_t>> id_data_map = {{imu_id, result}};
		ROS_INFO_STREAM( "DXMIO IMU are read"
			<< control_table_layout(parent_.width_log_, id_data_map, addr_imu_list) );
	}

	const double _sign = flip_z_axis_ ? -1.0 : 1.0;
	const array<double, 3> angular_velocity_handedness = {
		int32_bits_to_float(static_cast<int32_t>(result[0])) * _sign,
		int32_bits_to_float(static_cast<int32_t>(result[1])) * _sign,
		int32_bits_to_float(static_cast<int32_t>(result[2]))
	};
	const array<double, 3> linear_acceleration_handedness = {
		int32_bits_to_float(static_cast<int32_t>(result[3])),
		int32_bits_to_float(static_cast<int32_t>(result[4])),
		int32_bits_to_float(static_cast<int32_t>(result[5])) * _sign
	};
	angular_velocity_ = rotate_vector_by_quat(angular_velocity_handedness, quat_adjust_);
	linear_acceleration_ = rotate_vector_by_quat(linear_acceleration_handedness, quat_adjust_);

	// DXMIO quaternion is read in [w, x, y, z] order and converted to ROS [x, y, z, w].
	const double q0_w = int32_bits_to_float(static_cast<int32_t>(result[6]));
	const double q1_x = int32_bits_to_float(static_cast<int32_t>(result[7])) * _sign;
	const double q2_y = int32_bits_to_float(static_cast<int32_t>(result[8])) * _sign;
	const double q3_z = int32_bits_to_float(static_cast<int32_t>(result[9]));
	const array<double, 4> orientation_handedness = normalize_quat({q1_x, q2_y, q3_z, q0_w});
	orientation_ = normalize_quat(multiply_quat(orientation_handedness, quat_adjust_));

	return true;
}

void DynamixelHandler::ImuDXMIO::BroadcastImuData() {
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
	if ( pub_imu_ && rclcpp::ok() ) pub_imu_->publish(msg_imu);
}
