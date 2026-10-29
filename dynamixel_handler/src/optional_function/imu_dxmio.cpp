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
	bool terminator = true;
	vector<double> rpy_adjust_deg = {0.0, 0.0, 0.0};
	parent_.get_parameter_or("option/imu_dxmio.dxmio_id", id_imu_, uint8_t{200});
	parent_.get_parameter_or("option/imu_dxmio.model_number", model_number, int64_t(0x4024));
	parent_.get_parameter_or("option/imu_dxmio.terminator", terminator, true);
	parent_.get_parameter_or("option/imu_dxmio.frame_id", frame_id_, "base_link"s);
	parent_.get_parameter_or("option/imu_dxmio.pub_ratio/data", pub_ratio_["data"], 10u);
	parent_.get_parameter_or("option/imu_dxmio.pub_ratio/extra", pub_ratio_["extra"], 100u);
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
	// Save the node-level baudrate setting together with the terminator setting.
	if ( !dyn_comm_.tryWrite(addr_terminator, id_imu_, terminator) || !dyn_comm_.tryWrite(addr_write_nvm, id_imu_, 1) ) {
		ROS_WARN("  * IMU ID [%d] failed to write/save config", id_imu_);
		return;
	}
	while ( !dyn_comm_.tryPing(id_imu_) ) { if ( !rclcpp::ok() ) return; }
	auto current = dyn_comm_.tryRead(addr_terminator, id_imu_);
	if ( dyn_comm_.timeout_last_read() || dyn_comm_.comm_error_last_read() || current != terminator ) {
		ROS_WARN("  * IMU ID [%d] failed to verify config", id_imu_);
		return;
	}
	terminator_ = current != 0;
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

	set<string> items_read;
	for ( const auto& [item, ratio] : pub_ratio_ )
		if ( ratio && cnt % ratio == 0 ) items_read.insert(item);
	if ( !items_read.empty() && ReadImuData(id_imu_, items_read) )
		BroadcastImuData();
}

bool DynamixelHandler::ImuDXMIO::ReadImuData(uint8_t imu_id, const set<string>& items_read) {
	static auto& dyn_comm_ = parent_.dyn_comm_;
	const bool read_data = is_in("data"s, items_read);
	const bool read_extra = is_in("extra"s, items_read);
	// SyncRead can read the 58-byte span for these 10 fields in one response.
	vector<vector<DXMIOAddress>> addr_imu_lists(2);
	if ( read_data ) addr_imu_lists[0] = {addr_ax, addr_ay, addr_az, addr_gx, addr_gy, addr_gz, addr_quat_w, addr_quat_x, addr_quat_y, addr_quat_z};
	if ( read_extra ) addr_imu_lists[1] = {addr_gravity_x, addr_gravity_y, addr_gravity_z, addr_temperature};

	map<uint16_t, int64_t> result;
	for ( const auto& addr_imu_list : addr_imu_lists ) {
		if ( addr_imu_list.empty() ) continue;
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
		const auto& data = id_data_map.at(imu_id);
		for ( size_t i = 0; i < data.size(); i++ ) result[addr_imu_list[i].address()] = data[i];
	}

	const double _sign = flip_z_axis_ ? -1.0 : 1.0;
	const auto stamp = parent_.get_clock()->now();
	if ( read_data ) {
		const array<double, 3> angular_velocity_handedness = {
			int32_bits_to_float(static_cast<int32_t>(result[addr_gx.address()])) * _sign,
			int32_bits_to_float(static_cast<int32_t>(result[addr_gy.address()])) * _sign,
			int32_bits_to_float(static_cast<int32_t>(result[addr_gz.address()]))
		};
		const array<double, 3> linear_acceleration_handedness = {
			int32_bits_to_float(static_cast<int32_t>(result[addr_ax.address()])),
			int32_bits_to_float(static_cast<int32_t>(result[addr_ay.address()])),
			int32_bits_to_float(static_cast<int32_t>(result[addr_az.address()])) * _sign
		};
		angular_velocity_ = rotate_vector_by_quat(angular_velocity_handedness, quat_adjust_);
		linear_acceleration_ = rotate_vector_by_quat(linear_acceleration_handedness, quat_adjust_);

		// DXMIO quaternion is read in [w, x, y, z] order and converted to ROS [x, y, z, w].
		const double q0_w = int32_bits_to_float(static_cast<int32_t>(result[addr_quat_w.address()]));
		const double q1_x = int32_bits_to_float(static_cast<int32_t>(result[addr_quat_x.address()])) * _sign;
		const double q2_y = int32_bits_to_float(static_cast<int32_t>(result[addr_quat_y.address()])) * _sign;
		const double q3_z = int32_bits_to_float(static_cast<int32_t>(result[addr_quat_z.address()]));
		const array<double, 4> orientation_handedness = normalize_quat({q1_x, q2_y, q3_z, q0_w});
		orientation_ = normalize_quat(multiply_quat(orientation_handedness, quat_adjust_));
		// Update direction from quaternion; extra reads only update its magnitude.
		const array<double, 4> quat_conj = {-orientation_handedness[0], -orientation_handedness[1], -orientation_handedness[2], orientation_handedness[3]};
		const auto gravity_handedness = rotate_vector_by_quat({0.0, 0.0, _sign}, quat_conj);
		gravity_direction_ = rotate_vector_by_quat(gravity_handedness, quat_adjust_);

		stamp_imu_ = stamp;
	}
	if ( read_extra ) {
		const array<double, 3> gravity = {
			int32_bits_to_float(static_cast<int32_t>(result[addr_gravity_x.address()])),
			int32_bits_to_float(static_cast<int32_t>(result[addr_gravity_y.address()])),
			int32_bits_to_float(static_cast<int32_t>(result[addr_gravity_z.address()]))
		};
		gravity_norm_ = std::sqrt(gravity[0] * gravity[0] + gravity[1] * gravity[1] + gravity[2] * gravity[2]);
		// BNO085 raw gyro temperature: 0.5 degC/LSB, zero at 23 degC (as in E097C smpl19).
		temperature_degc_ = 23.0 + 0.5 * result[addr_temperature.address()];
	}

	return true;
}

void DynamixelHandler::ImuDXMIO::BroadcastImuData() {
	DxmioImu msg;
	msg.stamp = parent_.get_clock()->now();
	msg.id_list.push_back(id_imu_);
	Imu msg_imu;
	msg_imu.header.frame_id = frame_id_;
	msg_imu.header.stamp = stamp_imu_;
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
	msg.data.push_back(msg_imu);
	geometry_msgs::msg::Vector3 msg_gravity;
	msg_gravity.x = gravity_direction_[0] * gravity_norm_;
	msg_gravity.y = gravity_direction_[1] * gravity_norm_;
	msg_gravity.z = gravity_direction_[2] * gravity_norm_;
	msg.gravity.push_back(msg_gravity);
	msg.temperature_degc.push_back(temperature_degc_);
	msg.terminator.push_back(terminator_);
	if ( pub_imu_ && rclcpp::ok() ) pub_imu_->publish(msg);
}
