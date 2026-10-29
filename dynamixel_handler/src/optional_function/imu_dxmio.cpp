#include "imu_dxmio.hpp"

#include "myUtils/formatting_output.hpp"
#include "myUtils/make_iterator_convenient.hpp"
#include <cmath>

using namespace std::string_literals;

#define ROS_INFO(...)  RCLCPP_INFO(parent_.get_logger(), __VA_ARGS__)
#define ROS_WARN(...)  RCLCPP_WARN(parent_.get_logger(), __VA_ARGS__)
#define ROS_INFO_STREAM(...)  RCLCPP_INFO_STREAM(parent_.get_logger(), __VA_ARGS__)

using StateLock = std::lock_guard<std::recursive_mutex>;

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
	parent_.get_parameter_or("option/imu_dxmio.pub_ratio/motion", pub_ratio_motion_, 10u);
	parent_.get_parameter_or("option/imu_dxmio.pub_ratio/extra", pub_ratio_extra_, 100u);
	parent_.get_parameter_or("option/imu_dxmio.model_number", model_number, int64_t(0x4024));
	parent_.get_parameter_or("option/imu_dxmio.verbose/callback", verbose_callback_, false);
	parent_.get_parameter_or("option/imu_dxmio.verbose/write"   , verbose_write_   , false);
	parent_.get_parameter_or("option/imu_dxmio.verbose/read.raw", verbose_read_    , false);
	parent_.get_parameter_or("option/imu_dxmio.verbose/read.err", verbose_read_err_, false);

	static auto& dyn_comm_ = parent_.dyn_comm_;

	for ( auto id : dyn_comm_.Ping_broadcast() ) {
		if ( is_in(id, parent_.id_set_) || is_in(id, id_imu_list_) ) continue;
		auto model_read = dyn_comm_.tryRead(AddrCommon::model_number, id);
		if ( dyn_comm_.timeout_last_read() || dyn_comm_.comm_error_last_read() ) {
			ROS_WARN("  * IMU ID [%d] failed to read model_number", id);
			continue;
		}
		if ( model_read != model_number ) continue;
		ROS_INFO("  * IMU ID [%d] model_number [%d] is found", id, (int)model_number);
		// Save the node-level baudrate setting without changing the terminator.
		if ( !dyn_comm_.tryWrite(addr_write_nvm, id, 1) ) {
			ROS_WARN("  * IMU ID [%d] failed to save config", id);
			continue;
		}
		while ( !dyn_comm_.tryPing(id) ) { if ( !rclcpp::ok() ) return; }
		auto terminator_read = dyn_comm_.tryRead(addr_terminator, id);
		if ( dyn_comm_.timeout_last_read() || dyn_comm_.comm_error_last_read() ) {
			ROS_WARN("  * IMU ID [%d] failed to read config", id);
			continue;
		}
		id_imu_list_.push_back(id);
		frame_id_[id] = "dxmio_imu_" + std::to_string(id);
		quat_[id] = {0.0, 0.0, 0.0, 1.0};
		terminator_r_[id] = terminator_read != 0;
	}
	if ( id_imu_list_.empty() ) {
		ROS_WARN( " < ... IMU on DXMIO is failed to initialize  > ");
		return;
	}

	pub_imu_ = parent_.create_publisher<DxmioImu>("dynamixel/dxmio_imu/read", 4);
	rclcpp::SubscriptionOptions sub_options;
	sub_options.callback_group = parent_.cbg_command_;
	sub_imu_ = parent_.create_subscription<DxmioImu>("dynamixel/dxmio_imu/write", 4, bind(&ImuDXMIO::CallbackImu, this, _1), sub_options);

	is_dxmio_ready_ = true;
	ROS_INFO( " < ............ IMU on DXMIO is initialized  > ");
}

DynamixelHandler::ImuDXMIO::~ImuDXMIO(){

}

void DynamixelHandler::ImuDXMIO::CallbackImu(const DxmioImu::SharedPtr msg) {
	StateLock lock(parent_.mutex_state_);
	vector<id_t> valid_id_list;
	for ( auto id : msg->id_list ) if ( is_in(id, id_imu_list_) ) valid_id_list.push_back(id);
	if ( valid_id_list.empty() ) return;

	const bool has_frame = msg->frame_id.size() == msg->id_list.size();
	const bool has_flip  = msg->flip_z_axis.size() == msg->id_list.size();
	const bool has_rpy   = msg->adjust_rpy_deg.size() == msg->id_list.size();
	const bool has_terminator = msg->terminator.size() == msg->id_list.size();
	if ( verbose_callback_ ) {
		ROS_INFO("DXMIO IMU, '%zu' ID(s) are tryed to updated", valid_id_list.size());
		ROS_INFO_STREAM(id_list_layout(valid_id_list, "  ID"));
		if ( has_frame ) ROS_INFO("  - updated: frame_id"       ); else if ( !msg->frame_id.empty() ) ROS_WARN("  - skipped: frame_id (size mismatch)");
		if ( has_flip  ) ROS_INFO("  - updated: flip_z_axis"    ); else if ( !msg->flip_z_axis.empty() ) ROS_WARN("  - skipped: flip_z_axis (size mismatch)");
		if ( has_rpy   ) ROS_INFO("  - updated: adjust_rpy_deg" ); else if ( !msg->adjust_rpy_deg.empty() ) ROS_WARN("  - skipped: adjust_rpy_deg (size mismatch)");
		if ( has_terminator ) ROS_INFO("  - updated: terminator"); else if ( !msg->terminator.empty() ) ROS_WARN("  - skipped: terminator (size mismatch)");
	}
	for ( size_t i = 0; i < msg->id_list.size(); i++ ) {
		auto id = msg->id_list[i];
		if ( !is_in(id, id_imu_list_) ) continue;
		if ( has_frame ) frame_id_[id] = msg->frame_id[i];
		if ( has_flip ) flip_z_axis_[id] = msg->flip_z_axis[i];
		if ( has_rpy ) {
			const auto& rpy = msg->adjust_rpy_deg[i];
			rpy_adjust_[id] = {rpy.x, rpy.y, rpy.z};
		}
		if ( has_terminator ) {
			terminator_w_[id] = msg->terminator[i];
			updated_id_terminator_.insert(id);
		}
	}
	if ( verbose_callback_ ) ROS_INFO("------------------+------------------");
}

void DynamixelHandler::ImuDXMIO::MainProcess() {
	if (!is_dxmio_ready_) return;
	static int cnt = -1; cnt++;

	bool updated = false;
	{	StateLock lock(parent_.mutex_state_);
		updated |= SyncWriteTerminator(updated_id_terminator_);
		updated_id_terminator_.clear();
	}
	if ( pub_ratio_motion_ && cnt % pub_ratio_motion_ == 0 )
		updated |= ReadImuMotion(id_imu_list_);
	if ( pub_ratio_extra_ && cnt % pub_ratio_extra_ == 0 )
		updated |= ReadImuExtra(id_imu_list_);
	if ( updated )
		BroadcastImuData();
}

bool DynamixelHandler::ImuDXMIO::SyncWriteTerminator(const unordered_set<id_t>& id_set) {
	static auto& dyn_comm_ = parent_.dyn_comm_;
	map<id_t, vector<int64_t>> id_data_map;
	for ( auto id : id_set ) id_data_map[id] = {terminator_w_[id]};
	if ( id_data_map.empty() ) return false;
	parent_.SyncWrite_log({addr_terminator}, id_data_map, verbose_write_);
	for ( auto id : id_set ) {
		if ( !dyn_comm_.tryWrite(addr_write_nvm, id, 1) ) {
			ROS_WARN("DXMIO (id=[%d]) failed to save config", id);
			continue;
		}
		while ( !dyn_comm_.tryPing(id) ) { if ( !rclcpp::ok() ) return false; }
	}
	return SyncReadTerminator(id_set);
}

bool DynamixelHandler::ImuDXMIO::SyncReadTerminator(const unordered_set<id_t>& id_set) {
	static auto& dyn_comm_ = parent_.dyn_comm_;
	static const vector<DXMIOAddress> addr_config_list = {addr_terminator};
	const vector<id_t> id_list(id_set.begin(), id_set.end());
	auto id_data_map = dyn_comm_.SyncRead(addr_config_list, id_list);
	if ( dyn_comm_.timeout_last_read() || dyn_comm_.comm_error_last_read() ) {
		if ( verbose_read_err_ ) ROS_WARN("DXMIO failed to read config");
	}
	if ( verbose_read_ ) ROS_INFO_STREAM("DXMIO config are read" << control_table_layout(parent_.width_log_, id_data_map, addr_config_list));
	for ( auto [id, value] : id_data_map ) {
		terminator_r_[id] = value[0] != 0;
		if ( terminator_r_[id] != terminator_w_[id] ) ROS_WARN("DXMIO (id=[%d]) failed to verify config", id);
	}
	return !id_data_map.empty();
}

bool DynamixelHandler::ImuDXMIO::ReadImuMotion(const vector<id_t>& id_list) {
	static auto& dyn_comm_ = parent_.dyn_comm_;
	static const vector<DXMIOAddress> addr_imu_list = {
		addr_gx, addr_gy, addr_gz,
		addr_ax, addr_ay, addr_az,
		addr_quat_w, addr_quat_x, addr_quat_y, addr_quat_z
	};

	auto id_data_map = dyn_comm_.SyncRead(addr_imu_list, id_list);
	const bool is_timeout   = dyn_comm_.timeout_last_read();
	const bool has_comm_err = dyn_comm_.comm_error_last_read();

	if ( is_timeout || has_comm_err ) {
		if (verbose_read_err_) ROS_WARN("DXMIO IMU motion failed to read %s", is_timeout ? " (time out)" : " (some kind packet error)");
	}

	if (verbose_read_) {
		ROS_INFO_STREAM( "DXMIO IMU motion are read"
			<< control_table_layout(parent_.width_log_, id_data_map, addr_imu_list) );
	}

	StateLock lock(parent_.mutex_state_);
	const auto stamp = parent_.get_clock()->now();
	for ( const auto& [id, result] : id_data_map ) {
		const auto& rpy = rpy_adjust_[id];
		const auto quat_adjust = convert_rpy_to_quat(rpy[0], rpy[1], rpy[2]);
		// Convert axis convention before applying the mounting rotation.
		const double sign = flip_z_axis_[id] ? -1.0 : 1.0;
		const vec3d gyro = {
			int32_bits_to_float(static_cast<int32_t>(result[0])) * sign,
			int32_bits_to_float(static_cast<int32_t>(result[1])) * sign,
			int32_bits_to_float(static_cast<int32_t>(result[2]))
		};
		const vec3d acc = {
			int32_bits_to_float(static_cast<int32_t>(result[3])),
			int32_bits_to_float(static_cast<int32_t>(result[4])),
			int32_bits_to_float(static_cast<int32_t>(result[5])) * sign
		};
		gyro_[id] = rotate_vector_by_quat(gyro, quat_adjust);
		acc_[id] = rotate_vector_by_quat(acc, quat_adjust);
		// DXMIO [w, x, y, z] to ROS [x, y, z, w].
		const auto q = normalize_quat({int32_bits_to_float(static_cast<int32_t>(result[7])),
			int32_bits_to_float(static_cast<int32_t>(result[8])),
			int32_bits_to_float(static_cast<int32_t>(result[9])),
			int32_bits_to_float(static_cast<int32_t>(result[6]))});
		const auto quat = normalize_quat({q[0] * sign, q[1] * sign, q[2], q[3]});
		quat_[id] = normalize_quat(multiply_quat(quat, quat_adjust));
		const auto gravity_dir = rotate_vector_by_quat({0.0, 0.0, 1.0}, {-q[0], -q[1], -q[2], q[3]});
		gravity_dir_[id] = rotate_vector_by_quat({gravity_dir[0], gravity_dir[1], gravity_dir[2] * sign}, quat_adjust);
		stamp_imu_[id] = stamp;
	}
	return !id_data_map.empty();
}

bool DynamixelHandler::ImuDXMIO::ReadImuExtra(const vector<id_t>& id_list) {
	static auto& dyn_comm_ = parent_.dyn_comm_;
	static const vector<DXMIOAddress> addr_imu_list = {
		addr_gravity_x, addr_gravity_y, addr_gravity_z, addr_temperature
	};

	auto id_data_map = dyn_comm_.SyncRead(addr_imu_list, id_list);
	const bool is_timeout   = dyn_comm_.timeout_last_read();
	const bool has_comm_err = dyn_comm_.comm_error_last_read();

	if ( is_timeout || has_comm_err ) {
		if (verbose_read_err_) ROS_WARN("DXMIO IMU extra failed to read %s", is_timeout ? " (time out)" : " (some kind packet error)");
	}

	if (verbose_read_) {
		ROS_INFO_STREAM( "DXMIO IMU extra are read"
			<< control_table_layout(parent_.width_log_, id_data_map, addr_imu_list) );
	}

	StateLock lock(parent_.mutex_state_);
	for ( const auto& [id, result] : id_data_map ) {
		const vec3d gravity = {
			int32_bits_to_float(static_cast<int32_t>(result[0])),
			int32_bits_to_float(static_cast<int32_t>(result[1])),
			int32_bits_to_float(static_cast<int32_t>(result[2]))
		};
		gravity_norm_[id] = std::sqrt(gravity[0] * gravity[0] + gravity[1] * gravity[1] + gravity[2] * gravity[2]);
		temperature_degc_[id] = 23.0 + 0.5 * result[3];	// BNO085 raw gyro temperature: 0.5 degC/LSB, zero at 23 degC (as in E097C smpl19).
	}
	return !id_data_map.empty();
}

void DynamixelHandler::ImuDXMIO::BroadcastImuData() {
	DxmioImu msg;
	msg.stamp = parent_.get_clock()->now();
	{
		StateLock lock(parent_.mutex_state_);
		for ( auto id : id_imu_list_ ) {
			const auto& rpy = rpy_adjust_[id];
			msg.id_list.push_back(id);
			Imu msg_imu;
			msg_imu.header.frame_id = frame_id_[id];
			msg_imu.header.stamp = stamp_imu_[id];
			msg_imu.angular_velocity.x = gyro_[id][0];
			msg_imu.angular_velocity.y = gyro_[id][1];
			msg_imu.angular_velocity.z = gyro_[id][2];
			msg_imu.linear_acceleration.x = acc_[id][0];
			msg_imu.linear_acceleration.y = acc_[id][1];
			msg_imu.linear_acceleration.z = acc_[id][2];
			msg_imu.orientation.x = quat_[id][0];
			msg_imu.orientation.y = quat_[id][1];
			msg_imu.orientation.z = quat_[id][2];
			msg_imu.orientation.w = quat_[id][3];
			msg.data.push_back(msg_imu);
			geometry_msgs::msg::Vector3 msg_gravity;
			msg_gravity.x = gravity_dir_[id][0] * gravity_norm_[id];
			msg_gravity.y = gravity_dir_[id][1] * gravity_norm_[id];
			msg_gravity.z = gravity_dir_[id][2] * gravity_norm_[id];
			msg.gravity.push_back(msg_gravity);
			msg.temperature_degc.push_back(temperature_degc_[id]);
			msg.frame_id.push_back(frame_id_[id]);
			msg.flip_z_axis.push_back(flip_z_axis_[id]);
			geometry_msgs::msg::Vector3 msg_rpy;
			msg_rpy.x = rpy[0];
			msg_rpy.y = rpy[1];
			msg_rpy.z = rpy[2];
			msg.adjust_rpy_deg.push_back(msg_rpy);
			msg.terminator.push_back(terminator_r_[id]);
		}
	}
	if ( pub_imu_ && rclcpp::ok() ) pub_imu_->publish(msg);
}
