#include "rclcpp/rclcpp.hpp"

#include "dynamixel_handler_msgs/msg/dxl_commands_x.hpp"
#include "dynamixel_handler_msgs/msg/dxl_states.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

using dynamixel_handler_msgs::msg::DxlCommandsX;
using dynamixel_handler_msgs::msg::DxlStates;
using sensor_msgs::msg::JointState;

class Example0Node : public rclcpp::Node {
public:
    Example0Node() : Node("example0_joint_state_bridge") {
        sub_joint_command_ =
            create_subscription<JointState>("joint_command", 10,
                std::bind(&Example0Node::CallbackJointCommand, this, std::placeholders::_1));
        sub_states_ =
            create_subscription<DxlStates>("dynamixel/states", 10,
                std::bind(&Example0Node::CallbackStates, this, std::placeholders::_1));

        pub_cmd_x_ = create_publisher<DxlCommandsX>("dynamixel/commands/x", 10);
        pub_joint_states_ = create_publisher<JointState>("joint_states", 10);
    }

private:
    void CallbackJointCommand(const JointState::SharedPtr msg) {
        if (msg->name.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Empty JointState.name.");
            return;
        }

        const auto joint_count = msg->name.size();
        bool has_position = !msg->position.empty();
        bool has_velocity = !msg->velocity.empty();
        bool has_effort = !msg->effort.empty();
        if (has_position && msg->position.size() != joint_count) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "JointState.position size mismatch: %zu != %zu.", joint_count, msg->position.size());
            has_position = false;
        }
        if (has_velocity && msg->velocity.size() != joint_count) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "JointState.velocity size mismatch: %zu != %zu.", joint_count, msg->velocity.size());
            has_velocity = false;
        }
        if (has_effort && msg->effort.size() != joint_count) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "JointState.effort size mismatch: %zu != %zu.", joint_count, msg->effort.size());
            has_effort = false;
        }

        if (!has_position && !has_velocity && !has_effort) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No valid JointState field.");
            return;
        }

        DxlCommandsX cmd;
        auto& cmd_cpos = cmd.current_base_position_control;
        auto& cmd_epos = cmd.extended_position_control;
        auto& cmd_vel = cmd.velocity_control;
        auto& cmd_cur = cmd.current_control;
        bool ignored_effort = false;
        auto parse_uint16 = [](const std::string& text, uint16_t& value) {
            unsigned int parsed = 0;
            const auto result = std::from_chars(text.data(), text.data() + text.size(), parsed);
            if (result.ec != std::errc() || result.ptr != text.data() + text.size()
                || parsed > std::numeric_limits<uint16_t>::max()) return false;
            value = static_cast<uint16_t>(parsed);
            return true;
        };

        for (size_t i = 0; i < joint_count; ++i) {
            uint16_t id = 0;
            if (!parse_uint16(msg->name[i], id)) {
                RCLCPP_WARN_THROTTLE( get_logger(), *get_clock(), 2000, "Unknown joint name '%s'.", msg->name[i].c_str());
                continue;
            }

            if (has_position) {
                const double position_deg = msg->position[i] * 180.0 / M_PI;

                if (has_effort) {
                    cmd_cpos.id_list.push_back(id);
                    cmd_cpos.position_deg.push_back(position_deg);
                    cmd_cpos.current_ma.push_back(msg->effort[i] * 1000.0);
                    if (has_velocity) cmd_cpos.profile_vel_deg_s.push_back(std::abs(msg->velocity[i] * 180.0 / M_PI));
                } else {
                    cmd_epos.id_list.push_back(id);
                    cmd_epos.position_deg.push_back(position_deg);
                    if (has_velocity) cmd_epos.profile_vel_deg_s.push_back(std::abs(msg->velocity[i] * 180.0 / M_PI));
                }
                continue;
            }

            if (has_velocity) {
                cmd_vel.id_list.push_back(id);
                cmd_vel.velocity_deg_s.push_back(msg->velocity[i] * 180.0 / M_PI);
                if (has_effort) ignored_effort = true;
                continue;
            }

            cmd_cur.id_list.push_back(id);
            cmd_cur.current_ma.push_back(msg->effort[i] * 1000.0);
        }

        if (ignored_effort) RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignore effort for velocity command.");

        pub_cmd_x_->publish(cmd);
    }

    void CallbackStates(const DxlStates::SharedPtr msg) {
        if (msg->present.id_list.empty()) return;
        auto& ids = msg->present.id_list;

        JointState joint_state;
        joint_state.header.stamp = msg->stamp;
        for (const auto id : ids) joint_state.name.push_back(std::to_string(id));

        if (msg->present.position_deg.size() == ids.size()) 
            for (size_t i = 0; i < ids.size(); ++i) joint_state.position.push_back(msg->present.position_deg[i] * M_PI / 180.0);
        if (msg->present.velocity_deg_s.size() == ids.size()) 
            for (size_t i = 0; i < ids.size(); ++i) joint_state.velocity.push_back(msg->present.velocity_deg_s[i] * M_PI / 180.0);
        if (msg->present.current_ma.size() == ids.size()) 
            for (size_t i = 0; i < ids.size(); ++i) joint_state.effort.push_back(msg->present.current_ma[i] * 0.001);

        pub_joint_states_->publish(joint_state);
    }

    rclcpp::Subscription<JointState>::SharedPtr sub_joint_command_;
    rclcpp::Subscription<DxlStates>::SharedPtr sub_states_;
    rclcpp::Publisher<DxlCommandsX>::SharedPtr pub_cmd_x_;
    rclcpp::Publisher<JointState>::SharedPtr pub_joint_states_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Example0Node>());
    rclcpp::shutdown();
    return 0;
}
