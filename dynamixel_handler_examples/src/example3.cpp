#include "rclcpp/rclcpp.hpp"

#include "dynamixel_handler_msgs/msg/dxl_commands_x.hpp"
#include "dynamixel_handler_msgs/msg/dxl_states.hpp"

#include <chrono>
#include <csignal>
#include <cstdint>
#include <map>
#include <vector>

using dynamixel_handler_msgs::msg::DxlCommandsX;
using dynamixel_handler_msgs::msg::DxlStates;
using namespace std::chrono_literals;

namespace {
volatile std::sig_atomic_t g_shutdown_requested = 0;
void SignalHandler(int) { g_shutdown_requested = 1; }
}  // namespace

class Example3Node : public rclcpp::Node {
public:
    Example3Node() : Node("example3_node") {
        sub_states_ = create_subscription<DxlStates>(
            "dynamixel/states", 10, std::bind(&Example3Node::CallbackStates, this, std::placeholders::_1));
        pub_cmd_x_ = create_publisher<DxlCommandsX>("dynamixel/commands/x", 10);
        timer_ = create_wall_timer(500ms, std::bind(&Example3Node::MainLoop, this));
    }
    ~Example3Node() override {
        RestoreVelocityLimit();
        rclcpp::sleep_for(200ms);  // publishを送る時間を確保
    }

    void RestoreVelocityLimit() {
        if (initial_velocity_limit_deg_s_.empty()) return;

        auto cmd = DxlCommandsX();
        for (const auto& [id, limit] : initial_velocity_limit_deg_s_) {
            cmd.limit.id_list.push_back(id);
            cmd.limit.velocity_limit_deg_s.push_back(limit);
        }
        pub_cmd_x_->publish(cmd);
        RCLCPP_INFO(get_logger(), "Restored velocity_limit for %zu servo(s).", cmd.limit.id_list.size());
    }

private:
    struct ServoData {
        bool error = false;
        double pos_deg = 0.0;
        bool has_home = false;
        double home_deg = 0.0;
        bool forward = true;
    };

    void CallbackStates(const DxlStates::SharedPtr msg) {
        last_state_rx_time_ = get_clock()->now();
        if (!msg->present.id_list.empty()) {
            active_ids_ = msg->present.id_list;
        } else if (!msg->status.id_list.empty()) {
            active_ids_ = msg->status.id_list;
        }

        for (size_t i = 0; i < msg->status.id_list.size(); ++i) {
            const auto id = msg->status.id_list[i];
            auto& servo = servos_[id];
            servo.error = (i < msg->status.error.size()) ? msg->status.error[i] : false;
        }

        for (size_t i = 0; i < msg->present.id_list.size(); ++i) {
            if (i >= msg->present.position_deg.size()) {
                continue;
            }
            auto& servo = servos_[msg->present.id_list[i]];
            servo.pos_deg = msg->present.position_deg[i];
            if (!servo.has_home) {
                servo.home_deg = servo.pos_deg;
                servo.has_home = true;
            }
        }

        for (size_t i = 0; i < msg->limit.id_list.size(); ++i) {
            if (i >= msg->limit.velocity_limit_deg_s.size()) continue;
            const auto id = msg->limit.id_list[i];
            if (initial_velocity_limit_deg_s_.count(id) == 0)
                initial_velocity_limit_deg_s_[id] = msg->limit.velocity_limit_deg_s[i];
        }
    }

    void MainLoop() {
        if (last_state_rx_time_.nanoseconds() == 0) {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000, "Waiting for /dynamixel/states ...");
            return;
        }

        const auto state_age = (get_clock()->now() - last_state_rx_time_).seconds();
        if (state_age > 2.0) {
            initialized_ids_.clear();
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "State timeout %.2f sec. Skip command publish.", state_age);
            return;
        }

        const auto active_ids = active_ids_;
        if (active_ids.empty()) return;

        bool has_error = false;
        for (const auto id : active_ids) {
            if (servos_[id].error) {
                has_error = true;
                break;
            }
        }

        // エラー個体を先に解除（status.error フィールドで clear error を依頼）
        if (has_error) {
            auto cmd = DxlCommandsX();
            cmd.status.id_list = active_ids;
            cmd.status.error.assign(active_ids.size(), false);
            pub_cmd_x_->publish(cmd);
            return;
        }

        auto cmd = DxlCommandsX();

        // 初回はトルクON + 速度制限をまとめて設定
        if (active_ids != initialized_ids_) {
            for (const auto id : active_ids) {
                if (initial_velocity_limit_deg_s_.count(id) == 0) {
                    RCLCPP_INFO_THROTTLE(
                        get_logger(), *get_clock(), 2000,
                        "Waiting /dynamixel/states.limit to cache initial velocity_limit.");
                    return;
                }
            }
            cmd.status.id_list = active_ids;
            cmd.status.torque.assign(active_ids.size(), true);
            cmd.limit.id_list = active_ids;
            cmd.limit.velocity_limit_deg_s.assign(active_ids.size(), 80.0);
            pub_cmd_x_->publish(cmd);
            initialized_ids_ = active_ids;
            return;
        }

        // 速度制御: ホーム位置を中心に，位置フィードバックで進行方向を反転
        cmd.status.id_list = active_ids;
        cmd.status.torque.assign(active_ids.size(), true);
        cmd.velocity_control.id_list = active_ids;
        auto push_velocity = [&](double vel_deg_s) {
            cmd.velocity_control.velocity_deg_s.push_back(vel_deg_s);
            cmd.velocity_control.profile_acc_deg_ss.push_back(200.0);
        };
        for (const auto id : active_ids) {
            auto& servo = servos_[id];
            if (!servo.has_home) {
                push_velocity(0.0);
                continue;
            }
            if      (servo.pos_deg > servo.home_deg + 40.0) servo.forward = false;
            else if (servo.pos_deg < servo.home_deg - 40.0) servo.forward = true;
            push_velocity(servo.forward ? 60.0 : -60.0);
        }
        pub_cmd_x_->publish(cmd);
    }

    rclcpp::Time last_state_rx_time_;
    std::vector<uint16_t> initialized_ids_;
    std::vector<uint16_t> active_ids_;
    std::map<uint16_t, ServoData> servos_;
    std::map<uint16_t, double> initial_velocity_limit_deg_s_;

    rclcpp::Subscription<DxlStates>::SharedPtr sub_states_;
    rclcpp::Publisher<DxlCommandsX>::SharedPtr pub_cmd_x_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    auto node = std::make_shared<Example3Node>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    rclcpp::Rate rate(200.0);
    while (rclcpp::ok() && !g_shutdown_requested) {
        executor.spin_some();
        rate.sleep();
    }
    executor.remove_node(node);
    node.reset();  // デストラクタ内でvelocity_limit復元
    rclcpp::shutdown();
    return 0;
}
