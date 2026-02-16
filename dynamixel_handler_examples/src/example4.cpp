#include "rclcpp/rclcpp.hpp"

#include "dynamixel_handler_msgs/msg/dxl_commands_all.hpp"
#include "dynamixel_handler_msgs/msg/dxl_states.hpp"

#include <chrono>
#include <cstdint>
#include <map>
#include <vector>

using dynamixel_handler_msgs::msg::DxlCommandsAll;
using dynamixel_handler_msgs::msg::DxlStates;
using namespace std::chrono_literals;

class Example4Node : public rclcpp::Node {
public:
    Example4Node() : Node("example4_node") {
        sub_states_ = create_subscription<DxlStates>(
            "dynamixel/states", 10, std::bind(&Example4Node::CallbackStates, this, std::placeholders::_1));
        pub_cmd_all_ = create_publisher<DxlCommandsAll>("dynamixel/commands/all", 10);
        timer_ = create_wall_timer(100ms, std::bind(&Example4Node::MainLoop, this));
    }

private:
    struct ServoData {
        bool error = false;
        bool has_pos = false;
        double pos_deg = 0.0;
        bool forward = true;
    };

    void CallbackStates(const DxlStates::SharedPtr msg) {
        last_state_rx_time_ = get_clock()->now();
        active_ids_ = msg->status.id_list;

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
            servo.has_pos = true;
        }
    }

    void MainLoop() {
        if (last_state_rx_time_.nanoseconds() == 0) {
            RCLCPP_INFO_THROTTLE( get_logger(), *get_clock(), 2000, "Waiting for /dynamixel/states ...");
            return;
        }

        if ((get_clock()->now() - last_state_rx_time_).seconds() > 2.0) {
            initialized_ids_.clear();
            RCLCPP_WARN(get_logger(), "No /dynamixel/states received for more than 2 seconds. Resetting state.");
            return;
        }

        const auto active_ids = active_ids_;
        if (active_ids.empty())  return;

        std::vector<uint16_t> error_ids;
        for (const auto id : active_ids) if (servos_[id].error) error_ids.push_back(id);

        auto cmd = DxlCommandsAll();

        if (!error_ids.empty()) {
            cmd.status.id_list = error_ids;
            cmd.status.error.assign(error_ids.size(), false);
            pub_cmd_all_->publish(cmd);
            return;
        }

        // goal.* を使う場合は operating mode の明示設定が必要。
        if (active_ids != initialized_ids_) {
            cmd.status.id_list = active_ids;
            cmd.status.mode.assign(active_ids.size(), cmd.status.CONTROL_POSITION);
            cmd.status.torque.assign(active_ids.size(), true);
            pub_cmd_all_->publish(cmd);
            initialized_ids_ = active_ids;
            return;
        }

        constexpr double kTargetForwardDeg = 160.0;
        constexpr double kTargetReverseDeg = -160.0;
        cmd.goal.id_list = active_ids;
        for (const auto id : active_ids) {
            auto& servo = servos_[id];
            if (servo.has_pos) {
                if      (servo.pos_deg > kTargetForwardDeg - 5.0) servo.forward = false;
                else if (servo.pos_deg < kTargetReverseDeg + 5.0) servo.forward = true;
            }
            cmd.goal.position_deg.push_back(servo.forward ? kTargetForwardDeg : kTargetReverseDeg);
        }
        pub_cmd_all_->publish(cmd);
    }

    rclcpp::Time last_state_rx_time_;
    std::vector<uint16_t> initialized_ids_;
    std::vector<uint16_t> active_ids_;
    std::map<uint16_t, ServoData> servos_;

    rclcpp::Subscription<DxlStates>::SharedPtr sub_states_;
    rclcpp::Publisher<DxlCommandsAll>::SharedPtr pub_cmd_all_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Example4Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
