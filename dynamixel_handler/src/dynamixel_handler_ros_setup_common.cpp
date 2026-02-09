#include "dynamixel_handler.hpp"

void DynamixelHandler::SetupRosInterfaces(bool no_use_command_line) {
    // 他のノードとの通信用
    sub_dxl_all_cmds_ = create_subscription<DxlCommandsAll>(
        "dynamixel/commands/all", 10,
        [this](const DxlCommandsAll::SharedPtr msg) { CallbackCmdsAll(msg); });
    if (use_["x"]) {
        sub_dxl_x_cmds_ = create_subscription<DxlCommandsX>(
            "dynamixel/commands/x", 10,
            [this](const DxlCommandsX::SharedPtr msg) { CallbackCmdsX(msg); });
    }
    if (use_["p"]) {
        sub_dxl_p_cmds_ = create_subscription<DxlCommandsP>(
            "dynamixel/commands/p", 10,
            [this](const DxlCommandsP::SharedPtr msg) { CallbackCmdsP(msg); });
    }
    if (use_["pro"]) {
        sub_dxl_pro_cmds_ = create_subscription<DxlCommandsPro>(
            "dynamixel/commands/pro", 10,
            [this](const DxlCommandsPro::SharedPtr msg) { CallbackCmdsPro(msg); });
    }
    pub_dxl_states_ = create_publisher<DxlStates>("dynamixel/states", 4);

    // デバッグ用
    sub_shortcut_ = create_subscription<DynamixelShortcut>(
        "dynamixel/shortcut", 4,
        [this](const DynamixelShortcut& msg) { CallbackShortcut(msg); });
    pub_debug_ = create_publisher<DynamixelDebug>("dynamixel/debug", 4);

    if (no_use_command_line) {
        return;
    }

    // CallbackGroup は現状使っていないが、将来の subscriber 分離用に残す
    auto callback_group_subscriber = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_subscriber;
    (void)sub_options;

    sub_status_ = create_subscription<DynamixelStatus>(
        "dynamixel/command/status", 4,
        [this](const DynamixelStatus& msg) { CallbackCmd_Status(msg); });
    sub_goal_ = create_subscription<DynamixelGoal>(
        "dynamixel/command/goal", 4,
        [this](const DynamixelGoal& msg) { CallbackCmd_Goal(msg); });
    sub_gain_ = create_subscription<DynamixelGain>(
        "dynamixel/command/gain", 4,
        [this](const DynamixelGain& msg) { CallbackCmd_Gain(msg); });
    sub_limit_ = create_subscription<DynamixelLimit>(
        "dynamixel/command/limit", 4,
        [this](const DynamixelLimit& msg) { CallbackCmd_Limit(msg); });

    pub_status_ = create_publisher<DynamixelStatus>("dynamixel/state/status", 4);
    pub_present_ = create_publisher<DynamixelPresent>("dynamixel/state/present", 4);
    pub_goal_ = create_publisher<DynamixelGoal>("dynamixel/state/goal", 4);
    pub_gain_ = create_publisher<DynamixelGain>("dynamixel/state/gain", 4);
    pub_limit_ = create_publisher<DynamixelLimit>("dynamixel/state/limit", 4);
    pub_error_ = create_publisher<DynamixelError>("dynamixel/state/error", 4);

    if (use_["x"])   SetupRosInterfacesX();
    if (use_["p"])   SetupRosInterfacesP();
    if (use_["pro"]) SetupRosInterfacesPro();
}
