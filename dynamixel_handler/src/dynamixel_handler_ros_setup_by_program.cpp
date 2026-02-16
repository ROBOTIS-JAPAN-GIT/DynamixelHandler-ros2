#include "dynamixel_handler.hpp"
#include "dynamixel_handler_msgs/msg/dxl_commands_all.hpp"
#include "dynamixel_handler_msgs/msg/dxl_commands_p.hpp"
#include "dynamixel_handler_msgs/msg/dxl_commands_pro.hpp"
#include "dynamixel_handler_msgs/msg/dxl_commands_x.hpp"
#include "dynamixel_handler_msgs/msg/dxl_states.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_debug.hpp"
#include "dynamixel_handler_msgs/msg/dynamixel_shortcut.hpp"

void DynamixelHandler::SetupRosInterfaces_byProgram() {
    // CallbackGroup は現状使っていないが、将来の subscriber 分離用に残す
    // auto callback_group_subscriber = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // rclcpp::SubscriptionOptions sub_options;
    // sub_options.callback_group = callback_group_subscriber;
    // (void)sub_options;

    sub_dxl_all_cmds_ = create_subscription<DxlCommandsAll>( // 全シリーズ混在コマンドのサブスクライブ
        "dynamixel/commands/all", 10,
        [this](std::shared_ptr<DxlCommandsAll> msg) { CallbackCmdsAll(msg); });
    sub_shortcut_ = create_subscription<DynamixelShortcut>( // ショートカットコマンドのサブスクライブ
        "dynamixel/shortcut", 4,
        [this](const DynamixelShortcut& msg) { CallbackShortcut(msg); });

    pub_dxl_states_ = create_publisher<DxlStates     >("dynamixel/states", 4); // 全シリーズのステートのパブリッシュ
    pub_debug_      = create_publisher<DynamixelDebug>("dynamixel/debug" , 4);  // デバッグ情報のパブリッシュ

    if (use_[ "x" ]) sub_dxl_x_cmds_   = create_subscription<DxlCommandsX  >(
                        "dynamixel/commands/x", 10,
                        [this](std::shared_ptr<DxlCommandsX> msg) { CallbackCmdsX(msg); });
    if (use_[ "p" ]) sub_dxl_p_cmds_   = create_subscription<DxlCommandsP  >(
                        "dynamixel/commands/p", 10,
                        [this](std::shared_ptr<DxlCommandsP> msg) { CallbackCmdsP(msg); });
    if (use_["pro"]) sub_dxl_pro_cmds_ = create_subscription<DxlCommandsPro>(
                        "dynamixel/commands/pro", 10,
                        [this](std::shared_ptr<DxlCommandsPro> msg) { CallbackCmdsPro(msg); });
}
