#include "external_port.hpp"

#include "myUtils/formatting_output.hpp"
#include "myUtils/make_iterator_convenient.hpp"

#include <chrono>
#include <thread>
#include <algorithm>
using std::clamp;
#include <utility>
using std::exchange;

#define ROS_INFO(...)  RCLCPP_INFO(parent_.get_logger(), __VA_ARGS__)
#define ROS_WARN(...)  RCLCPP_WARN(parent_.get_logger(), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(parent_.get_logger(), __VA_ARGS__)
#define ROS_INFO_STREAM(...)  RCLCPP_INFO_STREAM(parent_.get_logger(), __VA_ARGS__)
#define ROS_WARN_STREAM(...)  RCLCPP_WARN_STREAM(parent_.get_logger(), __VA_ARGS__)
#define ROS_ERROR_STREAM(...) RCLCPP_ERROR_STREAM(parent_.get_logger(), __VA_ARGS__)
#define ROS_INFO_T(...)  RCLCPP_INFO_THROTTLE(parent_.get_logger(), *parent_.get_clock(), __VA_ARGS__)
#define ROS_WARN_T(...)  RCLCPP_WARN_THROTTLE(parent_.get_logger(), *parent_.get_clock(), __VA_ARGS__)
#define ROS_ERROR_T(...) RCLCPP_ERROR_THROTTLE(parent_.get_logger(), *parent_.get_clock(), __VA_ARGS__)
#define ROS_STOP(...) \
    {{ RCLCPP_FATAL(parent_.get_logger(), __VA_ARGS__); rclcpp::shutdown(); std::exit(EXIT_FAILURE); }}

using StateLock = std::lock_guard<std::recursive_mutex>;
static void rsleep(int millisec) { std::this_thread::sleep_for(std::chrono::milliseconds(millisec));}

void DynamixelHandler::ExternalPort::CallbackExternalPort(const DxlExternalPort::SharedPtr msg){
    static auto& id_set_ = parent_.id_set_;
    static auto& model_ = parent_.model_;

    // msg.id_list内のIDとmsg.port内のport番号の妥当性を確認
    vector<size_t> valid_indice;
    vector<uint8_t> valid_id_list, valid_port_list;
    set<tuple<uint8_t, uint8_t>> valid_pair_set;
    if ( msg->id_list.size() != msg->port.size() ) {
         ROS_WARN("External Port, ID and port size mismatch"); return;
    } // サイズ違いは問答無用でNG
    StateLock lock(parent_.mutex_state_);
    for (size_t i=0; i<msg->id_list.size(); i++) {
        auto id = msg->id_list[i];
        if ( !is_in(id, id_set_) ) continue; // 存在しないIDは無視
        if ( !has_external_port(model_[id]) ){
            ROS_WARN("ID [%d] has no external port", id); continue;
        }
        auto port = msg->port[i];
        if ( !is_in( port, ex_port_indice_[series_[id]]) ){
            ROS_WARN("ID [%d] does not have external port_%d", id, port); continue;
        }
        // 重複は (id, port) pair 単位で取り除く
        if ( !valid_pair_set.emplace(id, port).second ) continue;
        valid_indice.push_back(i);
        valid_id_list.push_back(id);
        valid_port_list.push_back(port);
    }
    if ( valid_indice.empty() ) return; // 有効なIDがない場合は何もしない
    // mode, data それぞれの指令が入力されたかどうかを確認
    const bool has_data = msg->id_list.size() == msg->data.size();
    const bool has_mode = msg->id_list.size() == msg->mode.size();
    if (verbose_callback_) {
        ROS_INFO("External Port, '%zu' port(s) are tryed to updated", valid_id_list.size());
        ROS_INFO_STREAM(id_list_layout(valid_id_list,   "  ID  "));
        ROS_INFO_STREAM(id_list_layout(valid_port_list, "  port"));
        if ( has_data ) ROS_INFO("  - updated: data"); else if ( !msg->data.empty() ) ROS_WARN("  - skipped: data (size mismatch)");
        if ( has_mode ) ROS_INFO("  - updated: mode"); else if ( !msg->mode.empty() ) ROS_WARN("  - skipped: mode (size mismatch)");
    }
    // データの反映
    for (auto i : valid_indice) {
        auto   ID = msg->id_list[i];
        auto port = msg->port[i];
        if ( has_mode ) {
            if ( msg->mode[i] == msg->MODE_UNSET ) {
                touched_port_export_[ID].erase(port); // modeがUNSETの場合はポートを削除
                continue;
            }
            auto mode_pre = export_w_[ID][port].mode; // 書き込み用のモード
                 if ( msg->mode[i] == msg->MODE_ANALOG_IN           ) export_w_[ID][port].mode = EXTERNAL_PORT_MODE_AIN;
            else if ( msg->mode[i] == msg->MODE_DIGITAL_OUT         ) export_w_[ID][port].mode = EXTERNAL_PORT_MODE_DOUT;
            else if ( msg->mode[i] == msg->MODE_DIGITAL_IN_PULLUP   ) export_w_[ID][port].mode = EXTERNAL_PORT_MODE_DIN_PULLUP;
            else if ( msg->mode[i] == msg->MODE_DIGITAL_IN_PULLDOWN ) export_w_[ID][port].mode = EXTERNAL_PORT_MODE_DIN_PULLDOWN;
            else {ROS_WARN("  Invalid mode [%s]", msg->mode[i].c_str()); continue;} // verbose_callback_ に関わらず出力
            touched_port_export_[ID].insert(port);
            if ( mode_pre!=export_w_[ID][port].mode ) updated_id_export_.mode.insert(ID);
        }
        if ( has_data ) {
            export_w_[ID][port].data = msg->data[i];
            updated_id_export_.data.insert(ID);
        }
    }
    if ( verbose_callback_ ) ROS_INFO("------------------+------------------");
}


void DynamixelHandler::ExternalPort::BroadcastExternalPort(){
    static auto& id_set_ = parent_.id_set_;
    DxlExternalPort msg;
    msg.stamp = parent_.get_clock()->now();
    { StateLock lock(parent_.mutex_state_);
        for ( auto id : id_set_ ) if (has_external_port(model_[id]))
            for ( auto port : ex_port_indice_[series_[id]] ) {
                msg.id_list.push_back(id);
                msg.port.push_back(port);
                if ( !is_in(port, touched_port_export_[id])) {
                    msg.mode.push_back(msg.MODE_UNSET);
                    msg.data.push_back(-1); // modeがUNSETの場合はdataは-1にする
                    continue;
                }
                switch ( export_r_[id][port].mode ){
                    case EXTERNAL_PORT_MODE_AIN         : msg.mode.push_back(msg.MODE_ANALOG_IN          ); break;
                    case EXTERNAL_PORT_MODE_DOUT        : msg.mode.push_back(msg.MODE_DIGITAL_OUT        ); break;
                    case EXTERNAL_PORT_MODE_DIN_PULLUP  : msg.mode.push_back(msg.MODE_DIGITAL_IN_PULLUP  ); break;
                    case EXTERNAL_PORT_MODE_DIN_PULLDOWN: msg.mode.push_back(msg.MODE_DIGITAL_IN_PULLDOWN); break;
                }
                msg.data.push_back(export_r_[id][port].data);
    }    }

    if ( pub_ex_port_ && rclcpp::ok() ) pub_ex_port_->publish(msg);
}

uint16_t DynamixelHandler::ExternalPort::ReadExternalPortMode(id_t id, port_t port){
    DynamixelAddress addr = AddrX::external_port_mode_1;
           if ( series_[id]==SERIES_X ) switch (port) {
        case 1: addr = AddrX::external_port_mode_1; break;
        case 2: addr = AddrX::external_port_mode_2; break;
        case 3: addr = AddrX::external_port_mode_3; break;
        default: return 0;
    } else if ( series_[id]==SERIES_P ) switch (port) {
        case 1: addr = AddrP::external_port_mode_1; break;
        case 2: addr = AddrP::external_port_mode_2; break;
        case 3: addr = AddrP::external_port_mode_3; break;
        case 4: addr = AddrP::external_port_mode_4; break;
        default: return 0;
    } else if ( series_[id]==SERIES_PRO ) switch (port) {
        case 1: addr = AddrPro::external_port_mode_1; break;
        case 2: addr = AddrPro::external_port_mode_2; break;
        case 3: addr = AddrPro::external_port_mode_3; break;
        case 4: addr = AddrPro::external_port_mode_4; break;
        default: return 0;
    } else       return 0;
    return parent_.dyn_comm_.tryRead(addr, id);
}

uint16_t DynamixelHandler::ExternalPort::ReadExternalPortData(id_t id, port_t port){
    DynamixelAddress addr = AddrX::external_port_data_1;
           if ( series_[id]==SERIES_X ) switch (port) {
        case 1: addr = AddrX::external_port_data_1; break;
        case 2: addr = AddrX::external_port_data_2; break;
        case 3: addr = AddrX::external_port_data_3; break;
        default: return 0;
    } else if ( series_[id]==SERIES_P ) switch (port) {
        case 1: addr = AddrP::external_port_data_1; break;
        case 2: addr = AddrP::external_port_data_2; break;
        case 3: addr = AddrP::external_port_data_3; break;
        case 4: addr = AddrP::external_port_data_4; break;
        default: return 0;
    } else if ( series_[id]==SERIES_PRO ) switch (port) {
        case 1: addr = AddrPro::external_port_data_1; break;
        case 2: addr = AddrPro::external_port_data_2; break;
        case 3: addr = AddrPro::external_port_data_3; break;
        case 4: addr = AddrPro::external_port_data_4; break;
        default: return 0;
    } else       return 0;
    return parent_.dyn_comm_.tryRead(addr, id);
}

bool DynamixelHandler::ExternalPort::WriteExternalPortMode(id_t id, port_t port, uint16_t data){
    DynamixelAddress addr = AddrX::external_port_mode_1;
           if ( series_[id]==SERIES_X ) switch (port) {
        case 1: addr = AddrX::external_port_mode_1; break;
        case 2: addr = AddrX::external_port_mode_2; break;
        case 3: addr = AddrX::external_port_mode_3; break;
        default: return false;
    } else if ( series_[id]==SERIES_P ) switch (port) {
        case 1: addr = AddrP::external_port_mode_1; break;
        case 2: addr = AddrP::external_port_mode_2; break;
        case 3: addr = AddrP::external_port_mode_3; break;
        case 4: addr = AddrP::external_port_mode_4; break;
        default: return false;
    } else if ( series_[id]==SERIES_PRO ) switch (port) {
        case 1: addr = AddrPro::external_port_mode_1; break;
        case 2: addr = AddrPro::external_port_mode_2; break;
        case 3: addr = AddrPro::external_port_mode_3; break;
        case 4: addr = AddrPro::external_port_mode_4; break;
        default: return false;
    } else       return false;
    return parent_.dyn_comm_.tryWrite(addr, id, data);
}

bool DynamixelHandler::ExternalPort::WriteExternalPortData(id_t id, port_t port, uint16_t data){
    DynamixelAddress addr = AddrX::external_port_data_1;
           if ( series_[id]==SERIES_X ) switch (port) {
        case 1: addr = AddrX::external_port_data_1; break;
        case 2: addr = AddrX::external_port_data_2; break;
        case 3: addr = AddrX::external_port_data_3; break;
        default: return false;
    } else if ( series_[id]==SERIES_P ) switch (port) {
        case 1: addr = AddrP::external_port_data_1; break;
        case 2: addr = AddrP::external_port_data_2; break;
        case 3: addr = AddrP::external_port_data_3; break;
        case 4: addr = AddrP::external_port_data_4; break;
        default: return false;
    } else if ( series_[id]==SERIES_PRO ) switch (port) {
        case 1: addr = AddrPro::external_port_data_1; break;
        case 2: addr = AddrPro::external_port_data_2; break;
        case 3: addr = AddrPro::external_port_data_3; break;
        case 4: addr = AddrPro::external_port_data_4; break;
        default: return false;
    } else       return false;
    return parent_.dyn_comm_.tryWrite(addr, id, clamp<uint16_t>(data, 0, 1));
}

template <> void DynamixelHandler::ExternalPort::SyncWriteExternalPortMode(unordered_set<id_t> updated_id_mode){
    vector<id_t> id_list_x = parent_.id_filter(updated_id_mode, SERIES_X);
    vector<id_t> id_list_p = parent_.id_filter(updated_id_mode, SERIES_P);
    vector<id_t> id_list_pro = parent_.id_filter(updated_id_mode, SERIES_PRO);
    if ( !id_list_x.empty() ) parent_.SyncWrite_log(AddrX::torque_enable, id_list_x, vector<int64_t>(id_list_x.size(), TORQUE_DISABLE), verbose_write_);
    if ( !id_list_p.empty() ) parent_.SyncWrite_log(AddrP::torque_enable, id_list_p, vector<int64_t>(id_list_p.size(), TORQUE_DISABLE), verbose_write_);
    if ( !id_list_pro.empty() ) parent_.SyncWrite_log(AddrPro::torque_enable, id_list_pro, vector<int64_t>(id_list_pro.size(), TORQUE_DISABLE), verbose_write_);
    SyncWriteExternalPortMode<AddrX>(updated_id_mode);
    SyncWriteExternalPortMode<AddrP>(updated_id_mode);
    SyncWriteExternalPortMode<AddrPro>(updated_id_mode);
    if ( !id_list_x.empty() ) parent_.SyncWrite_log(AddrX::torque_enable, id_list_x, vector<int64_t>(id_list_x.size(), TORQUE_ENABLE), verbose_write_);
    if ( !id_list_p.empty() ) parent_.SyncWrite_log(AddrP::torque_enable, id_list_p, vector<int64_t>(id_list_p.size(), TORQUE_ENABLE), verbose_write_);
    if ( !id_list_pro.empty() ) parent_.SyncWrite_log(AddrPro::torque_enable, id_list_pro, vector<int64_t>(id_list_pro.size(), TORQUE_ENABLE), verbose_write_);
}
template <typename Addr> void DynamixelHandler::ExternalPort::SyncWriteExternalPortMode(unordered_set<id_t> updated_id_mode){
    //* 書き込みに必要な変数を用意
    vector<DynamixelAddress> exmode_addr_list;  // 書き込むコマンドのアドレスのベクタ
    map<uint8_t, vector<int64_t>> id_exmode_vec_map; // id と 書き込むデータのベクタのマップ
    const auto& port_list = ex_port_indice_[Addr::series()];
    for (auto port : port_list) { // アドレスのベクタと，データのベクタの並びは対応している必要があるので，同一のループで作成する．
        switch ( port ) {
            case 1 : exmode_addr_list.push_back(Addr::external_port_mode_1 ); break;
            case 2 : exmode_addr_list.push_back(Addr::external_port_mode_2 ); break;
            case 3 : exmode_addr_list.push_back(Addr::external_port_mode_3 ); break;
            case 4 : exmode_addr_list.push_back(Addr::external_port_mode_4 ); break;
            default: /*ここに来たらエラ-*/ ROS_STOP("Unknown ExternalPortIndex");
        }
        for (auto id : updated_id_mode) if ( series_[id]==Addr::series() ) //updated_id_modeがex_portを持つ妥当なサーボのIDを持つことは，CallbackExternalPortで担保されている
            id_exmode_vec_map[id].push_back( export_w_[id][port].mode ); //export_w_がすべてのID全てのportに対して妥当な値を持っていることは初期化時に担保する
    }
    if ( id_exmode_vec_map.empty() ) return; // 書き込むデータがない場合は即時return
    //*SyncWriteでまとめて書き込み
    parent_.SyncWrite_log(exmode_addr_list, id_exmode_vec_map, verbose_write_);
    SyncReadExternalPortMode<Addr>(updated_id_mode); // 書き込んだ後に読み込みを行うことで、書き込みが成功したかどうかを確認する
}

template <> void DynamixelHandler::ExternalPort::SyncWriteExternalPortData(unordered_set<id_t> updated_id_data){
    SyncWriteExternalPortData<AddrX>(updated_id_data);
    SyncWriteExternalPortData<AddrP>(updated_id_data);
    SyncWriteExternalPortData<AddrPro>(updated_id_data);
}
template <typename Addr> void DynamixelHandler::ExternalPort::SyncWriteExternalPortData(unordered_set<id_t> updated_id_data){
    //* 書き込みに必要な変数を用意
    vector<DynamixelAddress> exdata_addr_list;  // 書き込むコマンドのアドレスのベクタ
    map<uint8_t, vector<int64_t>> id_exdata_vec_map; // id と 書き込むデータのベクタのマップ
    const auto& port_list = ex_port_indice_[Addr::series()];
    for (auto port : port_list) { // アドレスのベクタと，データのベクタの並びは対応している必要があるので，同一のループで作成する．
        switch ( port ) {
            case 1 : exdata_addr_list.push_back(Addr::external_port_data_1 ); break;
            case 2 : exdata_addr_list.push_back(Addr::external_port_data_2 ); break;
            case 3 : exdata_addr_list.push_back(Addr::external_port_data_3 ); break;
            case 4 : exdata_addr_list.push_back(Addr::external_port_data_4 ); break;
            default: /*ここに来たらエラ-*/ ROS_STOP("Unknown ExternalPortIndex");
        }
        for (auto id : updated_id_data) if ( series_[id]==Addr::series() ) //updated_id_dataがex_portを持つ妥当なサーボのIDを持つことは，CallbackExternalPortで担保されている
            id_exdata_vec_map[id].push_back( clamp<uint16_t>(export_w_[id][port].data, 0, 1) ); //export_w_がすべてのID全てのportに対して妥当な値を持っていることは初期化時に担保する
    }
    if ( id_exdata_vec_map.empty() ) return; // 書き込むデータがない場合は即時return
    //*SyncWriteでまとめて書き込み
    parent_.SyncWrite_log(exdata_addr_list, id_exdata_vec_map, verbose_write_);
}


template <> double DynamixelHandler::ExternalPort::SyncReadExternalPortMode(unordered_set<uint8_t> id_set){
    map<DynamixelSeries, size_t> num;
    for (auto id : id_set) num[series_[id]]++;
    double suc_rate_X = SyncReadExternalPortMode<AddrX>(id_set);
    double suc_rate_P = SyncReadExternalPortMode<AddrP>(id_set);
    double suc_rate_Pro = SyncReadExternalPortMode<AddrPro>(id_set);
    auto num_all = num[SERIES_X] + num[SERIES_P] + num[SERIES_PRO];
    return (num_all==0) ? 1.0 : (suc_rate_X * num[SERIES_X] + suc_rate_P * num[SERIES_P] + suc_rate_Pro * num[SERIES_PRO]) / num_all;
}
template <typename Addr> double DynamixelHandler::ExternalPort::SyncReadExternalPortMode(unordered_set<uint8_t> id_set){
    //* 読み込みに必要な変数を用意
    vector<DynamixelAddress> exmode_addr_list;
    const auto& port_list = ex_port_indice_[Addr::series()];
    for (auto p : port_list) switch ( p ) {
        case 1 : exmode_addr_list.push_back(Addr::external_port_mode_1 ); break;
        case 2 : exmode_addr_list.push_back(Addr::external_port_mode_2 ); break;
        case 3 : exmode_addr_list.push_back(Addr::external_port_mode_3 ); break;
        case 4 : exmode_addr_list.push_back(Addr::external_port_mode_4 ); break;
        default: /*ここに来たらエラ-*/ ROS_STOP("Unknown ExternalPortIndex");
    }

    vector<uint8_t> target_id_list;
    for (int id : id_set) if ( series_[id]==Addr::series() ) target_id_list.push_back(id);
    if ( target_id_list.empty() ) return 1.0; // 読み込むデータがない場合は即時return

    auto id_exmode_vec_map = parent_.SyncRead_log(exmode_addr_list, target_id_list, verbose_read_, verbose_read_err_);
    const int N_total = target_id_list.size();
    const int N_suc   = id_exmode_vec_map.size();
    // export_r_に反映
    StateLock lock(parent_.mutex_state_);
    for ( size_t i = 0; i < port_list.size(); i++) {
        for (const auto& [id, data_int] : id_exmode_vec_map)
            export_r_[id][port_list[i]].mode = data_int[i];
    }
    return N_suc/(double)N_total;
}

template <> double DynamixelHandler::ExternalPort::SyncReadExternalPortData(unordered_set<uint8_t> id_set){
    map<DynamixelSeries, size_t> num;
    for (auto id : id_set) num[series_[id]]++;
    double suc_rate_X = SyncReadExternalPortData<AddrX>(id_set);
    double suc_rate_P = SyncReadExternalPortData<AddrP>(id_set);
    double suc_rate_Pro = SyncReadExternalPortData<AddrPro>(id_set);
    auto num_all = num[SERIES_X] + num[SERIES_P] + num[SERIES_PRO];
    return (num_all==0) ? 1.0 : (suc_rate_X * num[SERIES_X] + suc_rate_P * num[SERIES_P] + suc_rate_Pro * num[SERIES_PRO]) / num_all;
}
template <typename Addr> double DynamixelHandler::ExternalPort::SyncReadExternalPortData(unordered_set<uint8_t> id_set){
    //* 読み込みに必要な変数を用意
    vector<DynamixelAddress> exdata_addr_list;
    const auto& port_list = ex_port_indice_[Addr::series()];
    for (auto p : port_list) switch ( p ) {
        case 1 : exdata_addr_list.push_back(Addr::external_port_data_1 ); break;
        case 2 : exdata_addr_list.push_back(Addr::external_port_data_2 ); break;
        case 3 : exdata_addr_list.push_back(Addr::external_port_data_3 ); break;
        case 4 : exdata_addr_list.push_back(Addr::external_port_data_4 ); break;
        default: /*ここに来たらエラ-*/ ROS_STOP("Unknown ExternalPortIndex");
    }

    vector<uint8_t> target_id_list;
    for (int id : id_set) if ( series_[id]==Addr::series() ) target_id_list.push_back(id);
    if ( target_id_list.empty() ) return 1.0; // 読み込むデータがない場合は即時return

    auto id_exdata_vec_map = parent_.SyncRead_log(exdata_addr_list, target_id_list, verbose_read_, verbose_read_err_);
    const int N_total = target_id_list.size();
    const int N_suc   = id_exdata_vec_map.size();
    // export_r_に反映
    StateLock lock(parent_.mutex_state_);
    for ( size_t i = 0; i < port_list.size(); i++) {
        for (const auto& [id, data_int] : id_exdata_vec_map)
            export_r_[id][port_list[i]].data = data_int[i];
    }
    return N_suc/(double)N_total;
}

DynamixelHandler::ExternalPort::ExternalPort(DynamixelHandler& parent) : parent_(parent) { // 本体を参照で保持, constを付けるとdyn_comm_
    ROS_INFO(" < Initializing External Port function ...   >");

    parent_.get_parameter_or("option/external_port.pub_ratio/mode", pub_ratio_mode_, 100u);
    parent_.get_parameter_or("option/external_port.pub_ratio/data", pub_ratio_data_, 10u);
    parent_.get_parameter_or("option/external_port.verbose/callback", verbose_callback_, false);
    parent_.get_parameter_or("option/external_port.verbose/write"   , verbose_write_   , false);
    parent_.get_parameter_or("option/external_port.verbose/read.raw", verbose_read_    , false);
    parent_.get_parameter_or("option/external_port.verbose/read.err", verbose_read_err_, false);

    Initialize(set<id_t>{});

    pub_ex_port_ = parent_.create_publisher<DxlExternalPort>("dynamixel/external_port/read", 4);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = parent_.cbg_command_;
    sub_ex_port_ = parent_.create_subscription<DxlExternalPort>("dynamixel/external_port/write", 4, bind(&DynamixelHandler::ExternalPort::CallbackExternalPort, this, _1), sub_options);

    ROS_INFO(" < ... External Port function is initialized >");
}
DynamixelHandler::ExternalPort::~ExternalPort(){ // デストラクタ,  終了処理を行う

}

void DynamixelHandler::ExternalPort::Initialize(const set<id_t>& cached_ids){
    static auto& id_set_ = parent_.id_set_;
    static auto& model_ = parent_.model_;
    unordered_set<id_t> added_ids;
    for (auto id : id_set_) if ( !is_in(id, cached_ids) && has_external_port(model_[id]) ) added_ids.insert(id);
    static const auto is_ok = [this]() { if(!rclcpp::ok()) ROS_STOP("Failed to initial read"); rsleep(50); };
    while (SyncReadExternalPortMode(added_ids)<1.0-1e-6) { is_ok(); ROS_INFO_T(5000, "    Reading ex. port modes...");} 
    while (SyncReadExternalPortData(added_ids)<1.0-1e-6) { is_ok(); ROS_INFO_T(5000, "    Reading ex. port data...");} 
    for (auto id : added_ids) export_w_[id] = export_r_[id];
}

void DynamixelHandler::ExternalPort::MainProcess(){
    static auto& ping_err_ = parent_.ping_err_;
    static auto& id_set_ = parent_.id_set_;
    static auto id_cache = id_set_;
    static int cnt = -1; cnt++;

    double success_rate = 0;
    unordered_set<uint8_t> target_id_set;
    {
        StateLock lock(parent_.mutex_state_);

        if ( id_cache != id_set_ ) Initialize( exchange(id_cache, id_set_) );

        SyncWriteExternalPortMode(updated_id_export_.mode);
        updated_id_export_.mode.clear();
        SyncWriteExternalPortData(updated_id_export_.data);
        updated_id_export_.data.clear();
        
        for (auto id : id_set_) if ( !touched_port_export_[id].empty() && ping_err_[id]==0 ) target_id_set.insert(id);
    }
    if ( pub_ratio_mode_ && cnt % pub_ratio_mode_ == 0 )
        success_rate += SyncReadExternalPortMode(target_id_set);
    if ( pub_ratio_data_ && cnt % pub_ratio_data_ == 0 )
        success_rate += SyncReadExternalPortData(target_id_set);
    if ( success_rate > 0.0 )
        BroadcastExternalPort();
}
