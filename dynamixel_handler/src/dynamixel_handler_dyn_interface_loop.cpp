#include "dynamixel_handler.hpp"
#include "myUtils/formatting_output.hpp"
#include "myUtils/logging_like_ros1.hpp"
#include "myUtils/make_iterator_convenient.hpp" // enum のインクリメントと， is_in 関数の実装

template <typename Container>
vector<uint8_t> DynamixelHandler::id_filter(const Container& id_set, series_t series){
    vector<id_t> id_list;
    for (auto&& id : id_set) if ( is_in( id, id_set_ ) && series_[id] == series) id_list.push_back(id);
    return id_list;
}
template vector<uint8_t> DynamixelHandler::id_filter<set<uint8_t>>(const set<uint8_t>& id_set, series_t series);
template vector<uint8_t> DynamixelHandler::id_filter<unordered_set<uint8_t>>(const unordered_set<uint8_t>& id_set, series_t series);

void DynamixelHandler::SyncWrite_log(
    const DynamixelAddress& addr, const vector<id_t>& id_list, const vector<int64_t>& data_list, bool verbose
){
    if (id_list.size() != data_list.size()) ROS_STOP("id_list.size() != data_list.size()");
    map <id_t, vector<int64_t>> id_data_vec_map;
    for (size_t i=0; i<id_list.size(); i++) id_data_vec_map[id_list[i]].push_back(data_list[i]);
    SyncWrite_log({addr}, id_data_vec_map, verbose);
}
void DynamixelHandler::SyncWrite_log(
    const vector<DynamixelAddress>& addr_list, const map<id_t, vector<int64_t>>& id_data_vec_map, bool verbose
){
    if ( verbose ) ROS_INFO_STREAM( "  '" << id_data_vec_map.size() << "' servo(s) will be written"
                                    << control_table_layout(width_log_, id_data_vec_map, addr_list) );
    dyn_comm_.SyncWrite(addr_list, id_data_vec_map); fflush(stdout);
}

map<uint8_t, int64_t> DynamixelHandler::SyncRead_log(
    const DynamixelAddress & addr, const vector<id_t>& id_list, bool verbose, bool verbose_err
){ 
    auto id_data_vec_map = SyncRead_log(vector<DynamixelAddress>({addr}), id_list, verbose, verbose_err);
    map<id_t, int64_t> id_data_map;
    for (const auto& [id, data_vec] : id_data_vec_map) id_data_map[id] = data_vec[0];
    return id_data_map;
}
map<uint8_t, vector<int64_t>> DynamixelHandler::SyncRead_log(
    const vector<DynamixelAddress>& addr_list, const vector<id_t>& id_list, bool verbose, bool verbose_err
){
    const auto id_data_vec_map = ( use_fast_read_ ) // fast read を使うかどうか． 途中で切り替えるとtimeout後に来るデータによってSyncReadが何度も失敗するので注意
        ? dyn_comm_.SyncRead_fast(addr_list, id_list)
        : dyn_comm_.SyncRead     (addr_list, id_list); fflush(stdout);
    const bool is_timeout_  = dyn_comm_.timeout_last_read();
    const bool is_comm_err_ = dyn_comm_.comm_error_last_read();
    //* 通信エラーの表示
    if ( verbose_err ) if ( is_timeout_ || is_comm_err_ ) {
        vector<id_t> failed_id_list;
        for ( auto id : id_list ) if ( !id_data_vec_map.count(id) ) failed_id_list.push_back(id);
        ROS_WARN_STREAM( "  '" << id_list.size() - id_data_vec_map.size() << "' servo(s) failed to read" 
                        << (is_timeout_ ? " (time out)" : " (some kind packet error)") << "\n"
                        << id_list_layout(failed_id_list) << "\n");
    }
    //* id_data_vec_mapの中身を確認
    if ( verbose ) if ( id_data_vec_map.size()>0 )
        ROS_INFO_STREAM( "'" << id_data_vec_map.size() << "' servo(s) are read"
                        << control_table_layout(width_log_, id_data_vec_map, addr_list) );
    return id_data_vec_map;
}

//* Main loop 内で使う全モータへの一括読み書き関数たち

/**
 * @func SyncWriteGoal
 * @brief 指定した範囲のコマンド値を書き込む
 * @param goal_indice_write 書き込むコマンドのEnumのリスト
*/
template <> void DynamixelHandler::SyncWriteGoal(set<GoalIndex> goal_indice_write, const unordered_set<id_t>& updated_id_goal){
    SyncWriteGoal<AddrX>(goal_indice_write, updated_id_goal);
    SyncWriteGoal<AddrP>(goal_indice_write, updated_id_goal);
    SyncWriteGoal<AddrPro>(goal_indice_write, updated_id_goal);
}
template <typename Addr> void DynamixelHandler::SyncWriteGoal(set<GoalIndex> goal_indice_write, const unordered_set<id_t>& updated_id_goal){
    if ( goal_indice_write.empty() ) return; // 空なら即時return
    //* 書き込む範囲のイテレータを取得, 分割書き込みが有効な場合書き込む範囲を1つ目のみに制限,残りは再帰的に処理する．
    auto [start,end] = minmax_element(goal_indice_write.begin(), goal_indice_write.end()); 
    /*flagによる分割*/ if ( use_split_write_ ) end = start; // 分割書き込みが有効な場合は書き込む範囲を1つ目のみに制限
    //* 書き込みに必要な変数を用意
    vector<DynamixelAddress> goal_addr_list;  // 書き込むコマンドのアドレスのベクタ
    map<id_t, vector<int64_t>> id_goal_vec_map; // id と 書き込むデータのベクタのマップ
    for (GoalIndex goal = *start; goal <= *end; goal++) { // アドレスのベクタと，データのベクタの並びは対応している必要があるので，同一のループで作成する．
        switch (goal) {
            case GOAL_PWM      : goal_addr_list.push_back(Addr::goal_pwm            ); break;
            case GOAL_CURRENT  : goal_addr_list.push_back(Addr::goal_current        ); break;
            case GOAL_VELOCITY : goal_addr_list.push_back(Addr::goal_velocity       ); break;
            case PROFILE_ACC   : goal_addr_list.push_back(Addr::profile_acceleration); break;
            case PROFILE_VEL   : goal_addr_list.push_back(Addr::profile_velocity    ); break;
            case GOAL_POSITION : goal_addr_list.push_back(Addr::goal_position       ); break;
            default: /*ここに来たらエラ-*/ ROS_STOP("Unknown GoalIndex");
        }
        const auto& addr = goal_addr_list.back();
        for (auto id : id_filter(updated_id_goal, Addr::series()))
            id_goal_vec_map[id].push_back( addr.val2pulse( goal_w_[id][goal], model_[id] ) );
    }
    if ( id_goal_vec_map.empty() ) return; // 書き込むデータがない場合は即時return
    //*SyncWriteでまとめて書き込み
    SyncWrite_log(goal_addr_list, id_goal_vec_map, verbose_["w_goal"] );
    //*再帰的に処理, use_split_write_=falseの場合は全て削除されるので,再帰しない
    goal_indice_write.erase(start, ++end);  // 今回書き込んだ範囲を消去
    SyncWriteGoal<Addr>(goal_indice_write, updated_id_goal);
}

/** 
 * @func SyncWriteGain
 */
template <> void DynamixelHandler::SyncWriteGain(set<GainIndex> gain_indice_write, const unordered_set<id_t>& updated_id_gain){
    SyncWriteGain<AddrX>(gain_indice_write, updated_id_gain);
    SyncWriteGain<AddrP>(gain_indice_write, updated_id_gain);
    SyncWriteGain<AddrPro>(gain_indice_write, updated_id_gain);
}
template <typename Addr> void DynamixelHandler::SyncWriteGain(set<GainIndex> gain_indice_write, const unordered_set<id_t>& updated_id_gain){
    if ( gain_indice_write.empty() ) return; // 空なら即時return
    //* 書き込む範囲のイテレータを取得, 分割書き込みが有効な場合書き込む範囲を1つ目のみに制限,残りは再帰的に処理する．
    auto [start,end] = minmax_element(gain_indice_write.begin(), gain_indice_write.end());
    /*flagによる分割*/  if ( use_split_write_ ) end = start; // 分割書き込みが有効な場合は書き込む範囲を1つ目のみに制限
    /*データ数による分割*/ if ( updated_id_gain.size() * (*end-*start+1) > 12*_num_gain ) end = start; // 一度に書き込むデータ数が多い場合は分割する
    //* 書き込みに必要な変数を用意
    vector<DynamixelAddress> gain_addr_list;  // 書き込むコマンドのアドレスのベクタ
    map<id_t, vector<int64_t>> id_gain_vec_map; // id と 書き込むデータのベクタのマップ
    for (GainIndex gain = *start; gain <= *end; gain++) { // アドレスのベクタと，データのベクタの並びは対応している必要があるので，同一のループで作成する．
        switch (gain) {
            case VELOCITY_I_GAIN     : gain_addr_list.push_back(Addr::velocity_i_gain     ); break;
            case VELOCITY_P_GAIN     : gain_addr_list.push_back(Addr::velocity_p_gain     ); break;
            case POSITION_D_GAIN     : gain_addr_list.push_back(Addr::position_d_gain     ); break;
            case POSITION_I_GAIN     : gain_addr_list.push_back(Addr::position_i_gain     ); break;
            case POSITION_P_GAIN     : gain_addr_list.push_back(Addr::position_p_gain     ); break;
            case FEEDFORWARD_ACC_GAIN: gain_addr_list.push_back(Addr::feedforward_2nd_gain); break;
            case FEEDFORWARD_VEL_GAIN: gain_addr_list.push_back(Addr::feedforward_1st_gain); break;
            default: /*ここに来たらエラ-*/ ROS_STOP("Unknown GainIndex");
        }
        const auto& addr = gain_addr_list.back();
        for (auto id : id_filter(updated_id_gain, Addr::series()))
            id_gain_vec_map[id].push_back( addr.val2pulse( gain_w_[id][gain], model_[id] ) );
    }
    if ( id_gain_vec_map.empty() ) return; // 書き込むデータがない場合は即時return
    //*SyncWriteでまとめて書き込み
    SyncWrite_log(gain_addr_list, id_gain_vec_map, verbose_["w_gain"]);
    //*再帰的に処理
    gain_indice_write.erase(start, ++end);  // 今回書き込んだ範囲を消去
    SyncWriteGain<Addr>(gain_indice_write, updated_id_gain);
}

/**
 * @func SyncWriteLimit
 * @brief 制限値をすべて書き込む, ROMに書き込むのでトルクをOFFにする
 * @param limit_indice_write 書き込む制限値のEnumのset
 */
template <> void DynamixelHandler::SyncWriteLimit(set<LimitIndex> limit_indice_write, const unordered_set<id_t>& updated_id_limit){
    vector<id_t> id_list_x = id_filter(updated_id_limit, SERIES_X);
    vector<id_t> id_list_p = id_filter(updated_id_limit, SERIES_P);
    vector<id_t> id_list_pro = id_filter(updated_id_limit, SERIES_PRO);
    if ( !id_list_x.empty() ) SyncWrite_log(AddrX::torque_enable, id_list_x, vector<int64_t>(id_list_x.size(), TORQUE_DISABLE), verbose_["w_limit"]);
    if ( !id_list_p.empty() ) SyncWrite_log(AddrP::torque_enable, id_list_p, vector<int64_t>(id_list_p.size(), TORQUE_DISABLE), verbose_["w_limit"]);
    if ( !id_list_pro.empty() ) SyncWrite_log(AddrPro::torque_enable, id_list_pro, vector<int64_t>(id_list_pro.size(), TORQUE_DISABLE), verbose_["w_limit"]);
    SyncWriteLimit<AddrX>(limit_indice_write, updated_id_limit);
    SyncWriteLimit<AddrP>(limit_indice_write, updated_id_limit);
    SyncWriteLimit<AddrPro>(limit_indice_write, updated_id_limit);
    if ( !id_list_x.empty() ) SyncWrite_log(AddrX::torque_enable, id_list_x, vector<int64_t>(id_list_x.size(), TORQUE_ENABLE), verbose_["w_limit"]);
    if ( !id_list_p.empty() ) SyncWrite_log(AddrP::torque_enable, id_list_p, vector<int64_t>(id_list_p.size(), TORQUE_ENABLE), verbose_["w_limit"]);
    if ( !id_list_pro.empty() ) SyncWrite_log(AddrPro::torque_enable, id_list_pro, vector<int64_t>(id_list_pro.size(), TORQUE_ENABLE), verbose_["w_limit"]);
}
template <typename Addr> void DynamixelHandler::SyncWriteLimit(set<LimitIndex> limit_indice_write, const unordered_set<id_t>& updated_id_limit){
    if ( limit_indice_write.empty() ) return; // 空なら即時return
    //* 書き込む範囲のイテレータを取得, 分割書き込みが有効な場合書き込む範囲を1つ目のみに制限,残りは再帰的に処理する．
    auto [start,end] = minmax_element(limit_indice_write.begin(), limit_indice_write.end()); 
    /*flagによる分割*/ if ( use_split_write_ ) end = start; // 分割書き込みを常に有効にする．
    /*データ数による分割*/ if ( updated_id_limit.size() * (*end-*start+1) > 12*_num_limit ) end = start; // 一度に書き込むデータ数が多い場合は分割する
    //* 書き込みに必要な変数を用意
    vector<DynamixelAddress> limit_addr_list;  // 書き込むコマンドのアドレスのベクタ
    map<id_t, vector<int64_t>> id_limit_vec_map; // id と 書き込むデータのベクタのマップ
    for (LimitIndex limit = *start; limit <= *end; limit++) { // アドレスのベクタと，データのベクタの並びは対応している必要があるので，同一のループで作成する．
        switch (limit) {
            case TEMPERATURE_LIMIT : limit_addr_list.push_back(Addr::temperature_limit ); break;
            case MAX_VOLTAGE_LIMIT : limit_addr_list.push_back(Addr::max_voltage_limit ); break;
            case MIN_VOLTAGE_LIMIT : limit_addr_list.push_back(Addr::min_voltage_limit ); break;
            case PWM_LIMIT         : limit_addr_list.push_back(Addr::pwm_limit         ); break;
            case CURRENT_LIMIT     : limit_addr_list.push_back(Addr::current_limit     ); break;
            case ACCELERATION_LIMIT: limit_addr_list.push_back(Addr::acceleration_limit); break;
            case VELOCITY_LIMIT    : limit_addr_list.push_back(Addr::velocity_limit    ); break;
            case MAX_POSITION_LIMIT: limit_addr_list.push_back(Addr::max_position_limit); break;
            case MIN_POSITION_LIMIT: limit_addr_list.push_back(Addr::min_position_limit); break;
            default: /*ここに来たらエラ-*/ ROS_STOP("Unknown LimitIndex");
        }
        const auto& addr = limit_addr_list.back();
        for (auto id : id_filter(updated_id_limit, Addr::series()))
            id_limit_vec_map[id].push_back( addr.val2pulse( limit_w_[id][limit], model_[id] ) );
    }
    if ( id_limit_vec_map.empty() ) return; // 書き込むデータがない場合は即時return
    //*SyncWriteでまとめて書き込み
    SyncWrite_log(limit_addr_list, id_limit_vec_map, verbose_["w_limit"]);
    //*再帰的に処理, use_split_write_=falseの場合は全て削除されるので,再帰しない
    limit_indice_write.erase(start, ++end);  // 今回書き込んだ範囲を消去
    SyncWriteLimit<Addr>(limit_indice_write, updated_id_limit);
}

using std::chrono::system_clock;
using std::chrono::duration_cast;
using std::chrono::microseconds;

/**
 * @func SyncReadPresent
 * @brief 指定した範囲の状態値を読み込む
 * @param present_indice_read 読み込む状態値のEnumのリスト
 * @return 読み取りの成功率, なんで再帰で頑張って実装してるんだろう．．．
*/
template <> tuple<double, uint8_t> DynamixelHandler::SyncReadPresent(set<PresentIndex> present_indice_read, const set<id_t>& id_set ){
    auto [suc_rate_X, num_x] = SyncReadPresent<AddrX>(present_indice_read, id_set);
    auto [suc_rate_P, num_p] = SyncReadPresent<AddrP>(present_indice_read, id_set);
    auto [suc_rate_PRO, num_pro] = SyncReadPresent<AddrPro>(present_indice_read, id_set);
    for ( auto id : id_filter(id_set, SERIES_UNKNOWN) ) {
        present_r_[id][PRESENT_PWM          ] = goal_w_[id][GOAL_PWM     ];
        present_r_[id][PRESENT_CURRENT      ] = goal_w_[id][GOAL_CURRENT ];
        present_r_[id][PRESENT_VELOCITY     ] = goal_w_[id][GOAL_VELOCITY];
        present_r_[id][PRESENT_POSITION     ] = goal_w_[id][GOAL_POSITION];
        present_r_[id][PRESENT_INPUT_VOLTAGE] = 25;
        present_r_[id][PRESENT_TEMPERATURE  ] = 25;
    }
    auto num_all = num_x + num_p + num_pro;
    return {(num_all)==0 ? 1.0 : (suc_rate_X*num_x + suc_rate_P*num_p + suc_rate_PRO*num_pro) / (num_all), num_all};
}
template <typename Addr> tuple<double, uint8_t> DynamixelHandler::SyncReadPresent(set<PresentIndex> present_indice_read, const set<id_t>& id_set ){
    vector<id_t> target_id_list = id_filter(id_set, Addr::series());
    if ( target_id_list.empty() ) return {1.0, 0}; // 読み込むデータがない場合は即時return
    //* 読み込む範囲のstate_addr_listのインデックスを取得
    auto [start, end] = minmax_element(present_indice_read.begin(), present_indice_read.end());
    if ( present_indice_read.empty() ) return {1.0, target_id_list.size()}; // ↑との対称性のためあえてminmax_elementの後に書いている, 再帰するときのreturnはここ．
    /*flagによる分割*/ if ( use_split_read_ ) end = start; // 分割読み込みが有効な場合は読み込む範囲を1つ目のみに制限
    //* 読み込みに必要な変数を用意
    vector<DynamixelAddress> state_addr_list;
    for (PresentIndex st=*start; st<=*end; st++) switch (st) {
        case PRESENT_PWM          : state_addr_list.push_back(Addr::present_pwm          ); break; 
        case PRESENT_CURRENT      : state_addr_list.push_back(Addr::present_current      ); break; 
        case PRESENT_VELOCITY     : state_addr_list.push_back(Addr::present_velocity     ); break;    
        case PRESENT_POSITION     : state_addr_list.push_back(Addr::present_position     ); break;   
        case VELOCITY_TRAJECTORY  : state_addr_list.push_back(Addr::velocity_trajectory  ); break;   
        case POSITION_TRAJECTORY  : state_addr_list.push_back(Addr::position_trajectory  ); break;  
        case PRESENT_INPUT_VOLTAGE: state_addr_list.push_back(Addr::present_input_voltage); break; 
        case PRESENT_TEMPERATURE  : state_addr_list.push_back(Addr::present_temperature  ); break;
        default: /*ここに来たらエラ-*/ ROS_STOP("Unknown PresentIndex");
    }
    // SyncReadでまとめて読み込み
    const auto id_st_vec_map = SyncRead_log(state_addr_list, target_id_list, verbose_["r_present"], verbose_["r_present_err"]);
    const int N_total = target_id_list.size();
    const int N_suc   = id_st_vec_map.size();
    //* present_r_に反映
    const unsigned int num_state_now  = *end-*start+1;
    for ( size_t i = 0; i < num_state_now; i++ ) {
        const auto addr = state_addr_list[i];
        for (const auto& [id, data_int] : id_st_vec_map)
            present_r_[id][*start+i] = addr.pulse2val( data_int[i], model_[id]);
    }
    // 今回読み込んだ範囲を消去して残りを再帰的に処理, use_split_read_=falseの場合は全て削除されるので,再帰しない
    present_indice_read.erase(start, ++end); // 今回読み込んだ範囲を消去
    const unsigned int num_state_next = present_indice_read.size();
    auto [suc_rate, tmp] = SyncReadPresent<Addr>(present_indice_read, id_set); // 2つめの要素はN_totalと等しいので利用しない．
    return {       suc_rate        *num_state_next / (num_state_now+num_state_next) 
            + N_suc/(double)N_total*num_state_now /  (num_state_now+num_state_next), N_total};
}

/**
 * @func SyncReadHardwareError
 * @brief ハードウェアエラーを読み込む
 * @return 読み取りの成功率
*/
template <> tuple<double, uint8_t> DynamixelHandler::SyncReadHardwareErrors(const set<id_t>& id_set){
    auto [suc_rate_X, num_x] = SyncReadHardwareErrors<AddrX>(id_set);
    auto [suc_rate_P, num_p] = SyncReadHardwareErrors<AddrP>(id_set);
    auto [suc_rate_PRO, num_pro] = SyncReadHardwareErrors<AddrPro>(id_set);
    auto num_all = num_x + num_p + num_pro;
    return {(num_all)==0 ? 1.0 : (suc_rate_X*num_x + suc_rate_P*num_p + suc_rate_PRO*num_pro) / (num_all), num_all};
}
template <typename Addr> tuple<double, uint8_t> DynamixelHandler::SyncReadHardwareErrors(const set<id_t>& id_set){
    vector<id_t> target_id_list = id_filter(id_set, Addr::series());
    if ( target_id_list.empty() ) return {1.0, 0}; // 読み込むデータがない場合は即時return
    
    auto id_error_map = SyncRead_log(Addr::hardware_error_status, target_id_list, false, false);
    if ( dyn_comm_.timeout_last_read() ) return {0.0, target_id_list.size() }; // 読み込み失敗

    //  hardware_error_とhas_hardware_error_に反映
    bool has_any_error = false;
    for (const auto& [id, error] : id_error_map ){
        hardware_err_[id].fill(false);
        if ((error >> HARDWARE_ERROR_INPUT_VOLTAGE     )& 0b1 ) hardware_err_[id][INPUT_VOLTAGE     ] = true;
        if ((error >> HARDWARE_ERROR_MOTOR_HALL_SENSOR )& 0b1 ) hardware_err_[id][MOTOR_HALL_SENSOR ] = true;
        if ((error >> HARDWARE_ERROR_OVERHEATING       )& 0b1 ) hardware_err_[id][OVERHEATING       ] = true;
        if ((error >> HARDWARE_ERROR_MOTOR_ENCODER     )& 0b1 ) hardware_err_[id][MOTOR_ENCODER     ] = true;
        if ((error >> HARDWARE_ERROR_ELECTRONICAL_SHOCK)& 0b1 ) hardware_err_[id][ELECTRONICAL_SHOCK] = true;
        if ((error >> HARDWARE_ERROR_OVERLOAD          )& 0b1 ) hardware_err_[id][OVERLOAD          ] = true;
        has_hardware_error_[id] = std::any_of(hardware_err_[id].begin(), hardware_err_[id].end(), [](bool b){return b;});
        if ( has_hardware_error_[id] ) has_any_error = true;
    }

    // コンソールへの表示
    if ( verbose_["r_hwerr"] ) if ( has_any_error ) {
        ROS_WARN( "Hardware error are detected");
        for (auto id : target_id_list) {
            if (hardware_err_[id][INPUT_VOLTAGE     ]) ROS_WARN ("  * servo ID [%d] has INPUT_VOLTAGE error"     ,id);
            if (hardware_err_[id][MOTOR_HALL_SENSOR ]) ROS_ERROR("  * servo ID [%d] has MOTOR_HALL_SENSOR error" ,id);
            if (hardware_err_[id][OVERHEATING       ]) ROS_ERROR("  * servo ID [%d] has OVERHEATING error"       ,id);
            if (hardware_err_[id][MOTOR_ENCODER     ]) ROS_ERROR("  * servo ID [%d] has MOTOR_ENCODER error"     ,id);
            if (hardware_err_[id][ELECTRONICAL_SHOCK]) ROS_ERROR("  * servo ID [%d] has ELECTRONICAL_SHOCK error",id);
            if (hardware_err_[id][OVERLOAD          ]) ROS_ERROR("  * servo ID [%d] has OVERLOAD error"          ,id);
        }
    }
    return {id_error_map.size()/double(target_id_list.size()), target_id_list.size()};
}

/**
 * @func SyncReadGain
 * @brief ゲインをすべて読み込む
 * @return 読み取りの成功率
*/
template <> tuple<double, uint8_t> DynamixelHandler::SyncReadGain(set<GainIndex> gain_indice_read, const set<id_t>& id_set){
    auto [suc_rate_X, num_x] = SyncReadGain<AddrX>(gain_indice_read, id_set);
    auto [suc_rate_P, num_p] = SyncReadGain<AddrP>(gain_indice_read, id_set);
    auto [suc_rate_PRO, num_pro] = SyncReadGain<AddrPro>(gain_indice_read, id_set);
    for (auto id : id_filter(id_set, SERIES_UNKNOWN)) gain_r_[id] = gain_w_[id]; // dummy servo の場合
    auto num_all = num_x + num_p + num_pro;
    return {(num_all)==0 ? 1.0 : (suc_rate_X*num_x + suc_rate_P*num_p + suc_rate_PRO*num_pro) / (num_all), num_all};
}
template <typename Addr> tuple<double, uint8_t> DynamixelHandler::SyncReadGain(set<GainIndex> gain_indice_read, const set<id_t>& id_set){
    //* 今回読み込むIDを取得
    vector<id_t> target_id_list = id_filter(id_set, Addr::series());
    if ( target_id_list.empty() ) return {1.0, 0}; // 読み込むデータがない場合は即時return
    //* 読み込む範囲のgain_addr_listのインデックスを取得
    auto [start, end] = minmax_element(gain_indice_read.begin(), gain_indice_read.end());
    if ( gain_indice_read.empty() ) return {1.0, target_id_list.size()}; // ↑との対称性のためあえてminmax_elementの後に書いている
    /*flagによる分割*/  if ( use_split_read_ ) end = start; // 分割読み込みを常に有効にする．
    /*データ数による分割*/ if ( target_id_list.size() * (*end-*start+1) > 12*_num_gain ) end = start; // 分割読み込みを常に有効にする．
    //* 読み込みに必要な変数を用意
    vector<DynamixelAddress> gain_addr_list;
    for (GainIndex g=*start; g<=*end; g++) switch ( g ) {
        case VELOCITY_I_GAIN     : gain_addr_list.push_back(Addr::velocity_i_gain     ); break;
        case VELOCITY_P_GAIN     : gain_addr_list.push_back(Addr::velocity_p_gain     ); break;
        case POSITION_D_GAIN     : gain_addr_list.push_back(Addr::position_d_gain     ); break;
        case POSITION_I_GAIN     : gain_addr_list.push_back(Addr::position_i_gain     ); break;
        case POSITION_P_GAIN     : gain_addr_list.push_back(Addr::position_p_gain     ); break;
        case FEEDFORWARD_ACC_GAIN: gain_addr_list.push_back(Addr::feedforward_2nd_gain); break;
        case FEEDFORWARD_VEL_GAIN: gain_addr_list.push_back(Addr::feedforward_1st_gain); break;
        default: /*ここに来たらエラ-*/ ROS_STOP("Unknown GainIndex");
    }

    auto id_gain_vec_map = SyncRead_log(gain_addr_list, target_id_list, verbose_["r_gain"], verbose_["r_gain_err"]);
    const int N_total = target_id_list.size();
    const int N_suc   = id_gain_vec_map.size();
    // gain_r_に反映
    const unsigned int num_gain_now  = *end-*start+1;
    for ( size_t i = 0; i < num_gain_now; i++ ) { 
        DynamixelAddress addr = gain_addr_list[i];
        for (const auto& [id, data_int] : id_gain_vec_map)
            gain_r_[id][*start+i] = static_cast<uint16_t>( addr.pulse2val( data_int[i], model_[id] ) ); // pulse2valはdoubleを返すので，uint16_tにキャスト
    }
    // 今回読み込んだ範囲を消去して残りを再帰的に処理, 
    gain_indice_read.erase(start, ++end); // 今回読み込んだ範囲を消去
    const unsigned int num_gain_next = gain_indice_read.size();
    auto [suc_rate, tmp] = SyncReadGain<Addr>(gain_indice_read, id_set);  // 2つめの要素はN_totalと等しいので利用しない．
    return {      suc_rate         *num_gain_next / (num_gain_now+num_gain_next) // 再帰的な意味で前回までの成功率に重しをかけたもの
            + N_suc/(double)N_total*num_gain_now  / (num_gain_now+num_gain_next), N_total};// 今回の成功率に今回読み込んだデータの数を重しとしてかけたもの
}

/**
 * @func SyncReadLimit
 * @brief 制限値をすべて読み込む
 * @return 読み取りの成功率
*/
template <> tuple<double, uint8_t> DynamixelHandler::SyncReadLimit(set<LimitIndex> limit_indice_read, const set<id_t>& id_set){
    auto [suc_rate_X, num_x] = SyncReadLimit<AddrX>(limit_indice_read, id_set);
    auto [suc_rate_P, num_p] = SyncReadLimit<AddrP>(limit_indice_read, id_set);
    auto [suc_rate_PRO, num_pro] = SyncReadLimit<AddrPro>(limit_indice_read, id_set);
    for (auto id : id_filter(id_set, SERIES_UNKNOWN)) limit_r_[id] = limit_w_[id]; // dummy servo の場合
    auto num_all = num_x + num_p + num_pro;
    return {(num_all)==0 ? 1.0 : (suc_rate_X*num_x + suc_rate_P*num_p + suc_rate_PRO*num_pro) / (num_all), num_all};
}
template <typename Addr> tuple<double, uint8_t> DynamixelHandler::SyncReadLimit(set<LimitIndex> limit_indice_read, const set<id_t>& id_set){
    //* 読み読むIDを取得
    vector<id_t> target_id_list = id_filter(id_set, Addr::series());
    if ( target_id_list.empty() ) return {1.0, 0}; // 読み込むデータがない場合は即時return
    //* 読み込む範囲のlimit_addr_listのインデックスを取得
    auto [start, end] = minmax_element(limit_indice_read.begin(), limit_indice_read.end());
    if ( limit_indice_read.empty() ) return {1.0, target_id_list.size()}; // ↑との対称性のためあえてminmax_elementの後に書いている
    /*flagによる分割*/  if ( use_split_read_ ) end = start; // 分割読み込みを常に有効にする．
    /*データ数による分割*/ if ( target_id_list.size() * (*end-*start+1) > 12*_num_limit ) end = start; // 分割読み込みを常に有効にする．
    //* 読み込みに必要な変数を用意
    vector<DynamixelAddress> limit_addr_list;
    for (LimitIndex l=*start; l<=*end; l++) switch ( l ) {
        case TEMPERATURE_LIMIT : limit_addr_list.push_back(Addr::temperature_limit ); break;
        case MAX_VOLTAGE_LIMIT : limit_addr_list.push_back(Addr::max_voltage_limit ); break;
        case MIN_VOLTAGE_LIMIT : limit_addr_list.push_back(Addr::min_voltage_limit ); break;
        case PWM_LIMIT         : limit_addr_list.push_back(Addr::pwm_limit         ); break;
        case CURRENT_LIMIT     : limit_addr_list.push_back(Addr::current_limit     ); break;
        case ACCELERATION_LIMIT: limit_addr_list.push_back(Addr::acceleration_limit); break;
        case VELOCITY_LIMIT    : limit_addr_list.push_back(Addr::velocity_limit    ); break;
        case MAX_POSITION_LIMIT: limit_addr_list.push_back(Addr::max_position_limit); break;
        case MIN_POSITION_LIMIT: limit_addr_list.push_back(Addr::min_position_limit); break;
        default: /*ここに来たらエラ-*/ ROS_STOP("Unknown LimitIndex");
    }

    auto id_limit_vec_map = SyncRead_log(limit_addr_list, target_id_list, verbose_["r_limit"], verbose_["r_limit_err"]);
    const int N_total = target_id_list.size();
    const int N_suc   = id_limit_vec_map.size();
    // limit_r_に反映
    const unsigned int num_limit_now  = *end-*start+1;
    for ( size_t i = 0; i < num_limit_now; i++ ) {
        DynamixelAddress addr = limit_addr_list[i];
        for (const auto& [id, data_int] : id_limit_vec_map)
            limit_r_[id][*start+i] = addr.pulse2val( data_int[i], model_[id] );
    }
    // 今回読み込んだ範囲を消去して残りを再帰的に処理,
    limit_indice_read.erase(start, ++end); // 今回読み込んだ範囲を消去
    const unsigned int num_limit_next = limit_indice_read.size();
    auto [suc_rate, tmp] = SyncReadLimit<Addr>(limit_indice_read, id_set);
    return {       suc_rate        *num_limit_next / (num_limit_now+num_limit_next) 
            + N_suc/(double)N_total*num_limit_now /  (num_limit_now+num_limit_next), N_total};
}

/**
 * @func SyncReadGoal
 * @brief 目標値をすべて読み込む
 * @return 読み取りの成功率
*/
template <> tuple<double, uint8_t> DynamixelHandler::SyncReadGoal(set<GoalIndex> goal_indice_read, const set<id_t>& id_set){
    auto [suc_rate_X, num_x] = SyncReadGoal<AddrX>(goal_indice_read, id_set);
    auto [suc_rate_P, num_p] = SyncReadGoal<AddrP>(goal_indice_read, id_set);
    auto [suc_rate_PRO, num_pro] = SyncReadGoal<AddrPro>(goal_indice_read, id_set);
    for (auto id : id_filter(id_set, SERIES_UNKNOWN)) goal_r_[id] = goal_w_[id]; // dummy servo の場合
    auto num_all = num_x + num_p + num_pro;
    return {(num_all)==0 ? 1.0 : (suc_rate_X*num_x + suc_rate_P*num_p + suc_rate_PRO*num_pro) / (num_all), num_all};
}
template <typename Addr> tuple<double, uint8_t> DynamixelHandler::SyncReadGoal(set<GoalIndex> goal_indice_read, const set<id_t>& id_set){
    //* 読み込むIDを取得
    vector<id_t> target_id_list = id_filter(id_set, Addr::series());
    if ( target_id_list.empty() ) return {1.0, 0}; // 読み込むデータがない場合は即時return
    //* 読み込む範囲のgoal_addr_listのインデックスを取得
    auto [start, end] = minmax_element(goal_indice_read.begin(), goal_indice_read.end());
    if ( goal_indice_read.empty() ) return {1.0, target_id_list.size()}; // ↑との対称性のためあえてminmax_elementの後に書いている
    /*flagによる分割*/ if ( use_split_read_ ) end = start; // 分割読み込みを常に有効にする．
    /*データ数による分割*/ if ( target_id_list.size() * (*end-*start+1) > 12*_num_goal ) end = start; // 分割読み込みを常に有効にする．
    //* 読み込みに必要な変数を用意
    vector<DynamixelAddress> goal_addr_list;
    for (GoalIndex g=*start; g<=*end; g++) switch ( g ) {
        case GOAL_PWM      : goal_addr_list.push_back(Addr::goal_pwm      ); break;
        case GOAL_CURRENT  : goal_addr_list.push_back(Addr::goal_current  ); break;
        case GOAL_VELOCITY : goal_addr_list.push_back(Addr::goal_velocity ); break;
        case PROFILE_ACC   : goal_addr_list.push_back(Addr::profile_acceleration); break;
        case PROFILE_VEL   : goal_addr_list.push_back(Addr::profile_velocity    ); break;
        case GOAL_POSITION : goal_addr_list.push_back(Addr::goal_position       ); break;
        default: /*ここに来たらエラ-*/ ROS_STOP("Unknown GoalIndex");
    }

    auto id_goal_vec_map = SyncRead_log(goal_addr_list, target_id_list, verbose_["r_goal"], verbose_["r_goal_err"]);
    const int N_total = target_id_list.size();
    const int N_suc   = id_goal_vec_map.size();
    // goal_r_に反映
    const unsigned int  num_goal_now  = *end-*start+1;
    for ( size_t i = 0; i < num_goal_now; i++) {
        DynamixelAddress addr = goal_addr_list[i];
        for (const auto& [id, data_int] : id_goal_vec_map)
            goal_r_[id][*start+i] = addr.pulse2val( data_int[i], model_[id] );
    }
    // 今回読み込んだ範囲を消去して残りを再帰的に処理, 
    goal_indice_read.erase(start, ++end); // 今回読み込んだ範囲を消去
    const unsigned int  num_goal_next = goal_indice_read.size();
    auto [suc_rate, tmp] = SyncReadGoal<Addr>(goal_indice_read, id_set);
    return {       suc_rate        *num_goal_next / (num_goal_now+num_goal_next) 
            + N_suc/(double)N_total*num_goal_now /  (num_goal_now+num_goal_next), N_total};
}

// 全てのモータの動作を停止させる．
template <> void DynamixelHandler::StopDynamixels(const set<id_t>& id_set){
    StopDynamixels<AddrX>(id_set);
    StopDynamixels<AddrP>(id_set);
    StopDynamixels<AddrPro>(id_set);
} 
template <typename Addr> void DynamixelHandler::StopDynamixels(const set<id_t>& id_set){
    vector<id_t> target_id_list = id_filter(id_set, Addr::series());
    if ( target_id_list.empty() ) return; // 読み込むデータがない場合は即時return

    ROS_INFO(" %s servo will be stopped",  Addr::series()==SERIES_X ? "X series" 
                                         : Addr::series()==SERIES_P ? "P series" 
                                         : Addr::series()==SERIES_PRO ? "PRO series" : "Unknown");
    if ( do_torque_off_ ) { // ノード停止時の挙動して， do_torque_off_ は do_stop_end_ を包含する．
        dyn_comm_.SyncWrite(Addr::torque_enable, target_id_list, vector<int64_t>(target_id_list.size(), TORQUE_DISABLE));
        ROS_INFO("  Torque of all servo are disabled");
    } else if ( do_stop_end_ ) { // ノード停止時に， サーボの動作を止める．
        // 1. PWM，電流，位置制御につては, bus_watchdogによって自動的に停止するため何もしない．
        // 2. 位置制御系ついては，現在値を書き込むことで停止させる( bus_watchdogによる位置制御の停止にはバグが多いため，利用しない． )
        dyn_comm_.SyncWrite(Addr::goal_position,  // エラーチェックはせず，読み込めたものだけ書き込む．
            dyn_comm_.SyncRead(Addr::present_position, target_id_list));
        ROS_INFO("  Reset goal position for stopping");
        // 3. さらに念のため， トルクを瞬間的にオンオフすることでも停止させる．
        vector<id_t> tq_on_list;
        for (auto id : target_id_list) if ( tq_mode_[id]==TORQUE_ENABLE ) tq_on_list.push_back(id);
        dyn_comm_.SyncWrite(Addr::torque_enable, tq_on_list, vector<int64_t>(tq_on_list.size(), TORQUE_DISABLE));
        dyn_comm_.SyncWrite(Addr::torque_enable, tq_on_list, vector<int64_t>(tq_on_list.size(), TORQUE_ENABLE ));
        ROS_INFO("  Reset torque enable for stopping");
    }
}

template <> void DynamixelHandler::CheckDynamixels(const set<id_t>& id_set){
    CheckDynamixels<AddrX>(id_set);
    CheckDynamixels<AddrP>(id_set);
    CheckDynamixels<AddrPro>(id_set);
}
template <typename Addr> void DynamixelHandler::CheckDynamixels(const set<id_t>& id_set){
    vector<id_t> target_id_list = id_filter(id_set, Addr::series());
    if ( target_id_list.empty() ) return; // 読み込むデータがない場合は即時return

    // bus_watchdogの書き込み．
    SyncWrite_log(Addr::bus_watchdog, target_id_list, vector<int64_t>(target_id_list.size(), 0/*OFF*/), verbose_["w_status"]);
    if ( do_stop_end_ ) { //　恒常的に動くモードの場合はbus_watchdogを有効にし，通信断絶で止まるようにする．
        map<id_t, vector<int64_t>> bus_watchdog_map;
        for (auto id : target_id_list) switch (op_mode_[id]){ 
            case OPERATING_MODE_VELOCITY: [[fallthrough]];
            case OPERATING_MODE_CURRENT : [[fallthrough]];
            case OPERATING_MODE_PWM     : 
                bus_watchdog_map[id].push_back(Addr::bus_watchdog.val2pulse(500/*ms*/, model_[id]));
        } // position系のモードもセットしたいが， homing_offsetのバグがあるので，一旦保留
        if ( !bus_watchdog_map.empty() ) SyncWrite_log({Addr::bus_watchdog}, bus_watchdog_map, verbose_["w_status"]);
    }

    // トルクの確認
    auto id_torque_map = SyncRead_log(Addr::torque_enable, target_id_list, verbose_["r_status"], verbose_["r_status_err"]);
    // 全ての状態が取得できていれば, 終了
    if ( id_torque_map.size() == target_id_list.size() ) { 
        for ( const auto& [id, torque] : id_torque_map ) tq_mode_[id] = static_cast<torque_t>(torque); 
        ping_err_.clear(); // 通信エラーがなければ，ping_err_をクリア
        return; // ガード節
    }
    // 取れていないデータがあれば，個別に応答を確認
    vector<id_t> alive_id_list;
    for ( auto id : target_id_list ) { // ping による確認だと， 直前の fast sync read で読み込み失敗した影響によってうまくいかない． 詳細はReadme参照
        auto id_torque_map = SyncRead_log(Addr::torque_enable, {id}, verbose_["r_status"], verbose_["r_status_err"]);
        if ( id_torque_map.empty() ) continue; // トルクの読み込みに失敗した場合は，そのモータは死んでいるとみなす．
        tq_mode_[id] = static_cast<torque_t>(id_torque_map[id]);
        alive_id_list.push_back(id);
    }
    // すべてのモータが死んでいる場合は，ping_err_=1で固定とする, おそらく根本で電源が切断されているため．
    // 生き残ったモータのping_err_をリセット
    // 一部の生き残っていないモータのping_err_をインクリメント
    for (auto id : target_id_list) { // 効率が悪いが, わかりやすさを優先
        if ( alive_id_list.empty() )    { ping_err_[id] = 1; continue; }
        if ( is_in(id, alive_id_list) ) { ping_err_[id] = 0; continue; }
        /*  !is_in(id, alive_id_list)  */ ping_err_[id]++;
        ROS_WARN("Servo ID [%d] is dead (%d count / %s)", id, (int)ping_err_[id],
            auto_remove_count_ ? (std::to_string(auto_remove_count_)+"count").c_str() : "inf");
    }
}
