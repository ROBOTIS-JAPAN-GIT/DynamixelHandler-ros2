#ifndef LAYOUT_OUTPUT_HPP
#define LAYOUT_OUTPUT_HPP

// ちょっとした文字列の整形を行う補助関数
using std::setw;
using std::prev;
using std::next;

template <typename T, typename U>
static string control_table_layout(int width, const map<T, vector<U>>& id_data_map, const vector<DynamixelAddress>& dp_list, const string& header=""){
    std::stringstream ss;
    ss << header;
    if (id_data_map.empty()) return ss.str();
    // width 以上のID数がある場合は，再帰させることで，縦に並べる
	width = min(width, (int)id_data_map.size());
    map<T, vector<U>> first(id_data_map.begin(), prev(id_data_map.end(), id_data_map.size() - width));
    map<T, vector<U>> second(next(id_data_map.begin(), width), id_data_map.end());
    // 分割した前半を処理
    ss << "\n" << "ADDR|"; 
    for (const auto& [id, data] : first) ss << "  [" << setw(3) << (int)id << "] "; 
    ss << "\n";
    for (size_t i = 0; i < dp_list.size(); ++i) {
        ss << "-" << setw(3) << dp_list[i].address() << "|" ;
        for (const auto& [id, data] : first) 
            if ( dp_list[i].is_dummy() ) ss << "      x ";
            else ss << std::setfill(' ') << setw(7) << data[i] << " "; 
        ss << "\n";
    }
    // 分割した前半に後半を処理したものを追加する
    return ss.str() + control_table_layout(width, second, dp_list);
}

template <typename T>
static string id_list_layout(const vector<T>& id_list, const string& header=" ID"){
    std::stringstream ss;
    ss << header << ": [ "; 
    for ( size_t i = 0; i < id_list.size(); i++ ) {
        if ( i != 0 ) ss << ", ";
        ss << (int)id_list[i]; 
    }
    ss << " ]";
    return ss.str();
}

#endif /* LAYOUT_OUTPUT_HPP */