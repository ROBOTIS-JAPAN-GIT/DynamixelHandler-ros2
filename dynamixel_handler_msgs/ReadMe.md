# Custom msg design

## Abstract
   プログラムとコマンドライン両方での使用する場合の利便性を考慮して設計を行った．
   コマンドラインで使用する場合は，できるだけ細かく分割されたmsgの方が，閲覧や送信が楽である．
   しかし，プログラムで使用する場合は，pub/subするトピック数を減らしたいため，全ての情報を一つのmsgにまとめることが有効である．
   そのため，コマンドラインで使用する細かいmsgを用意し，それらを複数まとめた大きなmsgをプログラムで使用することとした．

 - Usage in program
   - read: `/dynamixel/states` topic
   - write: `/dynamixel/commands/x` topic (for X series), `/dynamixel/commands/p` topic (for P series)
 - Usage in command line
   - read: `/dynamixel/state/...` topics, `/dynamixel/debug` topic
   - write: `/dynamixel/command/...` topics

-----

## Topic list 
全てのtopicのリストを示す．

#### プログラムから使う想定もの

   - `/dynamixel/states` : すべての状態をまとめたトピック
   - `/dynamixel/commands/x` : Xシリーズのコマンドをまとめたトピック
   - `/dynamixel/commands/p` : Pシリーズのコマンドをまとめたトピック
   - `/dynamixel/commands/all` : X,Pシリーズへまとめてコマンドするトピック
  
#### コマンドラインから使う想定のもの

   **デバック用**
   - `/dynamixel/debug` : サーボが動かないときにに確認したい情報をまとめたもの
   - `/dynamixel/shortcut` : トルクのオンオフ・エラー解除・IDの追加削除などを行うためのもの

   **状態確認用**
   - `/dynamixel/state/status` : サーボの状態を示す
   - `/dynamixel/state/present` : 現在の値を示す
   - `/dynamixel/state/goal` : 目標値を示す
   - `/dynamixel/state/gain` : ゲインを示す
   - `/dynamixel/state/limit` : 制限値を示す
   - `/dynamixel/state/error` : エラーを示す
   - `/dynamixel/state/extra` : その他の情報を示す

   **コマンド送信用**
   - `/dynamixel/command/x/pwm_control` : pwm制御モードでの指令を送る
   - `/dynamixel/command/x/current_control` : 電流制御モードでの指令を送る
   - `/dynamixel/command/x/velocity_control` : 速度制御モードでの指令を送る
   - `/dynamixel/command/x/position_control` : 位置制御モードでの指令を送る
   - `/dynamixel/command/x/extended_position_control` :　拡張位置制御モードでの指令を送る
   - `/dynamixel/command/x/current_base_position_control` :　電流制限付き位置制御モードでの指令を送る

   - `/dynamixel/command/p/pwm_control` : pwm制御モードでの指令を送る
   - `/dynamixel/command/p/current_control` : 電流制御モードでの指令を送る
   - `/dynamixel/command/p/velocity_control` : 速度制御モードでの指令を送る
   - `/dynamixel/command/p/position_control` : 位置制御モードでの指令を送る
   - `/dynamixel/command/p/extended_position_control` : 拡張位置制御モードでの指令を送る

   - `/dynamixel/command/status` : サーボの状態を変更する
   - `/dynamixel/command/goal` : サーボの目標値を変更する
   - `/dynamixel/command/gain` : ゲインを変更する
   - `/dynamixel/command/limit` : 制限値を変更する
   - `/dynamixel/command/extra` : その他の情報を変更する
  
-----

## How to use

### 情報をSubscribe(read)する場合
#### プログラムでの使用
```cpp
#include "dynamixel_handler_msgs/msg/dxl_states.hpp"
/// ...
/// 略, 動くコードは example を参照のこと
/// ...
dynamixel_handler_msgs::msg::DxlStates::SharedPtr msg; //すべての状態が確認できる
// subscribe した status の確認, 単に表示するだけ
for (size_t i = 0; i < msg->status.id_list.size(); i++) {
   printf("servo [%d], torque %s, has %s, ping is %s, mode is %s\n", 
      msg->status.id_list[i],
      msg->status.torque[i] ? "on" : "off",
      msg->status.error[i] ? "error" : "no error",
      msg->status.ping[i] ? "response" : "no response",
      msg->status.mode[i] // これは文字列なのでそのまま表示
   ); 
}
// subscribe した present value を変数に格納, `pub_outdated_present_value`パラメータが`true`の場合
for (size_t i=0; i < msg->present.id_list.size(); i++) {
   auto id = msg->present.id_list[i];
   cur_map[id] = msg->present.current_ma[i];
   vel_map[id] = msg->present.velocity_deg_s[i];
   pos_map[id] = msg->present.position_deg[i];
}
```
<details>
<summary> `pub_outdated_present_value`パラメータが`false`の場合 </summary>

```cpp
// この時，present valueはすべての要素があるとは限らないので以下のように確認が必要
auto& p = msg->present;
for (size_t i=0; i < p.id_list.size(); i++) {
   auto id = p.id_list[i];
   if ( !p.current_ma.empty()     ) cur_map[id] = p.current_ma[i];
   if ( !p.velocity_deg_s.empty() ) vel_map[id] = p.velocity_deg_s[i];
   if ( !p.position_deg.empty()   ) pos_map[id] = p.position_deg[i];
}
```
</details>

#### コマンドラインでの使用

```yaml  
$ ros2 topic echo --flow-style /dynamixel/debug #debug用, これがあると便利
status: # DynamixelStatus型
   id_list: [1, 2, 3, 4]
   torque: [true, false, false, false]
   error: [false, false, false, false]
   ping: [true, true, true, true]
   mode: ['position', 'velocity', 'current', 'velocity']
current_ma: # DynamixelDebugElement型
   present: [0.0, 0.0, 0.0, 0.0]
   goal: [0.0, 0.0, 0.0, 0.0]
velocity_deg_s: # DynamixelDebugElement型
   present: [0.0, 0.0, 0.0, 0.0]
   goal: [0.0, 0.0, 0.0, 0.0]
position_deg: # DynamixelDebugElement型
   present: [0.0, 0.0, 0.0, 0.0]
   goal: [0.0, 0.0, 0.0, 0.0]

$ ros2 topic echo --flow-style /dynamixel/state/limit #  DynamixelLimit型
id_list: [1,2,3,4]
temperature_limit_degc: [0.0, 0.0, 0.0, 0.0]
max_voltage_limit_v: [0.0, 0.0, 0.0, 0.0]
min_voltage_limit_v: [0.0, 0.0, 0.0, 0.0]
pwm_limit_percent: [0.0, 0.0, 0.0, 0.0]
current_limit_ma: [0.0, 0.0, 0.0, 0.0]
acceleration_limit_deg_ss: [0.0, 0.0, 0.0, 0.0]
velocity_limit_deg_s: [0.0, 0.0, 0.0, 0.0]
max_position_limit_deg: [0.0, 0.0, 0.0, 0.0]
min_position_limit_deg: [0.0, 0.0, 0.0, 0.0]

$ ros2 topic echo --flow-style /dynamixel/state/present #  DynamixelPresent型
id_list: [1,2,3,4] # `pub_outdated_present_value`パラメータが`true`の場合, 以下のように全ての要素がid_listと同じ長さになる．
pwm_pulse: [0.0, 0.0, 0.0, 0.0]
current_ma: [0.0, 0.0, 0.0, 0.0]
velocity_deg_s: [0.0, 0.0, 0.0, 0.0]
position_deg: [0.0, 0.0, 0.0, 0.0]
vel_trajectory_deg_s: [0.0, 0.0, 0.0, 0.0]
pos_trajectory_deg: [0.0, 0.0, 0.0, 0.0]
input_voltage_v: [0.0, 0.0, 0.0, 0.0]
temperature_degc: [0.0, 0.0, 0.0, 0.0]

# ... 略
```

### 指令をPublish(write)する場合

#### プログラムでの使用
```cpp
#include "dynamixel_handler_msgs/msg/dxl_commands_x.hpp"
/// ...
/// 略, 動くコードは example を参照のこと
/// ...
dynamixel_handler_msgs::msg::DxlCommandsX cmd;

// id = 1,2,3 のサーボを **torque_on**.
cmd.satatus.set__id_list( {1,2,3} )
           .set__torque( {true, true, true} );
// id:1 のサーボを電流制御モードで50degに移動
cmd.current_base_position_control.id_list.push_back(1);
cmd.current_base_position_control.position_deg.push_back(50);
// id:2 のサーボのdゲインを50に設定
cmd.gain.id_list.push_back(2); 
cmd.gain.position_d_gain_pulse.push_back(50.0); 

pub_dxl_cmd_->publish( cmd );
// =======================================
// より実践的には以下のように使う
#include "dynamixel_handler_msgs/msg/dxl_commands_x.hpp"
#include <map>
#include <tuple>
dynamixel_handler_msgs::msg::DxlCommandsX cmd;
std::map<uint8_t, std::tuple<float, float>> target_map = {
   {1, {50.0, 0.0}},
   {2, {100.0, 0.0}},
   {3, {150.0, 0.0}},
};
for ( auto [id, target] : target_map ) {
   auto [pos, cur] = target;
   cmd.current_base_position_control.id_list.push_back(id);
   cmd.current_base_position_control.position_deg.push_back(pos);
   cmd.current_base_position_control.current_ma.push_back(cur);
   cmd.current_base_position_control.profile_vel_deg_s.push_back(100.0);
   cmd.current_base_position_control.profile_acc_deg_ss.push_back(100.0);
}
pub_dxl_cmd_->publish( cmd );
```

#### コマンドラインでの使用
```bash
$ ros2 topic pub /dynamixel/shortcut dynamixel_handler_msgs/msg/DynamixelShortcut \
"command: 'torque_on'
id_list: [1,2,3,4]" -1

$ ros2 topic pub /dynamixel/command/current_base_position_control dynamixel_handler_msgs/msg/DynamixelControlXCurrentPosition \
"id_list: [1,2,3,4]
current_ma: [0.0, 0.0, 0.0, 0.0]
position_deg: [0.0, 0.0, 0.0, 0.0]
rotation: [0.0, 0.0, 0.0, 0.0]
profile_vel_deg_s: [0.0, 0.0, 0.0, 0.0]
profile_acc_deg_ss: [0.0, 0.0, 0.0, 0.0]" -1

# ... 略
```

-----

## Topic type detail

### `dynamixel_handler_msgs::msg::DxlStates` type 
 `/dynamixel/states` topic　の型
```cpp
builtin_interfaces/Time stamp

dynamixel_handler/DynamixelStatus status
dynamixel_handler/DynamixelPresent present
dynamixel_handler/DynamixelGoal goal
dynamixel_handler/DynamixelGain gain
dynamixel_handler/DynamixelLimit limit
dynamixel_handler/DynamixelError error
dynamixel_handler/DynamixelExtra extra
```
具体的な詳細については，[それぞれの要素の型定義](#それぞれの要素の型定義)を参照.　
↓ 出力例（これを見ればだいたいわかるはず）
```yaml 
$ ros2 topic echo --flow-style /dynamixel/states #このtopicはコマンドラインから見る想定ではない．
stamp: 0000
status: # DynamixelStatus型, pub_ratio/status に一回 read される．．
   id_list: [1, 2, 3, 4]
   torque: [true, false, false, false]
   error: [false, false, false, false]
   ping: [true, true, true, true]
   mode: ['position', 'velocity', 'current', 'velocity']
present: # DynamixelPresent型, pub_ratio/present.~ に一回 read され，1要素でも読み取ったら埋める
   id_list: [1, 2, 3, 4]
   pwm_percent: [0.0, 0.0, 0.0, 0.0] # pub_ratio/present.pwm に一回更新される．
   current_ma: [0.0, 0.0, 0.0, 0.0] # pub_ratio/present.current に一回更新される．
   velocity_deg_s: [0.0, 0.0, 0.0, 0.0] # pub_ratio/present.velocity に一回更新される．
   position_deg: [0.0, 0.0, 0.0, 0.0] # pub_ratio/present.position に一回更新される．
   vel_trajectory_deg_s: [0.0, 0.0, 0.0, 0.0] # pub_ratio/present.vel_trajectory に一回更新される．
   pos_trajectory_deg: [0.0, 0.0, 0.0, 0.0] # pub_ratio/present.pos_trajectory に一回更新される．
   input_voltage_v: [0.0, 0.0, 0.0, 0.0] # pub_ratio/present.input_voltage に一回更新される．
   temperature_degc: [0.0, 0.0, 0.0, 0.0] # pub_ratio/present.temperature に一回更新される．
goal: # DynamixelGoal型, pub_ratio/goalに一回 read され，読み取ったら埋める
   id_list: [1, 2, 3, 4]
   pwm_percent: [0.0, 0.0, 0.0, 0.0]
   current_ma: [0.0, 0.0, 0.0, 0.0]
   velocity_deg_s: [0.0, 0.0, 0.0, 0.0]
   profile_acc_deg_ss: [0.0, 0.0, 0.0, 0.0]
   profile_vel_deg_s: [0.0, 0.0, 0.0, 0.0]
   position_deg: [0.0, 0.0, 0.0, 0.0]
limit: # DynamixelLimit型, pub_ratio/limitに一回 read され，読み取りに成功したら埋める．
   id_list: []
   temperature_limit_degc: []
   max_voltage_limit_v: []
   min_voltage_limit_v: []
   pwm_limit_percent: []
   current_limit_ma: []
   acceleration_limit_deg_ss: []
   velocity_limit_deg_s: []
   max_position_limit_deg: []
   min_position_limit_deg: []
gain: # DynamixelGain型, pub_ratio/gainに一回 read され，読み取りに成功したら埋める．
   id_list: [1, 2, 3, 4]
   velocity_i_gain_pulse: [0, 0, 0, 0]
   velocity_p_gain_pulse: [0, 0, 0, 0]
   position_d_gain_pulse: [0, 0, 0, 0]
   position_i_gain_pulse: [0, 0, 0, 0]
   position_p_gain_pulse: [0, 0, 0, 0]
   feedforward_2nd_gain_pulse: [0, 0, 0, 0]
   feedforward_1st_gain_pulse: [0, 0, 0, 0]
error: # DynamxielError型, pub_ratio/errorに一回 read され，読み取りに成功したら埋める．
   id_list: [1, 2, 3, 4]
   input_voltage: [false, false, false, false]
   motor_hall_sensor: [false, false, false, false]
   overheating: [false, false, false, false]
   motor_encoder: [false, false, false, false]
   electronical_shock: [false, false, false, false]
   overload: [false, false, false, false]
extra: # DynamixelExtra型, 未実装
   id_list: []
   model: [] # 未実装
   firmware_version: [] # 未実装
   protocol_version: [] # 未実装
   drive_mode: # 未実装
      torque_on_by_goal_update: []
      profile_configuration: []
      reverse_mode: []
   shutdown: # 未実装
      overload_error: []
      electrical_shock_error: []
      motor_encorder_error: []
      overheating_error: []
      input_voltage_error: []
   restore_configuration: # 未実装
      ram_restore: []
      startup_torque_on: []
   homing_offset_deg: [] # 未実装, デフォルトで使用している
   bus_watchbdog_ms: [] # 未実装, デフォルトで使用している
   return_delay_time_us: [] # 未実装
   led: # 未実装
      red_percent: [] # Xシリーズは `0.0`と`100.0`の二値．切り上げ． 
      blue_percent: [] # Pシリーズのみ
      green_percent: [] # Pシリーズのみ
   shadow_id: [] # 未実装
   moving_threshold_deg_s: [] # 未実装
   status_return_level: [] # 未実装
   moving_status: # 未実装
      velocity_profile: []
      following_error: []
      profile_ongoing: []
      in_posision: []
   realtime_tick_us: [] # 未実装
   moving: [] # 未実装
   registered_instruction: [] # 未実装
   reboot: [] # 未実装 
```

### `dynamixel_handler_msgs::msg::DxlCommandsX` type 
`/dynamixel/commands/x` topic の型．
```cpp
dynamixel_handler/DynamixelControlXPwm                 pwm_control
dynamixel_handler/DynamixelControlXCurrent             current_control
dynamixel_handler/DynamixelControlXVelocity            velocity_control
dynamixel_handler/DynamixelControlXPosition            position_control
dynamixel_handler/DynamixelControlXExtendedPosition    extended_position_control
dynamixel_handler/DynamixelControlXCurrentBasePosition current_base_position_control
dynamixel_handler/DynamixelStatus status
dynamixel_handler/DynamixelGain   gain
dynamixel_handler/DynamixelLimit  limit
dynamixel_handler/DynamixelExtra  extra
```
具体的な詳細については，[それぞれの要素の型定義](#それぞれの要素の型定義)を参照.
↓ 出力例（これを見ればだいたいわかるはず）
```yaml
$ ros2 topic echo --flow-style /dynamixel/commands/x #このtopicはコマンドラインから送る想定ではない．
pwm_control: # DynamixelControlXPwm型
   id_list: [1]        # 1番のサーボをPWM制御モードに変更し，
   pwm_percent: [40.0] # goal_pwm アドレスに 40% に相当するパルス値を書き込む．
current_control: # DynamixelControlXCurrent型
   id_list: []
   current_ma: []
velocity_control: # DynamixelControlXVelocity型
   id_list: [2,3]                # 2,3番のサーボを速度制御モードに変更し，
   velocity_deg_s: [10.0, -50.0] # goal_velocity アドレスに 10, -50 deg/s に相当するパルス値を書き込む．
   profile_acc_deg_ss: []
position_control: # DynamixelControlXPosition型
   id_list: []
   position_deg: []
   profile_vel_deg_s: []
   profile_acc_deg_ss: []
extended_position_control: # DynamixelControlXExtendedPosition型
   id_list: []
   position_deg: []
   rotation: []
   profile_vel_deg_s: []
   profile_acc_deg_ss: []
current_base_position_control: # DynamixelControlXCurrentPosition型
   id_list: [4]                 # 4番のサーボを電流制限付き位置制御モードに変更し，    
   current_ma: [100.0]          # goal_current アドレスに 100mA に相当するパルス値を書き込む．
   position_deg: []             #
   rotation: [1.2]              # goal_position アドレスに 1.2*360 deg に相当するパルス値を書き込む．
   profile_vel_deg_s: [100.0]   # profile_velocity アドレスに 100 deg/s に相当するパルス値を書き込む．
   profile_acc_deg_ss: [1000.0] # profile_acceleration アドレスに 1000 deg/s^2 に相当するパルス値を書き込む．
status: # DynamixelStatus型
   id_list: []
   torque: [] # torque_onコマンド, torque_offコマンドと同等
   error: [] # clear_errorコマンドと同等
   ping: [] # add_id コマンド, remove_id コマンドと同等
   mode: [] # 各control系のコマンドと同等
gain: # DynamixelGain型
   id_list: [1,2,3,4]
   velocity_i_gain_pulse: []
   velocity_p_gain_pulse: []
   position_d_gain_pulse: []
   position_i_gain_pulse: [100, 100, 100, 100]
   position_p_gain_pulse: []
   feedforward_2nd_gain_pulse: []
   feedforward_1st_gain_pulse: []
limit: # DynamixelLimit型
   id_list: [1,2]
   temperature_limit_degc: []
   max_voltage_limit_v: []
   min_voltage_limit_v: []
   pwm_limit_percent: []
   current_limit_ma: [500, 800]
   acceleration_limit_deg_ss: []
   velocity_limit_deg_s: []
   max_position_limit_deg: []
   min_position_limit_deg: []
extra: # DynamixelExtra型, 未実装
   id_list: []
   model: [] # 未実装
   firmware_version: [] # 未実装
   protocol_version: [] # 未実装
   drive_mode: # 未実装
      torque_on_by_goal_update: []
      profile_configuration: []
      reverse_mode: []
   shutdown: # 未実装
      overload_error: []
      electrical_shock_error: []
      motor_encorder_error: []
      overheating_error: []
      input_voltage_error: []
   restore_configuration: # 未実装
      ram_restore: []
      startup_torque_on: []
   homing_offset_deg: [] # 未実装, デフォルトで使用している
   bus_watchbdog_ms: [] # 未実装, デフォルトで使用している
   return_delay_time_us: [] # 未実装
   led: # 未実装
      red_percent: [] # Xシリーズは `0.0`と`100.0`の二値．切り上げ． 
      blue_percent: [] # Pシリーズのみ
      green_percent: [] # Pシリーズのみ
   shadow_id: [] # 未実装
   moving_threshold_deg_s: [] # 未実装
   status_return_level: [] # 未実装
   moving_status: # 未実装
      velocity_profile: []
      following_error: []
      profile_ongoing: []
      in_posision: []
   realtime_tick_us: [] # 未実装
   moving: [] # 未実装
   registered_instruction: [] # 未実装
   reboot: [] # 未実装 
```

### `dynamixel_handler_msgs::msg::DxlCommandsP` type
 `/dynamixel/commands/p` topic の型．
```cpp
dynamixel_handler/DynamixelControlPPwm              pwm_control
dynamixel_handler/DynamixelControlPCurrent          current_control
dynamixel_handler/DynamixelControlPVelocity         velocity_control
dynamixel_handler/DynamixelControlPPosition         position_control
dynamixel_handler/DynamixelControlPExtendedPosition extended_position_control
dynamixel_handler/DynamixelStatus status
dynamixel_handler/DynamixelGain   gain
dynamixel_handler/DynamixelLimit  limit
dynamixel_handler/DynamixelExtra  extra
```

### `dynamixel_handler_msgs::msg::DxlCommandsAll` type
 `/dynamixel/commands/p` topic の型．
```cpp
dynamixel_handler/DynamixelStatus status
dynamixel_handler/DynamixelGain   goal
dynamixel_handler/DynamixelGain   gain
dynamixel_handler/DynamixelLimit  limit
dynamixel_handler/DynamixelExtra  extra
```

### それぞれの要素の型定義

#### `DynamixelShortcut` type
トルクのオンオフなどのコマンドを送るトピック `/dynamixel/shortcut` の型．
   ```yml
   string   command
   uint16[]  id_list
   # === high level commands : ユーザの利用を想定 ===
   string CLEAR_ERROR ="clear_error" # ("CE"): ハードウェアエラー(ex. overload)をrebootによって解除する．
                                     #         加えて，homing offsetを用いて現在角がジャンプしないように調整する．
   string TORQUE_OFF ="torque_off" # ("TOFF"): トルクをdisableにする．
   string TORQUE_ON  ="torque_on"  # ("TON") : 安全にトルクをenableにする．目標姿勢を現在姿勢へ一致させ，速度を0にする．
   string REMOVE_ID  ="remove_id"  # ("RMID"): 指定したIDのサーボを認識リストから削除する．
   string ADD_ID     ="add_id"     # ("ADID"): 指定したIDのサーボを認識リストに追加する．
   # === low level commands : 開発者向け ===
   string RESET_OFFSET="reset_offset" # : homing_offset アドレスに 0 を書き込む．
   string ENABLE ="enable"  # : torque enable アドレスに true を書き込む．
   string DISABLE="disable" # : torque enable アドレスに false を書き込む．
   string REBOOT ="reboot"  # : reboot インストラクションを送る．
   ```
各コマンドの内容はmsgの定数として定義されており，プログラム内から扱う場合，以下の様に記述できる．
   ```cpp
   dynamixel_handler_msgs::msg::DynamixelShortcut msg;
   msg.command = msg.TORQUE_ON; // typoはコンパイラが教えてくれる．
   // msg.command = "torque_on"; // もちろんこれもOKだが，typoのリスクがある．
   ```
また，括弧内はalias．すなわち，`command="clear_error"`とするのと`command="CE"`とするのは同じ．

#### `DynamixelDebug` type
デバック用の情報を読み込むためのトピック `/dynamixel/debug` の型．
   ```yml
   DynamixelStatus status
   DynamixelDebugElement current_ma
   DynamixelDebugElement velocity_deg_s
   DynamixelDebugElement position_deg
   ```

##### `DynamixelDebugElement` type
   ```yml
   float64[] present
   float64[] goal
   ```

#### `DynamixelControlXPwm` type
XシリーズをPWM制御モードで動かすためのトピック `/dynamixel/command/x/pwm_control` の型．
   ```yml
   uint16[] id_list
   float64[] pwm_percent
   ```

#### `DynamixelControlXCurrent` type
Xシリーズを電流制御モードで動かすためのトピック `/dynamixel/command/x/current_control` の型
   ```yml
   uint16[] id_list
   float64[] current_mA
   ```

#### `DynamixelControlXVelocity` type
Xシリーズを速度制御モードで動かすためのトピック　`/dynamixel/command/x/velocity_control`の型
   ```yml
   uint16[] id_list
   float64[] velocity_deg_s
   float64[] profile_acc_deg_ss
   ```

#### `DynamixelControlXPosition` type
Xシリーズを位置制御モードで動かすためのトピック　`/dynamixel/command/x/position_control`の型
   ```yml
   uint16[] id_list
   float64[] position_deg
   float64[] profile_vel_deg_s
   float64[] profile_acc_deg_ss
   ```

#### `DynamixelControlXExtendedPosition` type
Xシリーズを拡張位置制御モードで動かすためのトピック　`/dynamixel/command/x/extended_position_control`の型
   ```yml
   uint16[] id_list
   float64[] position_deg
   float64[] rotation # optional, 256までの回転数を指定できる
   float64[] profile_vel_deg_s
   float64[] profile_acc_deg_ss
   ```

#### `DynamixelControlXCurrentPosition` type　
Xシリーズを電流制限付き位置制御モードで動かすためのトピック `/dynamixel/command/x/current_position _control`の型
   ```yml
   uint16[] id_list
   float64[] current_ma
   float64[] position_deg
   float64[] rotation # optional, 256までの回転数を指定できる
   float64[] profile_vel_deg_s
   float64[] profile_acc_deg_ss
   ```

#### `DynamixelStatus` type
Status 関連を読み書きするためのトピック `/dynamixel/command/status` と `/dynamixel/state/status` の型
   ```yml
   uint16[] id_list
   bool[] torque
   bool[] error
   bool[] ping
   string[] mode
   #====== mode field に指定できる文字列 ======
   string CONTROL_PWM                   = "pwm"
   string CONTROL_CURRENT               = "current"
   string CONTROL_VELOCITY              = "velocity"
   string CONTROL_POSITION              = "position"
   string CONTROL_EXTENDED_POSITION     = "ex_position"
   string CONTROL_CURRENT_BASE_POSITION = "cur_position"
   ```

#### `DynamixelGoal` type
Goal値を読み書きするためのトピック `/dynamixel/command/goal` と `/dynamixel/state/goal` の型
   ```yml
   uint16[] id_list
   float64[] pwm_percent
   float64[] current_ma
   float64[] velocity_deg_s
   float64[] profile_acc_deg_ss
   float64[] profile_vel_deg_s
   float64[] position_deg
   ```

#### `DynamixelPresent` type
Present値を読み出すためのトピック `/dynamixel/state/present` の型
   ```yml
   uint16[] id_list
   float64[] pwm_percent
   float64[] current_ma
   float64[] velocity_deg_s
   float64[] position_deg
   float64[] vel_trajectory_deg_s
   float64[] pos_trajectory_deg
   float64[] input_voltage_v
   float64[] temperature_degc
   ```

#### `DynamixelGain` type
Gainを読み書きするためのトピック `/dynamixel/command/gain` と `/dynamixel/state/gain` の型
   ```yml
   uint16[] id_list
   float64[] velocity_i_gain_pulse
   float64[] velocity_p_gain_pulse
   float64[] position_d_gain_pulse  
   float64[] position_i_gain_pulse
   float64[] position_p_gain_pulse
   float64[] feedforward_2nd_gain_pulse
   float64[] feedforward_1st_gain_pulse
   ```
#### `DynamixelLimit` type
Limitを読み書きするためのトピック `/dynamixel/command/limit`　と　`/dynamixel/state/limit`　の型
   ```yml
   uint16[] id_list
   float64[] temperature_limit_degc
   float64[] max_voltage_limit_v
   float64[] min_voltage_limit_v
   float64[] pwm_limit_percent
   float64[] current_limit_ma
   float64[] acceleration_limit_deg_ss
   float64[] velocity_limit_deg_s
   float64[] max_position_limit_deg
   float64[] min_position_limit_deg
   ```

#### `DynamixelError` type
Errorを読みだすためのトピック `/dynamixel/state/error` の型
   ```yml
   uint16[] id_list
   bool[] input_voltage
   bool[] motor_hall_sensor
   bool[] overheating
   bool[] motor_encoder
   bool[] electronical_shock
   bool[] overload
   ```

---
---
---

## External port に関して

### Topic list

   - `/dynamixel/external_port/write` : 外部ポートへの書き込み・設定を行うmsg
   - `/dynamixel/external_port/read` : 外部ポートの読み取りを行うmsg

### How to use
```cpp
#include "dynamixel_handler_msgs/msg/dxl_external_port.hpp"
// ID: 1 のサーボのポート1と2にはLEDが接続されている
constexpr int ID_LIGHT = 1;
constexpr int PORT_LIGHT1 = 1;
constexpr int PORT_LIGHT2 = 2;
// ID: 1 のサーボのポート3はマグネットセンサが接続されている
constexpr int ID_MAGNET = 2;
constexpr int PORT_MAGNET = 3;
// 使い方1 書き込み
DxlExternalPort msg;
msg.set__id_list({ID_LIGHT            , ID_LIGHT            });
msg.set__port   ({PORT_LIGHT1         , PORT_LIGHT2         });
msg.set__mode   ({msg.MODE_DIGITAL_OUT, msg.MODE_DIGITAL_OUT});
msg.set__data   ({1                   , 0                   });
msg.id_list.push_back(ID_MAGNET);
msg.port.push_back(PORT_MAGNET);
msg.mode.push_back(msg.MODE_ANALOG_IN);
msg.data.push_back(0); // id_listとdataの要素数を合わせるためのダミー値
pub_ex_port_->publish( msg );

// 使い方2 読み込み
const int val_magnet = 0;
for ( size_t i = 0; i < msg.id_list.size(); i++ ) {
   if ( msg.id_list[i] != ID_MAGNET ) continue;
   if ( msg.port[i] != PORT_MAGNET ) continue;
   val_magnet = msg.data[i];
}
printf("magnet sensor is %d\n", val_magnet);
```

↓出力例(これを見ればだいたいわかるはず)
```yaml
$ ros2 topic echo --flow-style /dynamixel/external_port/write # DxlExternalPort型
stamp: 0000
id_list: [1, 1, 2]
port: [1, 2, 3]
mode: ["d_out", "d_out", "a_in"]
data: [1, 0, 0]

$ ros2 topic echo --flow-style /dynamixel/external_port/read # DxlExternalPort型
stamp: 0000
id_list: [1, 1, 1, 2, 2, 2]
port: [1, 2, 3, 1, 2, 3]
mode: ["d_out", "d_out", "d_in_pu", "d_in_pd", "d_in_pd", "a_in"]
data: [1, 0, 0, 1, 1, 2000]
---
```

### Topic type detail

#### `dynamixel_handler_msgs::msg::DxlExternalPort` type
外部ポートの設定を行うためのトピック `/dynamixel/external_port/write` と `/dynamixel/external_port/read` の型．
```yaml
builtin_interfaces/Time stamp
uint16[] id_list
uint16[] port
string[] mode
uint16[] data
#====== mode field に指定できる文字列 ======
string MODE_ANALOG_IN           = "a_in"
string MODE_DIGITAL_OUT         = "d_out"
string MODE_DIGITAL_IN_PULLUP   = "d_in_pu"
string MODE_DIGITAL_IN_PULLDOWN = "d_in_pd"
```