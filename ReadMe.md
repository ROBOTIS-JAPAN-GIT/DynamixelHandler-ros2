# DynamixelHandler-ros2

Robotis社の[Dynamixel](https://e-shop.robotis.co.jp/list.php?c_id=89)をROSから制御するための `dynamixel_handler`パッケージとその周辺パッケージを提供するリポジトリ.  

ROS2のみ対応，ROS1 ver は[こちら](https://github.com/ROBOTIS-JAPAN-GIT/DynamixelHandler-ros1).ただし，開発が分離しているので機能はやや異なる．

X, Pシリーズのみ対応，X320シリーズ，Yシリーズは順次対応予定．

## Table of Contents
  - [How to install](#how-to-install)
  - [How to use](#how-to-use)
  - [Topics](#topics)
  - [Parameters](#parameters)
  - [各種情報の分類と Control Table との対応](#各種情報の分類と-control-table-との対応)
  - [Baudrateの一括変更](#baudrateの一括変更)
  - [Trouble Shooting](#trouble-shooting)

## Features of this package
 - **Dynamixel制御に特化した最小単位のパッケージ**
    - `dynamixel_handler`パッケージが提供する **`dynamixel_handler`** ノードは，サーボモータとの通信を担う．
    - サーボの動作制御はユーザーが開発する **別の制御ノード**が行い，`dynamixel_handler`ノードは通信の仲介を行うイメージ．
    - ユーザーはシリアル通信の通信プロトコルや，コントロールテーブルついて知る必要が(あまり)ない．
    - Dynamixelの各種情報や機能が[適切に分類](#各種情報の分類と-control-table-との対応)され，ほぼすべての情報・機能を利用することが可能
  
 - **ROSトピックのみで制御できるシンプルなインターフェース**   
    トピック通信だけで利用できるので，このパッケージ自体のコードの編集は不要 
    - **Subscribe**: `/dynamixel/commands/x`(他、`/dynamixel/commands/p`など)
      - シリーズごとに定義された制御コマンドを送ることで動作制御．
      - 制御コマンドの内容に合わせてサーボの制御モードが自動変更．
    - **Publish**: `/dynamixel/states` 
      - [分類された各情報](#各種情報の分類と-control-table-との対応)を個別の周期で自動的に read & publish．
        - "Status": torqueのオンオフ, errorの有無, pingの成否, 制御モード (デフォルト 約2Hz)
        - "Error": overload エラー などのハードウェアエラー (デフォルト 約2Hz)  
        - "Present": current [mA], velocity [deg/s], position [deg] などの現在値 (デフォルト 約50Hz)
        - etc... (Goal, Limit, Gain, Extra)

 - **物理量ベースでのやり取り**
    - current [mA], velocity [deg/s], position [deg] などの物理量を直接扱える．
    - Dynamixelから read されたパルス値(0 ~ 4095 など)は物理量に変換してから publish される． 
    - Dynamixelへの目標値は物理量で入力され，内部でパルス値に変換されてから write される．  
     
    ※ 特に角度を rad (-π ~ π) ではなく degree (-180.0 ~ 180.0) で扱うので直感的．

 - **高速で安定したRead/Write** 
   - 読み書きをできるだけ一括で行うことで通信回数を削減  
      - 連続アドレスの一括読み書き
      - 複数サーボの一括読み書き (SyncRead/SyncWrite)
    - Fast Sync Read インストラクションによるReadの高速化
      - 通信形式についての細かい知識が無くてもros paramで設定するだけで利用可能．  
    - `dynamixel_handler`ノードの loop 周期に同期したRead/Write  
      - Topicの受信頻度に依存しないため，安定した通信が可能．

    ※ 12サーボ同時Read/Writeでも150Hz程度の通信が可能なことを確認  
    ※※ 高速通信には適切な [`latency timer`](#latency-timer)  (1~4ms) と `baudrate`(推奨 1,000,000bps 以上) が必要

 - **開発の手間を減らす便利機能**
    - 初期化時の動作 
      - 連結したDynamixelを自動で認識
      - 連結されるDynamixel数の指定 (Optional)
        - ロボットのサーボ数を事前に入力することで，断線などの異常を検知可能
      - エラーを自動でクリア (Optional)
      - トルクを自動でON (Optional)
      - baudrate の(簡易)一括設定 (Optional)
    - 終了時の動作
      - `dynamixel_handler`ノードを kill したタイミングで動作を停止 (Optional)
      - `dynamixel_handler`ノードを kill したタイミングでトルクをOFF (Optional)
    - Dummy Servo 機能 (Optional)
      - 未接続のサーボのIDを与えることで，そのIDのサーボの挙動を簡易シミュレート
      - 未接続のサーボを考慮したい時や実機を動かしたくない場合でも動作確認が可能
    - ハードウェアエラーのクリア
      - エラークリア時の回転数消失問題を homing offset により自動補正
    - baudrate の一括変更 (別ノードで提供)
      -  パッケージに同梱している `dynamixel_unify_baudrate`ノードで一括で変更可能

 - **ROSパラメータによる各種ログ表示制御**
   - Read/Write にかかる平均時間とSerial通信の成功率
   - CallBackした内容 (Optional)
   - Read/Write されるパルス値 (Optional)
   - Readに失敗したID (Optional)
   - etc...
  
  各機能の設定については ros param から可能なので，詳細は [Parameters](#parameters)の章を参照．

***************************

## How to install

### このリポジトリをgit clone
```bash
cd ~/ros2_ws/src
# sshの場合
git clone --recursive git@github.com:ROBOTIS-JAPAN-GIT/DynamixelHandler-ros2.git
# httpsの場合
git clone --recursive https://github.com/ROBOTIS-JAPAN-GIT/DynamixelHandler-ros2.git
# 旧バージョンを使いたい場合
git clone --recursive https://github.com/ROBOTIS-JAPAN-GIT/DynamixelHandler-ros2.git dynamixel_handler -b ver0.1.0
```

### ビルド
```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-up-to dynamixel_handler
source ~/ros2_ws/install/setup.bash # 初回 build 時のみ
```

***************************

## How to use

- コマンドラインを用いた `dynaimxel_handler`ノードの動作確認
- 他の制御用パッケージと連携した `dynaimxel_handler`ノードの利用

上記を達成するための手順を以下に示す．

### 1. Dynamixelの接続

Dynamixel Wizardでモータの動作確認ができる程度の状態を想定．

- Dynaimixel をディジーチェーンにしてU2D2経由でUSB接続されていること．
- ID に重複がないように事前に設定されていること.
- baudrate が全て統一されていること．

> [!TIP]
> `init/baudrate_auto_set`パラメータを`true`にすることで，接続時に自動で baudrate を変更することも可能．
> また，別ノードとして，baudrate を一括で変更するための [`dynamixel_unify_baudrate`ノード](#Baudrateの一括変更) も用意してある

### 2. `dynamixel_handler`ノードの起動

#### 2-1. 自分の環境とconfigの設定を合わせる
`config/config_dynamixel_handler.yml`の該当部分を編集し，保存．   
以下は baudrate: $57600$ かつ device name: `/dec/ttyUSB0`かつ latency timer: $16$ ms の場合
```yml
# config/config_dynamixel_handler.launch
/**:
    ros__parameters:
        # 通信機器の設定
        device_name: /dev/ttyUSB0 # 通信するデバイス名,
        baudrate: 57600    # 通信速度, Dynamixelのデフォルトは 57600 bps (で遅い) 
        latency_timer: 16 # 通信のインターバル, 多くの環境でデフォルトは 16 ms (で遅い)
```

通信環境の設定については [Parameters](#parameters) の章の[通信関係の設定](#通信関係の設定)を参照．

#### 2-2. ターミナルから実行

```bash
ros2 launch dynamixel_handler dynamixel_handler_launch.xml
```
```bash
# 出力例
# ... 略 ...
[dynamixel_handler-1] 00000.00000: Initializing DynamixelHandler ..... 
[dynamixel_handler-1] 00000.00000:  Succeeded to open USB device
[dynamixel_handler-1] 00000.00000:   ------------ name '/dev/ttyUSB0' 
[dynamixel_handler-1] 00000.00000:   -------- baudrate '57600'
[dynamixel_handler-1] 00000.00000:   --- latency_timer '16'
[dynamixel_handler-1] 00000.00000:  Expected number of Dynamixel is not set. 
[dynamixel_handler-1] 00000.00000:  > Free number of Dynamixel is allowed.
[dynamixel_handler-1] 00000.00000:  Auto scanning Dynamixel (id range [0] to [30]) ...
[dynamixel_handler-1] 00000.00000:    * P series servo id [1] is found 
[dynamixel_handler-1] 00000.00000:    ID [1] is enabled torque 
[dynamixel_handler-1] 00000.00000:    * X series servo id [6] is found 
[dynamixel_handler-1] 00000.00000:    ID [6] is enabled torque 
[dynamixel_handler-1] 00000.00000:    * X series servo id [7] is found 
[dynamixel_handler-1] 00000.00000:    ID [7] is enabled torque 
[dynamixel_handler-1] 00000.00000:  ... Finish scanning Dynamixel 
[dynamixel_handler-1] 00000.00000: ..... DynamixelHandler is initialized 
[dynamixel_handler-1] 00000.00000: Loop [0]: write=0.00ms read=11.72ms(p/f=100%/100%) 
[dynamixel_handler-1] 00000.00000: Loop [300]: write=0.01ms read=5.55ms(p/f=100%/100%) 
[dynamixel_handler-1] 00000.00000: Loop [600]: write=0.01ms read=5.47ms(p/f=100%/100%) 
[dynamixel_handler-1] 00000.00000: Loop [900]: write=0.01ms read=5.30ms(p/f=100%/100%)
```

連結したDynamixelが自動で探索され，見つかったDynamixelの初期設定が行われる．   
うまく見つからない場合は [Trouble Shooting](#trouble-shooting) を参照.     
初期化時の動作設定については [Parameters](#parameters) の章の[初期化・終了時等の挙動設定](#初期化終了時等の挙動設定)を参照.

### 3. Dynamixelの動作を制御

コマンドラインから指令する用の topic として`/dynamixel/command/...`と`/dynamixel/shortcut`トピックが用意されている．     

topic の詳細については [Topic](#subscribed-topics) の章を参照．

#### 例：ID:5の Dynamixel(Xシリーズ) を位置制御モードで角度を90degに制御

`/dynamixel/command/x/position_control`トピックにIDと角度を設定して publish．
```bash
ros2 topic pub /dynamixel/command/x/position_control \
 dynamixel_handler_msgs/msg/DynamixelControlXPosition \
 "{id_list: [5], position_deg: [90], profile_vel_deg_s: [], profile_acc_deg_ss: []}" -1
```
実機のサーボの角度が90degになれば成功．
> [!note]
> ID:5のDynamixelの制御モードは自動的に位置制御に変換される．

#### 例：ID:5の Dynamixel のトルクをOFF
`/dynamixel/shortcut`トピックに`torque_off`コマンドとIDを設定して publish．
```bash
ros2 topic pub /dynamixel/shortcut \
 dynamixel_handler_msgs/msg/DynamixelShortcut \
 "{command: 'torque_off', id_list: [5]}"
```
実機のサーボのトルクがOFFになれば成功．

`/dynamixel/shortcut`トピックで使えるコマンドについては[こちら](#shortcut-command-list) を参照．

### 4. Dynamixelの情報を取得

コマンドラインから確認する用の topic として`/dynamixel/state/...`と`/dynamixel/debug`トピックが一定周期で pub され続けている．   

topic の詳細については [Topic](#published-topics) の章を参照．    
また，read　周期については [Parameters](#parameters) の章の[実行時の動作設定](#実行時の動作設定)を参照．

#### 例: Dynamixel の現在値の確認

```bash
ros2 topic echo --flow-style /dynamixel/state/present # status, goal, gain, limit, error... など． 
```
```yml
--- # 出力例
id_list: [5, 6] # 認識されているサーボのID
pwm_percent: [0.0, 0.0] # 現在のPWM値
current_ma: [0.0, -2.69] # 現在の電流値
velocity_deg_s: [0.1, 0.1] # 現在の各速度
position_deg: [89.91210937499999, -0.2636718750000023] # 現在の角度
vel_trajectory_deg_s: [0.0, 0.0] # 目標速度 みたいなもの
pos_trajectory_deg: [0.0, 0.0] # 目標角度 みたいなもの
temperature_degc: [0.0, 0.0] # 現在の温度
input_voltage_v: [0.0, 0.0] # 現在の入力電圧
---
```
上記は電流，速度，位置を読み込むように設定した場合なのでそれ以外の要素は初期値の0になっている．   
read & publish される情報の選択については [Parameters](#parameters) の章の[実行時の動作設定](#実行時の動作設定)を参照．

#### 例: Dynamixel のデバック用情報の確認

```bash
ros2 topic echo --flow-style /dynamixel/debug
```
```yml
--- # 出力例
status: # /dynamixel/state/status と同じ
  id_list: [1, 6, 7, 8, 9] # 認識されているサーボのID
  torque: [true, true, true, true, true] # トルクがONかOFFか
  error: [false, false, false, false, false] # エラーが発生しているか
  ping: [true, true, true, true, true] # pingが通っているか
  mode: [velocity, cur_position, cur_position, cur_position, cur_position] # 制御モード
current_ma: # 現在の電流値と目標電流値
  present: [0.0, 3.0, 2.0, 9.0, 4.0] 
  goal: [0.0, 900.0, 910.0, 910.0, 910.0]
velocity_deg_s: # 現在の速度と目標速度
  present: [0.0, 0.0, 0.0, 1.374, 0.0]
  goal: [0.0, 439.68, 439.68, 439.68, 439.68]
position_deg: # 現在の角度と目標角度
  present: [1007.5781, -24.7852, 44.1211, -33.8379, -87.8906]
  goal: [1007.666, -24.7852, 44.1211, -33.8379, -87.8906]
---
```
トルクのオンオフ，制御モード，目標電流(実質的な最大電流)など，動作状況を確認するための情報が含まれる．

### 5. `dynamixel_handler_examples` パッケージの `example1`ノードを起動

以下では，`dynamixel_handler`ノードを別ノードと連携させて，Dynamixelの情報を取得しつつ，動作を制御する例を示す．
例に用いるのは [dynamixel_handler_examples](./dynamixel_handler_examples) が提供する `example1`ノードである．

[dynamixel_handler_examples](./dynamixel_handler_examples) パッケージをビルド
```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-up-to dynamixel_handler_examples
source ~/ros2_ws/install/setup.bash # 初回 build 時のみ
```

`example1`ノードを起動
```bash
ros2 launch dynamixel_handler_examples example1.xml
```

実行が成功すると，`example1`ノードが `/dynamixel/states`トピック (`DxlStates`型) を subscribe して，以下の情報が取得される．
 - `status`フィールドにより，トルクのオンオフ，エラーの有無，pingの成否，制御モード．
 - `present`フィールドにより，電流，速度，位置などの現在値．

取得された情報はターミナルに表示される．

また，`example1`ノードによって`/dynamixel/commands/x`トピック (`DxlCommandsX`型 ) が publish され，以下のように制御される．
 - `status`フィールドを用いてトルクをオン
 - `current_base_position_control`フィールド用いて
     - 電流を300mAに制限
     - +-45degで往復運動

IDにかかわらずすべてのサーボが上記の動作をしているはずである．

コードの解説や pkg の構成については[`dynamixel_handler_examplesのREADME`](./dynamixel_handler_examples/ReadMe.md)を参照．

***************************

## Topics

### Summay

一般的なユースケースでは以下の2つの topic を利用すれば十分である．

####  `/dynamixel/states`(dynamixel_handler_msgs::msg::DynamixelStates)
連結したサーボすべての状態をまとめた topic, `dynaimxel_handler`からpublishされる．

<details>
<summary> field 構成 </summary>

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
  # 未実装につき略
```

</details>

#### `/dynamixel/commands/x`(dynamixel_handler_msgs::msg::DynamixelCommandsX)
Xシリーズに対してまとめて指令する topic, `dynaimxel_handler`に subscribe される．
  
  <details>
  <summary> field 構成 </summary>
  
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
  # 未実装につき略
```

</details>

<br>

具体的な使い方ついては，[`dynamixel_handler_examples`](./dynamixel_handler_examples) を参照されたし．
 

### Published Topics

各情報は読み取り周期 `pub_ratio/{~}`に従って読み取られ，**読み取られた場合のみ** publish or データが埋められる．    
読み取りの周期は[実行時の動作設定](#実行時の動作設定)や[各種情報の分類](#各種情報の分類と-control-table-との対応)を参照．

#### プログラム向けの統合的なステータス情報

- **`/dynamixel/states`** ([`DxlStates`型](./dynamixel_handler_msgs#dynamixel_handler_msgsmsgdxlstates-type))  
  Xシリーズ・Pシリーズ共通のサーボ状態をまとめた topic．以下の field からなる：
  - `stamp`: データが読み取れた時刻  
  - `status`: `/dynamixel/state/status`に相当し，`pub_ratio/status`に一回データが埋められる．  
  - `present`: `/dynamixel/state/present`に相当し，`pub_ratio/present.{~}`の最小値に一回埋められる．  
  - `goal` : `/dynamixel/state/goal`に相当, `pub_ratio/goal`に一回データが埋められる．  
  - `gain` : `/dynamixel/state/gain`に相当, `pub_ratio/gain`に一回データが埋められる．  
  - `limit`: `/dynamixel/state/limit`に相当, `pub_ratio/limit`に一回データが埋められる．  
  - `error`: `/dynamixel/state/error`に相当, `pub_ratio/error`に一回データが埋められる．
 
- **`/dynamixel/external_port/read`** ([`DxlExternalPort`型](./dynamixel_handler_msgs#dynamixel_handler_msgsmsgdxlexternalport-type))    
  XH540とPシリーズが持つExternal Port機能を扱うための topic．以下の field からなる:
  - `stamp`: データが読み取れた時刻  
  - `id_list`: サーボのID
  - `port`: External Portのポート番号
  - `mode`: ポートのモード，analog in / digital out / digital in (pull up) / digital in (pull down)
  - `data`: ポートのデータ，モードに応じて 0--4096 (analog in) と 0 or 1 (digital ~) の値をとる．

#### コマンドライン確認用のステータス情報

  field は[メッセージの定義](./dynamixel_handler_msgs/ReadMe.md)を参照．  
  基本的に，`id_list`フィールドの長さとそれ以外の field の長さは一致する．   
  例外は`pub_outdated_present`パラメータが`false`が設定されている場合の`/dynaimxel/state/present`だけ．

- **`/dynamixel/state/status`** ([`DynamixelStatus`型](./dynamixel_handler_msgs#dynamixelstatus-type))  
  サーボの状態(トルク・エラー・ping・制御モード)を示す．     
  ※ エラーについてはハードウェアエラーの有無を示し，エラーの詳細は別途提供．

- **`/dynamixel/state/present`** ([`DynamixelPresent`型](./dynamixel_handler_msgs#dynamixelpresent-type))  
  サーボの現在値(位置、速度、電流など)を示す．    
  高速化のため，位置，速度などの要素個別で読み取り周期`pub_ratio/present.{~}`を設定できる．   
  ※ そのため最新のデータと非最新のデータが混在することになるが，非最新のデータを publish するかどうかは `pub_outdated_present`パラメータで設定可能．デフォルトは `true`なので古いデータも含め全ての field が埋まる．   

- **`/dynamixel/state/goal`** ([`DynamixelGoal`型](./dynamixel_handler_msgs#dynamixelgoal-type))  
  サーボの目標値(目標位置、目標速度など)を示す．   

- **`/dynamixel/state/gain`** ([`DynamixelGain`型](./dynamixel_handler_msgs#dynamixelgain-type))  
  サーボの制御ゲイン値を示す．   

- **`/dynamixel/state/limit`** ([`DynamixelLimit`型](./dynamixel_handler_msgs#dynamixellimit-type))  
  サーボの制限値(最大電流、最大速度など)を示す．   

- **`/dynamixel/state/error`** ([`DynamixelError`型](./dynamixel_handler_msgs#dynamixelerror-type))  
  サーボのハードウェアエラー情報の詳細を示す．   

- **`/dynamixel/debug`** ([`DynamixelDebug`型](./dynamixel_handler_msgs#dynamixeldebug-type))  
  デバッグ用の topic (サーボが動作しないときにコマンドラインで状況を確認する目的)．   

### Subscribed Topics

Subscribe 時にデータが一時保存され，直後のメインループ内で書き込みが行われるため，書き込みの最大周期は`loop_rate`[Hz]となる．  
詳細については[各種情報の分類](#各種情報の分類と-control-table-との対応)を参照

#### プログラム向け統合コマンド

  Subscribe したデータの各 field (`pwm_control`, `status`, ... など)の中で **`id_list`フィールドが埋まっているfieldのみ**処理される．   

- **`/dynamixel/commands/x`** ([`DxlCommandsX`型](./dynamixel_handler_msgs#dynamixel_handler_msgsmsgdxlcommandsx-type))    
  Xシリーズ用のコマンドを統合した topic．以下の field からなる：
  - `pwm_control`: `/dynamixel/command/x/pwm_control`に相当  
  - `current_control`: `/dynamixel/command/x/current_control`に相当  
  - `velocity_control`: `/dynamixel/command/x/velocity_control`に相当  
  - `position_control`: `/dynamixel/command/x/position_control`に相当  
  - `extended_position_control`: `/dynamixel/command/x/extended_position_control`に相当  
  - `current_base_position_control`: `/dynamixel/command/x/current_base_position_control`に相当  
  - `status`: `/dynamixel/command/status`に相当  
  - `gain`: `/dynamixel/command/gain`に相当  
  - `limit`: `/dynamixel/command/limit`に相当

- **`/dynamixel/commands/p`** ([`DxlCommandsP`型](./dynamixel_handler_msgs#dynamixel_handler_msgsmsgdxlcommandsp-type))    
  Pシリーズ用のコマンドを統合した topic．以下の field からなる： 
  - `pwm_control`: `/dynamixel/command/p/pwm_control`に相当  
  - `current_control`: `/dynamixel/command/p/current_control`に相当  
  - `velocity_control`: `/dynamixel/command/p/velocity_control`に相当  
  - `position_control`: `/dynamixel/command/p/position_control`に相当  
  - `extended_position_control`: `/dynamixel/command/p/extended_position_control`に相当  
  - `status`: `/dynamixel/command/status`に相当  
  - `gain`: `/dynamixel/command/gain`に相当  
  - `limit`: `/dynamixel/command/limit`に相当

- **`/dynamixel/commands/all`** ([`DxlCommandsAll`型](./dynamixel_handler_msgs#dynamixel_handler_msgsmsgdxlcommandsall-type))  
  X,Pシリーズを共通で扱うための topic. 以下の field からなる：
  - `status`: `/dynamixel/command/status`に相当  
  - `goal`: `/dynamixel/command/goal`に相当  
  - `gain`: `/dynamixel/command/gain`に相当  
  - `limit`: `/dynamixel/command/limit`に相当  
    ※ `{~}_control`系の field がないため制御モードの自動変更機能は無し.    
      `status.mode`で個別モードを指定し`goal.~`で各種目標値を与える．
    
- **`/dynamixel/external_port/write`** ([`DxlExternalPort`型](./dynamixel_handler_msgs#dynamixel_handler_msgsmsgdxlexternalport-type))    
  XH540とPシリーズが持つExternal Port機能を扱うための topic．以下の field からなる:
  - `stamp`: メッセージのタイムスタンプ  (無効)
  - `id_list`: 適用するサーボのID
  - `port`: 適用するExternal Portのポート番号
  - `mode`: ポートのモード，指定できるmodeは定数として定義されている．
  - `data`: ポートのデータ，モードが digital out の場合のみ有効
   
#### コマンドライン用個別コマンド

  field が省略されているものは[メッセージの定義](./dynamixel_handler_msgs/ReadMe.md)を参照．

  **Xシリーズ用の制御コマンド** : 関連するgoal値の設定＋制御モードの変更を行う. 
  - **`/dynamixel/command/x/pwm_control`** ([`DynamixelControlXPwm`型](./dynamixel_handler_msgs#dynamixelcontrolxpwm-type))  
  - **`/dynamixel/command/x/current_control`** ([`DynamixelControlXCurrent`型](./dynamixel_handler_msgs#dynamixelcontrolxcurrent-type))  
  - **`/dynamixel/command/x/velocity_control`** ([`DynamixelControlXVelocity`型](./dynamixel_handler_msgs#dynamixelcontrolxvelocity-type))  
  - **`/dynamixel/command/x/position_control`** ([`DynamixelControlXPosition`型](./dynamixel_handler_msgs#dynamixelcontrolxposition-type))  
  - **`/dynamixel/command/x/extended_position_control`** ([`DynamixelControlXExtendedPosition`型](./dynamixel_handler_msgs#dynamixelcontrolxextendedposition-type))  
  - **`/dynamixel/command/x/current_base_position_control`** ([`DynamixelControlXCurrentPosition`型](./dynamixel_handler_msgs#dynamixelcontrolxcurrentposition-type))

 **Pシリーズ用の制御コマンド** : 関連するgoal値の設定＋制御モードの変更を行う. 
  - **`/dynamixel/command/p/pwm_control`** ([`DynamixelControlPPwm`型](./dynamixel_handler_msgs#dynamixelcontrolppwm-type))  
  - **`/dynamixel/command/p/current_control`** ([`DynamixelControlPCurrent`型](./dynamixel_handler_msgs#dynamixelcontrolpcurrent-type))  
  - **`/dynamixel/command/p/velocity_control`** ([`DynamixelControlPVelocity`型](./dynamixel_handler_msgs#dynamixelcontrolpvelocity-type))  
  - **`/dynamixel/command/p/position_control`** ([`DynamixelControlPPosition`型](./dynamixel_handler_msgs#dynamixelcontrolpposition-type))  
  - **`/dynamixel/command/p/extended_position_control`** ([`DynamixelControlPExtendedPosition`型](./dynamixel_handler_msgs#dynamixelcontrolpextendedposition-type))

  **共通コマンド**
  - **`/dynamixel/command/status`** ([`DynamixelStatus`型](./dynamixel_handler_msgs#dynamixelstatus-type))       
    サーボの状態を設定する. 以下の field からなる：
    - `id_list`: 適用するサーボのID
    - `torque`: `true`/`false`で指定IDのトルクを安全にON/OFFする．
    - `error`: `false`, `true`のどちらが指定されていてもエラークリアする．
    - `ping`: `true`/`false`で指定したIDを認識リストへ追加/削除する．  
    - `mode`: [制御モードの文字列](./dynamixel_handler_msgs#dynamixelstatus-type)によって指定したIDの制御モードを変更．    
       ※ 各モードの`{~}_control`系の topic を送ることでも自動設定されるので，基本的には使わなくもてOK．   

  - **`/dynamixel/command/goal`** ([`DynamixelGoal`型](./dynamixel_handler_msgs#dynamixelgoal-type))      
    目標値(位置・速度・電流など)の設定する.  

  - **`/dynamixel/command/gain`** ([`DynamixelGain`型](./dynamixel_handler_msgs#dynamixelgain-type))    
    制御ゲイン値の設定する.  

  - **`/dynamixel/command/limit`** ([`DynamixelLimit`型](./dynamixel_handler_msgs#dynamixellimit-type))    
    制限値(最大速度、最大電流など)の設定する.    
    ※ limit はROM領域の値なので，書き込む場合torqueが強制的にOFFになることに注意．

  - **`/dynamixel/shortcut`** ([`DynamixelShortcut`型](./dynamixel_handler_msgs#dynamixelshortcut-type))    
    Dynamixelの起動、停止、エラー解除などのショートカットコマンド  
    - `command`: コマンド文字列, 指定できる文字列は[下記参照](#shortcut-command-list)．  
    - `id_list`: 適用するサーボのIDリスト, 省略すると認識されているすべてのIDを選択したのと同等となる．  
  
##### Shortcut Command list

`/dynamixel/shortcut`トピックの `command`フィールドに指定できる文字列．
`DynamixelShortcut`型の定義内で[定数として定義](./dynamixel_handler_msgs#dynamixelshortcut-type)されている．

- **高レベルコマンド**：ユーザの利用を想定
  - `torque_on`/ `TON` : 安全にトルクをenableにする．目標姿勢を現在姿勢へ一致させ，速度を0にする．
  - `torque_off`/ `TOFF`: トルクをdisableにする．
  - `clear_error`/ `CE`: ハードウェアエラー(ex. overload)をrebootによって解除する．   
    回転数の情報が喪失する問題を解消するために，homing offset用いて自動で補正する．
  - `remove_id`/ `RMID`: 指定したIDのサーボを認識リストから削除する．
  - `add_id`/ `ADID`   : 指定したIDのサーボを認識リストに追加する．

- **低レベルコマンド**：開発者向け
  - `reset_offset`: homing offset アドレスに 0 を書き込む．
  - `enable` : torque enable アドレスに true を書き込む．
  - `disable`: torque enable アドレスに false を書き込む．
  - `reboot` : reboot インストラクションを送る


***************************

## Parameters

### 通信関係の設定
```yml
# 通信機器の設定
  device_name: /dev/ttyUSB0 # 通信するデバイス名
  baudrate: 1000000 # 通信速度
  latency_timer: 4 # 通信のインターバル
```
基本的な通信の設定．自分の環境に合わせて設定する．

### 初期化・終了時等の挙動設定
```yml
# 通信の設定
  dyn_comm/retry_num: 10 # 通信失敗時のリトライ回数
  dyn_comm/inerval_msec: 5 # 通信失敗時のインターバル時間
  dyn_comm/verbose: false # 通信失敗時の詳細をエラーとして出すか
```
主に ping などのサーボ単体との通信失敗時の挙動を決める．    
基本的に数字が小さい方が初期化が早くなるが，サーボを発見できない可能性が高くなる．(デフォルト値は大きめに設定している)

`dyn_comm/verbose`が `true`の時は，通信失敗時に詳細なエラーを出力する．基本は `false`で問題ない．

```yml
# サーボの初期設定
  init/baudrate_auto_set: false # 探索前に，全てのサーボと全てのBaudrateに対して，baudrateの書き込みをするかどうか
  init/expected_servo_num: 0 # 期待するサーボの数，0ならいくつでもOK
  init/servo_auto_search:
      min_id: 0      # 探索するサーボのIDの最小値
      max_id: 30     # 探索するサーボのIDの最大値
      retry_times: 4 # 探索のリトライ回数
  init/hardware_error_auto_clean: true # 初期化時に Hardware error を自動でクリアするかどうか
  init/torque_auto_enable: true # 初期化時に Torque を自動でONにするかどうか
  term/torque_auto_disable: true # 終了時に Torque を自動でOFFにするかどうか
  term/servo_auto_stop: true # 終了時に電流・速度制御のサーボを停止するかどうか
```
`init/baudrate_auto_set`が `true`の時は，全てのサーボと全ての baudrate に対して，事前に `baudrate`の書き込みを行う．   
したがって，起動時にBaudrateが `baudrate`と異なるサーボがあっても自動的に `baudrate`に統一される．

`init/expected_servo_num`が `0`の時は，1つ以上servoが見つかるまでスキャンを繰り返す．  
`init/expected_servo_num`が `0`でない場合は，その数だけservoが見つかるまでスキャンを繰り返す．  
`init/servo_auto_search.retry_times`の回数分のスキャンが失敗した場合，初期化失敗で`dynamixel_handler`ノードは落ちる．

```yml
# デフォルト値の設定
  default/profile_acc: 600.0 # deg/s^2
  default/profile_vel: 100.0 # deg/s
  default/return_delay_time: 0.0 # us [double]
```
`default/profile_acc`と`default/profile_vel`は位置制御時の最大加速度と最大速度を決める．
この値が大きければキビキビとした動作になり，小さければ滑らかな動作になる．
`{~}_control`系の topic で動的に指定することも可能．

`default/return_delay_time`はサーボの応答遅延時間を設定する．基本的に0でいいはず．

### 実行時の動作設定
```yml
# ループの設定
  loop_rate: 100 # メインループの周期
  verbose_ratio: 300 # メインループのlog出力の割合(処理時間，通信の成功率), ex 100なら100回に1回出力
  pub_outdated_present_value: true # 最新でないpresent_XXXをpublishするかどうか. false ならば 直近のループでreadした Present の要素のみmsgに含められる．
  pub_ratio/present: # present_XXXを読み取り，/dynamixel/state/present トピックをpublishする割合
      pwm:                 0 # この回数に一回present_pwmを読み取る, 0=初回のみ
      current:             2 # この回数に一回present_currentを読み取る, 0=初回のみ
      velocity:            2 # この回数に一回present_velocityを読み取る, 0=初回のみ
      position:            2 # この回数に一回present_positionを読み取る, 0=初回のみ
      velocity_trajectory: 0 # この回数に一回velocity_trajectoryを読み取る, 0=初回のみ
      position_trajectory: 0 # この回数に一回position_trajectoryを読み取る, 0=初回のみ
      input_voltage:      29 # この回数に一回present_input_voltageを読み取る, 0=初回のみ
      temperature:        29 # この回数に一回present_temperatureを読み取る, 0=初回のみ
  pub_ratio/status: 47 # この回数に一回 Status を読み取り，/dynamixel/state/status トピックをpublishする, 0=初回のみ
  pub_ratio/goal: 11  # この回数に一回 Goal を読み取り，/dynamixel/state/goal トピックをpublishする, 0=初回のみ
  pub_ratio/gain: 101 # この回数に一回 Gain を読み取り，/dynamixel/state/gain トピックをpublishする, 0=初回のみ
  pub_ratio/limit: 307 # この回数に一回 Limit を読み取り，/dynamixel/state/limit トピックをpublishする, 0=初回のみ
  pub_ratio/error: 53 # この回数に一回 Hardware error を読み取り，/dynamixel/state/error トピックをpublishする, 0=初回のみ
```
`pub_ratio/{~}`の各情報 (status, present, goal, gain, limit, error) は topic 名と対応．   
read と publish 周期は`loop_rate`を`pub_ratio/{~}`で割った値となる．   
> 例: `loop_rate`= 100, `pub_ratio/status`= 47 の時 100/47 ≃ 2Hz．   

デフォルト値を素数にしているのは, serial read のタイミングが被って1ループの処理時間が長くなるのを防ぐため．

present値のみ高速化のために各アドレス(pwm, current, ... , temperature)の読み取り割合を設定できる．   
`~/present`トピックの publish 周期は`loop_rate`を`pub_ratio/present.{~}`の最小値で割った値となる．   
> 例: `loop_rate`= 100, `pub_ratio/present.current`= 2 の時 100/2 = 50Hz．

このため，present値の直近で読み取った最新の値と，古い値が混在することになる．   
`pub_outdated_present_value`が`true`の場合は古い値も含めて全てのアドレスの値を publish する．   
`pub_outdated_present_value`が`false`の場合は，直近のループで読み取った値のみを publish する．  

```yml
# Read/Write方式
  method/fast_read: false # Fast Sync Read を使うかどうか
  method/split_read: false # 複数の情報を分割して読み取るかどうか
  method/split_write: true # 複数の情報を分割して書き込むかどうか
```
`method/fast_read`が `true`の場合は Fast Sync Read を使い，`false`の場合は Sync Read を使う．それぞれの違いは[公式の動画](https://www.youtube.com/watch?v=claLIK8omIQ)を参照されたし．   
基本的に`true`の方が高速．ファームウェアが古い場合は `false`でないと情報が読み取れないのでデフォルトは`false`．   

`method/split_read`が`true`の場合は，各情報を分割して読み取り，`false`の場合は一括で読み取る．   
`false`の方が高速．`true`の方が安定．

`method/split_write`が`true`の場合は，各情報を分割して書き込み，`false`の場合は一括で書き込む．    
`false`の方が高速だが，あまり差はない．`true`の方が安定．

> [!NOTE]
> 上記のread方式による速度の違いについては[速度に関してメモ](#速度に関してメモ)も参照.

### log出力関係
```yml
# デバッグ用
  max_log_width: 6 # 以下のlog出力で，サーボ何個ごとに改行を入れるか
  verbose/callback: true # コールバック関数の呼び出しを出力
  verbose/write_goal: false # 書き込みするgoalデータのpulse値を出力
  verbose/write_gain: false # 書き込みするgainデータのpulse値を出力
  verbose/write_limit: false # 書き込みするlimitデータのpulse値を出力
  verbose/read_status/raw: false # 読み込んだstatusデータのpulse値を出力
  verbose/read_status/err: false # statusデータの読み込みエラーを出力
  verbose/read_present/raw: false # 読み込んだpresentデータのpulse値を出力
  verbose/read_present/err: false # presentデータの読み込みエラーを出力
  verbose/read_goal/raw: false # 読み込んだgoalデータのpulse値を出力
  verbose/read_goal/err: false # goalデータの読み込みエラーを出力
  verbose/read_gain/raw: false # 読み込んだgainデータのpulse値を出力
  verbose/read_gain/err: false # gainデータの読み込みエラーを出力
  verbose/read_limit/raw: false # 読み込んだlimitデータのpulse値を出力
  verbose/read_limit/err: false # limitデータの読み込みエラーを出力
  verbose/read_hardware_error: true # 検出したHardware errorを出力
```
基本的には上記の説明の通り．
`verbose/write_{~}`や`verbose/read_{~}`が`true`の場合は，書き込むアドレスと書き込むパルス値を直接確認できる．

### 開発用
```yml
# 開発用
  debug: false # true: デバイスとの接続に失敗してもエラーを出力しない/ false: エラーを出力して終了する
  no_use_command_line: false # true: プログラム用の topic のみ使用する/ false: コマンドライン用の topic も出力する
  dummy_servo_id_list: [6,7] # ダミーのサーボを作成する,同じIDのサーボが存在する場合でもダミーが優先される．
```
`debug`が`true`の場合は，デバイスとの接続失敗やサーボが見つからなかった場合でもプログラムが続行する．

`no_use_command_line`が`true`の場合は，コマンドライン用の topic を publish & subscribe しないので，`$ ros2 topic list`がすっきりする．起動時にDDSがネットワークにかける負荷も小さくなるかも？(未確認)

`dummy_servo_id_list`に指定したIDは，実際のサーボが存在しなくても，プログラム上ダミーのサーボが作成される．つまり何もつながなくても `/dynamixel/states`などの topic がpublishされる．       
実機がない場合での上位の node の動作確認などに便利．
指定したIDのサーボがつながれている場合でもIDが指定されていればダミーサーボが優先されるので，実際のサーボとの通信は行われない．      
ロボットを動かさずに上位の node の動作確認がしたいときに，ロボット全部のサーボのIDをダミーに指定してやると安全に動作確認ができる．

### Optional機能
Dynamixelの動作に直接関連しない，Optional 機能の設定．
現在は特定のモデルに存在する External Port の設定のみ．Dynamixelプロトコル対応のIMUなどの機能を追加する予定．
```yml
# Optonal機能
  option/external_port:
      use : false # External Portの機能を使うかどうか
      pub_ratio/data : 2   # この回数に一回 Data を読み取る.
      pub_ratio/mode : 100 # この回数に一回 Mode を読み取る, ROM値なので大きくても問題ない．
```

***************************

## 各種情報の分類と Control Table との対応

本パッケージでは，Dynamixelが持つ Control table 内の情報を，以下の様に分類して扱う．

### 状態 (status)
 - torque_enable  : サーボが制御を受け付けるかどうか．falseだと脱力してトルクを出さない．  
 - (ping)         : Control table の情報ではないが，statusとして扱っている．pingが通るかどうか．
 - (error)        : Control table の情報ではないが，statusとして扱っている．何らかのエラーを持っているかどうか．
 - operating_mode : サーボの制御モード，PWM, 電流制御, 速度制御, 位置制御, 拡張位置制御, 電流制御付き位置制御の6つのモードがある．
 -  
##### Subscrib / Write  
Xシリーズの場合，`/dynamixel/commands/x`の`status`フィールド or `/dynamixel/command/status`によって設定され，`loop_rate`の周期で書き込まれる．
operationg_modeのみ，対応する`/dynamixel/command/x/{~}_control`系の topic を subscribe することでも自動で設定される． 
##### Publish / Read  
`loop_rate`の内`pub_ratio/status`毎に1回の周期で読みだされ，`/dynamixel/states`の`status`フィールド and `/dynamixel/state/status`として publish される．

### 目標値 (goal)
 - goal_pwm             : 目標PWM値, PWM制御モードでのみ有効
 - goal_current         : 目標電流値, 電流制御モードと電流制御付き位置制御でのみ有効
 - goal_velocity        : 目標速度, 速度制御モードでのみ有効
 - goal_position        : 目標角度, 位置制御モードと拡張位置制御モード，電流制御付き位置制御で有効
 - profile_acceleration : 最大加速度, 速度制御・位置制御・拡張位置制御・電流制御付き位置制御モードで有効
 - profile_velocity     : 目標速度値, 位置制御モードと拡張位置制御モード，電流制御付き位置制御で有効 

##### Subscrib / Write
Xシリーズの場合，`/dynamixel/commands/x`or `/dynamixel/command/x/{~}_control`系の topic  or `/dynamixel/command/goal`によって設定され，`loop_rate`の周期で書き込まれる．   
##### Publish / Read
`loop_rate`の内`pub_ratio/goal`毎に1回の周期で読みだされ，`/dynamixel/states`の`goal`フィールド and `/dynamixel/state/goal`として publish される．

### 現在値 (present)
 - present_pwm          : 現在のPWM値
 - present_current      : 現在の電流値
 - present_velocity     : 現在の速度
 - present_position     : 現在の角度
 - velocity_trajectory  : 目標速度のようなもの
 - position_trajectory  : 目標角度のようなもの
 - present_input_voltage: 現在の入力電圧
 - present_temperature  : 現在の温度

##### Subscrib / Write
書き込みは不可．   
##### Publish / Read
各アドレスの情報は`loop_rate`の内`pub_ratio/present.{~}`に一回の周期で読みだされる．   
読みだされた情報は，`loop_rate`の内`pub_ratio/present.{~}`の最小値の割合で，`/dynamixel/states`の`present`フィールド and `/dynamixel/state/present`として publish される．

### ゲイン (gain)
 - velocity_i_gain       :
 - velocity_p_gain       :
 - position_d_gain       :
 - position_i_gain       :
 - position_p_gain       :
 - feedforward_acc_gain  :
 - feedforward_vel_gain  :
  
##### Subscrib / Write
Xシリーズの場合，`/dynamixel/commands/x`の`gain`フィールド or `/dynamixel/command/gain`によって設定され，`loop_rate`の周期で書き込まれる．
##### Publish / Read
`loop_rate`の内`pub_ratio/gain`毎に1回の周期で読みだされ，`/dynamixel/states`の`gain`フィールド and `/dynamixel/state/gain`として publish される．

> [!note]
> 制御モードによってデフォルト値が異なり，モードを変えると勝手に書き換えられてしまう．制御モードをまたぐ場合の処理については検討中．

### 制限 (limit)
 - temperature_limit : 温度がこの値を超えると Hardware error (overheating) が発生する．
 - max_voltage_limit : 入力電圧がこの値を超えると Hardware error (input_voltage) が発生する．
 - min_voltage_limit : 入力電圧がこの値を下回ると Hardware error (input_voltage) が発生する．   
 - pwm_limit         : 指定・発揮できるPWMの最大値
 - current_limit     : 指定・発揮できる最大電流値
 - acceleration_limit : 最大加速度 (読み書きが有効なのはPシリーズのみ)  
 - velocity_limit     : 最大速度
 - max_position_limit : 位置制御モードでの最大角度   
 - min_position_limit : 位置制御モードでの最小角度
  
##### Subscrib / Write
Xシリーズの場合，`/dynamixel/commands/x`の`limit`フィールド or `/dynamixel/command/limit`によって設定され，`loop_rate`の周期で書き込まれる．
##### Publish / Read
`loop_rate`の内`pub_ratio/limit`毎に1回の周期で読みだされ，`/dynamixel/states`の`limit`フィールド and `/dynamixel/state/limit`として publish される．

> [!note]
> ROM領域の値の書き込みは torque が on  状態ではできないため，limit の書き込みが発生する場合は一瞬だけ torque を off にする処理が入ることに注意．

### エラー (error)
 - hardware_error_status  : サーボのハードウェアエラー情報    
   検知されるエラーは以下の7種類．
   - input_voltage : 入力電圧が制限範囲外
   - motor_hall_sensor : モータのホールセンサの異常
   - overheating   : 温度が最大値を超えた
   - motor_encoder : モータのエンコーダの異常
   - electronical_shock : 電子回路内での異常
   - overload      : 過負荷

##### Subscrib / Write
書き込みは不可．  
##### Publish / Read
`loop_rate`の内`pub_ratio/error`毎に1回の周期で読みだされ，`/dynamixel/states`の`error`フィールド and `/dynamixel/state/error`として publish される．
  
### その他 (extra)
 - drive_mode             : 
 - return_delay_time      : Dynamixelの応答遅延時間，デフォルトで$500$usだが，この pkg では初期化時に$0$usに上書きされる．
 - homing_offset          : この値が現在の真の角度に加算されて出力される．この pkg では，reboot時の角度補正に用いられる．
 - moving_threshold       : 
 - startup_configuration  : not support, buckupがあるときPIDゲインやprofile系の値を自動で復元してくれるが，PIDのデフォルト値がモードによって異なる問題があるので使わない．
 - shutdown               : 
 - status_return_level    : not support, 常に2を前提とする
 - bus_watchbdog          : 指定時間通信が行われない場合に動作を停止する(トルクOFFとは異なる)，この pkg では node kill 時と通信断絶時にサーボを自動停止させる機能に用いられる．   
 - led                    : 
 - registered_instruction : 
 - realtime_tick          : 
 - moving                 : 
 - moving_status          : 

読み書きは未実装

> [!note] 
> (bus_watchdog の設定値が1以上の時) bus_watchdogの設定値 × 20ms 通信がないと自動で動作停止処理が実行される．
> しかし，位置制御系のモードかつ，homing_offset が設定されている状態でこの動作停止処理が走るとなぜか homing_offsetだけ回転する．
> firmwareのバグの模様，バージョンによってはこの問題が解消されているかもしれないが，危険なので位置制御系のモードで bus_watchdog を使うのは避けるべき．
> ということで `term/servo_auto_stop`が`true`かつ，PWM・電流・速度制御モードの場合のみ，$500$ms で停止するように bus_watchdog を設定し，それ以外のモードでは bus_watchdog は $0$ として利用しないようにしている．

### External Ports
 - external_port_data_{1,2,...} : 外部ポートのデータ，末尾の数字がポート番号に対応(Xシリーズは1,2,3，Pシリーズは1,2,3,4).
 - external_port_mode_{1,2,...} : 各外部ポートのモード. 以下の4つのmodeがある．
     - analog input : 0.0v ~ 3.3v のアナログ値を読み取り 0 ~ 4095 の値としてdataに格納する
     - digital output : data の値 0 or 1 に応じて 0v or $3.3$v を出力する
     - digital input (pullup) : プルアップされたデジタル値を読み取り，1 or 0 としてdataに格納する
     - digital input (pulldown) : プルダウンされたデジタル値を読み取る，1 or 0 としてdataに格納する

> [!note]
> X540シリーズとPシリーズのみに搭載される機能．

利用する場合は `option/external_port.use`を`true`に設定する．
##### Subscrib / Write
`/dynamixel/external_port/write`トピックによって設定され，`loop_rate`の周期で書き込まれる．
##### Publish / Read
デフォルトでは読み込みは行われず，`/dynamixel/external_port/write`トピックによって指定されたことのあるIDのみから読み込みが行われる．    
`loop_rate`の内`option/external_port.pub_ratio/mode`毎に1回の周期で mode が読みだされ，
`option/external_port.pub_ratio/data`毎に1回の周期で data が読みだされる．    
mode or data のどちらか一方でも読みだされた場合`/dynamixel/external_port/read`トピック として publish される．

***************************

## Baudrate の一括変更

`config/config_dynamixel_unify_baudrate.yml`の以下の部分を編集し，保存
```yml
# config/config_dynamixel_unify_baudrate.yml
/**:
    ros__parameters:
        # 通信機器の設定
        device_name: /dev/ttyUSB0 # 通信するデバイス名
        target_baudrate: 1000000 # 統一したい通信速度
```
ターミナルを開いて次を実行
```bash
ros2 launch dynamixel_handler dynamixel_unify_baudrate_launch.xml
```
全てのdynamixelの baudrate を`target_baudrate`に設定してくれる．

***************************

## Latency Timer

シリアル通信にはパケットの送受信の間に latency timer 分のインターバルが挟まる．
(USBデバイスのデフォルトは16msのようであり，高速な通信の妨げとなることが多い)
安定した通信のためには，使用するUBSデバイスの latency timer と ros param の `laytency_timer`を一致させる必要がある．

ros param の変更には，`config/config_dynamixel_handler.yml`の以下の部分を編集して保存する．
```yml
# config/config_dynamixel_handler.yml
latency_timer: 4 # 通信のインターバル
```

使用するUSBデバイスの latency timer はコマンドラインから次のコマンドを実行することで変更できる．
基本的に1度だけ実行すればよい．
```bash
echo ACTION==\"add\", SUBSYSTEM==\"usb-serial\", DRIVER==\"ftdi_sio\", ATTR{latency_timer}=\"4\" > 99-dynamixelsdk-usb.rules
sudo cp ./99-dynamixelsdk-usb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add
rm 99-dynamixelsdk-usb.rules
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer # ttyUSB0の部分は環境に合わせて変更すること
```

一時的であれば以下のようにしてもよい．
`ttyUSB0` の部分は自分の環境に合わせて編集すること．
```bash
echo 4 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

***************************

## 速度に関してメモ

### Sync Read vs Fast Sync Read(`method/fast_read`パラメータ)
結論としては，読み込むデータとサーボの数がそこまで多くないならFastを使う方がよい．

Fast Sync Readは受信するパケットが全サーボ分1つながりになっている．     
つまり1回の通信でやり取りするパケット数が少なくなるので，同時に読み書きするサーボが多くなると速度に違いが出てくる．    
少なくとも10サーボくらいで顕著にFast Sync Readの方が早くなる．   
ただ，サーボが多すぎると通信が不安定になることがある．

Fast Sync Read側のデメリットとしては，直列しているサーボのどこかで断線等が起きた場合に弱いという点が挙げられる．
Fast Sync Readはパケットがつながっているため，1つでも返事をしないサーボがあるとパケットが全滅してしまう．   
（これはlib_dynamixelのパケット処理の実装が悪いかもしれないが，知識不足ですぐに改善できなさそう．）    
通常のSync Readはパケットが独立しているため，断線するより前のサーボからの返事は受け取ることができる．    
断線や接続不良が危惧されるような状況では通信周期を犠牲にして，Sync Readを使わざるを得ないだろう．

### 複数アドレスの同時読み込み(`method/split_read`パラメータ)
後述の書き込みと異なり，こちらは分割ではなく同時にするのが良い．    
すなわち`method/split_read`は`false`を推奨する．

複数のアドレスからデータを読み込みたいとき，分割して読み込む場合はシリアル通信の処理時間が，アドレス数分だけ長くなる．    
100Hz以上で回そうと思うと，present_current, present_velocity, present_positionという基本の3つを取り出すだけでもきつい．    
自分の環境では，前述の3つくらいの同時読み込みであれば，120-180Hzくらいでる．200Hzは場合によって出るか出ないかというところ．    
分割読み込みでは60-80Hzくらいで頭打ちとなってしまった．
present系の8つのアドレスすべてから読み込んでも，同時読み込みなら100Hzくらいはでる．
分割読み込みだと30Hzも怪しい．

（上記は全て， 14サーボ直列，lib_dynamixel側のLATENCY_TIMER=2ms, デバイス側のlatency timer=2ms, baudrate=1M での結果）

### 複数アドレスの同時書き込み(`method/split_write`パラメータ)
書き込みに関しては，同時ではなく分割するのが良いだろう．    
すなわち`method/split_write`は`true`を推奨する．    

自分の環境では，`method/split_write`を`false`の状態で，12サーボに goal_current, goal_velocity, profile_acc, profile_vel, goal_position を同時にSync Writeしようとしたら，書き込みが失敗してうまく動かなかった．    
書き込むサーボが少なければ動く．     
また，`use_split_write`を`true`にして，分割で書き込み，1度に書き込むアドレスを減らしても動く．    
書き込みに関しては，分割して行っても処理時間はほぼ変わらない(1ms未満しか遅くならない)ので，基本は`true`としておくべき．


***************************
***************************


## Trouble Shooting

### 「dynamixel_handlerがない」もしくは，「メッセージがない」といったエラーが出る場合

ターミナルを立ち上げ直すか，以下を実行．
```bash
source ~/ros2_ws/install/setup.bash
```

### `dynamixel_handler`ノードを起動してもdynamixelが1つも見つからない場合

1. デバイス名の確認   
   どんな方法で確認しても良いが，Ubuntuの場合はDynamixelを認識するはずのUSBを抜く前後で `$ ls /dev/ttyUSB*`の出力を比較すれば，少なくとも正しいデバイス名がわかる．
1. デバイスの実行権限の確認   
   `$ sudo chmod 777 /dev/{your device name}`として改善すれば実行権限問題であることがわかる．
1. Dynaimxel側の baudrate の確認    
   Dynamixel wizardで確認するのが最も確実．
   とりあえず動くようにするには `init/baudrate_auto_set`パラメータを `true`にセットするか,
    [`dynamixel_unify_baudrate`ノード](#Baudrateの一括変更) で目的の baudrate に強制変換してしまうのが良い．

### `dynamixel_handler`ノードを起動したときにdynamixelが一部しか見つからない

1. laytency timer の確認    
   `$ cat /sys/bus/usb-serial/devices/{your device name}/latency_timer`を実行して出てきたデバイス側の latency timer の数値が，ros parameter の `latency_timer`と一致しているかどうかを確認する．  
   ros parameter 側の値を変えたい場合は config ファイルを修正すればよい．   
   デバイス側値を変えたい場合は [Latency Timer](#latency-timer) を参照．
3. 通信状態が悪すぎる場合   
   根本的にはケーブルやノイズ等を改善すべきだが，対症療法的に`dyn_comm/retry_num`を大きくすることでも改善する可能性がある．
   [Parameters](#parameters) の該当パラメータを参照

### ``cannot publish data``といったようなエラーが出た場合
デフォルトのDDSはFast-RTPSであるが，固有のバグを持っているらしく，実行時にエラーが発生する．
そのため，DDSをEclipse Cyclone DDSに変更しておく．

```bash
sudo apt update
sudo apt install ros-humble-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

※ ~/.bashrcの下部に``export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp``を追記しておくことで，\
ターミナルの立ち上げ時に毎回コマンドを打たなくて済む．

### wslにusbをアタッチしようとして以下のエラーが出たとき
`
usbipd: error: WSL 'usbip' client not correctly installed. See https://github.com/dorssel/usbipd-win/wiki/WSL-support for the latest instructions.
`
なんかようわからんが，以下のコマンドをwsl内で実行すると解決する．
```bash
sudo update-alternatives --install /usr/local/bin/usbip usbip `ls /usr/lib/linux-tools/*/usbip | tail -n1` 20
```

## Launchファイルと設定（yaml）
### launch_dynamixel_unify_baudrate.py
```bash
ros2 launch dynamixel_handler launch_dynamixel_unify_baudrate.py
```
対応する``yaml``は``config/config_dynamixel_handler.yaml``

### launch_dynamixel_handler.py
```bash
ros2 launch dynamixel_handler launch_dynamixel_handler.py
```
対応する``yaml``は``config/config_dynamixel_unify_baudrate.yaml``

※ 一度ビルドしていれば，yamlファイルの変更に伴うビルドは不要

