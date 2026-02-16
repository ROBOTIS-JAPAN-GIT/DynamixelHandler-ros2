# DynamixelHandler-ros2

Robotis社の[Dynamixel](https://e-shop.robotis.co.jp/list.php?c_id=89)をROSから制御するための `dynamixel_handler`パッケージとその周辺パッケージを提供するリポジトリ.  

ROS2(Humble)のみ対応，ROS1 ver は[こちら](https://github.com/ROBOTIS-JAPAN-GIT/DynamixelHandler-ros1).ただし，開発が分離しているので機能はやや異なる．

X, P, Proシリーズに対応，X320シリーズ，Yシリーズは順次対応予定．

## Table of Contents
  - [How to install](#how-to-install)
  - [How to use](#how-to-use)
  - [Topics](#topics)
  - [Parameters](#parameters)
  - [Optional機能](#optional機能)
  - [各種情報の分類と Control Table との対応](#各種情報の分類と-control-table-との対応)
  - [Baudrateの一括変更](#baudrateの一括変更)
  - [Trouble Shooting](#trouble-shooting)

## Features of this package
 - **Dynamixel制御に特化した最小単位のパッケージ**

    <details>
    <summary>詳細</summary>
    
    - `dynamixel_handler`パッケージが提供する **`dynamixel_handler`** ノードは，サーボモータとの通信を担う．
    - サーボの動作制御はユーザーが開発する **別の制御ノード**が行い，`dynamixel_handler`ノードは通信の仲介を行うイメージ．
    - ユーザーはシリアル通信の通信プロトコルや，コントロールテーブルついて知る必要が(あまり)ない．
    - Dynamixelの各種情報や機能が[適切に分類](#各種情報の分類と-control-table-との対応)され，ほぼすべての情報・機能を利用することが可能
    </details>
  
 - **ROSトピックのみで制御できるシンプルなインターフェース**   

    <details>
    <summary>詳細</summary>

    トピック通信だけで利用できるので，このパッケージ自体のコードの編集は不要 
    - **Subscribe**: `/dynamixel/commands/x`(他、`/dynamixel/commands/p`など)
      - シリーズごとに定義された制御コマンドを送ることで動作制御．
      - 制御コマンドの内容に合わせてサーボの制御モードが自動変更．
    - **Publish**: `/dynamixel/states` 
      - [分類された各情報](#各種情報の分類と-control-table-との対応)を個別の周期で自動的に read & publish．
        - "status": torqueのオンオフ, errorの有無, pingの成否, 制御モード (デフォルト 約2Hz)
        - "error": overload エラー などのハードウェアエラー (デフォルト 約2Hz)  
        - "present": current [mA], velocity [deg/s], position [deg] などの現在値 (デフォルト 約50Hz)
        - etc... (goal, limit, gain, extra)
    </details>

 - **物理量ベースでのやり取り**

    <details>
    <summary>詳細</summary>

    - current [mA], velocity [deg/s], position [deg] などの物理量を直接扱える．
    - Dynamixelから read されたパルス値(0 ~ 4095 など)は物理量に変換してから publish される． 
    - Dynamixelへの目標値は物理量で入力され，内部でパルス値に変換されてから write される．  
     
    ※ 特に角度を rad (-π ~ π) ではなく degree (-180.0 ~ 180.0) で扱うので直感的．
    </details>

 - **高速で安定したRead/Write** 

    <details>
    <summary>詳細</summary>
    
   - 読み書きをできるだけ一括で行うことで通信回数を削減  
      - 連続アドレスの一括読み書き
      - 複数サーボの一括読み書き (SyncRead/SyncWrite)
    - Fast Sync Read インストラクションによるReadの高速化
      - 通信形式についての細かい知識が無くてもros paramで設定するだけで利用可能．  
    - `dynamixel_handler`ノードの loop 周期に同期したRead/Write  
      - Topicの受信頻度に依存しないため，安定した通信が可能．

    ※ 12サーボ同時Read/Writeでも150Hz程度の通信が可能なことを確認  
    ※※ 高速通信には適切な [`latency timer`](#latency-timer)  (1~4ms) と `baudrate`(推奨 1,000,000bps 以上) が必要
    </details>

 - **開発の手間を減らす便利機能**

    <details>
    <summary>詳細</summary>
    
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
    </details>

 - **ROSパラメータによる各種ログ表示制御**

    <details>
    <summary>詳細</summary>
    
   - Read/Write にかかる平均時間とSerial通信の成功率
   - CallBackした内容 (Optional)
   - Read/Write されるパルス値 (Optional)
   - Readに失敗したID (Optional)
   - etc...
    </details>
  
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
> [!NOTE]
> 既存クローンでブランチ更新した場合（submodule の配置変更を含む変更を pull / merge した場合）は，
> リポジトリ直下で一度 `git submodule update --init --recursive` を実行してから build すること．

### ビルド
```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-up-to dynamixel_handler
source ~/ros2_ws/install/setup.bash # 初回 build 時のみ
```

***************************

## How to use

- コマンドラインを用いた `dynamixel_handler`ノードの動作確認
- 他の制御用パッケージと連携した `dynamixel_handler`ノードの利用

上記を達成するための手順を以下に示す．

### 1. Dynamixelの接続

Dynamixel Wizardでモータの動作確認ができる程度の状態を想定．

- Dynamixel をディジーチェーンにしてU2D2経由でUSB接続されていること．
- ID に重複がないように事前に設定されていること.
- baudrate が全て統一されていること．

> [!TIP]
> `init/baudrate_auto_set`パラメータを`true`にすることで，接続時に自動で baudrate を変更することも可能．
> また，別ノードとして，baudrate を一括で変更するための [`dynamixel_unify_baudrate`ノード](#Baudrateの一括変更) も用意してある

### 2. `dynamixel_handler`ノードの起動

#### 2-1. 自分の環境とconfigの設定を合わせる
リポジトリ内の`dynamixel_handler/config/config_dynamixel_handler.yaml`の該当部分を編集し，保存．   
以下は baudrate: $57600$ かつ device name: `/dev/ttyUSB0`かつ latency timer: $16$ ms の場合
```yaml
# config/config_dynamixel_handler.yaml
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
[dynamixel_handler-1] 00000.00000:  > series: X [use], P [no use], PRO [no use]
[dynamixel_handler-1] 00000.00000:   * P series servo id [1] is ignored 
[dynamixel_handler-1] 00000.00000:   * X series servo id [6] is found 
[dynamixel_handler-1] 00000.00000:    ID [6] is enabled torque 
[dynamixel_handler-1] 00000.00000:   * X series servo id [7] is found 
[dynamixel_handler-1] 00000.00000:    ID [7] is enabled torque 
[dynamixel_handler-1] 00000.00000:  ... Finish scanning Dynamixel 
[dynamixel_handler-1] 00000.00000: ..... DynamixelHandler is initialized 
[dynamixel_handler-1] 00000.00000: time=0.03ms/loop(5.11ms/read), success=100%(full=100%) 
[dynamixel_handler-1] 00000.00000: time=0.50ms/loop(5.11ms/read), success=100%(full=100%)  
[dynamixel_handler-1] 00000.00000: time=0.56ms/loop(5.11ms/read), success=100%(full=100%)  
[dynamixel_handler-1] 00000.00000: time=0.89ms/loop(5.11ms/read), success=100%(full=100%) 
```

連結したDynamixelが自動で探索され，見つかったDynamixelの初期設定が行われる．   
うまく見つからない場合は [Trouble Shooting](#trouble-shooting) を参照.     
初期化時の動作設定については [Parameters](#parameters) の章の[初期化・終了時等の挙動設定](#初期化終了時等の挙動設定)を参照.

### 3. Dynamixelの動作を制御

コマンドラインから指令する用の topic として`/dynamixel/command/...`と`/dynamixel/shortcut`トピックが用意されている．     
トピックを送るだけで動作の制御が可能である．

topic の詳細については [Topic](#subscribed-topics) の章を参照．

以下の例では ros2 topic コマンドを利用して topic をコマンドラインから publish しているが，別のノードを作成して同様の topic を publish することでも動作を制御できる．

#### 例：ID:5の Dynamixel(Xシリーズ) を位置制御モードで角度を90degに制御

`/dynamixel/command/x/position_control`トピックにIDと角度を設定して publish．
```bash
# 2-2. とは別のターミナルを開いて $ source ~/ros2_ws/install/setup.bash 実行後
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
# 2-2. とは別のターミナルを開いて $ source ~/ros2_ws/install/setup.bash 実行後
ros2 topic pub /dynamixel/shortcut \
 dynamixel_handler_msgs/msg/DynamixelShortcut \
 "{command: 'torque_off', id_list: [5]}"
```
実機のサーボのトルクがOFFになれば成功．

`/dynamixel/shortcut`トピックで使えるコマンドについては[こちら](#dynamixelshortcut-dynamixelshortcut型) を参照．

### 4. Dynamixelの情報を取得

コマンドラインから確認する用の topic として`/dynamixel/state/...`と`/dynamixel/debug`トピックが一定周期で pub され続けている．   

topic の詳細については [Topic](#published-topics) の章を参照．    
また，read　周期については [Parameters](#parameters) の章の[実行時の動作設定](#実行時の動作設定)を参照．

#### 例: Dynamixel の現在値の確認

```bash
# 2-2. とは別のターミナルを開いて $ source ~/ros2_ws/install/setup.bash 実行後
ros2 topic echo --flow-style /dynamixel/state/present # status, goal, gain, limit, error... など． 
```
```yaml
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

#### 例: Dynamixel のデバッグ用情報の確認

```bash
# 2-2. とは別のターミナルを開いて $ source ~/ros2_ws/install/setup.bash 実行後
ros2 topic echo --flow-style /dynamixel/debug
```
```yaml
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

### 6. ダミーサーボ機能を利用してデバッグする

`init/dummy_servo_list` パラメータに $1, \ldots, 254$ のIDを指定することで，実際のサーボが接続されていなくても，そのIDのサーボをダミーサーボとして簡易的にシミュレートすることができる．
最も簡単に利用する方法は，`dynamixel_handler`ノードの起動時にパラメータを指定することである．

```bash
ros2 run dynamixel_handler dynamixel_handler --ros-args -p init/dummy_servo_list:=[1,2]
```

実サーボとダミーサーボを同時に利用することも可能である．  
その場合はリポジトリ内の`dynamixel_handler/config/config_dynamixel_handler.yaml`の`init/dummy_servo_list`パラメータを編集する．
```yaml 
# config/config_dynamixel_handler.yaml
    # サーボの設定
        init/dummy_servo_list: [1, 2] # ダミーのサーボを作成するIDのリスト,同じIDのサーボが存在する場合でもダミーが優先される． -1や255は無視される．
```

## Topics

### Summary

一般的なユースケースでは以下の2つの topic を利用すれば十分である． 
- `/dynamixel/states`([dynamixel_handler_msgs::msg::DxlStates](./dynamixel_handler_msgs#dynamixel_handler_msgsmsgdxlstates-type))   
  連結したサーボすべての状態をまとめた topic, `dynamixel_handler`から publish される．
- `/dynamixel/commands/x`([dynamixel_handler_msgs::msg::DxlCommandsX](./dynamixel_handler_msgs#dynamixel_handler_msgsmsgdxlcommandsx-type))   
  Xシリーズのサーボを制御するための topic, `dynamixel_handler`に subscribe される．

ヘッダーファイルや型など，具体的な使い方ついては，[`dynamixel_handler_examples`](./dynamixel_handler_examples) を参照されたし．    
型そのものについての詳細は[メッセージの定義](./dynamixel_handler_msgs/ReadMe.md)を参照のこと．

<details>
<summary> P, Pro シリーズを利用する場合 </summary>

以下のトピックを利用する．

 - `/dynamixel/commands/p` ([`dynamixel_handler_msgs::msg::DxlCommandsP`](./dynamixel_handler_msgs#dynamixel_handler_msgsmsgdxlcommandsp-type))   
   Pシリーズのサーボを制御するための topic.
 - `/dynamixel/commands/pro` ([`dynamixel_handler_msgs::msg::DxlCommandsPro`](./dynamixel_handler_msgs#dynamixel_handler_msgsmsgdxlcommandspro-type))   
   Proシリーズのサーボを制御するための topic.
 - `/dynamixel/commands/all` ([`dynamixel_handler_msgs::msg::DxlCommandsAll`](./dynamixel_handler_msgs#dynamixel_handler_msgsmsgdxlcommandsall-type))   
   シリーズに関わらずすべてのサーボを制御するための topic.

PシリーズやProシリーズを利用するには，[Parameters](#parameters) の章の[初期化・終了時等の挙動設定](#初期化終了時等の挙動設定)における `init/used_servo_series`パラメータを`P: true`/`Pro: true`に設定する必要がある．

</details>

### Published Topics

`dynamixel_handler` ノードが publish する topic を以下に示す．

#### `/dynamixel/states` ([`DxlStates`型](./dynamixel_handler_msgs#dynamixel_handler_msgsmsgdxlstates-type))  
  全シリーズ共通のサーボ状態をまとめた topic．
  publish の周期と内容は [Parameters](#parameters) の章の[実行時の動作設定](#実行時の動作設定)の`pub_ratio`パラメータで設定される．
  すなわち，各情報は読み取り周期 `pub_ratio/{~}`に従って読み取られ，**読み取られた場合のみ** publish される．

| Field                  | Type                                                                 | Description                                                                                                | 
|------------------------|----------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------|
| `stamp`                | `builtin_interfaces/Time`                                            | メッセージのタイムスタンプ                                                                                         |
| `status`               | [`DynamixelStatus` ](./dynamixel_handler_msgs#dynamixelstatus-type ) | サーボの基本状態 (トルク, エラー有無, ping, 制御モード)．`pub_ratio/status` 周期でread．                             | `pub_ratio/status`に一回          | 
| `present`              | [`DynamixelPresent`](./dynamixel_handler_msgs#dynamixelpresent-type) | サーボの現在値 (PWM, 電流, 速度, 位置, etc.)．各要素のread周期は個別に設定可能．                                  | `pub_ratio/present.{~}`の最小値に一回 |
| `goal`                 | [`DynamixelGoal`   ](./dynamixel_handler_msgs#dynamixelgoal-type   ) | サーボの目標値 (PWM, 電流, 速度, 位置, etc.)．`pub_ratio/goal` 周期でread．                                     | `pub_ratio/goal`に一回          |
| `limit`                | [`DynamixelLimit`  ](./dynamixel_handler_msgs#dynamixelgain-type   ) | サーボの制限値 (温度, 電圧, PWM, 電流, 加速度, 速度, 位置)．`pub_ratio/limit` 周期でread．                        | `pub_ratio/limit`に一回          |
| `gain`                 | [`DynamixelGain`   ](./dynamixel_handler_msgs#dynamixellimit-type  ) | サーボの制御ゲイン (速度I/Pゲイン, 位置D/I/Pゲイン, FFゲイン)．`pub_ratio/gain` 周期でread．                            | `pub_ratio/gain`に一回          |
| `error`                | [`DynamixelError`  ](./dynamixel_handler_msgs#dynamixelerror-type  ) | サーボのハードウェアエラー詳細 (入力電圧, ホールセンサ, 過熱, エンコーダ, 電子回路, 過負荷)．`pub_ratio/error` 周期でread．   | `pub_ratio/error`に一回          |
| `extra`                | `DynamixelExtra`                                                     | その他の情報 (未実装)                                                                                           | 未実装                      |

<details>
<summary> `status` field の詳細 </summary>

##### `status` field ([DynamixelStatus型](./dynamixel_handler_msgs#dynamixelstatus-type)) 
`pub_ratio/status` 周期で read され, read されたタイミングのみ全ての情報が埋められる．  
 `id_list`フィールドの長さとそれ以外の field の長さは必ず一致する．すなわち，`id_list`　が空配列なら他の field も空配列であり，`id_list`が長さNの配列なら他の field も長さNの配列になる．

| Field     | Type      | Description                                  |
|-----------|-----------|----------------------------------------------|
| `id_list` | `uint16[]` | サーボIDのリスト                                |
| `torque`  | `bool[]`  | トルクがONかOFFか                               | 
| `error`   | `bool[]`  | ハードウェアエラーが発生しているか                |
| `ping`    | `bool[]`  | pingが通っているか                              | 
| `mode`    | `string[]`| 制御モードの文字列 (以下の Operating Mode list を参照) | 

Operating mode list
- `"pwm"`: PWM制御モード, Proシリーズは無し
- `"current"`: 電流制御モード
- `"velocity"`: 速度制御モード
- `"position"`: 位置制御モード
- `"ex_position"` = extended position: 拡張位置制御モード
- `"cur_position"`= current base position: 電流ベース位置制御モード, Xシリーズのみ

</details>

<details>
<summary> `present` field の詳細 </summary>

##### `present` field ([DynamixelPresent型](./dynamixel_handler_msgs#dynamixelpresent-type))

高速化のため，位置，速度などの要素個別で読み取り周期`pub_ratio/present.{~}`を設定できる．   
そのため最新のデータと非最新のデータが混在することになるが，非最新のデータを publish するかどうかは `pub_outdated_present_value` パラメータで設定可能．
- `pub_outdated_present_value: true` の場合: 非最新のデータも含めて publish されるため，`id_list`フィールドの長さとそれ以外の field の長さは必ず一致する．  
- `pub_outdated_present_value: false` の場合: 非最新のデータは publish されないため，"サーボの数と同じ長さの配列" の field と "空配列" の field が混在することになる．   

| Field                    | Type        | Description                                         | Note                                              |
|--------------------------|-------------|-----------------------------------------------------|---------------------------------------------------|
| `id_list`                | `uint16[]`   | サーボIDのリスト                                        |
| `pwm_percent`            | `float64[]` | 現在のPWM値 (%)                         | Proシリーズは非対応であり，".nan"で埋められる |
| `current_ma`             | `float64[]` | 現在の電流値 (mA)                                     |
| `velocity_deg_s`         | `float64[]` | 現在の速度 (deg/s)                                  |
| `position_deg`           | `float64[]` | 現在の位置 (deg)                                    |
| `vel_trajectory_deg_s`   | `float64[]` | profileによって生成された理想の速度 (deg/s) | Proシリーズは非対応であり，".nan"で埋められる |
| `pos_trajectory_deg`     | `float64[]` | profileによって生成された理想の位置 (deg)   | Proシリーズは非対応であり，".nan"で埋められる |
| `input_voltage_v`        | `float64[]` | 現在の入力電圧 (V)                                    |
| `temperature_degc`       | `float64[]` | 現在の温度 (°C)                                     |

  `pub_ratio/present` に `{current_ma: 1, velocity_deg_s: 1, position_deg: 1}` が設定されており，その他の `pub_ratio/present.{~}` が `0` に設定されている場合の出力例を示す．
  `pub_outdated_present_value` はデフォルトで `true` なので読み取られなかったデータも含め全ての field が埋まる．   
  ```yaml
  $ ros2 topic echo --flow-style /dynamixel/state/present 
  present: # DynamixelPresent型, pub_ratio/present.~ に一回 read され，1要素でも読み取ったら埋める
    id_list: [2, 3]
    pwm_percent: [0.0,  0.0]      # 現在のPWM値, Proシリーズは無し．
    current_ma: [100.9, 200.6]           # 現在の電流値
    velocity_deg_s: [10.0, 15.1]       # 現在の速度
    position_deg: [40.2, 3.1]         # 現在の位置
    vel_trajectory_deg_s: [0.0, 0.0] # profileによって生成された理想の速度，Proシリーズは無し．
    pos_trajectory_deg: [0.0, 0.0]   # profileによって生成された理想の位置, Proシリーズは無し．
    input_voltage_v: [0.0, 0.0]      # 現在の入力電圧
    temperature_degc: [0.0, 0.0]     # 現在の温度
  ```

  `pub_outdated_present_value` パラメータを `false` にすると以下のように読み取られなかった field は空配列となる．この場合は `pwm_percent`, `vel_trajectory_deg_s`, `pos_trajectory_deg`, `input_voltage_v`, `temperature_degc` は `pub_ratio/present.{~}` が `0` なので読み取られない．
  ```yaml
  $ ros2 topic echo --flow-style /dynamixel/state/present 
  present: # DynamixelPresent型, pub_ratio/present.~ に一回 read され，1要素でも読み取ったら埋める
    id_list: [2, 3]
    pwm_percent: []      # 現在のPWM値, Proシリーズは無し．
    current_ma: [100.9, 200.6]           # 現在の電流値
    velocity_deg_s: [10.0, 15.1]       # 現在の速度
    position_deg: [40.2, 3.1]         # 現在の位置
    vel_trajectory_deg_s: [] # profileによって生成された理想の速度，Proシリーズは無し．
    pos_trajectory_deg: []   # profileによって生成された理想の位置, Proシリーズは無し．
    input_voltage_v: []      # 現在の入力電圧
    temperature_degc: []     # 現在の温度
  ```

</details>

<details>
<summary> `goal` field の詳細 </summary>

##### `goal` field :
`pub_ratio/goal` 周期で read され, read されたタイミングのみ全ての情報が埋められる．
 `id_list`フィールドの長さとそれ以外の field の長さは必ず一致．すなわち，`id_list`　が空配列なら他の field も空配列であり，`id_list`が長さNの配列なら他の field も長さNの配列になる．
 
| Field                  | Type        | Description                                                     | Note                                              |
|------------------------|-------------|-----------------------------------------------------------------|----------------------------------------------------|
| `id_list`              | `uint16[]`   | サーボIDのリスト                                                    |
| `pwm_percent`          | `float64[]` | 目標PWM値 (%) | Proシリーズは非対応であり，".nan"で埋められる |
| `current_ma`           | `float64[]` | 目標電流値 (mA)                                                 |
| `velocity_deg_s`       | `float64[]` | 目標速度 (deg/s)                                              |
| `profile_acc_deg_ss`   | `float64[]` | profileの加速度 (deg/s^2)      | Proシリーズのgoal_accelerationに対応
| `profile_vel_deg_s`    | `float64[]` | profileの速度 (deg/s)          | Proシリーズは非対応であり，".nan"で埋められる |
| `position_deg`         | `float64[]` | 目標位置 (deg)                                                ||

</details>

<details>
<summary> `limit` field の詳細 </summary>

##### `limit` field :
`pub_ratio/limit` 周期で read され, read されたタイミングのみ全ての情報が埋められる．
`id_list`フィールドの長さとそれ以外の field の長さは必ず一致．すなわち，`id_list`　が空配列なら他の field も空配であり，`id_list`が長さNの配列なら他の field も長さNの配列になる．

| Field                       | Type        | Description                                                                 | Note                                              |
|-----------------------------|-------------|-----------------------------------------------------------------------------|----------------------------------------------------|
| `id_list`                   | `uint16[]`   | サーボIDのリスト                                                                | |
| `temperature_limit_degc`    | `float64[]`   | 温度上限 (°C)                                                                 | |
| `max_voltage_limit_v`       | `float64[]` | 入力電圧上限 (V)                                                                | |
| `min_voltage_limit_v`       | `float64[]` | 入力電圧下限 (V)                                                                | |
| `pwm_limit_percent`         | `float64[]` | PWM上限 (%) , goal_pwmはこの値に抑えられる． | Proシリーズは非対応であり，".nan"で埋められる |
| `current_limit_ma`          | `float64[]` | 電流値上限 (mA), goal_currentはこの値に抑えられる．                      | |
| `acceleration_limit_deg_ss` | `float64[]` | 加速度上限 (deg/s^2), profile_accelerationはこの値に抑えられる． | Xシリーズは非対応であり，".nan"で埋められる |
| `velocity_limit_deg_s`      | `float64[]` | 速度上限 (deg/s), goal_velocity  と profile_velocityはこの値に抑えられる ||
| `max_position_limit_deg`    | `float64[]` | 位置上限 (deg), 位置制御モードの場合，goal_positionはこの値に抑えられる． | |
| `min_position_limit_deg`    | `float64[]` | 位置下限 (deg), 位置制御モードの場合，goal_positionはこの値に抑えられる． | |
</details>

<details>
<summary> `gain` field の詳細 </summary>

##### `gain` field :
`pub_ratio/gain` 周期で read され, read されたタイミングのみ全ての情報が埋められる．
`id_list`フィールドの長さとそれ以外の field の長さは必ず一致．すなわち，`id_list`　が空配列なら他の field も空配であり，`id_list`が長さNの配列なら他の field も長さNの配列になる．

| Field                        | Type      | Description                          | Note                                              |
|------------------------------|-----------|--------------------------------------|----------------------------------------------------|
| `id_list`                    | `uint16[]` | サーボIDのリスト                           | |
| `velocity_i_gain_pulse`      | `uint16[]` | 速度制御Iゲイン         ||
| `velocity_p_gain_pulse`      | `uint16[]` | 速度制御Pゲイン         ||
| `position_d_gain_pulse`      | `uint16[]` | 位置制御Dゲイン | Proシリーズは非対応であり, "0"で埋められる|
| `position_i_gain_pulse`      | `uint16[]` | 位置制御Iゲイン | Proシリーズは非対応であり, "0"で埋められる|
| `position_p_gain_pulse`      | `uint16[]` | 位置制御Pゲイン  ||
| `feedforward_2nd_gain_pulse` | `uint16[]` | FF第2ゲイン | Proシリーズは非対応であり, "0"で埋められる|
| `feedforward_1st_gain_pulse` | `uint16[]` | FF第1ゲイン | Proシリーズは非対応であり, "0"で埋められる|
</details>

<details>
<summary> `error` field の詳細 </summary>

##### `error` field :
`pub_ratio/error` 周期で read され, read されたタイミングのみ全ての情報が埋められる．
`id_list`フィールドの長さとそれ以外の field の長さは必ず一致．すなわち，`id_list`　が空配列なら他の field も空配であり，`id_list`が長さNの配列なら他の field も長さNの配列になる．

| Field                | Type     | Description                               |
|----------------------|----------|-------------------------------------------|
| `id_list`            | `uint16[]`| サーボIDのリスト                              |
| `input_voltage`      | `bool[]` | 入力電圧が上限下限に引っかかっている                        |
| `motor_hall_sensor`  | `bool[]` | ホールセンサの異常                               |
| `overheating`        | `bool[]` | 現在温度が温度上限を超えている                          |
| `motor_encoder`      | `bool[]` | エンコーダの異常                               |
| `electronical_shock` | `bool[]` | 電子回路の異常                               |
| `overload`           | `bool[]` | 過負荷 (判定アルゴリズムは不明)                       |
</details>

<details>
<summary> `extra` field の詳細 </summary>

##### `extra` field :

| Field | Type | Description |
|-------|------|-------------|
| `# 未実装につき略` |      |             |

</details>

#### `/dynamixel/debug` ([`DynamixelDebug`型](./dynamixel_handler_msgs#dynamixeldebug-type))
  サーボが動作しないときにコマンドラインで状況を確認するためのデバッグ用の topic．  
  現在値と目標値の更新タイミングが異なるので注意．

| Field                  | Sub field   | Type | Description                                                                 |
|------------------------|-------------|------|-----------------------------------------------------------------------|
| `status`               |             | `DynamixelStatus` | サーボの基本状態 (トルク, エラー有無, ping, 制御モード)．`pub_ratio/status` 周期でread． |
| `current_ma`           | `present`   | `float64[]` | 現在の電流値 (mA)．`pub_ratio/present.current` 周期でread．                      |
|            | `goal`      | `float64[]` | 目標電流値 (mA)．`pub_ratio/goal` 周期でread．                          |
| `velocity_deg_s`       | `present`   | `float64[]` | 現在の速度 (deg/s)．`pub_ratio/present.velocity` 周期でread．                  |
|        | `goal`      | `float64[]` | 目標速度 (deg/s)．`pub_ratio/goal` 周期でread．                        |
| `position_deg`         | `present`   | `float64[]` | 現在の位置 (deg)．`pub_ratio/present.position` 周期でread．                        |
|          | `goal`      | `float64[]` | 目標位置 (deg)．`pub_ratio/goal` 周期でread．                            |


#### `/dynamixel/state/status` ([`DynamixelStatus`型](./dynamixel_handler_msgs#dynamixelstatus-type))  
`/dynamixel/states` の `status` フィールドを単独で publish する topic．  

#### `/dynamixel/state/present` ([`DynamixelPresent`型](./dynamixel_handler_msgs#dynamixelpresent-type))  
`/dynamixel/states` の `present` フィールドを単独で publish する topic．

#### `/dynamixel/state/goal` ([`DynamixelGoal`型](./dynamixel_handler_msgs#dynamixelgoal-type))  
`/dynamixel/states` の `goal` フィールドを単独で publish する topic．

#### `/dynamixel/state/gain` ([`DynamixelGain`型](./dynamixel_handler_msgs#dynamixellimit-type))  
`/dynamixel/states` の `gain` フィールドを単独で publish する topic．

#### `/dynamixel/state/limit` ([`DynamixelLimit`型](./dynamixel_handler_msgs#dynamixellimit-type))  
`/dynamixel/states` の `limit` フィールドを単独で publish する topic．

#### `/dynamixel/state/error` ([`DynamixelError`型](./dynamixel_handler_msgs#dynamixelerror-type))  
`/dynamixel/states` の `error` フィールドを単独で publish する topic．


### Subscribed Topics

`dynamixel_handler` ノードが subscribe する topic を以下に示す．    
Subscribe 時にデータが一時保存され，直後のメインループ内で書き込みが行われるため，書き込みの最大周期は`loop_rate`[Hz]となる．  

####  **`/dynamixel/commands/x`** ([`DxlCommandsX`型](./dynamixel_handler_msgs#dynamixel_handler_msgsmsgdxlcommandsx-type))    
  Xシリーズ用のコマンドを統合した topic．
 Subscribe したデータの各 field (`pwm_control`, `status`, ... など)の中で **`id_list`フィールドが埋まっているfieldのみ**処理される．   
  [Parameters](#parameters) の章の[初期化・終了時等の挙動設定](#初期化終了時等の挙動設定)における `init/used_servo_series` パラメータで `X: true` に設定されている場合に利用可能．

| Field                           | Type                                  | Description                                                                                                                               |
|---------------------------------|---------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------|
| `pwm_control`                   | [`DynamixelControlXPwm`](./dynamixel_handler_msgs#dynamixelcontrolxpwm-type)                | PWM制御モードへの変更と目標PWM値の設定.                                                                                                         |
| `current_control`               | [`DynamixelControlXCurrent`](./dynamixel_handler_msgs#dynamixelcontrolxcurrent-type)            | 電流制御モードへの変更と目標電流値の設定.                                                                                                         |
| `velocity_control`              | [`DynamixelControlXVelocity`](./dynamixel_handler_msgs#dynamixelcontrolxvelocity-type)           | 速度制御モードへの変更と目標速度などの設定.                                                                                               |
| `position_control`              | [`DynamixelControlXPosition`](./dynamixel_handler_msgs#dynamixelcontrolxposition-type)           | 位置制御モードへの変更と目標位置などの設定.                                                                                             |
| `extended_position_control`     | [`DynamixelControlXExtendedPosition`](./dynamixel_handler_msgs#dynamixelcontrolxextendedposition-type)   | 拡張位置制御モードへの変更と目標位置などの設定.                                                                                   |
| `current_base_position_control` | [`DynamixelControlXCurrentPosition`](./dynamixel_handler_msgs#dynamixelcontrolxcurrentposition-type)  | 電流制限付き位置制御モードへの変更と目標電流・目標位置などの設定.                                                                         |
| `status`                        | [`DynamixelStatus`](./dynamixel_handler_msgs#dynamixelstatus-type)                     | サーボの基本状態(トルクON/OFF, エラークリア, ID追加/削除, 制御モード変更)の設定.                                                                            |
| `gain`                          | [`DynamixelGain`](./dynamixel_handler_msgs#dynamixelgain-type)                       | 各種制御ゲインの設定.                                                                                             |
| `limit`                         | [`DynamixelLimit`](./dynamixel_handler_msgs#dynamixellimit-type)                      | 各種制限値の設定.                                                                         |
| `extra`                         | `DynamixelExtra`                      | その他の情報 (未実装).                                                                                                                      |


<details>
<summary> `pwm_control` field の詳細 </summary>

##### `pwm_control` field ([DynamixelControlXPwm型](./dynamixel_handler_msgs#dynamixelcontrolxpwm-type))
`id_list`と`pwm_percent`の長さは一致する必要がある．
| Field         | Type        | Description                                          |
|---------------|-------------|------------------------------------------------------|
| `id_list`     | `uint16[]`   | 適用するサーボIDのリスト.                                  |
| `pwm_percent` | `float64[]` | 目標PWM値 (%). `goal_pwm`アドレスに書き込まれる.             |

</details>

<details>
<summary> `current_control` field の詳細 </summary>

##### `current_control` field ([DynamixelControlXCurrent型](./dynamixel_handler_msgs#dynamixelcontrolxcurrent-type))
`id_list`と`current_ma`の長さは一致する必要がある．
| Field        | Type        | Description                                          |
|--------------|-------------|------------------------------------------------------|
| `id_list`    | `uint16[]`   | 適用するサーボIDのリスト.                                  |
| `current_ma` | `float64[]` | 目標電流値 (mA). `goal_current`アドレスに書き込まれる.       |

</details>

<details>
<summary> `velocity_control` field の詳細 </summary>

##### `velocity_control` field ([DynamixelControlXVelocity型](./dynamixel_handler_msgs#dynamixelcontrolxvelocity-type))
`velocity_deg_s`と`profile_acc_deg_ss`のどちらかのみが空配列であってもよい．
`id_list`と`velocity_deg_s` または `profile_acc_deg_ss`の長さは一致する必要がある．
| Field                | Type        | Description                                          |
|----------------------|-------------|------------------------------------------------------|
| `id_list`            | `uint16[]`   | 適用するサーボIDのリスト.                                  |
| `velocity_deg_s`     | `float64[]` | 目標速度 (deg/s). `goal_velocity`アドレスに書き込まれる.    |
| `profile_acc_deg_ss` | `float64[]` | プロファイル加速度 (deg/s^2). `profile_acceleration`アドレスに書き込まれる. |

</details>

<details>
<summary> `position_control` field の詳細 </summary>

##### `position_control` field ([DynamixelControlXPosition型](./dynamixel_handler_msgs#dynamixelcontrolxposition-type))
全ての要素が指定されている必要はない．
指定された要素と`id_list`の長さは一致する必要がある．
| Field                | Type        | Description                                          |
|----------------------|-------------|------------------------------------------------------|
| `id_list`            | `uint16[]`   | 適用するサーボIDのリスト.                                  |
| `position_deg`       | `float64[]` | 目標位置 (deg). `goal_position`アドレスに書き込まれる.       |
| `profile_vel_deg_s`  | `float64[]` | プロファイル速度 (deg/s). `profile_velocity`アドレスに書き込まれる. |
| `profile_acc_deg_ss` | `float64[]` | プロファイル加速度 (deg/s^2). `profile_acceleration`アドレスに書き込まれる. |

</details>

<details>
<summary> `extended_position_control` field の詳細 </summary>

##### `extended_position_control` field ([DynamixelControlXExtendedPosition型](./dynamixel_handler_msgs#dynamixelcontrolxextendedposition-type))
全ての要素が指定されている必要はない．
指定された要素と`id_list`の長さは一致する必要がある．
| Field                | Type        | Description                                          |
|----------------------|-------------|------------------------------------------------------|
| `id_list`            | `uint16[]`   | 適用するサーボIDのリスト.                                  |
| `position_deg`       | `float64[]` | 目標位置 (mA)．`goal_position`アドレスに書き込まれる．|
| `rotation`           | `float64[]` | 回転数. `goal_position`アドレスに `position_deg + rotation * 360` degに相当する値が書き込まれる. |
| `profile_vel_deg_s`  | `float64[]` | プロファイル速度 (deg/s). `profile_velocity`アドレスに書き込まれる. |
| `profile_acc_deg_ss` | `float64[]` | プロファイル加速度 (deg/s^2). `profile_acceleration`アドレスに書き込まれる. |

</details>

<details>
<summary> `current_base_position_control` field の詳細 </summary>

##### `current_base_position_control` field ([DynamixelControlXCurrentPosition型](./dynamixel_handler_msgs#dynamixelcontrolxcurrentposition-type))
全ての要素が指定されている必要はない．
指定された要素と`id_list`の長さは一致する必要がある．
| Field                | Type        | Description                                          |
|----------------------|-------------|------------------------------------------------------|
| `id_list`            | `uint16[]`   | 適用するサーボIDのリスト.                                  |
| `current_ma`         | `float64[]` | 目標電流値 (mA). `goal_current`アドレスに書き込まれる.       |
| `position_deg`       | `float64[]` | 目標位置 (deg). `goal_position`アドレスに書き込まれる.       |
| `rotation`           | `float64[]` | 回転数. `goal_position`アドレスに `position_deg + rotation * 360` deg (例: 1.2*360 deg)に相当する値が書き込まれる. |
| `profile_vel_deg_s`  | `float64[]` | プロファイル速度 (deg/s). `profile_velocity`アドレスに書き込まれる. |
| `profile_acc_deg_ss` | `float64[]` | プロファイル加速度 (deg/s^2). `profile_acceleration`アドレスに書き込まれる. |

</details>

 `status`, `gain`, `limit`, `extra` field は [`/dynamixel/commands/all` topic の説明](#dynamixelcommandlimit-dynamixellimit型)を参照

#### `/dynamixel/commands/p` ([`DxlCommandsP`型](./dynamixel_handler_msgs#dynamixel_handler_msgsmsgdxlcommandsp-type))    
  Pシリーズ用のコマンドを統合した topic．
 Subscribe したデータの各 field (`pwm_control`, `status`, ... など)の中で **`id_list`フィールドが埋まっているfieldのみ**処理される．    
  [Parameters](#parameters) の章の[初期化・終了時等の挙動設定](#初期化終了時等の挙動設定)における `init/used_servo_series` パラメータで `P: true` に設定されている場合に利用可能．

  
| Field                           | Type                                  | Description                                                                                                                               |
|---------------------------------|---------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------|
| `pwm_control`                   | [`DynamixelControlPPwm`](./dynamixel_handler_msgs#dynamixelcontrolppwm-type)                | PWM制御モードへの変更と目標PWM値の設定.                                                                                                         |
| `current_control`               | [`DynamixelControlPCurrent`](./dynamixel_handler_msgs#dynamixelcontrolpcurrent-type)            | 電流制御モードへの変更と目標電流値の設定.                                                                                                         |
| `velocity_control`              | [`DynamixelControlPVelocity`](./dynamixel_handler_msgs#dynamixelcontrolpvelocity-type)           | 速度制御モードへの変更と目標速度などの設定.                                                                                               |
| `position_control`              | [`DynamixelControlPPosition`](./dynamixel_handler_msgs#dynamixelcontrolpposition-type)           | 位置制御モードへの変更と目標位置などの設定.                                                                                             |
| `extended_position_control`     | [`DynamixelControlPExtendedPosition`](./dynamixel_handler_msgs#dynamixelcontrolpextendedposition-type)   | 拡張位置制御モードへの変更と目標位置などの設定.                                                                                   |
| `status`                        | [`DynamixelStatus`](./dynamixel_handler_msgs#dynamixelstatus-type)                     | サーボの基本状態(トルクON/OFF, エラークリア, ID追加/削除, 制御モード変更)の設定.                                                                            |
| `gain`                          | [`DynamixelGain`](./dynamixel_handler_msgs#dynamixelgain-type)                       | 各種制御ゲインの設定.                                                                                             |
| `limit`                         | [`DynamixelLimit`](./dynamixel_handler_msgs#dynamixellimit-type)                      | 各種制限値(温度, 電圧, PWM, 電流, 加速度, 速度, 位置上限/下限)の設定.                                                                         |
| `extra`                         | `DynamixelExtra`                      | その他の情報 (未実装).                                                                                                                      |

<details>
<summary> `pwm_control` field の詳細 </summary>

##### `pwm_control` field ([DynamixelControlPPwm型](./dynamixel_handler_msgs#dynamixelcontrolppwm-type))
`id_list`と`pwm_percent`の長さは一致する必要がある．
| Field         | Type        | Description                                          |
|---------------|-------------|------------------------------------------------------|
| `id_list`     | `uint16[]`   | 適用するサーボIDのリスト.                                  |
| `pwm_percent` | `float64[]` | 目標PWM値 (%). `goal_pwm`アドレスに書き込まれる.             |

</details>

<details>
<summary> `current_control` field の詳細 </summary>

##### `current_control` field ([DynamixelControlPCurrent型](./dynamixel_handler_msgs#dynamixelcontrolpcurrent-type))
`id_list`と`current_ma`の長さは一致する必要がある．
| Field        | Type        | Description                                          |
|--------------|-------------|------------------------------------------------------|
| `id_list`    | `uint16[]`   | 適用するサーボIDのリスト.                                  |
| `current_ma` | `float64[]` | 目標電流値 (mA). `goal_current`アドレスに書き込まれる.       |

</details>

<details>
<summary> `velocity_control` field の詳細 </summary>

##### `velocity_control` field ([DynamixelControlPVelocity型](./dynamixel_handler_msgs#dynamixelcontrolpvelocity-type))
全ての要素が指定されている必要はない．
指定された要素と`id_list`の長さは一致する必要がある．
| Field                | Type        | Description                                          |
|----------------------|-------------|------------------------------------------------------|
| `id_list`            | `uint16[]`   | 適用するサーボIDのリスト.                                  |
| `current_ma`     | `float64[]` | 目標電流値 (mA). `goal_current`アドレスに書き込まれる.       |
| `velocity_deg_s`     | `float64[]` | 目標速度 (deg/s). `goal_velocity`アドレスに書き込まれる.    |
| `profile_acc_deg_ss` | `float64[]` | プロファイル加速度 (deg/s^2). `profile_acceleration`アドレスに書き込まれる. |

</details>

<details>
<summary> `position_control` field の詳細 </summary>

##### `position_control` field ([DynamixelControlPPosition型](./dynamixel_handler_msgs#dynamixelcontrolpposition-type))
全ての要素が指定されている必要はない．
指定された要素と`id_list`の長さは一致する必要がある．
| Field                | Type        | Description                                          |
|----------------------|-------------|------------------------------------------------------|
| `id_list`            | `uint16[]`   | 適用するサーボIDのリスト.                                  |
| `current_ma`         | `float64[]` | 目標電流値 (mA). `goal_current`アドレスに書き込まれる.       |
| `velocity_deg_s`     | `float64[]` | 目標速度 (deg/s). `goal_velocity`アドレスに書き込まれる.    |
| `position_deg`       | `float64[]` | 目標位置 (deg). `goal_position`アドレスに書き込まれる.       |
| `profile_vel_deg_s`  | `float64[]` | プロファイル速度 (deg/s). `profile_velocity`アドレスに書き込まれる. |
| `profile_acc_deg_ss` | `float64[]` | プロファイル加速度 (deg/s^2). `profile_acceleration`アドレスに書き込まれる. |

</details>

<details>
<summary> `extended_position_control` field の詳細 </summary>

##### `extended_position_control` field ([DynamixelControlPExtendedPosition型](./dynamixel_handler_msgs#dynamixelcontrolpextendedposition-type))
全ての要素が指定されている必要はない．
指定された要素と`id_list`の長さは一致する必要がある．

| Field                | Type        | Description                                          |
|----------------------|-------------|------------------------------------------------------|
| `id_list`            | `uint16[]`   | 適用するサーボIDのリスト.                                  |
| `current_ma`         | `float64[]` | 目標電流値 (mA). `goal_current`アドレスに書き込まれる.       |
| `velocity_deg_s`     | `float64[]` | 目標速度 (deg/s). `goal_velocity`アドレスに書き込まれる.    |
| `position_deg`       | `float64[]` | 目標位置 (mA)．`goal_position`アドレスに書き込まれる．|
| `rotation`           | `float64[]` | 回転数. `goal_position`アドレスに `position_deg + rotation * 360` degに相当する値が書き込まれる. |
| `profile_vel_deg_s`  | `float64[]` | プロファイル速度 (deg/s). `profile_velocity`アドレスに書き込まれる. |
| `profile_acc_deg_ss` | `float64[]` | プロファイル加速度 (deg/s^2). `profile_acceleration`アドレスに書き込まれる. |

</details>

 `status`, `gain`, `limit`, `extra` field は [`/dynamixel/commands/all` topic の説明](#dynamixelcommandlimit-dynamixellimit型)を参照

#### `/dynamixel/commands/pro` ([`DxlCommandsPro`型](./dynamixel_handler_msgs#dynamixel_handler_msgsmsgdxlcommandspro-type))    
  Proシリーズ用のコマンドを統合した topic．
 Subscribe したデータの各 field (`current_control`, `status`, ... など)の中で **`id_list`フィールドが埋まっているfieldのみ**処理される．   
  [Parameters](#parameters) の章の[初期化・終了時等の挙動設定](#初期化終了時等の挙動設定)における `init/used_servo_series` パラメータで `Pro: true` に設定されている場合に利用可能．
  
| Field                           | Type                                  | Description                                                                                                                               |
|---------------------------------|---------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------|
| `current_control`               | [`DynamixelControlProCurrent`         ](./dynamixel_handler_msgs#dynamixelcontrolprocurrent-type) | 電流制御モードへの変更と目標電流値の設定.                                                                                                         |
| `velocity_control`              | [`DynamixelControlProVelocity`        ](./dynamixel_handler_msgs#dynamixelcontrolprovelocity-type) | 速度制御モードへの変更と目標速度などの設定.                                                                                               |
| `position_control`              | [`DynamixelControlProPosition`        ](./dynamixel_handler_msgs#dynamixelcontrolproposition-type) | 位置制御モードへの変更と目標位置などの設定.                                                                                             |
| `extended_position_control`     | [`DynamixelControlProExtendedPosition`](./dynamixel_handler_msgs#dynamixelcontrolproextendedposition-type) | 拡張位置制御モードへの変更と目標位置などの設定.                                                                                   |
| `status`                        | [`DynamixelStatus`                    ](./dynamixel_handler_msgs#dynamixelstatus-type) | サーボの基本状態(トルクON/OFF, エラークリア, ID追加/削除, 制御モード変更)の設定.                                                                            |
| `gain`                          | [`DynamixelGain`                      ](./dynamixel_handler_msgs#dynamixelgain-type) | 各種制御ゲインの設定.                                                                                             |
| `limit`                         | [`DynamixelLimit`                     ](./dynamixel_handler_msgs#dynamixellimit-type) | 各種制限値の設定.                                                                         |
| `extra`                         | `DynamixelExtra`                      | その他の情報 (未実装).                                                                                                                      |

<details>
<summary> `current_control` field の詳細 </summary>

##### `current_control` field ([DynamixelControlProCurrent型](./dynamixel_handler_msgs#dynamixelcontrolprocurrent-type))
`id_list`と`current_ma`の長さは一致する必要がある．
| Field        | Type        | Description                                          |
|--------------|-------------|------------------------------------------------------|
| `id_list`    | `uint16[]`   | 適用するサーボIDのリスト.                                  |
| `current_ma` | `float64[]` | 目標電流値 (mA). `goal_current`アドレスに書き込まれる.       |

</details>

<details>
<summary> `velocity_control` field の詳細 </summary>

##### `velocity_control` field ([DynamixelControlProVelocity型](./dynamixel_handler_msgs#dynamixelcontrolprovelocity-type))
全ての要素が指定されている必要はない．
指定された要素と`id_list`の長さは一致する必要がある．
| Field                | Type        | Description                                          |
|----------------------|-------------|------------------------------------------------------|
| `id_list`            | `uint16[]`   | 適用するサーボIDのリスト.                                  |
| `current_ma`     | `float64[]` | 目標電流値 (mA). `goal_current`アドレスに書き込まれる.       |
| `velocity_deg_s`     | `float64[]` | 目標速度 (deg/s). `goal_velocity`アドレスに書き込まれる.    |
| `acceleration_deg_ss` | `float64[]` | 目標加速度 (deg/s^2). `goal_acceleration`アドレスに書き込まれる. |

</details>

<details>
<summary> `position_control` field の詳細 </summary>

##### `position_control` field ([DynamixelControlProPosition型](./dynamixel_handler_msgs#dynamixelcontrolproposition-type))
全ての要素が指定されている必要はない．
指定された要素と`id_list`の長さは一致する必要がある．
| Field                | Type        | Description                                          |
|----------------------|-------------|------------------------------------------------------|
| `id_list`            | `uint16[]`   | 適用するサーボIDのリスト.                                  |
| `current_ma`         | `float64[]` | 目標電流値 (mA). `goal_current`アドレスに書き込まれる.       |
| `acceleration_deg_ss` | `float64[]` | 目標加速度 (deg/s^2). `goal_acceleration`アドレスに書き込まれる. |
| `velocity_deg_s`     | `float64[]` | 目標速度 (deg/s). `goal_velocity`アドレスに書き込まれる.    |
| `position_deg`       | `float64[]` | 目標位置 (deg). `goal_position`アドレスに書き込まれる.       |

</details>

<details>
<summary> `extended_position_control` field の詳細 </summary>

##### `extended_position_control` field ([DynamixelControlProExtendedPosition型](./dynamixel_handler_msgs#dynamixelcontrolproextendedposition-type))
全ての要素が指定されている必要はない．
指定された要素と`id_list`の長さは一致する必要がある．
| Field                | Type        | Description                                          |
|----------------------|-------------|------------------------------------------------------|
| `id_list`            | `uint16[]`   | 適用するサーボIDのリスト.                                  |
| `current_ma`         | `float64[]` | 目標電流値 (mA). `goal_current`アドレスに書き込まれる.       |
| `acceleration_deg_ss` | `float64[]` | 目標加速度 (deg/s^2). `goal_acceleration`アドレスに書き込まれる. |
| `velocity_deg_s`     | `float64[]` | 目標速度 (deg/s). `goal_velocity`アドレスに書き込まれる.    |
| `position_deg`       | `float64[]` | 目標位置 (mA)．`goal_position`アドレスに書き込まれる．|
| `rotation`           | `float64[]` | 回転数. `goal_position`アドレスに `position_deg + rotation * 360` degに相当する値が書き込まれる. |

</details>

 `status`, `gain`, `limit`, `extra` field は [`/dynamixel/commands/all` topic の説明](#dynamixelcommandlimit-dynamixellimit型)を参照

#### `/dynamixel/commands/all` ([`DxlCommandsAll`型](./dynamixel_handler_msgs#dynamixel_handler_msgsmsgdxlcommandsall-type))  
  全シリーズを共通で扱うための topic. シリーズ共通で扱うため，`status.mode` で制御モードを指定し`goal.~`で各種目標値を与える．
  
| Field                           | Type                                  | Description                                                                                                                               |
|---------------------------------|---------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------|
| `status`                        | `DynamixelStatus`                     | サーボの基本状態(トルクON/OFF, エラークリア, ID追加/削除, 制御モード変更)の設定.                                                                            |
| `goal`                          | `DynamixelGoal`                       | 各種目標値の設定.                                                                                                                         |
| `gain`                          | `DynamixelGain`                       | 各種制御ゲイン(速度I/Pゲイン, 位置D/I/Pゲイン, FFゲイン)の設定.                                                                                             |
| `limit`                         | `DynamixelLimit`                      | 各種制限値(温度, 電圧, PWM, 電流, 加速度, 速度, 位置上限/下限)の設定.                                                                         |
| `extra`                         | `DynamixelExtra`                      | その他の情報 (未実装).                                                                                                                      |

※ `{~}_control`系の field がないため制御モードの自動変更機能は無し.    


<details>
<summary> `status` field の詳細 </summary>

##### `status` field ([DynamixelStatus型](./dynamixel_handler_msgs#dynamixelstatus-type))
全ての要素が指定されている必要はない．
指定された要素と`id_list`の長さは一致する必要がある．
| Field     | Type      | Description                                                     |
|-----------|-----------|-----------------------------------------------------------------|
| `id_list` | `uint16[]` | 適用するサーボIDのリスト.                                             |
| `torque`  | `bool[]`  | トルクON/OFF. 例: ID 1はON, ID 2はOFF.                               |
| `error`   | `bool[]`  | エラークリア試行. `true`でも`false`でもエラー解除を試みる.                       |
| `ping`    | `bool[]`  | `true`なら対応IDを認識リストに追加, `false`なら削除.                 |
| `mode`    | `string[]`| 制御モード変更. 文字列で指定. (詳細は[DynamixelStatus型](./dynamixel_handler_msgs#dynamixelstatus-type)の説明を参照) |

※ `mode` については，各モードの`{~}_control`系の topic を送ることでも自動設定されるので，基本的には使わなくもてOK．   

</details>

<details>
<summary> `goal` field の詳細 </summary>

##### `goal` field の詳細 ([DynamixelGoal型](./dynamixel_handler_msgs#dynamixelgoal-type))
全ての要素が指定されている必要はない．
指定された要素と`id_list`の長さは一致する必要がある．
| Field                | Type        | Description                                          | Note |
|----------------------|-------------|------------------------------------------------------|------|
| `id_list`            | `uint16[]`   | 適用するサーボIDのリスト.                                  |      |
| `pwm_percent`     | `float64[]` | 目標PWM値 (%). `goal_pwm`アドレスに書き込まれる.             | Proシリーズは非対応 |
| `current_ma`         | `float64[]` | 目標電流値 (mA). `goal_current`アドレスに書き込まれる.       |      |
| `velocity_deg_s`     | `float64[]` | 目標速度 (deg/s). `goal_velocity`アドレスに書き込まれる.    |      |
| `profile_vel_deg_s`  | `float64[]` | プロファイル速度 (deg/s). `profile_velocity`アドレスに書き込まれる. |Proシリーズは非対応 |
| `profile_acc_deg_ss` | `float64[]` | プロファイル加速度 (deg/s^2). `profile_acceleration`アドレスに書き込まれる. | Proシリーズでは`goal_acceleration`に対応する． |
| `position_deg`       | `float64[]` | 目標位置 (deg). `goal_position`アドレスに書き込まれる.       |      |

</details>

<details>
<summary> `gain` field の詳細 </summary>

##### `gain` field の詳細 ([DynamixelGain型](./dynamixel_handler_msgs#dynamixelgaintype))
全ての要素が指定されている必要はない．
指定された要素と`id_list`の長さは一致する必要がある．
| Field                        | Type      | Description                                  | Note |
|------------------------------|-----------|----------------------------------------------| -----|
| `id_list`                    | `uint16[]` | 適用するサーボIDのリスト.            |                |
| `velocity_i_gain_pulse`      | `uint16[]` | 速度制御Iゲイン (パルス値).        | |
| `velocity_p_gain_pulse`      | `uint16[]` | 速度制御Pゲイン (パルス値).        | |
| `position_d_gain_pulse`      | `uint16[]` | 位置制御Dゲイン (パルス値).        | Proシリーズは非対応 |
| `position_i_gain_pulse`      | `uint16[]` | 位置制御Iゲイン (パルス値).        | Proシリーズは非対応 | 
| `position_p_gain_pulse`      | `uint16[]` | 位置制御Pゲイン (パルス値).        | 
| `feedforward_2nd_gain_pulse` | `uint16[]` | FF第2ゲイン (パルス値).         | Pシリーズは非対応 |
| `feedforward_1st_gain_pulse` | `uint16[]` | FF第1ゲイン (パルス値).         | Pシリーズは非対応 |

</details>

<details>
<summary> `limit` field の詳細 </summary>

##### `limit` field の詳細 ([DynamixelLimit型](./dynamixel_handler_msgs#dynamixellimit-type))
全ての要素が指定されている必要はない．
指定された要素と`id_list`の長さは一致する必要がある．

  ※ limit はROM領域の値なので，書き込む場合torqueが強制的にOFFになることに注意．

| Field                       | Type        | Description                                  | Note |
|-----------------------------|-------------|----------------------------------------------|------|
| `id_list`                   | `uint16[]`  | 適用するサーボIDのリスト.         | |
| `temperature_limit_degc`    | `float64[]` | 温度上限 (°C).               | |
| `max_voltage_limit_v`       | `float64[]` | 入力電圧上限 (V).            | |
| `min_voltage_limit_v`       | `float64[]` | 入力電圧下限 (V).            | | 
| `pwm_limit_percent`         | `float64[]` | PWM上限 (%)                 | Proシリーズは非対応 |
| `current_limit_ma`          | `float64[]` | 電流値上限 (mA).             | |
| `acceleration_limit_deg_ss` | `float64[]` | 加速度上限 (deg/s^2).        | Xシリーズは非対応 |
| `velocity_limit_deg_s`      | `float64[]` | 速度上限 (deg/s).            | |
| `max_position_limit_deg`    | `float64[]` | 位置上限 (deg).              | | 
| `min_position_limit_deg`    | `float64[]` | 位置下限 (deg).              | | 

</details>


<details>
<summary> `extra` field の詳細 </summary>

##### `extra` field の詳細 ([DynamixelExtra型](./dynamixel_handler_msgs#dynamixelextra-type))
| Field | Type | Description |
|-------|------|-------------|
| `# 未実装につき略` |      |             |

</details>

#### `/dynamixel/shortcut` ([`DynamixelShortcut`型](./dynamixel_handler_msgs#dynamixelshortcut-type))    
 Dynamixelの起動、停止、エラー解除などのショートカットコマンド  

|Field     | Type      | Description                                                                 |
|-----------|-----------|-----------------------------------------------------------------------------|
| `command` | `string`  | コマンドの文字列 (下記の Shortcut Command list 参照) |
| `id_list` | `uint16[]` | 適用するサーボのIDリスト． [] or [254] とすると認識されているすべてのIDを選択したのと同等となる． |

<details>
<summary> Shortcut Command list </summary>

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

</details>

#### `/dynamixel/command/status` ([`DynamixelStatus`型](./dynamixel_handler_msgs#dynamixelstatus-type))       
/dynamixel/commands/{~}` の `status` フィールドを単独で subscribe する topic．  

#### `/dynamixel/command/goal` ([`DynamixelGoal`型](./dynamixel_handler_msgs#dynamixelgoal-type))      
/dynamixel/commands/{~}` の `goal` フィールドを単独で subscribe する topic．

#### `/dynamixel/command/gain` ([`DynamixelGain`型](./dynamixel_handler_msgs#dynamixelgain-type))    
/dynamixel/commands/{~}` の `gain` フィールドを単独で subscribe する topic．

#### `/dynamixel/command/limit` ([`DynamixelLimit`型](./dynamixel_handler_msgs#dynamixellimit-type))    
/dynamixel/commands/{~}` の `limit` フィールドを単独で subscribe する topic．

#### Xシリーズ用の制御コマンド
   関連するgoal値の設定＋制御モードの変更を行う. 
  - **`/dynamixel/command/x/pwm_control`** ([`DynamixelControlXPwm`型](./dynamixel_handler_msgs#dynamixelcontrolxpwm-type))  
  - **`/dynamixel/command/x/current_control`** ([`DynamixelControlXCurrent`型](./dynamixel_handler_msgs#dynamixelcontrolxcurrent-type))  
  - **`/dynamixel/command/x/velocity_control`** ([`DynamixelControlXVelocity`型](./dynamixel_handler_msgs#dynamixelcontrolxvelocity-type))  
  - **`/dynamixel/command/x/position_control`** ([`DynamixelControlXPosition`型](./dynamixel_handler_msgs#dynamixelcontrolxposition-type))  
  - **`/dynamixel/command/x/extended_position_control`** ([`DynamixelControlXExtendedPosition`型](./dynamixel_handler_msgs#dynamixelcontrolxextendedposition-type))  
  - **`/dynamixel/command/x/current_base_position_control`** ([`DynamixelControlXCurrentPosition`型](./dynamixel_handler_msgs#dynamixelcontrolxcurrentposition-type))

#### Pシリーズ用の制御コマンド
 関連するgoal値の設定＋制御モードの変更を行う. 
  - **`/dynamixel/command/p/pwm_control`** ([`DynamixelControlPPwm`型](./dynamixel_handler_msgs#dynamixelcontrolppwm-type))  
  - **`/dynamixel/command/p/current_control`** ([`DynamixelControlPCurrent`型](./dynamixel_handler_msgs#dynamixelcontrolpcurrent-type))  
  - **`/dynamixel/command/p/velocity_control`** ([`DynamixelControlPVelocity`型](./dynamixel_handler_msgs#dynamixelcontrolpvelocity-type))  
  - **`/dynamixel/command/p/position_control`** ([`DynamixelControlPPosition`型](./dynamixel_handler_msgs#dynamixelcontrolpposition-type))  
  - **`/dynamixel/command/p/extended_position_control`** ([`DynamixelControlPExtendedPosition`型](./dynamixel_handler_msgs#dynamixelcontrolpextendedposition-type))

 #### Proシリーズ用の制御コマンド
  関連するgoal値の設定＋制御モードの変更を行う. 
  - **`/dynamixel/command/pro/current_control`** ([`DynamixelControlProCurrent`型](./dynamixel_handler_msgs#dynamixelcontrolprocurrent-type))  
  - **`/dynamixel/command/pro/velocity_control`** ([`DynamixelControlProVelocity`型](./dynamixel_handler_msgs#dynamixelcontrolprovelocity-type))  
  - **`/dynamixel/command/pro/position_control`** ([`DynamixelControlProPosition`型](./dynamixel_handler_msgs#dynamixelcontrolproposition-type))  
  - **`/dynamixel/command/pro/extended_position_control`** ([`DynamixelControlProExtendedPosition`型](./dynamixel_handler_msgs#dynamixelcontrolproextendedposition-type))


***************************

## Parameters

### 通信関係の設定
```yaml
# 通信機器の設定
  device_name: /dev/ttyUSB0 # 通信するデバイス名
  baudrate: 1000000 # 通信速度
  latency_timer: 4 # 通信のインターバル
```
基本的な通信の設定．自分の環境に合わせて設定する．

### 初期化・終了時等の挙動設定
```yaml
# サーボの初期設定
  init/dummy_servo_list: [-1] # ダミーのサーボを作成するIDのリスト,同じIDのサーボが存在する場合でもダミーが優先される． [0, 254]の範囲外は無視される．
  init/baudrate_auto_set: false # 探索前に，全てのサーボと全てのBaudrateに対して，baudrateの書き込みをするかどうか
  init/used_servo_series: {X: true, P: false, Pro: false} # 使用するサーボのシリーズ
  init/expected_servo_num: 0
  init/servo_auto_search: {min_id: 0, max_id: 10, retry_times: 0} # サーボのIDを自動で探索するかどうか
  init/hardware_error_auto_clean: true # 初期化時に Hardware error を自動でクリアするかどうか
  init/torque_auto_enable: true # 初期化時に Torque を自動でONにするかどうか
  term/torque_auto_disable: true # 終了時に Torque を自動でOFFにするかどうか
  term/servo_auto_stop: true # 終了時に電流・速度制御のサーボを停止するかどうか
```
`init/dummy_servo_list`に指定したIDは，実際のサーボが存在しなくても，プログラム上ダミーのサーボが作成される．つまり何もつながなくても `/dynamixel/states`などの topic がpublishされる．       
実機がない場合での上位の node の動作確認などに便利．
指定したIDのサーボがつながれている場合でもIDが指定されていればダミーサーボが優先されるので，実際のサーボとの通信は行われない．      
ロボットを動かさずに上位の node の動作確認がしたいときに，ロボット全部のサーボのIDをダミーに指定してやると安全に動作確認ができる．
利用できるIDは[0, 254]の範囲で，それ以外のIDは無視される．空配列を与えるとエラーが起きるので注意．

`init/baudrate_auto_set`が `true`の時は，全てのサーボと全ての baudrate に対して，事前に `baudrate`の書き込みを行う．   
したがって，起動時にBaudrateが `baudrate`と異なるサーボがあっても自動的に `baudrate`に統一される．

`init/expected_servo_num`が `0`の時は，1つ以上servoが見つかるまでスキャンを繰り返す．  
`init/expected_servo_num`が `0`でない場合は，その数だけservoが見つかるまでスキャンを繰り返す．  
`init/servo_auto_search.retry_times`の回数分のスキャンが失敗した場合，初期化失敗で`dynamixel_handler`ノードは落ちる．

```yaml
# デフォルト値の設定
  default/profile_acc: 600.0 # deg/s^2
  default/profile_vel: 100.0 # deg/s
  default/return_delay_time: 0.0 # us [double]
```
`default/profile_acc`と`default/profile_vel`は位置制御時の最大加速度と最大速度を決める．
この値が大きければキビキビとした動作になり，小さければ滑らかな動作になる．
`{~}_control`系の topic で動的に指定することも可能．

`default/return_delay_time`はサーボの応答遅延時間を設定する．基本的に0でいいはず．

```yaml
# 通信の設定
  dyn_comm/retry_num: 10 # 通信失敗時のリトライ回数
  dyn_comm/inerval_msec: 5 # 通信失敗時のインターバル時間
  dyn_comm/verbose: false # 通信失敗時の詳細をエラーとして出すか
```
主に ping などのサーボ単体との通信失敗時の挙動を決める．    
基本的に数字が小さい方が初期化が早くなるが，サーボを発見できない可能性が高くなる．(デフォルト値は大きめに設定している)

`dyn_comm/verbose`が `true`の時は，通信失敗時に詳細なエラーを出力する．基本は `false`で問題ない．

### 実行時の動作設定
```yaml
# ループの設定
  loop_rate: 100 # メインループの周期
  verbose_ratio: 300 # メインループのlog出力の割合(処理時間，通信の成功率), ex 100なら100回に1回出力
  pub_outdated_present_value: true # 最新でないpresent_XXXをpublishするかどうか. false ならば 直近のループでreadした Present の要素のみmsgに含められる．
  pub_ratio/present: # present_XXXを読み取り，/dynamixel/state/present トピックをpublishする割合
      pwm:                 0 # この回数に一回present_pwmを読み取る, 0=メインループで読み取らない
      current:             2 # この回数に一回present_currentを読み取る, 0=メインループで読み取らない
      velocity:            2 # この回数に一回present_velocityを読み取る, 0=メインループで読み取らない
      position:            2 # この回数に一回present_positionを読み取る, 0=メインループで読み取らない
      velocity_trajectory: 0 # この回数に一回velocity_trajectoryを読み取る, 0=メインループで読み取らない
      position_trajectory: 0 # この回数に一回position_trajectoryを読み取る, 0=メインループで読み取らない
      input_voltage:      29 # この回数に一回present_input_voltageを読み取る, 0=メインループで読み取らない
      temperature:        29 # この回数に一回present_temperatureを読み取る, 0=メインループで読み取らない
  pub_ratio/status: 47 # この回数に一回 Status を読み取り，/dynamixel/state/status トピックをpublishする, 0=メインループで読み取らない
  pub_ratio/goal: 11  # この回数に一回 Goal を読み取り，/dynamixel/state/goal トピックをpublishする, 0=メインループで読み取らない
  pub_ratio/gain: 101 # この回数に一回 Gain を読み取り，/dynamixel/state/gain トピックをpublishする, 0=メインループで読み取らない
  pub_ratio/limit: 307 # この回数に一回 Limit を読み取り，/dynamixel/state/limit トピックをpublishする, 0=メインループで読み取らない
  pub_ratio/error: 53 # この回数に一回 Hardware error を読み取り，/dynamixel/state/error トピックをpublishする, 0=メインループで読み取らない
```
`pub_ratio/{~}`の各情報 (status, present, goal, gain, limit, error) は topic 名と対応．   
read と publish 周期は`loop_rate`を`pub_ratio/{~}`で割った値となる．   
> 例: `loop_rate`= 100, `pub_ratio/status`= 47 の時 100/47 ≃ 2Hz．   
> `pub_ratio` が `0` の項目はメインループで read されない（初期化時の値は保持される）．

デフォルト値を素数にしているのは, serial read のタイミングが被って1ループの処理時間が長くなるのを防ぐため．

present値のみ高速化のために各アドレス(pwm, current, ... , temperature)の読み取り割合を設定できる．   
`~/present`トピックの publish 周期は`loop_rate`を`pub_ratio/present.{~}`の最小値で割った値となる．   
> 例: `loop_rate`= 100, `pub_ratio/present.current`= 2 の時 100/2 = 50Hz．

このため，present値の直近で読み取った最新の値と，古い値が混在することになる．   
`pub_outdated_present_value`が`true`の場合は古い値も含めて全てのアドレスの値を publish する．   
`pub_outdated_present_value`が`false`の場合は，直近のループで読み取った値のみを publish する．  

```yaml
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
```yaml
# デバッグ用
  max_log_width: 6 # 以下のlog出力で，サーボ何個ごとに改行を入れるか
  verbose/callback: true # コールバック関数の呼び出しを出力
  verbose/write_goal: false # 書き込みするgoalデータのpulse値を出力
  verbose/write_gain: false # 書き込みするgainデータのpulse値を出力
  verbose/write_limit: false # 書き込みするlimitデータのpulse値を出力
  verbose/read_status:  {raw: false, err: false}
  verbose/read_present: {raw: false, err: false}
  verbose/read_goal:    {raw: false, err: false}
  verbose/read_gain:    {raw: false, err: false}
  verbose/read_limit:   {raw: false, err: false}
  verbose/read_hardware_error: true # 検出したHardware errorを出力
```
基本的には上記の説明の通り．
`verbose/write_{~}`や`verbose/read_{~}`が`true`の場合は，書き込むアドレスと書き込むパルス値を直接確認できる．

### 開発用
```yaml
# 開発用
  debug: false # true: デバイスとの接続に失敗してもエラーを出力しない/ false: エラーを出力して終了する
  no_use_command_line: false # true: プログラム用の topic のみ使用する/ false: コマンドライン用の topic も出力する
```
`debug`が`true`の場合は，デバイスとの接続失敗やサーボが見つからなかった場合でもプログラムが続行する．

`no_use_command_line`が`true`の場合は，コマンドライン用の topic を publish & subscribe しないので，`$ ros2 topic list`がすっきりする．起動時にDDSがネットワークにかける負荷も小さくなるかも？(未確認)

***************************

## Optional機能

Dynamixel本体の標準的な制御（status/goal/present/gain/limit/error）とは独立した拡張機能．  
現状は `External Port` と `OpenCR IMU` に対応している．

### 概要

| Feature | 有効化パラメータ | Publish | Subscribe | 初期化時の挙動 |
|---|---|---|---|---|
| External Port | `option/external_port.use` | `dynamixel/external_port/read` | `dynamixel/external_port/write` | 対象サーボが無くても継続 |
| OpenCR IMU | `option/imu_opencr.use` | `dynamixel/imu/raw` | `dynamixel/imu/calibration_gyro` | WARNを出して継続 |

### External Port

X540シリーズと P/Pro シリーズの外部ポート機能を扱う optional 機能．  
利用する場合は `option/external_port.use` を `true` に設定する．

```yaml
option/external_port:
    use: false
    pub_ratio/data: 10  # Dataを読む周期（loop回数基準）
    pub_ratio/mode: 100 # Modeを読む周期（loop回数基準）
    verbose/callback: false
    verbose/write: false
    verbose/read: {raw: false, err: false}
```

- Publish: `dynamixel/external_port/read`
- Subscribe: `dynamixel/external_port/write`
- Data項目: `external_port_data_{1,2,...}`
- Mode項目: `external_port_mode_{1,2,...}`
- デフォルトでは読み込み対象IDは空で，`/dynamixel/external_port/write` で指定されたIDのみを読む

#### `/dynamixel/external_port/read` ([`DxlExternalPort`型](./dynamixel_handler_msgs#dynamixel_handler_msgsmsgdxlexternalport-type))    

`option/external_port.pub_ratio/mode` と `option/external_port.pub_ratio/data` のいずれかで読み取りが発生したループで publish される．

| Field | Type | Description |
|-------|------|-------------|
| `stamp` | `builtin_interfaces/Time` | データが読み取れた時刻 |
| `id_list` | `uint16[]` | サーボのID |
| `port` | `uint16[]` | External Portのポート番号 |
| `mode` | `string[]` | ポートのモード |
| `data` | `int16[]` | ポートのデータ (`analog`: 0~4096, `digital`: 0/1, `unset`: -1) |

mode 文字列の意味:

- `"a_in"`: analog in
- `"d_out"`: digital out
- `"d_in_pu"`: digital in (pull up)
- `"d_in_pd"`: digital in (pull down)
- `"unset"`: ポート未設定

#### `/dynamixel/external_port/write` ([`DxlExternalPort`型](./dynamixel_handler_msgs#dynamixel_handler_msgsmsgdxlexternalport-type))    

サーボのIDとポート番号の組で External Port を指定する．  
`id_list` と `port` の長さは一致が必要で，`mode` / `data` は必要な方だけ指定できる（指定した場合は `id_list` と同じ長さが必要）．

| Field | Type | Description |
|---|---|---|
| `stamp` | `builtin_interfaces/Time` | メッセージのタイムスタンプ（未使用） |
| `id_list` | `uint16[]` | 適用するサーボID |
| `port` | `uint16[]` | ポート番号 (X: 1-3, P/Pro: 1-4) |
| `mode` | `string[]` | ポートのモード文字列 |
| `data` | `int16[]` | ポートのデータ（`digital out`時に0/1を使用） |

### OpenCR IMU

OpenCR上のIMUを `sensor_msgs/msg/Imu` として publish する optional 機能．

```yaml
option/imu_opencr:
    use: false
    opencr_id: 40
    frame_id: base_link
    pub_ratio: 2
    model_number: 0
    adjust/rpy_deg: [0.0, 0.0, 180.0]
    adjust/flip_z_axis: true
    verbose/callback: false
    verbose/write: false
    verbose/read: {raw: false, err: false}
```

- Publish: `dynamixel/imu/raw` (`sensor_msgs/msg/Imu`)
- Subscribe: `dynamixel/imu/calibration_gyro` (`std_msgs/msg/Empty`)
- 起動時に `opencr_id` への `ping` と `model_number` を確認し，不一致時はWARNを出して機能を無効化
- `calibration_gyro` 受信時はOpenCRへ校正コマンドを書き込み，現状実装では5秒待機
- `adjust/rpy_deg` と `adjust/flip_z_axis` で座標補正を適用
- OpenCRファームのControl Tableアドレスと `src/optional_function/imu_opencr.cpp` は必ず対応させること

#### `dynamixel/imu/raw` (`sensor_msgs/msg/Imu`)

`loop_rate` に対して `pub_ratio` ごとに read を実行し，read 成功時のみ publish する．

| Field | Type | Description |
|---|---|---|
| `header.stamp` | `builtin_interfaces/Time` | publish時刻 (`now`) |
| `header.frame_id` | `string` | `option/imu_opencr.frame_id` |
| `orientation.{x,y,z,w}` | `float64` | 補正後クォータニオン（正規化済み） |
| `angular_velocity.{x,y,z}` | `float64` | 補正後角速度 |
| `linear_acceleration.{x,y,z}` | `float64` | 補正後加速度 |
| `orientation_covariance` | `float64[9]` | 現状は未設定（デフォルト値） |
| `angular_velocity_covariance` | `float64[9]` | 現状は未設定（デフォルト値） |
| `linear_acceleration_covariance` | `float64[9]` | 現状は未設定（デフォルト値） |

#### `dynamixel/imu/calibration_gyro` (`std_msgs/msg/Empty`)

受信時に `opencr_id` 宛てにジャイロ校正コマンドを書き込む．  
メッセージ本体のフィールドはなく，トリガー用途のみで使う．  
コマンド送信後は現状実装で5秒待機する．



***************************

## 各種情報の分類と Control Table との対応

本パッケージでは，Dynamixelが持つ Control table 内の情報を，以下の様に分類して扱う．

### 状態 (status)
 - torque_enable  : サーボが制御を受け付けるかどうか．falseだと脱力してトルクを出さない．  
 - (ping)         : Control table の情報ではないが，statusとして扱っている．pingが通るかどうか．
 - (error)        : Control table の情報ではないが，statusとして扱っている．何らかのエラーを持っているかどうか．
 - operating_mode : サーボの制御モード，PWM, 電流制御, 速度制御, 位置制御, 拡張位置制御, 電流制御付き位置制御の6つのモードがある．
 
##### Subscribe / Write  
Xシリーズの場合，`/dynamixel/commands/x`の`status`フィールド or `/dynamixel/command/status`によって設定され，`loop_rate`の周期で書き込まれる．
operating_modeのみ，対応する`/dynamixel/command/x/{~}_control`系の topic を subscribe することでも自動で設定される． 
##### Publish / Read  
`loop_rate`の内`pub_ratio/status`毎に1回の周期で読みだされ，`/dynamixel/states`の`status`フィールド and `/dynamixel/state/status`として publish される．

### 目標値 (goal)
 - goal_pwm             : 目標PWM値, PWM制御モードでのみ有効
 - goal_current         : 目標電流値, 電流制御モードと電流制御付き位置制御でのみ有効
 - goal_velocity        : 目標速度, 速度制御モードでのみ有効
 - goal_position        : 目標角度, 位置制御モードと拡張位置制御モード，電流制御付き位置制御で有効
 - profile_acceleration : 最大加速度, 速度制御・位置制御・拡張位置制御・電流制御付き位置制御モードで有効
   - (goal_acceleration): Proシリーズでの表記.
 - profile_velocity     : 目標速度値, 位置制御モードと拡張位置制御モード，電流制御付き位置制御で有効 (Proシリーズにはないが，goal_velocityで代用できる)

##### Subscribe / Write
Xシリーズの場合，`/dynamixel/commands/x`or `/dynamixel/command/x/{~}_control`系の topic  or `/dynamixel/command/goal`によって設定され，`loop_rate`の周期で書き込まれる．   
##### Publish / Read
`loop_rate`の内`pub_ratio/goal`毎に1回の周期で読みだされ，`/dynamixel/states`の`goal`フィールド and `/dynamixel/state/goal`として publish される．

### 現在値 (present)
 - present_pwm          : 現在のPWM値 (Proシリーズにはない)
 - present_current      : 現在の電流値
 - present_velocity     : 現在の速度
 - present_position     : 現在の角度
 - velocity_trajectory  : 目標速度のようなもの (Proシリーズにはない)
 - position_trajectory  : 目標角度のようなもの (Proシリーズにはない)
 - present_input_voltage: 現在の入力電圧
 - present_temperature  : 現在の温度

##### Subscribe / Write
書き込みは不可．   
##### Publish / Read
各アドレスの情報は`loop_rate`の内`pub_ratio/present.{~}`に一回の周期で読みだされる．   
読みだされた情報は，`loop_rate`の内`pub_ratio/present.{~}`の最小値の割合で，`/dynamixel/states`の`present`フィールド and `/dynamixel/state/present`として publish される．

### ゲイン (gain)
 - velocity_i_gain       :
 - velocity_p_gain       :
 - position_d_gain       :　Proシリーズでは無効
 - position_i_gain       :　Proシリーズでは無効
 - position_p_gain       :
 - feedforward_acc_gain  :　Proシリーズでは無効
 - feedforward_vel_gain  :　Proシリーズでは無効
  
##### Subscribe / Write
Xシリーズの場合，`/dynamixel/commands/x`の`gain`フィールド or `/dynamixel/command/gain`によって設定され，`loop_rate`の周期で書き込まれる．
##### Publish / Read
`loop_rate`の内`pub_ratio/gain`毎に1回の周期で読みだされ，`/dynamixel/states`の`gain`フィールド and `/dynamixel/state/gain`として publish される．

> [!note]
> 制御モードによってデフォルト値が異なり，モードを変えると勝手に書き換えられてしまう．制御モードをまたぐ場合の処理については検討中．

### 制限 (limit)
 - temperature_limit : 温度がこの値を超えると Hardware error (overheating) が発生する．
 - max_voltage_limit : 入力電圧がこの値を超えると Hardware error (input_voltage) が発生する．
 - min_voltage_limit : 入力電圧がこの値を下回ると Hardware error (input_voltage) が発生する．   
 - pwm_limit         : 指定・発揮できるPWMの最大値 (Proシリーズにはない)
 - current_limit     : 指定・発揮できる最大電流値
 - acceleration_limit : 最大加速度 (Xシリーズにはない)
 - velocity_limit     : 最大速度
 - max_position_limit : 位置制御モードでの最大角度 
 - min_position_limit : 位置制御モードでの最小角度
  
##### Subscribe / Write
Xシリーズの場合，`/dynamixel/commands/x`の`limit`フィールド or `/dynamixel/command/limit`によって設定され，`loop_rate`の周期で書き込まれる．
##### Publish / Read
`loop_rate`の内`pub_ratio/limit`毎に1回の周期で読みだされ，`/dynamixel/states`の`limit`フィールド and `/dynamixel/state/limit`として publish される．

> [!note]
> ROM領域の値の書き込みは torque が on  状態ではできないため，limit の書き込みが発生する場合は一瞬だけ torque を off にする処理が入ることに注意．

### エラー (error)
 - hardware_error_status  : サーボのハードウェアエラー情報    
   検知されるエラーは以下の7種類．
   - input_voltage      : 入力電圧が制限範囲外
   - motor_hall_sensor  : モータのホールセンサの異常
   - overheating        : 温度が最大値を超えた
   - motor_encoder      : モータのエンコーダの異常
   - electronical_shock : 電子回路内での異常
   - overload           : 過負荷

##### Subscribe / Write
書き込みは不可．  
##### Publish / Read
`loop_rate`の内`pub_ratio/error`毎に1回の周期で読みだされ，`/dynamixel/states`の`error`フィールド and `/dynamixel/state/error`として publish される．
  
### その他 (extra)
 - drive_mode             : (Proシリーズにはない) 
 - return_delay_time      : Dynamixelの応答遅延時間，デフォルトで$500$usだが，この pkg では初期化時に$0$usに上書きされる．
 - homing_offset          : この値が現在の真の角度に加算されて出力される．この pkg では，reboot時の角度補正に用いられる．
 - moving_threshold       : 
 - startup_configuration  : not support, buckupがあるときPIDゲインやprofile系の値を自動で復元してくれるが，PIDのデフォルト値がモードによって異なる問題があるので使わない．
 - shutdown               : 
 - status_return_level    : not support, 常に2を前提とする
- bus_watchdog          : 指定時間通信が行われない場合に動作を停止する(トルクOFFとは異なる)，この pkg では node kill 時と通信断絶時にサーボを自動停止させる機能に用いられる．(Proシリーズにはない)
 - led                    : 
 - registered_instruction : 
 - realtime_tick          : 
 - moving                 : 
 - moving_status          : 

 pub/subによる読み書きは未実装

> [!note] 
> (bus_watchdog の設定値が1以上の時) bus_watchdogの設定値 × 20ms 通信がないと自動で動作停止処理が実行される．
> しかし，位置制御系のモードかつ，homing_offset が設定されている状態でこの動作停止処理が走るとなぜか homing_offsetだけ回転する．
> firmwareのバグの模様，バージョンによってはこの問題が解消されているかもしれないが，危険なので位置制御系のモードで bus_watchdog を使うのは避けるべき．
> ということで `term/servo_auto_stop`が`true`かつ，PWM・電流・速度制御モードの場合のみ，$500$ms で停止するように bus_watchdog を設定し，それ以外のモードでは bus_watchdog は $0$ として利用しないようにしている．

## Baudrate の一括変更

`config/config_dynamixel_unify_baudrate.yaml`の以下の部分を編集し，保存
```yaml
# config/config_dynamixel_unify_baudrate.yaml
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

> [!NOTE]
> `dynamixel_unify_baudrate` は Python ノードであり，内部で `_mylib_dynamixel`（pybind11モジュール）を利用する．
> `_mylib_dynamixel` が見つからない場合は，`dynamixel_handler` パッケージを再buildすること．

***************************

## Latency Timer

シリアル通信にはパケットの送受信の間に latency timer 分のインターバルが挟まる．
(USBデバイスのデフォルトは16msのようであり，高速な通信の妨げとなることが多い)
安定した通信のためには，使用するUSBデバイスの latency timer と ros param の `latency_timer` を一致させる必要がある．

ros param の変更には，`config/config_dynamixel_handler.yaml`の以下の部分を編集して保存する．
```yaml
# config/config_dynamixel_handler.yaml
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


## Trouble Shooting

### 「dynamixel_handlerがない」もしくは，「メッセージがない」といったエラーが出る場合

ターミナルを立ち上げ直すか，以下を実行．
```bash
source ~/ros2_ws/install/setup.bash
```

### `colcon build` で `src/internal/mylib_dynamixel/...` が見つからない場合

submodule 未初期化が原因のことが多い．以下を実行してから再buildする．

```bash
cd ~/ros2_ws/src/DynamixelHandler-ros2
git submodule update --init --recursive
cd ~/ros2_ws
colcon build --packages-select dynamixel_handler
```

### `dynamixel_unify_baudrate` 実行時に `_mylib_dynamixel` が見つからない場合

`dynamixel_handler` の build が未完了か，ビルド成果物が古い．以下を再実行する．

```bash
cd ~/ros2_ws
colcon build --packages-select dynamixel_handler
source ~/ros2_ws/install/setup.bash
```

### `dynamixel_handler`ノードを起動してもdynamixelが1つも見つからない場合

1. デバイス名の確認   
   どんな方法で確認しても良いが，Ubuntuの場合はDynamixelを認識するはずのUSBを抜く前後で `$ ls /dev/ttyUSB*`の出力を比較すれば，少なくとも正しいデバイス名がわかる．
1. デバイスの実行権限の確認   
   `$ sudo chmod 777 /dev/{your_device_name}`として改善すれば実行権限問題であることがわかる．
1. Dynamixel側の baudrate の確認    
   Dynamixel wizardで確認するのが最も確実．
   とりあえず動くようにするには `init/baudrate_auto_set`パラメータを `true`にセットするか,
    [`dynamixel_unify_baudrate`ノード](#Baudrateの一括変更) で目的の baudrate に強制変換してしまうのが良い．

### `dynamixel_handler`ノードを起動したときにdynamixelが一部しか見つからない

1. latency timer の確認    
   `$ cat /sys/bus/usb-serial/devices/{your_device_name}/latency_timer`を実行して出てきたデバイス側の latency timer の数値が，ros parameter の `latency_timer`と一致しているかどうかを確認する．  
   ros parameter 側の値を変えたい場合は config ファイルを修正すればよい．   
   デバイス側値を変えたい場合は [Latency Timer](#latency-timer) を参照．
3. 通信状態が悪すぎる場合   
   根本的にはケーブルやノイズ等を改善すべきだが，対症療法的に`dyn_comm/retry_num`を大きくすることでも改善する可能性がある．
   [Parameters](#parameters) の該当パラメータを参照

### `dynamixel_handler`ノードを起動したときにサーボの初期化が終わらない．
```
[dynamixel_handler-1] 1748119256.347349302: Initializing DynamixelHandler ..... 
[dynamixel_handler-1] 1748119256.352399757:  Succeeded to open device 
[dynamixel_handler-1] 1748119256.352459648:   ------------ name '/dev/ttyUSB0'
[dynamixel_handler-1] 1748119256.352463615:   -------- baudrate '1000000'
[dynamixel_handler-1] 1748119256.352466370:   --- latency_timer '1'
[dynamixel_handler-1] 1748119256.352996238:  Expected servo number is not set. 
[dynamixel_handler-1] 1748119256.353100349:  > Free number of Dynamixel is allowed 
[dynamixel_handler-1] 1748119256.353110024:  Auto scanning Dynamixel (id range '0' to '10') ...
[dynamixel_handler-1] 1748119256.353129157:  > series: X [use], P [no use], PRO [no use]
[dynamixel_handler-1] 1748119256.397073588:   *  X  series servo ID [1] is found
[dynamixel_handler-1] 1748119256.397073588:     Reading  present values...
[dynamixel_handler-1] 1748119256.397073588:     Reading  present values...
...
```
上記のような表記が続く．

1. Read methodの確認
   `method/fast_read`が`true`になっているか確認する．
   一部のモータは Fast Sync Readに対応しておらず，そうしたモータの場合初期の通信が失敗して無限ループに入ってしまう．
   [Parameters](#parameters)　の`method/fast_read`を`false`にして，通常のSync Readに切り替えることで解決する．

### dynamixel は見つかったが通信の成功率が極端に低い

1. Dynamixelの接続状態の確認
   TTLとRS485を並列に同時に使うと起きる．
1. latency timerの確認
   デバイス側の latency timer の値よりも，ros parameter の `latency_timer`の値が小さいときに起きる．
1. 別のノードによるUSBデバイスの占有
   他のノードがUSBデバイスを占有していると，通信がうまくいかないことがある．
   その場合は，`$ ros2 node list`で他のノードを確認し，必要に応じて停止する．

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
以下のコマンドをwsl内で実行すると解決する．
```bash
sudo update-alternatives --install /usr/local/bin/usbip usbip `ls /usr/lib/linux-tools/*/usbip | tail -n1` 20
```

```text
usbipd: error: WSL kernel is not USBIP capable; update with 'wsl --update'.
```
以下のコマンドをwsl内で実行すると解決する．
```bash
sudo modprobe vhci_hcd
```

## Launchファイルと設定（yaml）
### launch_dynamixel_unify_baudrate.py
```bash
ros2 launch dynamixel_handler launch_dynamixel_unify_baudrate.py
```
対応する``yaml``は``config/config_dynamixel_unify_baudrate.yaml``

### launch_dynamixel_handler.py
```bash
ros2 launch dynamixel_handler launch_dynamixel_handler.py
```
対応する``yaml``は``config/config_dynamixel_handler.yaml``

※ 一度ビルドしていれば，yamlファイルの変更に伴うビルドは不要


