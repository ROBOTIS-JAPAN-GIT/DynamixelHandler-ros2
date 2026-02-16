# 開発者向けReadMe

## プログラムの構成

Dynamixelとやり取りを行うライブラリは[別のリポジトリ](https://github.com/SHINOBI-organization/lib_dynamixel)として管理しており，git submoduleの機能を使って取り込んでいる．
現在の取り込み先は `src/internal/mylib_dynamixel` である．

`develop/faster_build` マージ後は，`dynamixel_unify_baudrate` が Python ノード実装になっており，
内部で pybind11 モジュール `_mylib_dynamixel` を利用する構成になっている．

このパッケージはROS2ノードとして動作し，srcディレクトリ以下に主要な実装が配置されています．各ファイルの役割は以下の通りです:

- **src/main.cpp**  
  ノードのエントリーポイント．
  `DynamixelHandler` の生成，executor への登録，spin を行う最小構成．

- **src/dynamixel_handler.cpp**  
  `DynamixelHandler` クラス本体の実装．
  初期化処理，パラメータ読込，publisher/subscriber生成，optional機能の起動，
  および `MainLoop()` の実処理（read/write，publish）を担当する．

- **src/dynamixel_handler.hpp**  
  本パッケージの中心となるクラスの宣言を行い，Dynamixelとの通信および各種データ（状態，コマンド，制御パラメータ）の管理を実装しています．

- **src/dynamixel_handler_ros_interface_sub.cpp**  
  ROSのsubscribeインターフェースを担当し，各種コマンドメッセージの受信と一時的なコマンド情報の保持についての処理を実装しています．

- **src/dynamixel_handler_ros_interface_pub.cpp**  
  ROSのpublishインターフェースを担当し，Dynamixelの状態やエラー情報，デバッグ情報などを各トピックに出力する処理を実装しています．

- **src/dynamixel_handler_ros_setup_for_program.cpp**  
  プログラム連携向けのROS interface初期化を担当し，
  `dynamixel/commands/{all,x,p,pro}` と `dynamixel/states` / `dynamixel/debug` の生成を行う．

- **src/dynamixel_handler_ros_setup_for_cli.cpp**  
  CLI運用向けのROS interface初期化を担当し，
  `dynamixel/command/...` 系・`dynamixel/state/...` 系など細分化トピックの生成を行う．

- **src/dynamixel_handler_dyn_interface_loop.cpp**  
  動作ループ内での一括読み書き処理（SyncRead，SyncWrite）を担当し，各サーボの状態やエラー情報の更新，コマンドの指令，状態の取得などを実装しています．

- **src/dynamixel_handler_dyn_interface_once.cpp**  
  動作ループ外での一括読み書き処理（Read，Write）を担当し，各サーボの状態やエラー情報の更新，コマンドの指令，状態の取得などを実装しています．

- **src/optional_function/**  
  optional機能の実装ディレクトリ．
  現在は `external_port.cpp/.hpp` と `imu_opencr.cpp/.hpp` を含む．

- **src/sub_node/dynamixel_unify_baudrate.py**  
  baudrate一括変更ノード本体（Python）．
  pybind11モジュール `_mylib_dynamixel` を介してシリアル通信を行う．

さらに，`src/internal/myUtils` 内のユーティリティ群は，ログ出力やフォーマッティング，イテレータの利便性向上など，本パッケージ全体で共通して利用される補助的な機能を提供している．
`src/internal/mylib_dynamixel` は通信ライブラリ本体（submodule）であり，`dynamixel_communicator` と `_mylib_dynamixel` のビルド元になっている．

***************************

## 未実装機能
 - extra に分類した情報の read/writeの実装
 - write するタイミングの検討について
   - 現在の方法：sub callback でストアしメインループで write
     - [＋] write回数が抑えられる．
       - 各IDへの command が別の topic に乗ってきても，node 側で 1/loop_rate [sec] 分の command をまとめてくれる
     - [＋] write の周期が一定以下になり，read の圧迫や負荷の変動が起きづらい
     - [－] 一度 command をストアするので，topic の sub から 最大 1/loop_rate [sec] の遅延が生じてしまう．
       - 8ms未満くらいは遅れるが，そもそものtopicの遅延の方が支配的?(topic遅延が6ms，callback->writeが遅延2ms)
   - もう一つの方法：sub callback で直接 write
     - [＋] callback後の遅延は生じない
     - [－] topic の pub の仕方によってはwrite回数が増えてしまう
       - 例えば，ID:5へ指令する command topic と ID:6が別のノードからpubされているとすると，callbackは2回呼ばれる．一度ストアしてからまとめてWrite方式だとwriteは1回だが，callbackで直接Write方式だとwriteも2回

## 開発メモ

### Node終了時の停止について

- bus_watchdogで停止できそう？
  - RAM値なので簡単に書き込みできる
  - 初期化時の最初に書き込まないと，bus_watchdogがエラー中のサーボのgoal値を書き換えられない
  - Velocity・current制御モードの場合だけ停止するようにしていたが，全部停止するようにしてもいいかもしれない．
- bus_watchdogのバグあり．
  - XC330は停止しない？
  - XM540は停止はするけどhoming_offsetの値分動いてから停止する．なんで？？？ (Firmware update 46 to 48 で治った？治ってないやんけ！)


1. 通信断絶時の停止について
   - 速度・電流・PWM制御モードではデフォルトでbus_watchdogによって止まるようにしたい．
   - `term/servo_auto_stop`パラメータで制御する？
   - 取りあえずbus_watchdogでだいたいは大丈夫なはず．
     - XM540の位置制御系では，homing_offsetが設定時のバグが残っているが，速度・電流・PWM制御系では問題ない．
   - (例外) XC330はそもそも止まらない．
     - こいつのみ別途止める必要がある？ないか．
     - まあ，通信断絶
  
2. ノードの明示的な停止について
    - こいつも `term/servo_auto_stop` で制御する．
    - 速度・電流・PWM制御モードについてはbus_watchdogでクリアしている．
      - 念のため即時停止するように最も短いbus_watchdogを再設定するか？
    - 位置制御系については現在位置を指令しなおすしか手がないかな．

3. トルクの瞬間的なオンオフでは止まらなそう．
   - XC330は止まらずに最初の目標値目指して回り続ける (goal値はオンオフされた瞬間のものに更新されてる)
   - XM540は止まるときと，止まらずに加速するときがある．わけわからん．
  
### 遭遇したバグ

Fast Sync Read で 1...6 のサーボに read を送った状況で，ID:1モータが断線していたとする．  
当然，Fast Sync Read の return packet であるひとつながりのパケットは帰ってこないのでタイムアウト．   
その直後，1...6の各モータに対してPingを送るとなぜか**全て**のモータがタイムアウトする．  
「Fast Sync Read + 1つ目のIDの断線によりタイムアウト + 左記の直後に Ping」という状況だと，断線してないモータもタイムアウトしてしまうという問題が起きる．

 - Sync Read だとこの問題は起きない．
 - Fast Sync Read で指定するIDの先頭であるID:1 **以外** のモータを断線させても問題は起きない．
 - Fast Sync Read で指定するIDの順番を逆順にしたうえで，ID:1を断線させても問題は起きない．
 - Fast Sync Read で指定するIDの順番を逆順にしたうえで，ID:6を断線させると問題が起きる．

### Yシリーズ対応に向けて

- 方針としては何とか頑張ってX, Pシリーズと共通化する方向で行きたい．
- ただし，X, Pシリーズの方がYシリーズよりも機能が多いので，msgのfieldはYシリーズに合わせる形になるかも．

Commands 系: 個別に用意すればいいので問題なし?
  - goal: 順番は異なるが機能は同じ, profileのvelベースとtimeベースが分離してたりするが機能は同じ．
    - 結局Goal系はIndirect Addressで再構築しないと難しそう．
    - Profile_vel, Profile_accは, Profile_vel, Profile_acc, Profile_time, Profile_acc_timeの4つに分裂しているが，結局Drive_modeの設定でどのペアが使われるかが変わるので，以下のようにして他のシリーズと共用の形式にまとめられそう．
    ```
    float64[] profile_vel_deg_s  # こちらが埋まっている場合は drive_mode を velocity base　に
    float64[] profile_acc_deg_ss # こちらが埋まっている場合は drive_mode を velocity base　に
    float64[] profile_time_ms     # こちらが埋まっている場合は drive_mode を time base に + Yシリーズの場合はIndirect Address の方もvel->timeに書き換え
    float64[] profile_acc_time_ms # こちらが埋まっている場合は drive_mode を time base に + Yシリーズの場合はIndirect Address の方もvel->timeに書き換え
    ```
  - offset: PWM, Current, Velocityについて謎の項目が生えてる．
States 系: 1つのmsgに載せられるかは要検討
  - gain: 順番は異なるが同じ
  - present: 温度がモータとインバータで別れてる．他は同じ．項目を追加すれば対応可能．
  - limit: 同上
  - error: 種類の増加はともかく，エラーデータ構造が異なっている．種類増加は単純にフィールド追加で対応，データ構造はYシリーズのみ読み込み方を変えることで対応できそう．

順番違い系はIndirectAddressで何とかするか...？


エラークリア用のパケットが増えてる

## optional_function 実装ルール（開発者向け）

`optional_function` は「Dynamixelの基本制御に依存しない拡張機能」を追加するための層として扱う．
既存の `ExternalPort` と `ImuOpenCR` は同じ流儀で実装しているので，以下を共通ルールとする．

### 追加時の実装手順

#### 1) 新規作成するファイル

```text
src/optional_function/<function_name>.hpp
src/optional_function/<function_name>.cpp
```

#### 2) 既存ファイルの編集ポイント

`src/dynamixel_handler.hpp`

```cpp
class <FunctionClass>;
std::unique_ptr<FunctionClass> <function_ptr>_;
```

`src/dynamixel_handler.cpp`

```cpp
#include "optional_function/<function_name>.hpp"

bool use_<function_name>;
get_parameter_or("option/<function_name>.use", use_<function_name>, false);
if (use_<function_name>) {
    <function_ptr>_ = std::make_unique<FunctionClass>(*this);
}

// MainLoop() 内
if (<function_ptr>_) <function_ptr>_->MainProcess(); // 既存命名に合わせる
```

`config/config_dynamixel_handler*.yaml`

```yaml
option/<function_name>:
    use: false
    pub_ratio: 10
    verbose/callback: false
    verbose/write: false
    verbose/read: {raw: false, err: false}
```

README (`../ReadMe.md` / `ReadMe.md`) も同時に更新する．

### パラメータ設計のルール

#### config 側（yaml）のルール

- 有効化フラグは `option/<function_name>.use` を必須とする．
- 周期制御は `pub_ratio`（または `pub_ratio/<kind>`）で統一する．
- verbose系は `verbose/callback`, `verbose/write`, `verbose/read.{raw,err}` を基本形とする．

#### cpp 側（読込実装）のルール

```cpp
parent_.get_parameter_or("option/<function_name>.pub_ratio", pub_ratio_, 10u);
parent_.get_parameter_or("option/<function_name>.verbose/callback", verbose_callback_, false);
parent_.get_parameter_or("option/<function_name>.verbose/write",    verbose_write_,    false);
parent_.get_parameter_or("option/<function_name>.verbose/read.raw", verbose_read_,     false);
parent_.get_parameter_or("option/<function_name>.verbose/read.err", verbose_read_err_, false);
```

- パラメータ読込は `get_parameter_or` を使い，デフォルト値をコード上で明示する．
- configキー名とcppキー名を必ず一致させる．

### 初期化と失敗時挙動

#### 初期化時

- optional機能の初期化失敗でノード全体を停止しない（`ROS_STOP`しない）．
- 失敗時は `WARN` を出し，当該機能のみ無効化して続行する．
- 依存先（例: firmware の model/address）が不一致な場合は，理由をログに残して無効化する．

### 実行時処理のルール

- `MainProcess()` は極力軽量にし，ループ周期を阻害しない．
- read成功時のみpublishし，通信失敗時は `verbose/read.err` 設定に従って警告を出す．
- callback内で待機が必要な場合は用途を明確化し，ログを出す（例: IMU校正の待機）．

### 命名と責務分離

- クラス名は機能単位で明確にする（例: `ExternalPort`, `ImuOpenCR`）．
- 機能固有のtopic/param名は `dynamixel/<function>` および `option/<function>` に寄せる．
- サーボ制御の共通ロジック（goal/present/gain/limit/error）に不要な依存を持ち込まない．

### 実装前後チェックリスト

#### ファイル構成

1. `optional_function/<function_name>.hpp/.cpp` を作成したか．
2. `dynamixel_handler.hpp` に前方宣言と `unique_ptr` を追加したか．
3. `dynamixel_handler.cpp` に生成条件と `MainLoop` 呼び出しを追加したか．

#### 挙動

1. 初期化失敗時に「機能のみ無効化」で続行できるか．
2. `verbose` 設定で read/write/callback のログ粒度を制御できるか．
3. read失敗時に通信エラーを握りつぶさず，必要時に警告できるか．

#### ドキュメント

1. `config` と README の記述が実装と一致しているか．
2. topic名・param名・デフォルト値がREADMEに反映されているか．
