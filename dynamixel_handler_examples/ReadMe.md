# dynamixel_handler_examples

## Build

```bash 
cd ~/ros2_ws
colcon build --symlink-install --packages-up-to dynamixel_handler_examples
source ~/.bashrc # 初回 build 時のみ
```

## Pkg configuration

<details>
<summary>package.xml</summary>

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>dynamixel_handler_examples </name>
  <version>0.0.0</version>
  <description>The dynamixel_handler_examples package</description>
  <maintainer email="michikawa.ryohei@gmail.com">michikawa07</maintainer>
  <license>TODO</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <build_depend>dynamixel_handler_msgs</build_depend>

  <exec_depend>ros2launch</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>

</package>
```
</details>

<details>
<summary>Cmakelist.txt</summary>

```cmake
cmake_minimum_required(VERSION 3.8)
project(dynamixel_handler_examples)

#C++17を使えるように宣言
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_handler_msgs REQUIRED)

## Specify dependencies
set(dependencies
  rclcpp
  dynamixel_handler_msgs
)

## Build
include_directories(
  include
)

add_executable(example1
  src/example1.cpp
)

ament_target_dependencies(example1 ${dependencies})

## Install
install(TARGETS
  example1
  DESTINATION lib/${PROJECT_NAME}
)

# install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

</details>



## Example1　

### Launch

```bash
ros2 launch dynamixel_handler_examples example1.launch.py
``` 
上記コマンドで起動される `example1` node は，
  - トルクのオンオフ指令のpublish
  - 電流制限付き位置制御で往復運動指令をpublish
  - サーボ状態をsubscribeしてトルクなどの状態を表示
  - サーボ状態をsubscribeして現在値の保存と表示
を行う．

### Code Explanation

`dynamixel_handler_msgs` pkg が提供する msg 型を用いて，Dynamixelの動作制御や情報取得をするプログラムの例を示す．

> プログラムから動作制御するための topic として `/dynamixel/commands/x` (`DxlCommandsX`型) が用意されている．     
> (Pシリーズを制御する場合は `/dynamixel/commands/p` を利用, 両方を併用する場合は `/dynamixel/commands/all` を利用)   
>
> また，プログラムから情報取得するための topic として `/dynamxiel/states` (`DxlStates`型) が用意されている．    
> (シリーズ問わず全ての情報を利用できる)
> 
> 個別の topic を使うことも可能だが，publisher, subscriber の数が増えて coding の手間が増えるので非推奨．

ここでは，最低限の使い方として
  - トルクのオンオフ
  - 電流制限付き位置制御で往復運動
  - トルクなどの状態を表示
  - 現在値の保存と表示

を行う簡単なプログラムの一例を示す．       

<details>
<summary>コード全体</summary>

```cpp
// example1.cpp
#include "rclcpp/rclcpp.hpp"

#include "dynamixel_handler_msgs/msg/dxl_states.hpp"
#include "dynamixel_handler_msgs/msg/dxl_commands_x.hpp"
using namespace dynamixel_handler_msgs::msg;

#include <map>
#include <chrono>
using namespace std::chrono_literals;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node  = std::make_shared<rclcpp::Node>("example1_node");

    rclcpp::Time updated_time;
    std::map<uint8_t, double> dxl_pos, dxl_vel, dxl_cur;
    auto sub_st = node->create_subscription<DxlStates>("dynamixel/states", 10, 
        [&](const DxlStates::SharedPtr msg){
            // トルクのオンオフ，エラーの有無，pingの成否，制御モードなどの情報を表示．
            for (size_t i = 0; i < msg->status.id_list.size(); i++) {
                RCLCPP_INFO(node->get_logger(), "- servo [%d], torque %s, has %s, ping is %s, mode is %s", 
                    msg->status.id_list[i],
                    msg->status.torque[i] ? "on" : "off",
                    msg->status.error[i] ? "error" : "no error",
                    msg->status.ping[i] ? "response" : "no response",
                    msg->status.mode[i].c_str()
                ); 
            }
            // データがreadされた時刻の保存, 位置，速度，電流の現在値の保存
            if(!msg->present.id_list.empty()) updated_time = msg->stamp;
            for (size_t i = 0; i < msg->present.id_list.size(); i++) {
                auto id = msg->present.id_list[i];
                dxl_pos[id] = msg->present.position_deg[i];
                dxl_vel[id] = msg->present.velocity_deg_s[i];
                dxl_cur[id] = msg->present.current_ma[i];
            }
    });

    auto pub_cmd = node->create_publisher<DxlCommandsX>("dynamixel/commands/x", 10);
    auto timer = node->create_wall_timer(1.0s, [&](){
        RCLCPP_INFO(node->get_logger(), "* Present value updated time %f", updated_time.seconds());
        for (const auto& [id, _] : dxl_pos) {
            RCLCPP_INFO(node->get_logger(), "* servo [%d], pos %f, vel %f, cur %f", id, dxl_pos[id], dxl_vel[id], dxl_cur[id]);
        }

        auto cmd = DxlCommandsX();
        for (const auto& [id, pos] : dxl_pos) {
            // トルクをオンに (毎回送る必要はないが，すでにONの場合はスキップされるので問題ない)
            cmd.status.id_list.push_back(id);
            cmd.status.torque.push_back(true);
            // 電流を300mAに制限しつつ， +-45degで往復運動させる．
            auto target = (pos < 0) ? 45 : -45;
            auto& cmd_ctrl = cmd.current_base_position_control; // 長いので参照を用いて省略
            cmd_ctrl.id_list.push_back(id);
            cmd_ctrl.current_ma.push_back(300/*mA*/);       // 目標電流，この値を超えないように制御される
            cmd_ctrl.position_deg.push_back(target/*deg*/); // 目標角度
        }
        if (!cmd.status.id_list.empty()) pub_cmd->publish(cmd);
    });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```
</details>

#### 動作指令の publish 部分について

`DxlCommandsX`型のメッセージ利用するためのヘッダファイルをインクルード．
```cpp
#include "dynamixel_handler_msgs/msg/dxl_commands_x.hpp"
using namespace dynamixel_handler_msgs::msg; // 長くなるので名前空間を省略すると便利
```

`dynaimxel/command/x` topic を publish するための publisher を作成．
```cpp
auto pub_cmd = node->create_publisher<DxlCommandsX>("dynamixel/commands/x", 10);
```

`DxlCommandsX`型のメッセージを作成し，以下を指令するmsgをpublishする．
 - `cmd.status` を用いてトルクをオン
 - `cmd.current_base_position_control` 用いて
     - 電流を300mAに制限
     - +-45degで往復運動させる．
```cpp
auto cmd = DxlCommandsX(); // 空のメッセージを作成
for (const auto& [id, pos] : dxl_pos) {
    // トルクをオンに (毎回送る必要はないが，すでにONの場合はスキップされるので問題ない)
    cmd.status.id_list.push_back(id);
    cmd.status.torque.push_back(true); // true でトルクオン, false でトルクオフ
    // 電流を300mAに制限しつつ， +-45degで往復運動させる．
    auto target = (pos < 0) ? 45 : -45;
    auto& cmd_ctrl = cmd.current_base_position_control; // 長いので参照を用いて省略
    cmd_ctrl.id_list.push_back(id);
    cmd_ctrl.current_ma.push_back(300/*mA*/);       // 目標電流，この値を超えないように制御される
    cmd_ctrl.position_deg.push_back(target/*deg*/); // 目標角度
}
if (!cmd.status.id_list.empty()) pub_cmd->publish(cmd);
```

`auto& cmd_ctrl = cmd.current_base_position_control;`を`auto& cmd_ctrl = cmd.position_control;`に変更すると，`cmd_ctrl.current_ma.push_back(300/*mA*/);`の行でコンパイルエラーが発生する．   
すなわち，各制御モードごとにどの目標値が有効なのか暗記しなくても，コンパイラが教えてくれる．

#### 現在情報の subscribe 部分について

`DxlStates`型のメッセージを利用するためのヘッダファイルをインクルード．
```cpp
#include "dynamixel_handler_msgs/msg/dxl_states.hpp"
using namespace dynamixel_handler_msgs::msg; // 長くなるので名前空間を省略すると便利
```

`dynamixel/states` topic を subscribe するための subscriber を作成．  
今回は簡略化のためにCallback関数をラムダ式で記述しているが，通常は関数を定義してそれを渡す．
```cpp
auto sub_st = node->create_subscription<DxlStates>("dynamixel/states", 10, 
    [&](const DxlStates::SharedPtr msg){ // ラムダ式によるCallback関数
        //... 省略 ...
});
```

Callback関数内で，トルクのオンオフ，エラーの有無，pingの成否，制御モードを表示．
```cpp
auto sub_st = node->create_subscription<DxlStates>("dynamixel/states", 10, 
  [&](const DxlStates::SharedPtr msg){ // ラムダ式によるCallback関数
      // トルクのオンオフ，エラーの有無，pingの成否，制御モードなどの情報を表示．
      for (size_t i = 0; i < msg->status.id_list.size(); i++) {
          RCLCPP_INFO(node->get_logger(), 
              "- servo [%d], torque %s, has %s, ping is %s, mode is %s", 
              msg->status.id_list[i],
              msg->status.torque[i] ? "on" : "off", // cpp で boolen の表示は面倒なので文字列に変換
              msg->status.error[i] ? "error" : "no error", // 同上
              msg->status.ping[i] ? "response" : "no response", // 同上
              msg->status.mode[i].c_str() // std::string は c_str() で char* に変換
          ); 
      }
      //... 省略 ...
});
```

現在値の保存するための変数を用意し，Callback関数内で保存．Timerで定期的に現在値を表示．
```cpp
rclcpp::Time updated_time; // より厳密な制御のために，データがreadされた時刻を利用できる
std::map<uint8_t, double> dxl_pos, dxl_vel, dxl_cur; // id とそれぞれの値を保存する map を用意すると便利
auto sub_st = node->create_subscription<DxlStates>("dynamixel/states", 10, 
    [&](const DxlStates::SharedPtr msg){
        //... 省略 ...
        // データがreadされた時刻の保存
        if(!msg->present.id_list.empty()) updated_time = msg->stamp;
        // 位置，速度，電流の現在値の保存
        for (size_t i = 0; i < msg->present.id_list.size(); i++) {
            auto id = msg->present.id_list[i];
            dxl_pos[id] = msg->present.position_deg[i];
            dxl_vel[id] = msg->present.velocity_deg_s[i];
            dxl_cur[id] = msg->present.current_ma[i];
        } // 一度 map に保存することで，サーボのIDでアクセスできるようになるので便利
});
//... 省略 ...
auto timer = node->create_wall_timer(1.0s, [&](){ // 1.0sごとに実行される．
    // データがreadされた時刻を表示
    RCLCPP_INFO(node->get_logger(), "* Present value updated time %f", updated_time.seconds());
    // 各サーボの現在値を表示
    for (const auto& [id, _] : dxl_pos) {
        RCLCPP_INFO(node->get_logger(), //　ID をキーにして保存した値を利用
        "* servo [%d], pos %f, vel %f, cur %f", id, dxl_pos[id], dxl_vel[id], dxl_cur[id]);
    }
    // ... 省略 ...
});
```