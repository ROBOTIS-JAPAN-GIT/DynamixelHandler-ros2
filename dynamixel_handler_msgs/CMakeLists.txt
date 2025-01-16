cmake_minimum_required(VERSION 3.8)
project(dynamixel_handler_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED) # 自作msgの作成のために必要
find_package(std_msgs REQUIRED) # 自作msgのHeaderのために必要

# 以下の部分は自作msgの定義のために必要
set(msg_files
  "msg/DxlStates.msg"
  "msg/DxlCommandsX.msg"
  "msg/DxlCommandsP.msg"
  "msg/DxlCommandsAll.msg"

  "msg/DynamixelStatus.msg"
  "msg/DynamixelPresent.msg"
  "msg/DynamixelGoal.msg"
  "msg/DynamixelGain.msg"
  "msg/DynamixelLimit.msg"
  "msg/DynamixelError.msg"
  "msg/DynamixelExtra.msg"
  "msg/DynamixelExtraDrivemode.msg"
  "msg/DynamixelExtraShutdown.msg"
  "msg/DynamixelExtraRestureconfig.msg"
  "msg/DynamixelExtraMovingstatus.msg"
  "msg/DynamixelExtraLed.msg"

  "msg/DynamixelDebug.msg"
  "msg/DynamixelDebugElement.msg"
  "msg/DynamixelShortcut.msg"
  
  "msg/DynamixelControlXPwm.msg"
  "msg/DynamixelControlXCurrent.msg"
  "msg/DynamixelControlXVelocity.msg"
  "msg/DynamixelControlXPosition.msg"
  "msg/DynamixelControlXExtendedPosition.msg"
  "msg/DynamixelControlXCurrentBasePosition.msg"
  "msg/DynamixelControlPPwm.msg"
  "msg/DynamixelControlPCurrent.msg"
  "msg/DynamixelControlPVelocity.msg"
  "msg/DynamixelControlPPosition.msg"
  "msg/DynamixelControlPExtendedPosition.msg"

  "msg/DxlExternalPort.msg"
)
set(srv_files

)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

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
