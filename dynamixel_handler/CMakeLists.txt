cmake_minimum_required(VERSION 3.8)
project(dynamixel_handler)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dynamixel_handler_msgs REQUIRED)


# インクルードディレクトリの設定
include_directories(
  include/mylib_dynamixel/src
)

# =============================== dynamixel_unify_baudrate =========================
add_executable(
  dynamixel_unify_baudrate 
	include/mylib_dynamixel/download/port_handler_linux_fix.cpp
	include/mylib_dynamixel/download/port_handler_fix.cpp
	include/mylib_dynamixel/src/dynamixel_communicator_main.cpp
	include/mylib_dynamixel/src/dynamixel_communicator_retry.cpp
  
	src/sub_node/dynamixel_unify_baudrate.cpp
)
target_include_directories(
  dynamixel_unify_baudrate PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )
target_compile_features(
  dynamixel_unify_baudrate 
  PUBLIC c_std_99 cxx_std_17
)

ament_target_dependencies(
	dynamixel_unify_baudrate
  rclcpp
  std_msgs
  dynamixel_handler_msgs
)

install(
  TARGETS dynamixel_unify_baudrate
  DESTINATION lib/${PROJECT_NAME}
  )


# ============================ dynamixel_handler ================================

# 実行可能ファイルの名前を変更
add_executable(
  ${PROJECT_NAME} 
	include/mylib_dynamixel/download/port_handler_linux_fix.cpp
	include/mylib_dynamixel/download/port_handler_fix.cpp
	include/mylib_dynamixel/src/dynamixel_communicator_main.cpp
	include/mylib_dynamixel/src/dynamixel_communicator_retry.cpp
  
	src/dynamixel_handler_ros_interface_sub.cpp
	src/dynamixel_handler_ros_interface_pub.cpp
	src/dynamixel_handler_dyn_interface_loop.cpp
	src/dynamixel_handler_dyn_interface_once.cpp

  src/optional_function/external_port.cpp

	src/main.cpp
)
target_include_directories(
  ${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )
target_compile_features(
  ${PROJECT_NAME} 
  PUBLIC c_std_99 cxx_std_17
)

ament_target_dependencies(
	${PROJECT_NAME}
  rclcpp
  std_msgs
  dynamixel_handler_msgs
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# ==================================================================================


install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
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
