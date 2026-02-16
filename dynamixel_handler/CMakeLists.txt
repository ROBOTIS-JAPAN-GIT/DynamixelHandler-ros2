cmake_minimum_required(VERSION 3.8)
project(dynamixel_handler)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dynamixel_handler_msgs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 CONFIG REQUIRED)

# =============================== dynamixel_communicator =========================
add_library(
  dynamixel_communicator STATIC
  src/internal/mylib_dynamixel/download/port_handler_linux_fix.cpp
  src/internal/mylib_dynamixel/download/port_handler_fix.cpp
  src/internal/mylib_dynamixel/src/dynamixel_parameters.cpp
  src/internal/mylib_dynamixel/src/dynamixel_communicator_main.cpp
  src/internal/mylib_dynamixel/src/dynamixel_communicator_retry.cpp
)
target_include_directories(
  dynamixel_communicator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/internal/mylib_dynamixel/src>
)
target_compile_features(
  dynamixel_communicator PUBLIC cxx_std_17
)
set_target_properties(dynamixel_communicator PROPERTIES POSITION_INDEPENDENT_CODE ON)

# =============================== dynamixel_unify_baudrate =========================
add_library(_mylib_dynamixel MODULE src/internal/mylib_dynamixel/bindings/python/dynamixel_communicator_pybind.cpp)
set_target_properties(_mylib_dynamixel PROPERTIES PREFIX "" OUTPUT_NAME "_mylib_dynamixel")
target_link_libraries(
  _mylib_dynamixel
  PRIVATE
    dynamixel_communicator
    pybind11::module
    Python3::Python
)

install(
  TARGETS _mylib_dynamixel
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS src/sub_node/dynamixel_unify_baudrate.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dynamixel_unify_baudrate
)


# ============================ dynamixel_handler ================================

# 実行可能ファイルの名前を変更
add_executable(
  ${PROJECT_NAME} 
	src/dynamixel_handler_ros_interface_sub.cpp
	src/dynamixel_handler_ros_interface_pub.cpp
	src/dynamixel_handler_ros_setup_for_program.cpp
	src/dynamixel_handler_ros_setup_for_cli.cpp
	src/dynamixel_handler_dyn_interface_loop.cpp
	src/dynamixel_handler_dyn_interface_once.cpp
	src/dynamixel_handler.cpp

  src/optional_function/external_port.cpp
  src/optional_function/imu_opencr.cpp

	src/main.cpp
)
target_include_directories(
  ${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/internal
)
target_compile_features(
  ${PROJECT_NAME} 
  PUBLIC cxx_std_17
)

ament_target_dependencies(
		${PROJECT_NAME}
  rclcpp
  std_msgs
  sensor_msgs
  dynamixel_handler_msgs
)
target_link_libraries(
  ${PROJECT_NAME}
  dynamixel_communicator
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

ament_package()
