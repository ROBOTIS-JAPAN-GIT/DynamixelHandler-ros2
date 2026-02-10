cmake_minimum_required(VERSION 3.8)
project(dynamixel_handler)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to testing OFF unless explicitly provided by the caller.
if(NOT DEFINED BUILD_TESTING)
  set(BUILD_TESTING OFF CACHE BOOL "Build tests and linters")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dynamixel_handler_msgs REQUIRED)

# Optional utility executable. ON by default for convenience.
# Set this to OFF when prioritizing shorter local build time.
option(BUILD_DYNAMIXEL_UNIFY_BAUDRATE "Build dynamixel_unify_baudrate executable" ON)
option(ENABLE_DYNAMIXEL_PCH "Enable precompiled header for dynamixel_handler target" ON)

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
  dynamixel_communicator PUBLIC c_std_99 cxx_std_17
)
if(ENABLE_DYNAMIXEL_PCH AND CMAKE_VERSION VERSION_GREATER_EQUAL "3.16")
  target_precompile_headers(
    dynamixel_communicator
    PRIVATE
      "$<$<COMPILE_LANGUAGE:CXX>:${CMAKE_CURRENT_SOURCE_DIR}/src/internal/mylib_dynamixel/src/dynamixel_communicator.h>"
  )
endif()

# =============================== dynamixel_unify_baudrate =========================
if(BUILD_DYNAMIXEL_UNIFY_BAUDRATE)
  add_executable(
    dynamixel_unify_baudrate
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
  target_link_libraries(
    dynamixel_unify_baudrate
    dynamixel_communicator
  )

  install(
    TARGETS dynamixel_unify_baudrate
    DESTINATION lib/${PROJECT_NAME}
  )
endif()


# ============================ dynamixel_handler ================================

# 実行可能ファイルの名前を変更
add_executable(
  ${PROJECT_NAME} 
	src/dynamixel_handler_ros_interface_sub.cpp
	src/dynamixel_handler_ros_interface_pub.cpp
	src/dynamixel_handler_ros_setup_by_program.cpp
	src/dynamixel_handler_ros_setup_by_cli.cpp
	src/dynamixel_handler_dyn_interface_loop.cpp
	src/dynamixel_handler_dyn_interface_once.cpp
	src/dynamixel_handler.cpp

  src/optional_function/external_port.cpp

	src/main.cpp
)
target_include_directories(
  ${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/internal
)
target_compile_features(
  ${PROJECT_NAME} 
  PUBLIC c_std_99 cxx_std_17
)
if(ENABLE_DYNAMIXEL_PCH AND CMAKE_VERSION VERSION_GREATER_EQUAL "3.16")
  target_precompile_headers(
    ${PROJECT_NAME}
    PRIVATE
      "$<$<COMPILE_LANGUAGE:CXX>:${CMAKE_CURRENT_SOURCE_DIR}/src/dynamixel_handler.hpp>"
  )
endif()

ament_target_dependencies(
		${PROJECT_NAME}
  rclcpp
  std_msgs
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
