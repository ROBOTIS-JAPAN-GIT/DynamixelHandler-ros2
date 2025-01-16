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