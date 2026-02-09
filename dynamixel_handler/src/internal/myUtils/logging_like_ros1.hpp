#ifndef LOGGING_LIKE_ROS1_HP
#define LOGGING_LIKE_ROS1_HP

#define ROS_INFO(...)  RCLCPP_INFO(this->get_logger(), __VA_ARGS__)
#define ROS_WARN(...)  RCLCPP_WARN(this->get_logger(), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(this->get_logger(), __VA_ARGS__)
#define ROS_INFO_STREAM(...)  RCLCPP_INFO_STREAM(this->get_logger(), __VA_ARGS__)
#define ROS_WARN_STREAM(...)  RCLCPP_WARN_STREAM(this->get_logger(), __VA_ARGS__)
#define ROS_ERROR_STREAM(...) RCLCPP_ERROR_STREAM(this->get_logger(), __VA_ARGS__)
#define ROS_INFO_T(...)  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), __VA_ARGS__)
#define ROS_WARN_T(...)  RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), __VA_ARGS__)
#define ROS_ERROR_T(...) RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), __VA_ARGS__)
#define ROS_STOP(...) \
    {{ RCLCPP_FATAL(this->get_logger(), __VA_ARGS__); rclcpp::shutdown(); std::exit(EXIT_SUCCESS); }}
#endif /* LOGGING_LIKE_ROS1_HP */