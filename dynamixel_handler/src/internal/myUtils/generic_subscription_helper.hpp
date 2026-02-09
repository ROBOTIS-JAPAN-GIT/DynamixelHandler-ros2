#ifndef DYNAMIXEL_HANDLER_GENERIC_SUBSCRIPTION_HELPER_HPP
#define DYNAMIXEL_HANDLER_GENERIC_SUBSCRIPTION_HELPER_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

#include <utility>

namespace dynamixel_handler_detail {

template <typename MsgT, typename CallbackT>
rclcpp::SubscriptionBase::SharedPtr CreateGenericSubscriptionWithTypedCallback(
    rclcpp::Node* node,
    const char* topic_name,
    const char* topic_type,
    size_t qos_depth,
    CallbackT&& callback) {
    return node->create_generic_subscription(
        topic_name, topic_type, rclcpp::QoS(qos_depth),
        [callback = std::forward<CallbackT>(callback)](std::shared_ptr<rclcpp::SerializedMessage> serialized_msg) mutable {
            MsgT msg;
            rclcpp::Serialization<MsgT> serializer;
            serializer.deserialize_message(serialized_msg.get(), &msg);
            callback(msg);
        });
}

}  // namespace dynamixel_handler_detail

#endif
