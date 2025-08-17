#include <string>
#include "your_custom_bt_node/plugins/condition/detect_object_condition.hpp"

namespace nav2_behavior_tree
{

DetectObjectCondition::DetectObjectCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  object_detection_topic_("/object_detection"),
  object_detected_(false)
{
  getInput("DetectObject_topic", object_detection_topic_);
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  callback_group_ = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);

  callback_group_executor_.add_callback_group(callback_group_, node->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;

  object_detection_sub_ = node->create_subscription<std_msgs::msg::Bool>(
    object_detection_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&DetectObjectCondition::objectDetectionCallback, this, std::placeholders::_1),
    sub_option);
}

BT::NodeStatus DetectObjectCondition::tick()
{
  callback_group_executor_.spin_some();
  if (object_detected_) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

void DetectObjectCondition::objectDetectionCallback(std_msgs::msg::Bool::SharedPtr msg)
{
  object_detected_ = msg->data;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::DetectObjectCondition>("IsDetectObject");
}
