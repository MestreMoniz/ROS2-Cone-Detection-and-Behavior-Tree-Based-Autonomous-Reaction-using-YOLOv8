#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__DETECT_OBJECT_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__DETECT_OBJECT_CONDITION_HPP_

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace nav2_behavior_tree
{

class DetectObjectCondition : public BT::ConditionNode
{
public:
  DetectObjectCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  DetectObjectCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "DetectObject_topic", std::string("/object_detection"), "Object detection topic")
    };
  }

private:
  void objectDetectionCallback(std_msgs::msg::Bool::SharedPtr msg);

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr object_detection_sub_;
  std::string object_detection_topic_;
  bool object_detected_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__DETECT_OBJECT_CONDITION_HPP_
