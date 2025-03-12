//This node listens to /detected_cone (published by cone_detector_node.cpp) and updates the behavior tree.
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class DetectConeAction : public BT::SyncActionNode
{
public:
    DetectConeAction(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("detect_cone_bt_node");
        subscriber_ = node_->create_subscription<std_msgs::msg::String>(
            "/detected_cone", 10, std::bind(&DetectConeAction::cone_callback, this, std::placeholders::_1));
    }

    static BT::PortsList providedPorts()
    {
        return {BT::OutputPort<std::string>("object_detected")};
    }

    BT::NodeStatus tick() override
    {
        if (last_detected_cone_.empty())
        {
            return BT::NodeStatus::FAILURE; // No cone detected
        }

        setOutput("object_detected", last_detected_cone_);
        return BT::NodeStatus::SUCCESS;
    }

private:
    void cone_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        last_detected_cone_ = msg->data;
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
    std::string last_detected_cone_;
};

// Register the node
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<DetectConeAction>("DetectCone");
}
