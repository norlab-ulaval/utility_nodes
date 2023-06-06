#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class WaitingNode : public rclcpp::Node
{
public:
    WaitingNode():
            Node("waiting_node")
    {
        receivedMessage = false;
        subscription = this->create_subscription<std_msgs::msg::String>("topic_in", 1, std::bind(&WaitingNode::callback, this, std::placeholders::_1));
    }

    void callback(const std_msgs::msg::String& msg)
    {
        receivedMessage = true;
    }

    bool hasReceivedMessage() const
    {
        return receivedMessage;
    }

private:
    bool receivedMessage;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<WaitingNode> node = std::make_shared<WaitingNode>();
    while(rclcpp::ok() && !node->hasReceivedMessage())
    {
        rclcpp::spin_some(node);
    }
    rclcpp::shutdown();
    return 0;
}
