#include <rclcpp/rclcpp.hpp>

class TopicHzNode : public rclcpp::Node
{
public:
    TopicHzNode():
            Node("topic_hz_node")
    {
        this->declare_parameter<int>("window_size", 10);
        windowSize = this->get_parameter("window_size").as_int();
        if(windowSize <= 1)
        {
            RCLCPP_FATAL(this->get_logger(), "Invalid window_size, exiting...");
            exit(1);
        }

        this->declare_parameter<std::string>("topic_name", "");
        std::string topicName = this->get_parameter("topic_name").as_string();
        if(topicName.empty())
        {
            RCLCPP_FATAL(this->get_logger(), "The topic_name argument was not provided, exiting...");
            exit(1);
        }
        if(topicName[0] != '/')
        {
            topicName = "/" + topicName;
        }

        std::map<std::string, std::vector<std::string>> topicNamesAndTypes;
        bool topicFound = false;
        std::chrono::time_point<std::chrono::steady_clock> startTime = std::chrono::steady_clock::now();
        while(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - startTime).count() < 5 && !topicFound)
        {
            topicNamesAndTypes = this->get_topic_names_and_types();
            topicFound = topicNamesAndTypes.count(topicName) != 0 && !this->get_publishers_info_by_topic(topicName).empty();
        }
        if(!topicFound)
        {
            RCLCPP_FATAL_STREAM(this->get_logger(), "Cannot find requested topic.");
            exit(1);
        }

        std::string topicType = topicNamesAndTypes[topicName][0];
        rclcpp::QoS qos = this->get_publishers_info_by_topic(topicName)[0].qos_profile();
        subscription = this->create_generic_subscription(topicName, topicType, qos.keep_last(1), std::bind(&TopicHzNode::subscriptionCallback, this, std::placeholders::_1));
    }

private:
    void subscriptionCallback(std::shared_ptr<rclcpp::SerializedMessage> msg)
    {
        window.emplace_back(this->get_clock()->now());

        std::chrono::time_point<std::chrono::steady_clock> currentTime = std::chrono::steady_clock::now();
        if(window.size() >= windowSize && std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastStatTime).count() >= 1000)
        {
            double delaySum = 0;
            for(auto it = ++window.begin(); it != window.end(); ++it)
            {
                delaySum += it->seconds() - std::prev(it)->seconds();
            }
            double averageRate = (window.size() - 1) / delaySum;
            window.clear();

            RCLCPP_INFO(this->get_logger(), "Average rate: %.2f Hz", averageRate);
            lastStatTime = currentTime;
        }
    }

    int windowSize;
    std::list<rclcpp::Time> window;
    rclcpp::GenericSubscription::SharedPtr subscription;
    std::chrono::time_point<std::chrono::steady_clock> lastStatTime;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TopicHzNode>());
    rclcpp::shutdown();
    return 0;
}
