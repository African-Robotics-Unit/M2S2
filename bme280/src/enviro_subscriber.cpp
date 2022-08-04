#include "rclcpp/rclcpp.hpp"
#include "m2s2_custom_interfaces/msg/enviro_data.hpp"
 
class EnviroSubscriberNode: public rclcpp::Node 
{
public:
    EnviroSubscriberNode() : Node("enviro_subscriber_node")
    {
        subscriber_ = this->create_subscription<m2s2_custom_interfaces::msg::EnviroData>(
            "enviro_data", 10, 
            std::bind(&EnviroSubscriberNode::callbackEnviroSensor, this, std::placeholders::_1)); //args: topicName, qSize, callback function
            RCLCPP_INFO(this->get_logger(), "Enviro subscriber has been started");
    }
 
private:
    void callbackEnviroSensor(const m2s2_custom_interfaces::msg::EnviroData::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "%f", msg->temperature);
        RCLCPP_INFO(this->get_logger(), "%f", msg->humidity);
        RCLCPP_INFO(this->get_logger(), "%f", msg->pressure);
        RCLCPP_INFO(this->get_logger(), "%s", msg->header.stamp);

    }

    // ROS Stuff
    //declare subscription called subscriber_
	rclcpp::Subscription<m2s2_custom_interfaces::msg::EnviroData>::SharedPtr subscriber_;
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EnviroSubscriberNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

//need 2 things: subscriber object and callabck 
// for every callback for a subscriber, you use:  const, name of message ::SharedPtr
// for every callback use std::bind(&NodeName::callbackFuncName, this) if callback function takes in parameters, add third argument: std::placeholders::_1