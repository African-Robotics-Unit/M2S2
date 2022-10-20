#include <chrono>
#include <inttypes.h>
#include <memory>
#include <iostream>
#include "rclcpp/time.hpp"
#include "rclcpp/rclcpp.hpp"

//include all messages for syncing
#include "flir_lepton_msgs/msg/temperature_raw.hpp"
//#include "m2s2_custom_interfaces/msg/thermal_raw.hpp" //new message in m2s2_custom interfaces for thermal stuff (use when new data is collected)
#include "sensor_msgs/msg/image.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/cache.h>
#include <message_filters/time_synchronizer.h>
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/synchronizer.h"


class CameraSynchroniserNode : public rclcpp::Node 
{
public:
    CameraSynchroniserNode() : Node("camera_sync_node") 
    {
    RCLCPP_INFO(this->get_logger(), "Camera Sync node started");

    rclcpp::QoS qos(10);
    auto rmw_qos_profile = qos.get_rmw_qos_profile();


    // Define quality of service: all messages that you want to receive must have the same
    //rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
    //custom_qos_profile.depth = 1;
    //custom_qos_profile.reliability = rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    //custom_qos_profile.history = rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    //custom_qos_profile.durability = rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
   
    
    // Initialise subscribers to the topics of interest
    thermal_raw_sub_.subscribe(this, "thermal_raw", rmw_qos_profile);
    image_resized_sub_.subscribe(this, "image_resized_raw", rmw_qos_profile);

    // Individual subscribers
    thermal_raw_sub_.registerCallback(&CameraSynchroniserNode::thermal_callback, this);
    image_resized_sub_.registerCallback(&CameraSynchroniserNode::image_resized_callback, this);
    
    // Create an approximate time filter
    syncApproximate.reset(new ApproxSync(approximate_policy(10), thermal_raw_sub_, image_resized_sub_));  

    // Register the approximate time callback.
    syncApproximate->registerCallback(&CameraSynchroniserNode::approximate_sync_callback, this);
    syncApproximate->setMaxIntervalDuration(rclcpp::Duration(5,0)); 
    

    // Publish synced image messages  
    this->publisher_image_raw_synced_   = this->create_publisher<sensor_msgs::msg::Image>("synced_image", qos);
    this->publisher_thermal_raw_synced_ = this->create_publisher<flir_lepton_msgs::msg::TemperatureRaw>("synced_thermal", qos);

    }
 
private:
    
    
    void thermal_callback(const flir_lepton_msgs::msg::TemperatureRaw::SharedPtr &temp_raw_msg)
    {
        RCLCPP_INFO(this->get_logger(), "I heard the following THERMAL message: %u. %u", temp_raw_msg->header.stamp.sec, temp_raw_msg->header.stamp.nanosec);

    }

    void image_resized_callback(const sensor_msgs::msg::Image::SharedPtr &image_resized_msg)
    {
        RCLCPP_INFO(this->get_logger(), "I heard the following IMAGE message: %u. %u", image_resized_msg->header.stamp.sec, image_resized_msg->header.stamp.nanosec);

        // Cache stuff...    
        //cache_.add(image_resized_msg);
    }

    void approximate_sync_callback(const flir_lepton_msgs::msg::TemperatureRaw temp_raw_msg, const sensor_msgs::msg::Image image_resized_msg)
    {   

        RCLCPP_INFO(this->get_logger(), "Received APPROXIMATE msgs");
        RCLCPP_INFO(this->get_logger(), "I heard and synchronized the following timestamps: %u.%u, %u.%u", temp_raw_msg.header.stamp.sec, temp_raw_msg.header.stamp.nanosec, image_resized_msg.header.stamp.sec, image_resized_msg.header.stamp.nanosec);
        
        // Publish synced image messages
        this->publisher_image_raw_synced_->publish(image_resized_msg);
        this->publisher_thermal_raw_synced_->publish(temp_raw_msg);

        //cache_.getLatestTime();

    }

    // Subscribers to individual topics
    message_filters::Subscriber<flir_lepton_msgs::msg::TemperatureRaw> thermal_raw_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> image_resized_sub_;

    // Publishers for synced image messages
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image_raw_synced_;
    rclcpp::Publisher<flir_lepton_msgs::msg::TemperatureRaw>::SharedPtr publisher_thermal_raw_synced_;

    // Message filter stuff
    typedef message_filters::sync_policies::ApproximateTime<flir_lepton_msgs::msg::TemperatureRaw, sensor_msgs::msg::Image> approximate_policy;
    typedef message_filters::Synchronizer<approximate_policy> ApproxSync;
    std::shared_ptr<ApproxSync> syncApproximate;

    // Cache stuff
    //message_filters::Cache<sensor_msgs::msg::Image> cache_;

};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraSynchroniserNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

