#include <chrono>
#include <inttypes.h>
#include <memory>
#include <iostream>
#include <string>
#include "rclcpp/time.hpp"
#include "rclcpp/rclcpp.hpp"

//include all messages for syncing
#include "flir_lepton_msgs/msg/temperature_raw.hpp" //#include "m2s2_custom_interfaces/msg/thermal_raw.hpp" //new message in m2s2_custom interfaces for thermal stuff (use when new data is collected)
#include "sensor_msgs/msg/image.hpp"
#include "m2s2_custom_interfaces/msg/enviro_data.hpp"
#include "audio_common_msgs/msg/audio_data.hpp"

//radar
//lidar
//pinoir

#include "m2s2_custom_interfaces/msg/time_sync_id.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/cache.h>
#include <message_filters/time_synchronizer.h>
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/synchronizer.h"


class SynchroniserNode : public rclcpp::Node 
{
public:
    SynchroniserNode() : Node("sync_node") {
    RCLCPP_INFO(this->get_logger(), "Sync node started");
    this->count = 0;


    rclcpp::QoS qos(10);
    auto rmw_qos_profile = qos.get_rmw_qos_profile();


    // Define quality of service: all messages that you want to receive must have the same
    //rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
    //custom_qos_profile.depth = 1;
    //custom_qos_profile.reliability = rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    //custom_qos_profile.history = rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    //custom_qos_profile.durability = rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
   
    
    // Initialise subscribers to the topics of interest
    audio_data_sub_.subscribe(this, "audio/audio", rmw_qos_profile);
    thermal_raw_sub_.subscribe(this, "thermal_raw", rmw_qos_profile);
    image_resized_sub_.subscribe(this, "image_resized_raw", rmw_qos_profile);
    enviro_data_sub_.subscribe(this, "enviro_data", rmw_qos_profile);


    // Individual subscribers
    audio_data_sub_.registerCallback(&SynchroniserNode::audio_callback, this);
    thermal_raw_sub_.registerCallback(&SynchroniserNode::thermal_callback, this);
    image_resized_sub_.registerCallback(&SynchroniserNode::image_resized_callback, this);
    enviro_data_sub_.registerCallback(&SynchroniserNode::enviro_callback, this);

    
    // Create an approximate time filter
    syncApproximate.reset(new ApproxSync(approximate_policy(10), audio_data_sub_, thermal_raw_sub_, image_resized_sub_, enviro_data_sub_));  

    // Register the approximate time callback.
    syncApproximate->registerCallback(&SynchroniserNode::approximate_sync_callback, this);
    syncApproximate->setMaxIntervalDuration(rclcpp::Duration(0,5)); 
    

    // Publisher for TimeSyncId message 
    this->synced_msgs_pub_= this->create_publisher<m2s2_custom_interfaces::msg::TimeSyncId>("synced_ids", qos);

    }
 
private:
    
    
    void thermal_callback(const flir_lepton_msgs::msg::TemperatureRaw::SharedPtr &thermal_raw_msg)
    {
        RCLCPP_INFO(this->get_logger(), "I heard the following THERMAL message: %u. %u", thermal_raw_msg->header.stamp.sec, thermal_raw_msg->header.stamp.nanosec);

    }

    void image_resized_callback(const sensor_msgs::msg::Image::SharedPtr &image_resized_msg)
    {
        RCLCPP_INFO(this->get_logger(), "I heard the following IMAGE message: %u. %u", image_resized_msg->header.stamp.sec, image_resized_msg->header.stamp.nanosec);

    }

    void audio_callback(const audio_common_msgs::msg::AudioData::SharedPtr &audio_msg)
    {
        RCLCPP_INFO(this->get_logger(), "I heard the following AUDIO message: %u. %u", audio_msg->header.stamp.sec, audio_msg->header.stamp.nanosec);

    }

    void enviro_callback(const m2s2_custom_interfaces::msg::EnviroData::SharedPtr &enviro_msg)
    {
        RCLCPP_INFO(this->get_logger(), "I heard the following ENVIRO message: %u. %u", enviro_msg->header.stamp.sec, enviro_msg->header.stamp.nanosec);

    }

    void approximate_sync_callback(const audio_common_msgs::msg::AudioData audio_msg, const flir_lepton_msgs::msg::TemperatureRaw thermal_raw_msg, const sensor_msgs::msg::Image image_resized_msg, const m2s2_custom_interfaces::msg::EnviroData enviro_msg)
    {   

        RCLCPP_INFO(this->get_logger(), "Received APPROXIMATE msgs");
        RCLCPP_INFO(this->get_logger(), "I heard and synchronized the following timestamps: %u.%u, %u.%u, %u.%u, %U.%u", audio_msg.header.stamp.sec, audio_msg.header.stamp.nanosec, thermal_raw_msg.header.stamp.sec, thermal_raw_msg.header.stamp.nanosec, image_resized_msg.header.stamp.sec, image_resized_msg.header.stamp.nanosec, enviro_msg.header.stamp.sec, enviro_msg.header.stamp.nanosec);
        
        // Populate TimeSyncId message 
        this->thermal_id = std::stoi(thermal_raw_msg.header.frame_id);
        this->image_id = std::stoi(image_resized_msg.header.frame_id);
        this->audio_id = std::stoi(audio_msg.header.frame_id);
        this->enviro_id = std::stoi(enviro_msg.header.frame_id);
        //this->lidar_id = 
        //this->radar_id = 
        //this->pi_noir_id = 

        this->frame_id = std::to_string(this->count);

    
        this->time_synced_ids.thermal_id = this->thermal_id;
        this->time_synced_ids.image_id = this->image_id;
        this->time_synced_ids.audio_id = this->audio_id;
        this->time_synced_ids.enviro_id = this->enviro_id;
        //this->time_synced_ids.lidar_id = this->lidar_id;
        //this->time_synced_ids.radar_id = this->radar_id;
        //this->time_synced_ids.pi_noir_id = this->pi_noir_id;

        this->time_synced_ids.header.frame_id = this->frame_id;
        this->time_synced_ids.header.stamp = audio_msg.header.stamp;
        
        // Publish TimeSyncId message
        this->synced_msgs_pub_->publish(this->time_synced_ids);

        this->count++;


    }

    // Subscribers to individual topics
    message_filters::Subscriber<flir_lepton_msgs::msg::TemperatureRaw> thermal_raw_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> image_resized_sub_;
    message_filters::Subscriber<audio_common_msgs::msg::AudioData> audio_data_sub_;
    message_filters::Subscriber<m2s2_custom_interfaces::msg::EnviroData> enviro_data_sub_;

    // Publisher for synced id message
    rclcpp::Publisher<m2s2_custom_interfaces::msg::TimeSyncId>::SharedPtr synced_msgs_pub_;
    m2s2_custom_interfaces::msg::TimeSyncId time_synced_ids;   
    int thermal_id;
    int image_id;
    int audio_id;
    int enviro_id;   
    //int lidar_id;
    //int radar_id;
    //int pi_noir_id;
    std::string frame_id;
    int count;

    // Message filter stuff
    typedef message_filters::sync_policies::ApproximateTime<audio_common_msgs::msg::AudioData, flir_lepton_msgs::msg::TemperatureRaw, sensor_msgs::msg::Image, m2s2_custom_interfaces::msg::EnviroData> approximate_policy;
    typedef message_filters::Synchronizer<approximate_policy> ApproxSync;
    std::shared_ptr<ApproxSync> syncApproximate;

};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SynchroniserNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

