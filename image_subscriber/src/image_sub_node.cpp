#include <chrono>
#include <inttypes.h>
#include <memory>
#include <iostream>
#include "rclcpp/time.hpp"
#include "rclcpp/rclcpp.hpp"

//#include "m2s2_custom_interfaces/msg/thermal_raw.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "image_transport/image_transport.hpp"

// OpenCV Includes
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include "cv_bridge/cv_bridge.h"
#include <opencv2/core.hpp>

using std::placeholders::_1;
using namespace cv;



class ImageSubscriberNode : public rclcpp::Node 
{
public:
    ImageSubscriberNode() : Node("image_sub_node") 
    {

    // Topic
    this->declare_parameter("topic", std::strin g("/image_raw"));
    this->get_parameter("topic", this->topic);
    
    // output directory
    this->declare_parameter("output_directory", std::string("/home/aru/m2s2_ws/20221013/0"));
    this->get_parameter("output_directory", this->output_directory); 


    this->frame_count = 0;
    RCLCPP_INFO(this->get_logger(), "Image subscriber node started");

    rmw_qos_profile_t rmw_qos_profile_ximea_custom = rmw_qos_profile_sensor_data;
    // rmw_qos_profile_ximea_custom.depth = 1;

    rclcpp::QoS qos(rclcpp::KeepLast(100), rmw_qos_profile_ximea_custom);
    auto rmw_qos_profile = qos.get_rmw_qos_profile();
    

    // Initialise subscriber
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(topic, qos, std::bind(&ImageSubscriberNode::image_callback, this, _1));
    }


      
 
private:
   
    void image_callback(const sensor_msgs::msg::Image::SharedPtr image_msg)
    {
        //convert and store frames to .jpeg
        //write as cv image for saving purposes
        std::string image_encoding = "bgr8"; 
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(image_msg, image_encoding);

        std::string secs = std::to_string(image_msg->header.stamp.sec);
        std::string nsecs = std::to_string(image_msg->header.stamp.nanosec);
        std::string frame_id = image_msg->header.frame_id;
        std::string timestr = secs + "_" + nsecs;
        std::string img_name = (this->output_directory + "/" + frame_id + "_" + timestr + ".jpeg");
        imwrite(img_name, cv_ptr->image);

        RCLCPP_INFO(this->get_logger(), "Image saved [%i]", this->frame_count++);
    }

    // Vars
	int frame_count; 
    std::string topic;
    std::string output_directory;

    // Subscribers to individual topics
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageSubscriberNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

