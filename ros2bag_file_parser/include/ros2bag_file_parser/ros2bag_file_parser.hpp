#ifndef _ROS2BAG_FILE_PARSER_
#define _ROS2BAG_FILE_PARSER_

#include <memory>
#include <string>
#include <iostream>
#include <armadillo>


#include "rclcpp/rclcpp.hpp"

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

#include "rcpputils/filesystem_helper.hpp"
#include "rcutils/time.h"

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/typesupport_helpers.hpp"
#include "rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp"
#include "rosbag2_cpp/types/introspection_message.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "flir_lepton_msgs/msg/temperature_raw.hpp"

// OpenCV Includes
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include "cv_bridge/cv_bridge.h"
#include <opencv2/core.hpp>

namespace ros2bag_file_parser {

class Ros2BagFIleParser : public rclcpp::Node, std::enable_shared_from_this<Ros2BagFIleParser>
{
public:
    Ros2BagFIleParser();

    ~Ros2BagFIleParser();

private:

    // Topic specific parameters
    std::string rosbag_directory;
    std::string output_directory;
    std::string topic;

    bool analyse;
    
    // Rosbag storage options that never really change
    rosbag2_cpp::StorageOptions storage_options;
    rosbag2_cpp::ConverterOptions converter_options;

    void set_default_parameters();

    template <typename T> T deserialize_message(auto bag_message);

    void deserialize_message_by_topic(auto bag_message, std::string topic);

    void parse();
}; // class ros2bag_file_parser


class FileParserUtil 
{
public:
    FileParserUtil(std::string _od);

    ~FileParserUtil();

    void * getEmptyMessage(std::string message_type, void * msg_);

    void save(sensor_msgs::msg::Image image_);

    void save(flir_lepton_msgs::msg::TemperatureRaw thermal_image);
private:
    std::string output_directory;
}; // class file_parser_util

} // namespace ros2bag_file_parser

#endif