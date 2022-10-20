#include "ros2bag_file_parser/ros2bag_file_parser.hpp"

#include <iostream>
#include <fstream>

using namespace cv;
namespace ros2bag_file_parser {

// *************************
// Ros2bagFileParser Class

//Constructor
Ros2BagFIleParser::Ros2BagFIleParser() :
    Node("ros2bag_file_parser_node")
{   
    this->set_default_parameters();
    
    this->storage_options.uri = this->rosbag_directory;
    this->storage_options.storage_id = "sqlite3";
    this->converter_options.output_serialization_format = "cdr";

    this->parse();
}

// Destructor, leave as defualt for now
Ros2BagFIleParser::~Ros2BagFIleParser() = default;

// FUCKTIONS
template <typename T> T Ros2BagFIleParser::deserialize_message(auto bag_message){

    try{
        if (bag_message->topic_name == this->topic) {
            T extracted_test_msg;
            rclcpp::Serialization<T> serialization;
            rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
            serialization.deserialize_message(
                        &extracted_serialized_msg, &extracted_test_msg);

            return extracted_test_msg;
        } else {
            throw (this->topic);
        }
    } catch (std::string _my_topic){
        RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid Topic : " << _my_topic);
    }
} // func deserialize_message

void Ros2BagFIleParser::parse(){
    rosbag2_cpp::Reader reader(std::make_unique<rosbag2_cpp::readers::SequentialReader>());
    reader.open(this->storage_options, this->converter_options);
    
    while(reader.has_next()){
        auto bag_message = reader.read_next();
        if (bag_message->topic_name == this->topic) {
            this->deserialize_message_by_topic(bag_message, this->topic);
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "DONE PROCCESSING MESSAGE");
    }

}

void Ros2BagFIleParser::deserialize_message_by_topic(auto bag_message, std::string topic){

    FileParserUtil file_parser_util(this->output_directory);
    
    if (topic == "/image_resized_raw"){
        sensor_msgs::msg::Image deserialized_message = 
            this->deserialize_message<sensor_msgs::msg::Image>(bag_message);
        file_parser_util.save(deserialized_message);
    }

    if (topic == "/thermal_raw"){
        flir_lepton_msgs::msg::TemperatureRaw deserialized_message = 
            this->deserialize_message<flir_lepton_msgs::msg::TemperatureRaw>(bag_message);
        file_parser_util.save(deserialized_message);
    }

}

void Ros2BagFIleParser::set_default_parameters(){

    // Topic
    this->declare_parameter("topic", std::string("/image_raw"));
    this->get_parameter("topic", this->topic);
    // bag_directory
    this->declare_parameter("rosbag_directory", std::string("/home/aru/m2s2_ws/20220909/2_ebony"));
    this->get_parameter("rosbag_directory", this->rosbag_directory);
    // output directory
    this->declare_parameter("output_directory", std::string("/home/aru/m2s2_ws/20220909/TEST"));
    this->get_parameter("output_directory", this->output_directory);
    // Analytics file
    this->declare_parameter("analyse", false);
    this->get_parameter("analyse", this->analyse);

}

// **************************************
// Utility class

// Constructor
FileParserUtil::FileParserUtil(std::string _od) :
    output_directory(_od) 
{}

// Leave as default for now
FileParserUtil::~FileParserUtil() = default;

// FUCKTIONS

// RGB IMAGE PROCESSING
void FileParserUtil::save(sensor_msgs::msg::Image image_){ 
        //write as cv image for saving purposes
        std::string image_encoding = "bgr8"; // should declare as a parameter
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(image_, image_encoding);

        std::string secs = std::to_string(image_.header.stamp.sec);
        std::string nsecs = std::to_string(image_.header.stamp.nanosec);
        std::string frame_id = image_.header.frame_id;
        std::string timestr = secs + "_" + nsecs;
        std::string img_name = (this->output_directory + "/" + timestr + ".jpeg");
        imwrite(img_name, cv_ptr->image);

        std::ofstream file_out;

        file_out.open("/home/aru/m2s2_ws/stats/out.txt", std::ios_base::app);
        if (file_out.is_open())
            file_out << frame_id << "," << timestr << std::endl;
        file_out.close();
}

void FileParserUtil::save(flir_lepton_msgs::msg::TemperatureRaw thermal_image){
    //THERMAL IMAGE PROCESSING 
        //1. convert data to cv matrix to store as 16 bit .png image
        cv::Mat thermal_16 = cv::Mat(thermal_image.height, thermal_image.width, CV_16U, &thermal_image.data[0]);
        
        imwrite(
            this->output_directory +
             "/thermal_16/" + 
             std::to_string(thermal_image.header.stamp.sec) + 
             std::to_string(thermal_image.header.stamp.nanosec) + 
             ".png"
            , thermal_16
        );

        //2. Normalise 16_bit data to store as grayscale 8 bit image
        cv::Mat thermal_8;
        cv::Mat gray_image;
        
        normalize(thermal_16, thermal_8, 0, 65535, cv::NORM_MINMAX);
        thermal_8.convertTo(thermal_8, CV_8U, 1./256.);
        cvtColor(thermal_8, gray_image, cv::COLOR_GRAY2RGB);

        imwrite(
            this->output_directory +
             "/thermal_8/" + 
             std::to_string(thermal_image.header.stamp.sec) + 
             std::to_string(thermal_image.header.stamp.nanosec) + 
             ".png"
            , gray_image
        );
        
}

} // namespace ros2bag_file_parser