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

#include "sensor_msgs/msg/image.hpp"
#include "flir_lepton_msgs/msg/temperature_raw.hpp"

// OpenCV Includes
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include "cv_bridge/cv_bridge.h"
#include <opencv2/core.hpp>

using namespace cv;

int main(int argc, char** argv)
{
  (void) argc;
  (void) argv;
  
  using MsgType = flir_lepton_msgs::msg::TemperatureRaw;                  //TODO: take in as command option at runtime 
  std::string rosbag_directory = "/home/aru/m2s2_ws/20220909/2_ebony";    //TODO: take in as command option at runtime 
  std::string topic = "/thermal_raw";                                     //TODO: take in as command option at runtime
  std::string output_directory = "/home/aru/m2s2_ws/20220909/TEST";       //TODO: take in as command option at runtime 
  //std::string image_encoding = "bgr8";                                  //TODO: take in as command option at runtime     options:

  int frame_count = 1;


  rosbag2_cpp::StorageOptions storage_options;
  rosbag2_cpp::ConverterOptions converter_options;

  storage_options.uri = rosbag_directory;
  storage_options.storage_id = "sqlite3";

  converter_options.output_serialization_format = "cdr";


  {
    rosbag2_cpp::Reader reader(std::make_unique<rosbag2_cpp::readers::SequentialReader>());
    reader.open(storage_options, converter_options);
    while (reader.has_next()) {
      auto bag_message = reader.read_next();

      //std::cout<<"Found topic name " << bag_message->topic_name << std::endl;

      if (bag_message->topic_name == topic) {

        MsgType extracted_test_msg;
        rclcpp::Serialization<MsgType> serialization;
        rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
        serialization.deserialize_message(
                                          &extracted_serialized_msg, &extracted_test_msg);

        std::cout<<"Found data in topic " << bag_message->topic_name << ": " << extracted_test_msg.header.stamp.sec << std::endl;
        std::cout<<"Frame No " << frame_count << std::endl;

        /*
        // RGB IMAGE PROCESSING 
        //write as cv image for saving purposes
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(extracted_test_msg, image_encoding);

        std::string secs = std::to_string(extracted_test_msg.header.stamp.sec);
        std::string nsecs = std::to_string(extracted_test_msg.header.stamp.nanosec);
        std::string timestr = secs + nsecs;
        std::string img_name = (output_directory + "/" + timestr + ".jpeg");
        imwrite(img_name, cv_ptr->image);
        */

        //THERMAL IMAGE PROCESSING 
        //1. convert data to cv matrix to store as 16 bit .png image
        cv::Mat thermal_16 = cv::Mat(extracted_test_msg.height, extracted_test_msg.width, CV_16U, &extracted_test_msg.data[0]);
        
        std::string secs = std::to_string(extracted_test_msg.header.stamp.sec);
        std::string nsecs = std::to_string(extracted_test_msg.header.stamp.nanosec);
        std::string timestr = secs + nsecs;
        std::string img_name = (output_directory + "/" + timestr + ".png");
        imwrite(img_name, thermal_16);

        /*
        //2. Normalise 16_bit data to store as grayscale 8 bit image
        cv::Mat thermal_16 = cv::Mat(extracted_test_msg.height, extracted_test_msg.width, CV_16U, &extracted_test_msg.data[0]);
        cv::Mat thermal_8;
        cv::Mat gray_image;
        
        normalize(thermal_16, thermal_8, 0, 65535, cv::NORM_MINMAX);
        thermal_8.convertTo(thermal_8, CV_8U, 1./256.);
        cvtColor(thermal_8, gray_image, cv::COLOR_GRAY2RGB);


        std::string secs = std::to_string(extracted_test_msg.header.stamp.sec);
        std::string nsecs = std::to_string(extracted_test_msg.header.stamp.nanosec);
        std::string timestr = secs + nsecs;
        std::string img_name = (output_directory + "/" + timestr + ".jpeg");
        imwrite(img_name, gray_image);
        
        */


        frame_count++;


      }

    }
  }
  return 0;
}