#ifndef XIMEA_ROS_CAM_XIMEAROSCAM_HPP
#define XIMEA_ROS_CAM_XIMEAROSCAM_HPP

// ROS2 Stuff
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/fill_image.hpp"

// Camera Stuff
#include "camera_info_manager/camera_info_manager.hpp"
#include "image_transport/image_transport.hpp"

// Ximea API
#include <m3api/xiApi.h>

// Other includes
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include <map>
#include <yaml-cpp/yaml.h>
#include <sys/stat.h>
#include <memory>

// OpenCV Includes
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include "cv_bridge/cv_bridge.h"
#include <opencv2/core.hpp>

namespace ximea_ros2_cam {

class XimeaROSCam : public rclcpp::Node, std::enable_shared_from_this<XimeaROSCam> {
 public:
    // Node Constructor
    XimeaROSCam();

    // Node Destructor
    ~XimeaROSCam();

 private:
   /**
    * @brief Initialization event for the ROS Nodelet
    *
    * Initialization event for the ROS Nodelet. It contains all of the
    * init_ functions to definparameters, subscriptions,
    * publishers, etc...
    *
    */
   // virtual void onInit();

   /**
    * @brief  Initialize all node handles.
    *
    * Initialize all node handles. Includes public and private nodes.
    */
   void initNodeHandles();

   /**
    * @brief  Initialize diagnostics.
    *
    * Initialize diagnostics and its parameters.
    */
   void initDiagnostics();

   /**
    * @brief  Initialize all publishers.
    *
    * Initialize all publishers.
    */
   void initPubs();

   /**
    * @brief  Initialize all timers.
    *
    * Initialize all timers.
    */
   void initTimers();

   // Camera params and Functions

   void initCam();
   void openCam();

   static std::map<std::string, int> ImgFormatMap;
   static std::map<std::string, int> BytesPerPixelMap;
   static std::map<std::string, std::string> ImgEncodingMap;
   static std::map<int, int> CamMaxPixelWidth;
   static std::map<int, int> CamMaxPixelHeight;

   // Capture the camera(s) frames
   std::string cam_config_file_;    // camera config file

   // Camera variable list
   // Inactive variables
   rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr cam_img_counter_pub_;     // Image counter
   uint32_t img_count_;                     // Image count
   bool cam_info_loaded_;                    // is camera info loaded?
   std::shared_ptr<camera_info_manager::CameraInfoManager>
                     cam_info_manager_;   // Cam info manager handle
   rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_pub_;             // Cam info publisher handle
   rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_xi_image_info_pub_; // xiGetImage info publisher handle
   //image_transport::ImageTransport cam_it_; // Image transport handle
   image_transport::CameraPublisher cam_pub_;     // Image publisher handle
   
   //stuff for resized image 
   image_transport::CameraPublisher image_resized_pub_;                                                //resized image publisher handle
   cv::Mat img_mat, img_resized;
   std_msgs::msg::Header resized_header;

   //stuff for compressed image
   rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_compressed_pub_;

   
   // compressed image params
   std::string cam_compressed_format_;      // "png" (lossless) or "jpeg"
   int cam_compressed_jpeg_quality_;        // 1-100 (1 = min quality)
   int cam_compressed_png_level_;           // 1-9 (9 = max compression)
   // camera
   std::string cam_name_;                   // Main topic name for cam
   std::string cam_format_;                 // Camera image format
   int cam_format_int_;                     // Camera image format int val
   std::string cam_encoding_;               // Camera image encoding
   int cam_bytesperpixel_;                  // Camera image bytes per pixel
   std::string cam_serialno_;               // Camera serial no
   std::string cam_frameid_;
   float poll_time_;			     // For launching cameras in succession
   float poll_time_frame_;                  // For each image buffer check
   int cam_model_;
   std::string cam_calib_file_;
   int cam_trigger_mode_;
   int cam_hw_trigger_edge_;
   bool cam_autoexposure_;
   int cam_exposure_time_;
   float cam_manualgain_;
   int cam_autotime_limit_;
   float cam_autoexposure_priority_;
   float cam_autogain_limit_;
   bool cam_binning_en_;
   int cam_downsample_factor_;
   int cam_roi_left_;
   int cam_roi_top_;
   int cam_roi_width_;
   int cam_roi_height_;
   bool cam_framerate_control_;   // framerate control - enable or disable
   int cam_framerate_set_;      // framerate control - setting fps
   int cam_img_cap_timeout_;       // max time to wait for img
   // white balance mode: 0 - none, 1 - use coeffs, 2 = auto
   int cam_white_balance_mode_;
   float cam_white_balance_coef_r_; // white balance coefficient (rgb)
   float cam_white_balance_coef_g_;
   float cam_white_balance_coef_b_;

   // Diagnostics
   //bool enable_diagnostics;
   //diagnostic_updater::Updater diag_updater;
   //std::shared_ptr<diagnostic_updater::TopicDiagnostic> cam_pub_diag;
   double pub_frequency_tolerance;
   double pub_frequency;
   double frequency_min;
   double frequency_max;
   double age_min;
   double age_max;

   // Bandwidth Limiting
   int cam_num_in_bus_;            // # cameras in a single bus
   float cam_bw_safetyratio_;        // ratio used based on a camera avail bw

   // Active variables
   bool is_active_;                // camera actively acquiring images?
   HANDLE xi_h_;                   // camera xiAPI handle
   float min_fps_;                 // camera calculated min fps
   float max_fps_;                 // camera calculated max fps

   // Focal Parameters
   int lens_mode_;
   float lens_aperture_value_;
   int lens_aperture_index_;
   int lens_focus_movement_value_;
   int lens_focus_move_;
   int lens_focal_length_;

   // Output Messages
   bool publish_xi_image_info_;     // publish xiGetImage handle?

   // Callback function for Camera Frame
   rclcpp::TimerBase::SharedPtr xi_open_device_cb_;
   void openDeviceCb();

   rclcpp::TimerBase::SharedPtr t_frame_cb_;
   void frameCaptureCb();

};  // class XimeaROSCam

}  // namespace ximea_ros_cam

#endif  // XIMEA_ROS_CAM_XIMEAROSCAM_HPP
