#include "ximea_ros2_cam/ximea_ros2_cam.hpp"

using namespace cv;
namespace ximea_ros2_cam {

std::map<std::string, int> XimeaROSCam::ImgFormatMap = {
    {"XI_MONO8",      XI_MONO8},
    {"XI_MONO16",     XI_MONO16},
    {"XI_RGB24",      XI_RGB24},
    {"XI_RGB32",      XI_RGB32},
    {"XI_RGB_PLANAR", XI_RGB_PLANAR},
    {"XI_RAW8",       XI_RAW8},
    {"XI_RAW16",      XI_RAW16}
};

std::map<std::string, int> XimeaROSCam::BytesPerPixelMap = {
    {"XI_MONO8",      1},
    {"XI_MONO16",     2},
    {"XI_RGB24",      3},
    {"XI_RGB32",      4},
    {"XI_RGB_PLANAR", 3},
    {"XI_RAW8",       1},
    {"XI_RAW16",      2}
};

std::map<std::string, std::string> XimeaROSCam::ImgEncodingMap = {
    {"XI_MONO8",      "mono8"},
    {"XI_MONO16",     "mono16"},
    {"XI_RGB24",      "bgr8"},
    {"XI_RGB32",      "bgra8"},
    {"XI_RGB_PLANAR", "not_applicable"},
    {"XI_RAW8",       "mono8"},
    {"XI_RAW16",      "mono16"}
};

std::map<int, int> XimeaROSCam::CamMaxPixelWidth = { {0, 2048} };
std::map<int, int> XimeaROSCam::CamMaxPixelHeight = { {0, 1088} };

// Constructor
XimeaROSCam::XimeaROSCam() : 
    Node("ximea_cam_node"),
    img_count_(0),
    cam_info_loaded_(false),
    cam_trigger_mode_(0),
    cam_framerate_control_(false),
    cam_white_balance_mode_(0),
    age_min(0.0),
    is_active_(false),
    cam_model_(0),
    xi_h_(NULL) {
    
    this->initCam();
    this->initPubs();
    this->initTimers();
    this->openDeviceCb();

    RCLCPP_INFO(this->get_logger(), "Node initialized..");
}

XimeaROSCam::~XimeaROSCam() {
    // Init variables
    XI_RETURN xi_stat;

    RCLCPP_INFO(this->get_logger(), "Shutting down ximea_ros_cam node...");
    // Stop acquisition and close device if handle is available
    if (this->xi_h_ != NULL) {


        //Save Camera context
        std::string cam_context_path = "/home/aru/repos/ximea_stuff/ecal_ximea/img/cam_context.bin";
        char* cam_context=NULL;
	    #define SIZE_OF_CONTEXT_BUFFER (10*1024*1024) // 10MiB
	    cam_context = (char*)malloc(SIZE_OF_CONTEXT_BUFFER);
	    xi_stat= xiGetParamString(this->xi_h_, XI_PRM_API_CONTEXT_LIST, cam_context, SIZE_OF_CONTEXT_BUFFER);

        std::ofstream file(cam_context_path, std::ios::out | std::ios::binary);
        file.write(cam_context, SIZE_OF_CONTEXT_BUFFER);
        file.close();

        RCLCPP_INFO_STREAM(this->get_logger(), "Done Writing camera context file");
        std::string s(cam_context);
        RCLCPP_INFO_STREAM(this->get_logger(), "Cam Context: " << s);
        // Stop image acquisition
        this->is_active_ = false;
        xi_stat = xiStopAcquisition(this->xi_h_);
        
        // Close camera device
        xiCloseDevice(this->xi_h_);
        this->xi_h_ = NULL;

        RCLCPP_INFO_STREAM(this->get_logger(), "Closed device: " << this->cam_serialno_);
    }
    RCLCPP_INFO(this->get_logger(), "ximea_ros2_cam node shutdown complete.");

    // To avoid warnings
    (void)xi_stat;
}

void XimeaROSCam::initPubs() {
    // Report start of function
    RCLCPP_INFO(this->get_logger(), "Loading Publishers ... ");

    this->cam_img_counter_pub_ = this->create_publisher<std_msgs::msg::UInt32>(
            "image_count", 10);
            
    // Report end of function
    RCLCPP_INFO(this->get_logger(), "... Publishers Loaded. ");
}

// initTimers() - initialize the timers
void XimeaROSCam::initTimers() {
    // Report start of function
    RCLCPP_INFO(this->get_logger(), "Loading Timers ... ");

    // Load camera polling callback timer ((Ensure that with multiple cameras,
    // each time is about 2 seconds spaced apart)
   /* this->xi_open_device_cb_ =
        this->create_wall_timer(std::chrono::duration<float>(this->poll_time_),
        std::bind(&XimeaROSCam::openDeviceCb, this)); */

    RCLCPP_INFO_STREAM(this->get_logger(), "xi_open_device_cb_: " << this->xi_open_device_cb_);

    // Load camera frame capture callback timer
    this->t_frame_cb_ =
        this->create_wall_timer(std::chrono::duration<float>(this->poll_time_frame_),
        std::bind(&XimeaROSCam::frameCaptureCb, this));

    RCLCPP_INFO_STREAM(this->get_logger(), "t_frame_cb_: " << this->t_frame_cb_);

    // Report end of function
    RCLCPP_INFO(this->get_logger(), "... Timers Loaded.");
}

void XimeaROSCam::initCam() {
    RCLCPP_INFO(this->get_logger(), "Loading Camera Configuration");

    // Assume that all of the config is embedded in the camera private namespace
    // Load all parameters and store them into their corresponding vars

    //      -- apply camera name --
    this->declare_parameter("cam_name", std::string("INVALID"));
    this->get_parameter("cam_name", this->cam_name_);
    RCLCPP_INFO_STREAM(this->get_logger(), "cam_name: " << this->cam_name_);
    
    //      -- apply camera specific parameters --
    this->declare_parameter( "serial_no", std::string("INVALID"));
    this->get_parameter("serial_no", this->cam_serialno_);
    RCLCPP_INFO_STREAM(this->get_logger(), "serial number: " << this->cam_serialno_);
    this->declare_parameter( "frame_id", std::string("'0'"));
    this->get_parameter("frame_id", this->cam_frameid_);
    RCLCPP_INFO_STREAM(this->get_logger(), "frame id: " << this->cam_frameid_);
    this->declare_parameter( "calib_file", std::string("INVALID"));
    this->get_parameter("calib_file", this->cam_calib_file_);
    RCLCPP_INFO_STREAM(this->get_logger(), "calibration file: " << this->cam_calib_file_);
    this->declare_parameter("poll_time", -1.0f);
    this->get_parameter("poll_time", this->poll_time_);
    RCLCPP_INFO_STREAM(this->get_logger(), "poll_time: " << this->poll_time_);
    this->declare_parameter("poll_time_frame", 0.0f);
    this->get_parameter("poll_time_frame", this->poll_time_frame_);
    RCLCPP_INFO_STREAM(this->get_logger(), "poll_time_frame: " << this->poll_time_frame_);

    //      -- apply compressed image parameters (from image_transport) --
    this->declare_parameter( "image_transport_compressed_format", std::string("INVALID"));
    this->get_parameter( "image_transport_compressed_format", this->cam_compressed_format_);
    RCLCPP_INFO_STREAM(this->get_logger(), "image_transport_compressed_format: "
        << this->cam_compressed_format_);
    this->declare_parameter( "image_transport_compressed_jpeg_quality", -1);
    this->get_parameter("image_transport_compressed_jpeg_quality", this->cam_compressed_jpeg_quality_);
    RCLCPP_INFO_STREAM(this->get_logger(), "image_transport_compressed_jpeg_quality: "
        << this->cam_compressed_jpeg_quality_);
    this->declare_parameter("image_transport_compressed_png_level", -1);
    this->get_parameter("image_transport_compressed_png_level", this->cam_compressed_png_level_);
    RCLCPP_INFO_STREAM(this->get_logger(), "image_transport_compressed_png_level: "
        << this->cam_compressed_png_level_);

    //      -- apply image format parameters --
    this->declare_parameter( "format", std::string("INVALID"));
    this->get_parameter("format", this->cam_format_);
    RCLCPP_INFO_STREAM(this->get_logger(), "format: " << this->cam_format_);
    this->cam_format_int_ = ImgFormatMap[this->cam_format_];
    RCLCPP_INFO_STREAM(this->get_logger(), "format_int: " << this->cam_format_int_);
    this->cam_bytesperpixel_ = BytesPerPixelMap[this->cam_format_];
    RCLCPP_INFO_STREAM(this->get_logger(), "cam_bytesperpixel_: " << this->cam_bytesperpixel_);
    this->cam_encoding_ = ImgEncodingMap[this->cam_format_];
    RCLCPP_INFO_STREAM(this->get_logger(), "cam_encoding_: " << this->cam_encoding_);

    //      -- apply bandwidth parameters --
    this->declare_parameter("num_cams_in_bus", -1);
    this->get_parameter("num_cams_in_bus", this->cam_num_in_bus_);
    RCLCPP_INFO_STREAM(this->get_logger(),"cam_num_in_bus_: " << this->cam_num_in_bus_);
    this->declare_parameter("bw_safetyratio", -1.0f);
    this->get_parameter("bw_safetyratio", this->cam_bw_safetyratio_);
    RCLCPP_INFO_STREAM(this->get_logger(),"cam_bw_safetyratio_: " << this->cam_bw_safetyratio_);

    //      -- apply triggering parameters --
    this->declare_parameter("cam_trigger_mode", -1);
    this->get_parameter("cam_trigger_mode", this->cam_trigger_mode_);
    RCLCPP_INFO_STREAM(this->get_logger(), "cam_trigger_mode_: " << this->cam_trigger_mode_);
    this->declare_parameter("hw_trigger_edge", -1);
    this->get_parameter("hw_trigger_edge", this->cam_hw_trigger_edge_);
    RCLCPP_INFO_STREAM(this->get_logger(), "cam_hw_trigger_edge_: " << this->cam_hw_trigger_edge_);

    //      -- apply framerate (software cap) parameters --
    this->declare_parameter( "frame_rate_control", false);
    this->get_parameter("frame_rate_control", this->cam_framerate_control_);
    RCLCPP_INFO_STREAM(this->get_logger(),  "cam_framerate_control_: " << this->cam_framerate_control_);
    this->declare_parameter("frame_rate_set", -1);
    this->get_parameter("frame_rate_set", this->cam_framerate_set_);
    RCLCPP_INFO_STREAM(this->get_logger(),  "cam_framerate_set_: " << this->cam_framerate_set_);
    this->declare_parameter("img_capture_timeout", -1);
    this->get_parameter("img_capture_timeout", this->cam_img_cap_timeout_);
    RCLCPP_INFO_STREAM(this->get_logger(),  "cam_img_cap_timeout_: " << this->cam_img_cap_timeout_);

    //      -- apply exposure parameters --
    this->declare_parameter("auto_exposure", false);
    this->get_parameter("auto_exposure", this->cam_autoexposure_);
    RCLCPP_INFO_STREAM(this->get_logger(),  "cam_autoexposure_: " << this->cam_autoexposure_);
    this->declare_parameter("manual_gain", -1.0f);
    this->get_parameter("manual_gain", this->cam_manualgain_);
    RCLCPP_INFO_STREAM(this->get_logger(),  "cam_manualgain_: " << this->cam_manualgain_);
    this->declare_parameter("exposure_time", -1);
    this->get_parameter("exposure_time", this->cam_exposure_time_);
    RCLCPP_INFO_STREAM(this->get_logger(),  "cam_exposure_time_: " << this->cam_exposure_time_);
    this->declare_parameter( "auto_exposure_priority", -1.0f);
    this->get_parameter("auto_exposure_priority", this->cam_autoexposure_priority_);
    RCLCPP_INFO_STREAM(this->get_logger(),  "cam_autoexposure_priority_: "
        << this->cam_autoexposure_priority_);
    this->declare_parameter("auto_time_limit", -1);
    this->get_parameter("auto_time_limit", this->cam_autotime_limit_);
    RCLCPP_INFO_STREAM(this->get_logger(),  "cam_autotime_limit_: " << this->cam_autotime_limit_);
    this->declare_parameter( "auto_gain_limit", -1.0f);
    this->get_parameter( "auto_gain_limit", this->cam_autogain_limit_);
    RCLCPP_INFO_STREAM(this->get_logger(),  "cam_autogain_limit_: " << this->cam_autogain_limit_);

    //      -- apply white balance parameters --
    this->declare_parameter( "white_balance_mode", -1);
    this->get_parameter( "white_balance_mode", this->cam_white_balance_mode_);
    RCLCPP_INFO_STREAM(this->get_logger(),  "cam_white_balance_mode_: "
        << this->cam_white_balance_mode_);
    this->declare_parameter( "white_balance_coef_red", -1.0f);
    this->get_parameter( "white_balance_coef_red", this->cam_white_balance_coef_r_);
    RCLCPP_INFO_STREAM(this->get_logger(),  "cam_white_balance_coef_r_: "
        << this->cam_white_balance_coef_r_);
    this->declare_parameter("white_balance_coef_green", -1.0f);
    this->get_parameter( "white_balance_coef_green", this->cam_white_balance_coef_g_);
    RCLCPP_INFO_STREAM(this->get_logger(),  "cam_white_balance_coef_g_: "
        << this->cam_white_balance_coef_g_);
    this->declare_parameter("white_balance_coef_blue", -1.0f);
    this->get_parameter( "white_balance_coef_blue", this->cam_white_balance_coef_b_);
    RCLCPP_INFO_STREAM(this->get_logger(),  "cam_white_balance_coef_b_: "
        << this->cam_white_balance_coef_b_);

    //      -- apply ROI parameters --
    this->declare_parameter("roi_left", -1);
    this->get_parameter("roi_left", this->cam_roi_left_);
    RCLCPP_INFO_STREAM(this->get_logger(),  "cam_roi_left_: " << this->cam_roi_left_);
    this->declare_parameter("roi_top", -1);
    this->get_parameter("roi_top", this->cam_roi_top_);
    RCLCPP_INFO_STREAM(this->get_logger(),  "cam_roi_top_: " << this->cam_roi_top_);
    this->declare_parameter("roi_width", -1);
    this->get_parameter("roi_width", this->cam_roi_width_);
    RCLCPP_INFO_STREAM(this->get_logger(),  "cam_roi_width_: " << this->cam_roi_width_);
    this->declare_parameter("roi_height", -1);
    this->get_parameter("roi_height", this->cam_roi_height_);
    RCLCPP_INFO_STREAM(this->get_logger(),  "cam_roi_height_: " << this->cam_roi_height_);

    // Other basic init values
    this->is_active_ = false;
    this->xi_h_ = NULL;

    // Setup image transport (camera publisher) and camera info topics
    rmw_qos_profile_t rmw_qos_profile_ximea_custom = rmw_qos_profile_sensor_data;
    rmw_qos_profile_ximea_custom.depth = 1;

    this->cam_pub_ = image_transport::create_camera_publisher(this, "image_raw"); //, rmw_qos_profile_ximea_custom);
    
    //create publisher for resized image 
    this->image_resized_pub_ = image_transport::create_camera_publisher(this, "image_resized_raw");
    
    //create publisher for compressed image
    //this->image_compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("image_compressed", 10);



    // only load and publish calib file if it isn't empty
    // assume camera info is not loaded
    // Setup camera info manager for calibration
    this->cam_info_loaded_ = false;
    this->cam_info_manager_ =
        std::make_shared<camera_info_manager::CameraInfoManager>(this, this->cam_name_);
    if (this->cam_info_manager_->loadCameraInfo(this->cam_calib_file_)) {
        this->cam_info_loaded_ = true;
    }
    // loaded camera info properly
    if (this->cam_info_loaded_) {
        // advertise
        this->cam_info_pub_ =
            this->create_publisher<sensor_msgs::msg::CameraInfo>(
                "camera_info", 1);
    }

    // Enable auto bandwidth calculation to ensure bandwidth limiting and
    // framerate setting are supported
    xiSetParamInt(0, XI_PRM_AUTO_BANDWIDTH_CALCULATION, XI_ON);
}

void XimeaROSCam::openCam() {
    // Init variables
    XI_RETURN xi_stat;

    // leave if there isn't a valid handle
    if (this->xi_h_ == NULL) { return; }

    // Apply parameters to camera
    //      -- Set image format --
    xi_stat = xiSetParamInt(this->xi_h_,
                            XI_PRM_IMAGE_DATA_FORMAT,
                            this->cam_format_int_);

    // //      -- Set auto white balance if requested --
    // if (this->cam_auto_white_balance_) {
    //     xi_stat = xiSetParamInt(this->xi_h_, XI_PRM_AUTO_WB, 1);
    // } else {
    //     xi_stat = xiSetParamInt(this->xi_h_, XI_PRM_AUTO_WB, 0);
    // }

    //      -- White balance mode --
    // Note: Setting XI_PRM_MANUAL right before or after setting coeffs
    // actually overrides the coefficients! This is because calculating
    // the manual coeffs takes time, so when the coefficients are set,
    // they will be overwritten once the manual coeff values are calculated.
    // This also is the same when XI_PRM_MANUAL is set to 0 as well.
    if (this->cam_white_balance_mode_ == 2) {
        RCLCPP_INFO_STREAM(this->get_logger(),  "WHITE BALANCE MODE SET TO AUTO.");
        xi_stat = xiSetParamInt(this->xi_h_, XI_PRM_AUTO_WB, 1);
    } else if (this->cam_white_balance_mode_ == 1) {
        RCLCPP_INFO_STREAM(this->get_logger(),  "WHITE BALANCE MODE SET TO APPLY COEFFS.");
        xi_stat = xiSetParamInt(this->xi_h_, XI_PRM_AUTO_WB, 0);
        xi_stat = xiSetParamFloat(this->xi_h_, XI_PRM_WB_KR,
                                  this->cam_white_balance_coef_r_);
        xi_stat = xiSetParamFloat(this->xi_h_, XI_PRM_WB_KG,
                                  this->cam_white_balance_coef_g_);
        xi_stat = xiSetParamFloat(this->xi_h_, XI_PRM_WB_KB,
                                  this->cam_white_balance_coef_b_);
    } else if (this->cam_white_balance_mode_ == 0) {
        RCLCPP_INFO_STREAM(this->get_logger(),  "WHITE BALANCE MODE SET TO NONE.");
        xi_stat = xiSetParamInt(this->xi_h_, XI_PRM_AUTO_WB, 0);
    } else {
        // should not be here!
        RCLCPP_INFO_STREAM(this->get_logger(),  "WHITE BALANCE MODE IS NOT 0 TO 2!");
    }
    
    // Camera hardware trigger mode enabled?
    if (this->cam_trigger_mode_ == 2) {
        if (this->cam_hw_trigger_edge_ == 0) {
            // Select trigger to be rising edge
            xi_stat = xiSetParamInt(this->xi_h_,
                                    XI_PRM_TRG_SOURCE, XI_TRG_EDGE_RISING);
        } else if (this->cam_hw_trigger_edge_ == 1) {
            // Select trigger to be falling edge
            xi_stat = xiSetParamInt(this->xi_h_,
                                    XI_PRM_TRG_SOURCE, XI_TRG_EDGE_FALLING);
        } else { // default to rising
            // Select trigger to be rising edge
            xi_stat = xiSetParamInt(this->xi_h_,
                                    XI_PRM_TRG_SOURCE, XI_TRG_EDGE_RISING);

        }

        // Select input pin 1 to be for GP input trigger
        xi_stat = xiSetParamInt(this->xi_h_, XI_PRM_GPI_SELECTOR, 1);
        xi_stat = xiSetParamInt(this->xi_h_, XI_PRM_GPI_MODE, XI_GPI_TRIGGER);
    } else if (this->cam_trigger_mode_ == 1) {
        // Select software triggering
        // NOT FULLY IMPLEMENTED YET
        xi_stat = xiSetParamInt(this->xi_h_, XI_PRM_TRG_SOURCE, XI_TRG_SOFTWARE);
    } else {
        // Disable any triggering
        xi_stat = xiSetParamInt(this->xi_h_, XI_PRM_TRG_SOURCE, XI_TRG_OFF);
    }
    //      -- Set exposure --
    // If auto exposure is set to 0, then set manual exposure, otherwise set
    // auto exposure.
    //      -- Set manual exposure --
    if (!this->cam_autoexposure_) {
        RCLCPP_INFO_STREAM(this->get_logger(),  "Setting manual exposure: EXPOSURE AMOUNT: " <<
                        this->cam_exposure_time_ << " GAIN: " <<
                        this->cam_manualgain_);
        // manual exposure
        xi_stat = xiSetParamInt(this->xi_h_,
                                XI_PRM_AEAG,
                                0);
        xi_stat = xiSetParamInt(this->xi_h_,
                                XI_PRM_EXPOSURE,
                                this->cam_exposure_time_);
        // exposure gain limit
        xi_stat = xiSetParamFloat(this->xi_h_,
                                  XI_PRM_GAIN,
                                  this->cam_manualgain_);
    //      -- Set auto exposure --
    } else {
        RCLCPP_INFO_STREAM(this->get_logger(),  "Setting auto exposure: EXPOSURE TIME LIMIT: " <<
                        this->cam_autotime_limit_ << " GAIN LIMIT: " <<
                        this->cam_autogain_limit_ << " AUTO PRIORITY: " <<
                        this->cam_autoexposure_priority_);
        // auto exposure
        xi_stat = xiSetParamInt(this->xi_h_,
                                XI_PRM_AEAG,
                                1);
        // auto priority
        xi_stat = xiSetParamFloat(this->xi_h_,
                                  XI_PRM_EXP_PRIORITY,
                                  this->cam_autoexposure_priority_);
        // auto exposure time limit
        xi_stat = xiSetParamFloat(this->xi_h_,
                                  XI_PRM_AE_MAX_LIMIT,
                                  this->cam_autotime_limit_);
        // auto exposure gain limit
        xi_stat = xiSetParamFloat(this->xi_h_,
                                  XI_PRM_AG_MAX_LIMIT,
                                  this->cam_autogain_limit_);
    }

    //      -- Set region of interest --
    int max_cam_width = CamMaxPixelWidth[this->cam_model_];
    RCLCPP_INFO_STREAM(this->get_logger(),  "MAX WIDTH: " << max_cam_width);
    int max_cam_height = CamMaxPixelHeight[this->cam_model_];
    RCLCPP_INFO_STREAM(this->get_logger(),  "MAX HEIGHT: " << max_cam_height);

    // Check bounds
    if (this->cam_roi_left_ < 0 || this->cam_roi_left_ > max_cam_width ||
        this->cam_roi_top_ < 0 || this->cam_roi_top_ > max_cam_height ||
        this->cam_roi_width_ < 0 || this->cam_roi_width_ > max_cam_width ||
        this->cam_roi_height_ < 0 || this->cam_roi_height_ > max_cam_height ||
        this->cam_roi_left_ + this->cam_roi_width_ > max_cam_width ||
        this->cam_roi_top_ + this->cam_roi_height_ > max_cam_height) {

        RCLCPP_INFO_STREAM(this->get_logger(), "cam_roi_left_: " << this->cam_roi_left_);
        RCLCPP_INFO_STREAM(this->get_logger(), "cam_roi_top_: " << this->cam_roi_top_);
        RCLCPP_INFO_STREAM(this->get_logger(), "cam_roi_width_: " << this->cam_roi_width_);
        RCLCPP_INFO_STREAM(this->get_logger(), "cam_roi_height_: " << this->cam_roi_height_);

        RCLCPP_INFO_STREAM(this->get_logger(), "cam_roi_left_: " << this->cam_roi_left_);
        RCLCPP_INFO_STREAM(this->get_logger(), "cam_roi_top_: " << this->cam_roi_top_);
        RCLCPP_INFO_STREAM(this->get_logger(), "cam_roi_width_: " << this->cam_roi_width_);
        RCLCPP_INFO_STREAM(this->get_logger(), "cam_roi_height_: " << this->cam_roi_height_);


        // Out of bounds, throw error here
        return;
    }

    // Set ROI
    xi_stat = xiSetParamInt(this->xi_h_, XI_PRM_WIDTH, this->cam_roi_width_);
    xi_stat = xiSetParamInt(this->xi_h_, XI_PRM_HEIGHT, this->cam_roi_height_);
    xi_stat = xiSetParamInt(this->xi_h_, XI_PRM_OFFSET_X, this->cam_roi_left_);
    xi_stat = xiSetParamInt(this->xi_h_, XI_PRM_OFFSET_Y, this->cam_roi_top_);
    // xiGetParamInt(xiH_, XI_PRM_WIDTH XI_PRM_INFO_INCREMENT, &tmp);

    // Compute available bandwidth for this camera
    int avail_bw = 0;           // Mbits per second
    xi_stat = xiGetParamInt(this->xi_h_, XI_PRM_AVAILABLE_BANDWIDTH, &avail_bw);

    // If we have more than one camera per bus/controller, we divide the
    // available bw to accomodate for the same amount of cameras
    if (this->cam_num_in_bus_ > 1) {
        avail_bw = (int) ((double)avail_bw / (double)this->cam_num_in_bus_);
    }

    // Set bandwidth limit for camera and apply a safety ratio
    RCLCPP_INFO_STREAM(this->get_logger(),  "Limiting bandwidth to: " <<
            (int)((float)avail_bw*this->cam_bw_safetyratio_) << " Mbits/sec");
    xi_stat = xiSetParamInt(this->xi_h_,
                            XI_PRM_LIMIT_BANDWIDTH,
                            (int)((float)avail_bw*this->cam_bw_safetyratio_));
    xi_stat = xiSetParamInt(this->xi_h_, XI_PRM_LIMIT_BANDWIDTH_MODE , XI_ON);


    //      -- Framerate control  --
    // For information purposes, obtain min and max calculated possible fps
    xi_stat = xiGetParamFloat(this->xi_h_,
                              XI_PRM_FRAMERATE XI_PRM_INFO_MIN,
                              &this->min_fps_);
    xi_stat = xiGetParamFloat(this->xi_h_,
                              XI_PRM_FRAMERATE XI_PRM_INFO_MAX,
                              &this->max_fps_);

    // If we are not in trigger mode, determine if we want to limit fps
    if (this->cam_trigger_mode_ == 0) {
        if (this->cam_framerate_control_) {
            RCLCPP_INFO_STREAM(this->get_logger(),  "Setting frame rate control to: " <<
                        this->cam_framerate_set_ << " Hz");

            xi_stat = xiSetParamInt(this->xi_h_,
                                    XI_PRM_ACQ_TIMING_MODE,
                                    XI_ACQ_TIMING_MODE_FRAME_RATE);
            // Apply frame rate (we assume MQ camera here)
            xi_stat = xiSetParamInt(this->xi_h_,
                                      XI_PRM_FRAMERATE,
                                      this->cam_framerate_set_);
        } else {
            // default to free run
            xi_stat = xiSetParamInt(this->xi_h_,
                                    XI_PRM_ACQ_TIMING_MODE,
                                    XI_ACQ_TIMING_MODE_FREE_RUN);
        }
    }

    //      -- Optimize transport buffer commit/size based on payload  --
    // // For usb controllers that can handle it...
    // src: https://www.ximea.com/support/wiki/apis/Linux_USB30_Support
    // xiSetParamInt(handle, XI_PRM_ACQ_TRANSPORT_BUFFER_COMMIT, 32);
    // xiGetParamInt( handle, XI_PRM_ACQ_TRANSPORT_BUFFER_SIZE XI_PRM_INFO_MAX,
    //  &buffer_size);
    // xiSetParamInt(handle, XI_PRM_ACQ_TRANSPORT_BUFFER_SIZE, buffer_size);

    // // For high frame rate performance
    // src: https://www.ximea.com/support/wiki/usb3/...
    //      ...How_to_optimize_software_performance_on_high_frame_rates
    
    // set maximum number of queue
    int number_of_field_buffers = 0;
    xiGetParamInt(this->xi_h_, XI_PRM_BUFFERS_QUEUE_SIZE XI_PRM_INFO_MAX, &number_of_field_buffers);
    xiSetParamInt(this->xi_h_, XI_PRM_BUFFERS_QUEUE_SIZE, number_of_field_buffers);
    
    int payload=0;
    xi_stat = xiGetParamInt(this->xi_h_, XI_PRM_IMAGE_PAYLOAD_SIZE, &payload);
    
    // ---------------------------------------------------
//select transport buffer size depending on payload 

   int transport_buffer_size_default = 0;
   int transport_buffer_size_increment = 0;
   int transport_buffer_size_minimum = 0;
    
   xi_stat = xiGetParamInt(this->xi_h_, XI_PRM_ACQ_TRANSPORT_BUFFER_SIZE, &transport_buffer_size_default);
   xi_stat = xiGetParamInt(this->xi_h_, XI_PRM_ACQ_TRANSPORT_BUFFER_SIZE XI_PRM_INFO_INCREMENT, &transport_buffer_size_increment);
   xi_stat = xiGetParamInt(this->xi_h_, XI_PRM_ACQ_TRANSPORT_BUFFER_SIZE XI_PRM_INFO_MIN, &transport_buffer_size_minimum);
   
   if(payload < transport_buffer_size_default + transport_buffer_size_increment){
   	
   	int transport_buffer_size = payload;
   	if (transport_buffer_size_increment){
   		int remainder = transport_buffer_size % transport_buffer_size_increment;
   		if (remainder)
   			transport_buffer_size += transport_buffer_size_increment - remainder;
   	}
   	if (transport_buffer_size < transport_buffer_size_minimum)
   		transport_buffer_size = transport_buffer_size_minimum;
   	xi_stat = xiSetParamInt(this->xi_h_, XI_PRM_ACQ_TRANSPORT_BUFFER_SIZE, transport_buffer_size);
   	
   	
   }
    //      -- Start camera acquisition --
    RCLCPP_INFO(this->get_logger(),  "Starting Acquisition...");
    xi_stat = xiStartAcquisition(this->xi_h_);
    RCLCPP_INFO(this->get_logger(),  "Acquisition started...");

    this->is_active_ = true;                    // set active to be true

    // To avoid warnings
    (void)xi_stat;
}

void XimeaROSCam::openDeviceCb() {
    XI_RETURN xi_stat;

    RCLCPP_INFO_STREAM(this->get_logger(), "Polling Ximea Cam. Serial #: " << this->cam_serialno_);

    xi_stat = xiOpenDeviceBy(XI_OPEN_BY_SN,
            this->cam_serialno_.c_str(),
            &this->xi_h_);

    if (xi_stat == XI_OK && this->xi_h_ != NULL) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Poll successful. Loading serial #: "
                        << this->cam_serialno_);
        this->xi_open_device_cb_.reset();
        XimeaROSCam::openCam();
    }

    // To avoid warnings
    (void)xi_stat;
}

// Start aquiring data
void XimeaROSCam::frameCaptureCb() {

    // Init variables
    XI_RETURN xi_stat;
    XI_IMG xi_img;
    char *img_buffer;
    rclcpp::Time timestamp;
    std::string time_str;

    xi_img.size = sizeof(XI_IMG);
    xi_img.bp = NULL;
    xi_img.bp_size = 0;

    // Acquisition started
    //RCLCPP_INFO(this->get_logger(), "CAPTURING FRAME");
    if (this->is_active_) {
        // Acquire image
        xi_stat = xiGetImage(this->xi_h_,
                             this->cam_img_cap_timeout_,
                             &xi_img);
        // Add timestamp
        timestamp = now();

        // Was the image retrieval successful?
        if (xi_stat == XI_OK) {
        	
    
            /*RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 3,
                "Capturing image from Ximea camera serial no: "
                << this->cam_serialno_
                << ". WxH: "
                << xi_img.width
                << " x "
                << xi_img.height << ".");
                */
        
            // Setup image
            img_buffer = reinterpret_cast<char *>(xi_img.bp);
	
            // Populate ROS message
            sensor_msgs::msg::Image img;
            sensor_msgs::fillImage(img,
                                    this->cam_encoding_,
                                    xi_img.height,
                                    xi_img.width,
                                    xi_img.width * this->cam_bytesperpixel_,
                                    img_buffer);
            img.header.frame_id = std::to_string(this->img_count_++);
            img.header.stamp = timestamp;
            
            
            //Publish image
            sensor_msgs::msg::CameraInfo cam_info =
                    this->cam_info_manager_->getCameraInfo();      
            this->cam_pub_.publish(img, cam_info);
            RCLCPP_INFO(this->get_logger(), "Image published: [%i]", this->img_count_);
            
            
            /*
            // CompressedImage
            this->img_mat = cv::Mat(xi_img.height, xi_img.width, CV_8UC3, img_buffer);
            cv_bridge::CvImage img_bridge = 
                  cv_bridge::CvImage(img.header, this->cam_encoding_, this->img_mat);

            sensor_msgs::msg::CompressedImage img_msg;
            img_bridge.toCompressedImageMsg(img_msg); //dst_format = JPG
            this->image_compressed_pub_->publish(img_msg);
            */
            


            /**
            //RESIZING 
            this->resized_header.stamp = timestamp;

            // add frame count to frame_id
            this->cam_frameid_ = std::to_string(this->img_count_);
            this->resized_header.frame_id = this->cam_frameid_;

            sensor_msgs::msg::CameraInfo cam_resized_info = this->cam_info_manager_->getCameraInfo();
            sensor_msgs::msg::Image img_small; 
            cv_bridge::CvImage cv_img;

            //resize image by converting to cv image 
            this->img_mat = cv::Mat(xi_img.height, xi_img.width, CV_8UC3, img_buffer);
            resize(this->img_mat, this->img_resized, Size(640, 512), cv::INTER_LINEAR);
            cv_img = cv_bridge::CvImage(this->resized_header, this->cam_encoding_, this->img_resized);
            cv_img.toImageMsg(img_small); // from cv_bridge to sensor_msgs::Image
            
            //publish image 
            cam_resized_info.height = 512;
            cam_resized_info.width = 640;
            this->image_resized_pub_.publish(img_small, cam_resized_info);
            RCLCPP_INFO(this->get_logger(), "Small image published");

            this->img_count_++;
            */

            

        }
    
    }

    // To avoid warnings
    (void)xi_stat;
} // XIMEA_CAM_CLASS

} //XIMEA_CAM_NAMESPACE
