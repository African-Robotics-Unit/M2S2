#include "ximea_ros2_cam/ximea_ros2_cam.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ximea_ros2_cam::XimeaROSCam>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}