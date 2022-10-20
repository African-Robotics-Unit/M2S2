#include "ximea_ros2_cam/ximea_ros2_cam.hpp"

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::executors::StaticSingleThreadedExecutor executor;

  auto cam_node = std::make_shared<ximea_ros2_cam::XimeaROSCam>();

  executor.add_node(cam_node);
  executor.spin();
  
  rclcpp::shutdown();

  return 0;
}
