#include "ximea_ros2_cam/ximea_ros2_cam_dev.hpp"

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::executors::StaticSingleThreadedExecutor executor;

  auto cam_node = std::make_shared<ximea_ros2_cam::XimeaROSCam>();
  //auto bag_node = std::make_shared<ximea_ros2_cam::BagRecorder>();

  executor.add_node(cam_node);
  //executor.add_node(bag_node);
  executor.spin();
  

  //rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
