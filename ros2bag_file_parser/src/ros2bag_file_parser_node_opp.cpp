#include "ros2bag_file_parser/ros2bag_file_parser.hpp"

int main(int argc, char ** argv) {

    rclcpp::init(argc, argv);
    rclcpp::executors::StaticSingleThreadedExecutor executor;

    auto ros2bag_file_parser_node = std::make_shared<ros2bag_file_parser::Ros2BagFIleParser>();

    executor.add_node(ros2bag_file_parser_node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
} // end of main