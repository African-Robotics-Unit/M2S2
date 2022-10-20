cmake_minimum_required(VERSION 3.5)
project(ros2bag_file_parser)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(flir_lepton_msgs REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#add_executable(ros2bag_file_parser_node src/ros2bag_file_parser_node.cpp src/ros2bag_file_parser_opp.cpp)
add_executable(ros2bag_file_parser_node src/ros2bag_file_parser_node_opp.cpp src/ros2bag_file_parser_opp.cpp)
ament_target_dependencies(ros2bag_file_parser_node
"rclcpp"
"rosbag2_cpp"
"sensor_msgs"
"cv_bridge"
"flir_lepton_msgs"
)

target_include_directories(ros2bag_file_parser_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

install(TARGETS
  ros2bag_file_parser_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
