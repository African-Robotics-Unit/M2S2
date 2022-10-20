cmake_minimum_required(VERSION 3.5)
project(sync_subscriber)

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
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(m2s2_custom_interfaces REQUIRED)
find_package(message_filters REQUIRED)
find_package(audio_common_msgs REQUIRED)
find_package(flir_lepton_msgs REQUIRED) #remove when new data is collected to use m2s2_custom_interfaces message

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(sync_node src/sync_node.cpp)
ament_target_dependencies(sync_node 
"rclcpp"
"m2s2_custom_interfaces"
"sensor_msgs"
"std_msgs"
"message_filters"
"audio_common_msgs"
"flir_lepton_msgs"
)

add_executable(camera_sync_node src/camera_sync_node.cpp)
ament_target_dependencies(camera_sync_node 
"rclcpp"
"m2s2_custom_interfaces"
"sensor_msgs"
"std_msgs"
"message_filters"
"flir_lepton_msgs"
)

install(TARGETS
  sync_node
  camera_sync_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
