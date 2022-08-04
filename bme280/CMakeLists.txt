cmake_minimum_required(VERSION 3.5)
project(bme280)

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
find_package(m2s2_custom_interfaces REQUIRED)

add_library(bme280 src/driver/bme280_linux.c src/driver/bme280.c)
target_include_directories(bme280 PUBLIC src/driver/)

add_executable(enviro_node src/enviro_sensor.cpp)
ament_target_dependencies(enviro_node rclcpp m2s2_custom_interfaces)

add_executable(enviro_subscriber_node src/enviro_subscriber.cpp)
ament_target_dependencies(enviro_subscriber_node rclcpp m2s2_custom_interfaces)

target_link_libraries(enviro_node bme280)

install(TARGETS
	enviro_node
  enviro_subscriber_node
	DESTINATION lib/${PROJECT_NAME})

ament_package()
