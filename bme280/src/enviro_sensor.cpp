
#include "rclcpp/rclcpp.hpp"
#include "m2s2_custom_interfaces/msg/enviro_data.hpp"
#include <string>

extern "C" {
#include "driver/bme280_linux.h"
}


class EnviroNode : public rclcpp::Node{

public:
	EnviroNode() : Node("enviro_node"){
	
		//frame count to be stored in frame_id of Header  
		this->frame_count = 0;

		this->declare_parameter<std::string>("i2c_bus", "/dev/i2c-1");
		this->get_parameter("i2c_bus", i2c_bus);
		
		// Initiate communication
		int bme_rslt;
		if ( (bme_rslt = static_cast<int>(init_BME280(i2c_bus.c_str(), &id, &dev))) != 0 ){
			RCLCPP_ERROR_STREAM(this->get_logger(), 
			"Failed to Initiate Communication with code " << bme_rslt);
		}
		
		publisher_ = this->create_publisher<m2s2_custom_interfaces::msg::EnviroData>("enviro_data", 10);
		timer_ = this->create_wall_timer(
			std::chrono::milliseconds(100), std::bind(&EnviroNode::bme_timer_callback, this));
	}
		
private:
	// BME PARAMS
	std::string i2c_bus;
	struct bme280_dev dev;		//Stores device settings and params
	struct bme280_data comp_data; // Stores data read from the sensor
	struct identifier id;		//I2C information of the device.
	int frame_count; 

	// ROS Stuff
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<m2s2_custom_interfaces::msg::EnviroData>::SharedPtr publisher_;
	
	void bme_timer_callback(){
		int bme_rslt;
		
		// Custom msg interface for the BME280
		auto data_ = m2s2_custom_interfaces::msg::EnviroData();
		
		// Reading from the BME280 regs, need to delay for debouncing
		dev.delay_us(70, dev.intf_ptr);
		if ( (bme_rslt = static_cast<int>(bme280_get_sensor_data(BME280_ALL, &comp_data, &dev))) != 0 ){
			RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to Read from the Sensor with code" << bme_rslt );
		}
		//print_sensor_data(&comp_data);
		
		RCLCPP_INFO(this->get_logger(), "Publishing Environment Data");
		data_.temperature = comp_data.temperature;
		data_.pressure = 0.01f * comp_data.pressure;
		data_.humidity = comp_data.humidity;

		//add timestamp 
		data_.header.stamp = now();

		data_.header.frame_id = std::to_string(this->frame_count);
		
		publisher_->publish(data_);

		this->frame_count++;
	}
};

int main(int argc, char **argv){
	rclcpp::init(argc, argv);
	auto node = std::make_shared<EnviroNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}

//TO DO:
//create post processing subscriber to sqllite db
