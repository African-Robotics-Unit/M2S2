![logo](docs/resources/ARU_logo_rectangle.png)

# M2S2 - Multi Modal Sensor SUite 

## Description 
This repo contains [ROS2](https://docs.ros.org/en/foxy/index.html) drivers for various sensors to collect raw timestamped RGB Camera, Radar, Thermal, Event, LiDAR, Audio, IMU, Temperature, Humidity and Pressure measurements. 


<hr/>

## Hardware 
- RGB Camera: [Ximea -- MQ022CG-CM](https://www.ximea.com/products/usb3-vision-cameras-xiq-line/mq022cg-cm) (Driver: [ximea_ROS2_driver](src/ximea_ROS2_driver))
- Radar: [Texas Instruments DCA1000 Raw ADC capture board](https://www.ti.com/tool/DCA1000EVM) (Driver: [radar_ROS2_driver](src/radar_ROS2_driver))
- Thermal: [FLIR Boson 640](https://www.flir.eu/products/boson/) (Driver: [flir_boson_ROS2_driver](src/flir_boson_ROS2_driver))
- Event: [Dynamic Vision iniVation DVXplorer](https://inivation.com/solution/dvp/) (Driver: [dvxplorer_ROS2_driver](src/dvxplorer_ROS2_driver))
- LiDAR: [Livox Avia](https://www.livoxtech.com/avia) (Driver: [livox_lidar_ROS2_driver](src/livox_lidar_ROS2_driver))
- Audio: [Wildtronics Pro Mono Parabolic Microphone](https://www.wildtronics.com/parabolic.html#.Y3zIiNLP1H4) (Driver: [audio_ROS2_driver](src/audio_ROS2_driver))
- Temperature, Humidity & Pressure: [BOSCH BME280 Temperature Humidity Pressure sensor](https://www.bosch-sensortec.com/products/environmental-sensors/humidity-sensors-bme280/)(Driver: [bme280_ROS2_driver](src/bme280_ROS2_driver))

<hr/> 

## Prerequisites
- [ROS2 foxy](https://docs.ros.org/en/foxy/Installation.html)

- [JetPack v5](https://developer.nvidia.com/embedded/jetpack). We are currently using Jetpack v5.0.2.

- For dependencies of specific drivers, please refer to the corresponding README file. 

<hr/>

## Setup

### Jetson AGX Orin Developer Kit
Our system was set up using a [Jetson AGX Orin Developer Kit](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/)

### ROS2 Foxy
For ROS2 installation, please refer to the official ROS2 installation guide: [ROS2-foxy installation guide](https://docs.ros.org/en/foxy/Installation.html)

### RMW-ecal
Using [eCAL RWM](https://github.com/eclipse-ecal/rmw_ecal) as an alternative to ROS2 DDS middleware implementations showed significant perfomance improvements.

Install [eCAL](https://eclipse-ecal.github.io/ecal/getting_started/setup.html)
```bash
sudo add-apt-repository ppa:ecal/ecal-latest
sudo apt-get update 
sudo apt-get install ecal
```

Clone the latest release of the [ecal-rmw](https://github.com/eclipse-ecal/rmw_ecal) repository into your ROS2 workspace and build it. 

```bash
cd ~/ros2_ws
git clone https://github.com/eclipse-ecal/rmw_ecal.git
cd ~/ros2_ws/rmw_ecal
colcon build --packages-skip rmw_ecal_proto_cpp
vim ~/.bashrc   # add "source ~/ros2_ws/rmw_ecal/install/setup.bash"
source ~/.bashrc
```

Add the following line into your `.bashrc` to run all nodes using eCAL middleware
```bash
export RMW_IMPLEMENTATION=rmw_ecal_dynamic_cpp
```

<hr/>

## Build 



<hr/>

## Usage


<hr/>

## Deserialise Data 
Using the eCAL RMW, one can record data using their powerful[recording](https://eclipse-ecal.github.io/ecal/getting_started/recorder.html) tool. All data is stored as an ecal_measurement in .hdf5 files. 

Our [m2s2_ecal_deserializers](m2s2_ecal_deserializers) provides deserializers to convert raw M2S2 measurements into readable data.

NB: if you are using rosbag2 checkout our [ros2bag deserialisers](https://github.com/African-Robotics-Unit/ros2bag_file_parser).


