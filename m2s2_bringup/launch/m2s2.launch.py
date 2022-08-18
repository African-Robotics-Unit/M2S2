
import os
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    ld = LaunchDescription()
    # ARGS

    # PACKAGES: include launch files from other packages

    # Audio Capture To File 
    # audio_capture_to_file_launch = IncludeLaunchDescription(
    #     AnyLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('audio_capture'),
    #             'launch/capture_to_file.launch.xml'))
    # )

    #Audio Capture to ROS Bag
    audio_capture_to_bag_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('audio_capture'),
                'launch/capture.launch.xml'))
    )

    #Enviro capture to ROS Bag
    enviro_capture_to_bag_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('m2s2_bringup'),
                'launch/enviro_sensor.launch.xml'))
    )

    # Ximea Cam capture to ROS Bag
    ximea_capture_to_bag_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ximea_ros2_cam'),
                'launch/xiCam.launch.xml'))
    )

    # ld.add_action(audio_capture_to_file_launch)
    ld.add_action(audio_capture_to_bag_launch)
    ld.add_action(enviro_capture_to_bag_launch)
    ld.add_action(ximea_capture_to_bag_launch)
    return ld 