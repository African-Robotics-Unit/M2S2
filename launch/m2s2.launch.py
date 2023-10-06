
import os
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():

    cam_context_path_arg = DeclareLaunchArgument(
        'cam_context',
        default_value=''
    )

    ld = LaunchDescription([
        cam_context_path_arg
    ])

    # PACKAGES: include launch files from other packages

    # Realsense Cam
    realsense_capture_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch/rs_launch.py')
        )
    )

    # Ximea Cam capture 
    ximea_capture_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ximea_ros2_cam'),
                'launch/xiCam.launch.xml'))
    )

    # FLIR Boson Cam Capture
    boson_capture_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
		get_package_share_directory('flir_boson_ros2'),
	       'launch/flir_boson.launch.xml'))
    )


    # Radar Capture
    radar_capture_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
		get_package_share_directory('radar_bringup'),
	       'radar_proc_launch.py'))
    )
    

    # Livox Avia Lidar Capture
    lidar_capture_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
		get_package_share_directory('livox_ros2_driver'),
	       'launch/livox_lidar_launch.py'))
    )

    # DvXplorer Capture
    event_capture_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
		get_package_share_directory('dvxplorer_ros2'),
	       'launch/dvxplorer_capture_mono.launch.xml'))
    )


    # Audio Capture 
    audio_capture_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('audio_capture'),
                'launch/capture.launch.xml'))
    )


    # Add Nodes

    ld.add_action(realsense_capture_launch)
    ld.add_action(audio_capture_launch)
    ld.add_action(event_capture_launch)
    ld.add_action(lidar_capture_launch)
    ld.add_action(ximea_capture_launch)
    ld.add_action(boson_capture_launch)
    ld.add_action(radar_capture_launch)


    return ld
