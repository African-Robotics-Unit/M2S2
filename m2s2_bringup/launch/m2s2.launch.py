
import os
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():

    # ARGS
    bag_name = LaunchConfiguration("bag_name")
    image_topic = LaunchConfiguration("image_topic")
    image_info_topic = LaunchConfiguration("image_info_topic")
    audio_topic = LaunchConfiguration("audio_topic")
    enviro_topic = LaunchConfiguration("enviro_topic")
    
    bag_name_arg = DeclareLaunchArgument(
        'bag_name',
        default_value='bags/test'
    )
    
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/image_raw'
    )

    image_info_topic_arg = DeclareLaunchArgument(
        'image_info_topic',
        default_value='camera_info'
    )

    audio_topic_args = DeclareLaunchArgument(
        'audio_topic',
        default_value='audio/audio'
    )

    enviro_topic_arg = DeclareLaunchArgument(
        'enviro_topic',
        default_value='enviro_data'
    )

    ld = LaunchDescription([
        bag_name_arg,
        image_topic_arg,
        image_info_topic_arg,
        audio_topic_args,
        enviro_topic_arg
    ])


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

    # Record all topics at once

    record_all_topics_action = ExecuteProcess(
            cmd=[
                'ros2',
                'bag',
                'record',
                image_topic,
                image_info_topic,
                audio_topic,
                enviro_topic , '-o',
                bag_name],
            output='screen'
    )

    # Add Nodes
    ld.add_action(audio_capture_to_bag_launch)
    ld.add_action(enviro_capture_to_bag_launch)
    ld.add_action(ximea_capture_to_bag_launch)
    ld.add_action(record_all_topics_action)
    return ld 