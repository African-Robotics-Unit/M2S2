from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('m2s2_bringup'),
                    'launch/m2s2.launch.py'
                ])
            ]),
            launch_arguments={
                'bag_name': 'bags/test',
                'image_topic': '/image_resized_raw',
                'image_info_topic': '/camera_info',
                'audio_topic': '/audio/audio',
                'enviro_topic': '/enviro_data'
            }.items()
        )
    ])
