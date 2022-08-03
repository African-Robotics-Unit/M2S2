
import os
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    
    # ARGS

    # PACKAGES: include launch files from other packages

    # Audio Capture
    audio_capture_launch_include = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('audio_capture'),
                'launch/capture_to_file.launch.xml'))
    )

    # NODES: include lightweight nodes that do not need separate launch files



    return LaunchDescription([audio_capture_launch_include])