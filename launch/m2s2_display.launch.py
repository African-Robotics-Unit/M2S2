
import os
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()


    # DvXplorer Renderer
    event_render_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
		get_package_share_directory('dvs_renderer'),
	       'launch/dvxplorer_mono.launch.xml'))
    )

    # Audio Play
    audio_play_launch = IncludeLaunchDescription(
         AnyLaunchDescriptionSource(
             os.path.join(
                 get_package_share_directory('audio_play'),
                 'launch/play.launch.xml'))
    )
    
    rqt_gui = Node(
            package='rqt_gui',
            executable='rqt_gui',
            output='screen',
            #arguments=['--display-config', rviz_config_path]
    )
    
    livox_rviz = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['--display-config', '/home/uctseaice/ws_livox/src/livox_ros2_driver/config/livox_lidar.rviz']
        )

    # Add Nodes
    ld.add_action(event_render_launch)
    ld.add_action(audio_play_launch)
    ld.add_action(rqt_gui)
    ld.add_action(livox_rviz)



    return ld
