
import os
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():

    # ARGS
    bag_name = LaunchConfiguration("bag")
    image_topic = LaunchConfiguration("image_topic")
    boson_thermal_topic = LaunchConfiguration("boson_thermal_topic")
    lepton_thermal_topic = LaunchConfiguration("lepton_thermal_topic")
    radar_topic = LaunchConfiguration("radar_topic")
    lidar_topic = LaunchConfiguration("lidar_topic")
    event_topic = LaunchConfiguration("event_topic")
    audio_topic = LaunchConfiguration("audio_topic")
    enviro_topic = LaunchConfiguration("enviro_topic")
    
    bag_name_arg = DeclareLaunchArgument(
        'bag_name',
        default_value='bags/test'
    )
    
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='ximea/image_raw'
    )

    boson_thermal_topic_arg = DeclareLaunchArgument(
        'boson_thermal_topic',
        default_value='boson/thermal_raw'
    )

    lepton_thermal_topic_arg = DeclareLaunchArgument(
        'lepton_thermal_topic',
        default_value='lepton/thermal_raw'
    )

    radar_topic_arg = DeclareLaunchArgument(
        'radar_topic',
        default_value='/radar_data'
    )

    lidar_topic_arg = DeclareLaunchArgument(
        'lidar_topic',
        default_value='livox/lidar'
    )    

    event_topic_arg = DeclareLaunchArgument(
        'event_topic',
        default_value='dv/events'
    )    

    audio_topic_args = DeclareLaunchArgument(
        'audio_topic',
        default_value='audio/audio'
    )

    enviro_topic_arg = DeclareLaunchArgument(
        'enviro_topic',
        default_value='/enviro_data'
    )

    cam_context_arg = DeclareLaunchArgument(
        'cam_context',
        default_value='/home/orin/m2s2_ws/Experiments/20230124/cam_context/test.bin'
    )

    ld = LaunchDescription([
        bag_name_arg,
        image_topic_arg,
        boson_thermal_topic_arg,
        lepton_thermal_topic_arg,
        radar_topic_arg,
        lidar_topic_arg,
        event_topic_arg,
        audio_topic_args,
        enviro_topic_arg,
        cam_context_arg
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

    # FLIR Lepton Cam Capture
    lepton_capture_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
		get_package_share_directory('flir_lepton_ros2'),
	       'flir_lepton.launch.xml'))
    )
    
    # Radar Capture
    radar_capture_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
		get_package_share_directory('radar_bringup'),
	       'launch/radar_proc_launch.py'))
    )
    

    # Livox Avia Lidar Capture
    lidar_capture_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
		get_package_share_directory('livox_ros2_driver'),
	       'launch/livox_lidar_rviz_launch.py'))
    )

    # DvXplorer Capture
    event_capture_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
		get_package_share_directory('dvxplorer_ros2'),
	       'launch/dvxplorer_capture_mono.launch.xml'))
    )

    # DvXplorer Renderer
    event_render_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
		get_package_share_directory('dvs_renderer'),
	       'launch/dvxplorer_mono.launch.xml'))
    )

    #Audio Capture 
    audio_capture_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('audio_capture'),
                'launch/capture.launch.xml'))
    )

    #Audio Play
    audio_play_launch = IncludeLaunchDescription(
         AnyLaunchDescriptionSource(
             os.path.join(
                 get_package_share_directory('audio_play'),
                 'launch/play.launch.xml'))
    )

    #Enviro capture 
    enviro_capture_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('bme280_ros2'),
                'launch/bme280.launch.xml'))
    )

    #Display 
    rqt_gui = Node(
            package='rqt_gui',
            executable='rqt_gui',
            output='screen',
            #arguments=['--display-config', rviz_config_path]
    )
    

    # Record lidar and event data to ros2 bag 
    #record_all_topics_action = ExecuteProcess(
    #        cmd=[
    #            'ros2',
    #            'bag',
    #            'record',
    #            event_topic,
    #            lidar_topic, 
    #            '-o', bag_name],
    #        output='screen'
    #)

    # Add Nodes
    ld.add_action(boson_capture_launch)
    ld.add_action(lepton_capture_launch)
    ld.add_action(lidar_capture_launch)
    ld.add_action(event_capture_launch)
    ld.add_action(event_render_launch)
    #ld.add_action(audio_capture_launch)
    #ld.add_action(enviro_capture_launch)
    #ld.add_action(ximea_capture_launch)
    ld.add_action(realsense_capture_launch)
    #ld.add_action(audio_play_launch)
    ld.add_action(rqt_gui)



    return ld
