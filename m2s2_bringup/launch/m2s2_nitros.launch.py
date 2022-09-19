import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    rectify_node = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        name='rectify',
        namespace='',
        parameters=[{
            'output_width': 1280,
            'output_height': 1024,
        }]
    )


    convertor_node = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        name='image_format_node',
        namespace='',
        parameters=[{
            'encoding_desired': 'rgb8'
        }]
    )

    ximea_cam_params_path = os.path.join(
        get_package_share_directory('ximea_ros2_cam'),
        'config',
        'xiCam_config.yaml'
    )
    ximea_cam_node = ComposableNode(
        package='ximea_ros2_cam',
        plugin='ximea_ros2_cam::XimeaROSCam',
        name='ximea_ros2_cam_node',
        parameters=[
            {
                'serial_no': '29970251',
                'cam_name': 'ximea_cam',
                'calib_file': '',
                'frame_id': '0',
                'num_cams_in_bus': '2',
                'bw_safetyratio': '1.0',
                'publish_xi_image_info': 'true',
                'poll_time': '2.0',
                'poll_time_frame': '0.0001'
            },
            ximea_cam_params_path
        ]
    )

    container = ComposableNodeContainer(
        name='image_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            rectify_node,
            convertor_node,
            ximea_cam_node
        ]
    )

    some_node = Node(
        package='ximea_ros2_cam',
        namespace='ximea_ros2_cam',
        executable='ximea_ros2_cam_node',
        name='ximea_ros2_cam_node'
    )

    return launch.LaunchDescription([container, some_node])
