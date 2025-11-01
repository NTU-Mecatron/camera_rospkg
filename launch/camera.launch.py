from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('camera_rospkg')
    calib = os.path.join(pkg, 'config', 'calibration.yaml')

    # Launch argument for namespace
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='camera_rospkg',
        description='Namespace for the camera node'
    )

    return LaunchDescription([
        namespace_arg,
        Node(
            package='camera_rospkg',
            executable='camera_publisher_node',
            name='camera_publisher',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{
                'device': '/dev/video0',
                'width': 1280,
                'height': 720,
                'fps': 30.0,
                'pixel_format' : 'MJPG',
                'io_method' : 'mmap',
                'frame_id': 'camera_optical_frame',
                'rectify': True,
                'camera_name': 'camera',
                'calibration_url': calib,  # Disable calibration for now - OpenCV format not compatible

                # FFMPEG image transport parameters for foxglove
                'ffmpeg_image_transport.encoder': 'h264_v4l2m2m', # NVIDIA hardware encoder for H264
                'ffmpeg_image_transport.bit_rate': 10000000, # 10 Mbps
                'ffmpeg_image_transport.gop_size': 15 # Keyframe interval
            }],
        )
    ])