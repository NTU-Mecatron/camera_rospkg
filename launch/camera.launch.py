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
                'device': '/dev/video4',
                'width': 1280,
                'height': 720,
                'fps': 30.0,
                'topCrop' : 50,
                'bottomCrop' : 50,
                'frame_id': 'camera_optical_frame',
                'rectify': True,
                'camera_name': 'camera',
                'calibration_url': f'file://{calib}'
            }],
        )
    ])
