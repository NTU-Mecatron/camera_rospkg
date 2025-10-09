from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('camera_rospkg')
    calib = os.path.join(pkg, 'config', 'calibration.yaml')

    return LaunchDescription([
        Node(
            package='camera_rospkg',
            executable='camera_publisher_node',
            name='camera_publisher',
            output='screen',
            parameters=[{
                'device': '/dev/video0',
                'width': 1280,
                'height': 720,
                'fps': 30.0,
                'frame_id': 'camera_optical_frame',
                'rectify': True,
                'camera_name': 'camera',
                'calibration_url': ''
            }],
        )
    ])
