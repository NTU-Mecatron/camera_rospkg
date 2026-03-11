from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("camera_rospkg")
    default_calibration = os.path.join(pkg_share, "config", "calibration.yaml")

    namespace = LaunchConfiguration("namespace")
    device = LaunchConfiguration("device")
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    fps = LaunchConfiguration("fps")
    frame_id = LaunchConfiguration("frame_id")
    rectify = LaunchConfiguration("rectify")
    calibration_url = LaunchConfiguration("calibration_url")

    camera_parameters = [{
        "device": device,
        "frame_id": frame_id,
        "width": ParameterValue(width, value_type=int),
        "height": ParameterValue(height, value_type=int),
        "fps": ParameterValue(fps, value_type=float),
        "rectify": ParameterValue(rectify, value_type=bool),
        "calibration_url": calibration_url,
    }]

    container = ComposableNodeContainer(
        name="camera_container",
        namespace=namespace,
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="camera_rospkg",
                plugin="camera_rospkg::CameraPublisher",
                name="camera_publisher",
                parameters=camera_parameters,
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace",
            default_value="camera",
            description="Namespace for the camera container and camera topics.",
        ),
        DeclareLaunchArgument(
            "device",
            default_value="/dev/video0",
            description="Camera device path or numeric index.",
        ),
        DeclareLaunchArgument(
            "width",
            default_value="640",
            description="Requested capture width in pixels.",
        ),
        DeclareLaunchArgument(
            "height",
            default_value="480",
            description="Requested capture height in pixels.",
        ),
        DeclareLaunchArgument(
            "fps",
            default_value="30.0",
            description="Requested capture frame rate.",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="camera_optical_frame",
            description="Frame ID stamped into all published messages.",
        ),
        DeclareLaunchArgument(
            "rectify",
            default_value="true",
            choices=["true", "false"],
            description="Enable image rectification when calibration is available.",
        ),
        DeclareLaunchArgument(
            "calibration_url",
            default_value=default_calibration,
            description="Calibration YAML file path.",
        ),
        container,
    ])
