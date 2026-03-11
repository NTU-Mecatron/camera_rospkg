from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LifecycleTransition
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from lifecycle_msgs.msg import Transition
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("camera_rospkg")
    default_calibration = os.path.join(pkg_share, "config", "calibration.yaml")

    camera_parameters = [{
        "device": ParameterValue(LaunchConfiguration("device"), value_type=str),
        "frame_id": ParameterValue(LaunchConfiguration("frame_id"), value_type=str),
        "width": ParameterValue(LaunchConfiguration("width"), value_type=int),
        "height": ParameterValue(LaunchConfiguration("height"), value_type=int),
        "fps": ParameterValue(LaunchConfiguration("fps"), value_type=float),
        "rectify": ParameterValue(LaunchConfiguration("rectify"), value_type=bool),
        "calibration_url": ParameterValue(LaunchConfiguration("calibration_url"), value_type=str),
    }]

    container = ComposableNodeContainer(
        name="camera_container",
        namespace=LaunchConfiguration("namespace"),
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

    autostart = TimerAction(
        period=1.0,
        condition=IfCondition(LaunchConfiguration("autostart")),
        actions=[
            LifecycleTransition(
                lifecycle_node_names=[[LaunchConfiguration("namespace"), "/camera_publisher"]],
                transition_ids=[Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE],
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace",
            default_value="camera",
            description="Namespace for the camera container and camera topics.",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            choices=["true", "false"],
            description="Automatically configure and activate the lifecycle node after launch.",
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
        autostart,
    ])
