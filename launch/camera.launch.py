from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("camera_rospkg")
    default_calibration = os.path.join(pkg_share, "config", "calibration.yaml")
    node_name = ["/", LaunchConfiguration("namespace"), "/camera_publisher"]

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
                namespace=LaunchConfiguration("namespace"),
                parameters=[{
                    "device": LaunchConfiguration("device"),
                    "calibration_url": default_calibration,
                }],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
    )

    autoconfigure = TimerAction(
        period=2.0,
        condition=IfCondition(LaunchConfiguration("autostart")),
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "lifecycle", "set", "--no-daemon", "--spin-time", "10",
                    node_name, "configure"
                ],
                output="screen",
            ),
        ],
    )

    autoactivate = TimerAction(
        period=5.0,
        condition=IfCondition(LaunchConfiguration("autostart")),
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "lifecycle", "set", "--no-daemon", "--spin-time", "10",
                    node_name, "activate"
                ],
                output="screen",
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace",
            default_value="camera_rospkg",
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
            description="Camera device path or video file path used before lifecycle configure.",
        ),
        container,
        autoconfigure,
        autoactivate,
    ])
