from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    pkg_share = get_package_share_directory("camera_rospkg")
    default_params_file = os.path.join(pkg_share, "config", "camera_params.yaml")
    default_calibration = os.path.join(pkg_share, "config", "calibration.yaml")

    # Read autostart directly from the default params file at generation time.
    with open(default_params_file, "r") as f:
        _params_yaml = yaml.safe_load(f)
    autostart = bool(_params_yaml.get("autostart", True))

    # Read calibration_url from params file; fall back to package default if empty.
    calibration_url = ""
    for key, val in _params_yaml.items():
        if isinstance(val, dict) and "ros__parameters" in val:
            calibration_url = val["ros__parameters"].get("calibration_url", "")
            break
    if not calibration_url:
        calibration_url = default_calibration

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
                parameters=[LaunchConfiguration("params_file"), {"calibration_url": calibration_url}],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
    )

    actions = [
        DeclareLaunchArgument(
            "namespace",
            default_value="camera_rospkg",
            description="Namespace for the camera container and camera topics.",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description=(
                "Path to a ROS2 params YAML file for the camera node. "
                "Defaults to config/camera_params.yaml inside the camera_rospkg share directory."
            ),
        ),
        container,
    ]

    if autostart:
        actions += [
            TimerAction(
                period=2.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2", "lifecycle", "set", "--no-daemon", "--spin-time", "10",
                            node_name, "configure"
                        ],
                        output="screen",
                    ),
                ],
            ),
            TimerAction(
                period=5.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2", "lifecycle", "set", "--no-daemon", "--spin-time", "10",
                            node_name, "activate"
                        ],
                        output="screen",
                    ),
                ],
            ),
        ]

    return LaunchDescription(actions)
