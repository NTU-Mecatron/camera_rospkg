import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableLifecycleNode


def generate_launch_description():
    pkg_share = get_package_share_directory("camera_rospkg")

    container = ComposableNodeContainer(
        name="camera_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableLifecycleNode(
                package="camera_rospkg",
                plugin="camera_rospkg::CameraPublisher",
                name="camera_publisher",
                namespace=LaunchConfiguration("namespace"),
                parameters=[
                    LaunchConfiguration("params_file"),
                    {"calibration_url": os.path.join(pkg_share, "config", "calibration.yaml")},
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
                autostart=False,
            ),
        ],
        output="screen",
    )

    def lifecycle_startup_actions(context):
        namespace = LaunchConfiguration("namespace").perform(context).strip("/")
        full_node_name = f"/{namespace}/camera_publisher" if namespace else "/camera_publisher"

        if LaunchConfiguration("autostart").perform(context).strip().lower() != "true":
            return []

        configure_cmd = ExecuteProcess(
            cmd=["ros2", "lifecycle", "set", full_node_name, "configure"], output="screen")
        activate_cmd = ExecuteProcess(
            cmd=["ros2", "lifecycle", "set", full_node_name, "activate"], output="screen")

        return [
            TimerAction(period=2.0, actions=[configure_cmd]),
            TimerAction(period=4.0, actions=[activate_cmd]),
        ]

    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace",
            default_value="camera_rospkg",
            description="Namespace for the camera lifecycle node. Defaults to 'camera_rospkg'."
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(pkg_share, "config", "camera_params.yaml"),
            description="Path to a ROS2 params YAML file for the camera node. Defaults to config/camera_params.yaml."
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Whether to automatically transition the camera node to 'active' state."
        ),
        container,
        OpaqueFunction(function=lifecycle_startup_actions),
    ])
