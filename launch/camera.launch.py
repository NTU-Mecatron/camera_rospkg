from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableLifecycleNode


def generate_launch_description():
    config_dir = PathJoinSubstitution([FindPackageShare('camera_rospkg'), 'config'])
    config_file = PathJoinSubstitution([config_dir, 'camera_params.yaml'])
    calibration_file = PathJoinSubstitution([config_dir, 'calibration.yaml'])

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
                    config_file,
                    {"calibration_url": calibration_file},
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
            default_value="camera",
            description="Namespace for the camera lifecycle node."
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Whether to automatically transition the camera node to 'active' state."
        ),
        container,
        OpaqueFunction(function=lifecycle_startup_actions),
    ])
