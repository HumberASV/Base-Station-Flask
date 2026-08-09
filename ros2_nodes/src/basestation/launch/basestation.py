from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")
    factory = LaunchConfiguration("factory")

    # annotated_node = Node(
    #     package="basestation",
    #     executable="stream",
    #     name="udp_stream",
    #     arguments=[
    #         "--image_topic", "zedx/zed_node/stereo/color/rect/image",
    #         "--objects_topic","zedx/zed_node/obj_det/objects"
    #         ]
    # )

    app_node = Node(
        package="basestation",
        executable="flask",
        name="app",
        arguments=["--host", host, "--port", port, "--factory", factory],
    )

    return LaunchDescription([
        DeclareLaunchArgument("host", default_value="192.168.137.1",
                               description="Host IP addr"),
        DeclareLaunchArgument("port", default_value="8080",
                               description="Host port addr"),
        DeclareLaunchArgument("factory", default_value="false",
                               description="Stream factory (fake) telemetry instead of connecting to ROS2/the ASV"),
        # TODO(Carson): Launch description for isDashboard
        # annotated_node,
        app_node,
    ])
