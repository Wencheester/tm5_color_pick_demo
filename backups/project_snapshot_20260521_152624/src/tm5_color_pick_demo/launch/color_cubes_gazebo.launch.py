import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    demo_pkg = "tm5_color_pick_demo"
    gazebo_pkg = "tm_gazebo"

    demo_dir = get_package_share_directory(demo_pkg)
    gazebo_dir = get_package_share_directory(gazebo_pkg)
    world_file = os.path.join(demo_dir, "worlds", "color_cubes.sdf")

    gui = LaunchConfiguration("gui")
    camera_view = LaunchConfiguration("camera_view")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(demo_pkg), "xacro", "tm5-700_color_pick.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    start_gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": [" -s -r -v 3 ", world_file]}.items(),
        condition=UnlessCondition(gui),
    )

    start_gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": [" -r -v 4 ", world_file]}.items(),
        condition=IfCondition(gui),
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-allow_renaming",
            "true",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.0",
            "-R",
            "0.0",
            "-P",
            "0.0",
            "-Y",
            "0.0",
        ],
    )

    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/tool_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/tool_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        remappings=[
            ("/tool_camera/image", "/tool_camera/image_raw"),
        ],
    )

    camera_viewer = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="tool_camera_view",
        output="screen",
        arguments=["/tool_camera/image_raw"],
        condition=IfCondition(camera_view),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui",
                default_value="false",
                description="Start Gazebo GUI.",
            ),
            DeclareLaunchArgument(
                "camera_view",
                default_value="false",
                description="Open rqt_image_view for /tool_camera/image_raw.",
            ),
            SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1"),
            SetEnvironmentVariable("RMW_FASTRTPS_USE_SHM", "0"),
            AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(gazebo_dir, "models")),
            start_gz_sim_headless,
            start_gz_sim_gui,
            robot_state_publisher_node,
            TimerAction(period=2.0, actions=[spawn_robot]),
            camera_bridge,
            TimerAction(period=4.0, actions=[camera_viewer]),
        ]
    )
