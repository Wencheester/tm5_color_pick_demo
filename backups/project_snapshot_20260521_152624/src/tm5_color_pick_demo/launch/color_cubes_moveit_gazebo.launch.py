import os
import sys

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except OSError:
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except OSError:
        return None


def generate_launch_description():
    args = []
    if len(sys.argv) >= 5:
        args.extend(sys.argv[4:])

    demo_pkg = "tm5_color_pick_demo"
    moveit_config_pkg = "tm5-700_moveit_config"
    gazebo_pkg = "tm_gazebo"

    run_simulator = LaunchConfiguration("sim")
    run_rviz = LaunchConfiguration("use_rviz")
    camera_view = LaunchConfiguration("camera_view")

    demo_dir = get_package_share_directory(demo_pkg)
    gazebo_dir = get_package_share_directory(gazebo_pkg)
    world_file = os.path.join(demo_dir, "worlds", "color_cubes.sdf")
    rviz_config_file = os.path.join(gazebo_dir, "rviz", "view_robot.rviz")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(demo_pkg), "xacro", "tm5-700_color_pick.urdf.xacro"]
            ),
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_description_semantic = {
        "robot_description_semantic": load_file(demo_pkg, "config/tm5-700_color_pick_gz.srdf")
    }
    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(moveit_config_pkg, "config/kinematics.yaml")
    }

    ompl_planning_pipeline_config = {
        "planning_pipelines": ["ompl"],
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
        },
    }
    ompl_planning_yaml = load_yaml(moveit_config_pkg, "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["ompl"].update(ompl_planning_yaml)

    controllers_yaml = load_yaml(moveit_config_pkg, "config/controllers.yaml")
    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.1,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    joint_limits_yaml = {
        "robot_description_planning": load_yaml(moveit_config_pkg, "config/joint_limits.yaml")
    }

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            joint_limits_yaml,
            {"use_sim_time": True},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            joint_limits_yaml,
            {"use_sim_time": True},
        ],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    joint_state_broadcaster_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(run_rviz),
    )

    tm_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "tmr_arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
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

    start_gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": [" -r -v 4 ", world_file]}.items(),
        condition=IfCondition(run_simulator),
    )

    ros_gz_bridge = Node(
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

    tm_driver_node = Node(
        package="tm_driver",
        executable="tm_driver",
        output="screen",
        arguments=args,
        condition=UnlessCondition(run_simulator),
    )

    planning_scene_objects = Node(
        package=demo_pkg,
        executable="planning_scene_objects",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sim",
                default_value="true",
                description="Use the virtual TM Robot simulation in Gazebo.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz with the MoveIt configuration.",
            ),
            DeclareLaunchArgument(
                "camera_view",
                default_value="false",
                description="Open rqt_image_view for /tool_camera/image_raw.",
            ),
            SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1"),
            SetEnvironmentVariable("RMW_FASTRTPS_USE_SHM", "0"),
            AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(gazebo_dir, "models")),
            robot_state_publisher_node,
            joint_state_broadcaster_spawner,
            joint_state_broadcaster_handler,
            tm_arm_controller_spawner,
            spawn_robot,
            start_gz_sim,
            ros_gz_bridge,
            TimerAction(period=4.0, actions=[camera_viewer]),
            tm_driver_node,
            run_move_group_node,
            TimerAction(period=8.0, actions=[planning_scene_objects]),
        ]
    )
