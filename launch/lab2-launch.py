from launch_ros.actions import Node, SetParameter
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    sim_share = get_package_share_directory("ME465_Sim")
    description_share = get_package_share_directory("ME465_Description")
    lab2_share = get_package_share_directory("ME465_Lab2")
    with open(os.path.join(lab2_share, "simulation.yaml")) as f:
        params = yaml.safe_load(f)
    return LaunchDescription([
        DeclareLaunchArgument(
            name="simulation",
            default_value="false",
            description="Start simulation",
        ),
        DeclareLaunchArgument(
            name="visualization",
            default_value="true",
            description="Show visualization",
        ),
        DeclareLaunchArgument(
            name="record-bag",
            default_value="false",
            description="Filename to save data to",
        ),
        DeclareLaunchArgument(
            name="play-bag",
            default_value="false",
            description="Filename to read data from",
        ),
        SetParameter(
            name="use_sim_time",
            value=LaunchConfiguration("simulation"),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sim_share, "launch", "simulation-launch.py"),
            ),
            condition=IfCondition(LaunchConfiguration("simulation")),
        ),
        Node(
            condition=IfCondition(LaunchConfiguration("simulation")),
            package="apriltag_ros",
            executable="apriltag_node",
            parameters=[params["apriltag"]["ros__parameters"]],
            remappings=(
                ("/image_rect", "/camera2/image_raw"),
                ("/camera_info", "/camera2/camera_info"),
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(description_share, "launch", "description-launch.py"),
            ),
        ),
        Node(
            package="ME465_Lab2",
            executable="lab2_node",
            parameters=[
                {"map": [-1.5, 0.0, 2.0]},
            ],
        ),
        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration("record-bag")),
            cmd=(
                "ros2",
                "bag",
                "record",
                "-o",
                "lab2.bag",
                "/detected",
                "/commands/velocity",
            ),
        ),
        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration("play-bag")),
            cmd=(
                "ros2",
                "bag",
                "play",
                "lab2.bag",
            ),
        ),
        Node(
            condition=IfCondition(LaunchConfiguration("visualization")),
            package="ME465_Lab2",
            executable="plot_node",
        ),
        Node(
            condition=UnlessCondition(LaunchConfiguration("play-bag")),
            package="ME465_Lab2",
            executable="detection_node",
            parameters=[
                {"size": 0.2},
            ],
            remappings=[
                ("/camera_info", "/camera2/camera_info"),
            ],
        ),
    ])
