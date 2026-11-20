from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("autoware_carmaker_interface"),
        "config",
        "interface.param.yaml",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("enable_cameras", default_value="false"),
            DeclareLaunchArgument("publish_vehicle_control", default_value="true"),
            Node(
                package="autoware_carmaker_interface",
                executable="interface_node",
                name="autoware_carmaker_interface",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                    {"enable_cameras": ParameterValue(LaunchConfiguration("enable_cameras"), value_type=bool)},
                    {"publish_vehicle_control": ParameterValue(LaunchConfiguration("publish_vehicle_control"), value_type=bool)},
                ],
            ),
        ]
    )

