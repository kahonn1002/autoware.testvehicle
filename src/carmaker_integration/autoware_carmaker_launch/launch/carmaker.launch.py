import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def generate_launch_description():
    autoware_share = get_package_share_directory("autoware_launch")
    carmaker_launch_share = get_package_share_directory("autoware_carmaker_launch")
    interface_share = get_package_share_directory("autoware_carmaker_interface")
    vehicle_share = get_package_share_directory("carmaker_fsai_vehicle_launch")
    diagnostic_graph_path = os.path.join(
        carmaker_launch_share,
        "config",
        "autoware-carmaker.yaml",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_path = LaunchConfiguration("map_path")
    rviz = LaunchConfiguration("rviz")
    perception = LaunchConfiguration("perception")

    autoware = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(autoware_share, "launch", "autoware.launch.xml")),
        launch_arguments={
            "map_path": map_path,
            "vehicle_model": "carmaker_fsai_vehicle",
            "sensor_model": "carmaker_sensor_kit",
            "use_sim_time": use_sim_time,
            "launch_vehicle": "true",
            "launch_vehicle_interface": "false",
            "launch_system": "true",
            "system_run_mode": "planning_simulation",
            "launch_system_monitor": "false",
            "launch_dummy_diag_publisher": "true",
            "diagnostic_graph_aggregator_graph_path": diagnostic_graph_path,
            "launch_sensing": "false",
            "launch_sensing_driver": "false",
            "launch_localization": "false",
            "launch_perception": perception,
            "launch_planning": "true",
            "launch_control": "true",
            "launch_api": "true",
            "is_simulation": "true",
            "enable_all_modules_auto_mode": "true",
            "perception_mode": "lidar",
            # The pointcloud-based occupancy node requires exact timestamp
            # equality between raw and segmented clouds. CarMaker's 10 ms
            # ROS job can repeat its 20 ms LiDAR scan, so use Autoware's
            # simulator configuration and consume the segmented cloud only.
            "occupancy_grid_map_method": "laserscan_based",
            "input_obstacle_pointcloud": "true",
            "input_obstacle_and_raw_pointcloud": "false",
            "use_traffic_light_recognition": "false",
            "traffic_light_recognition/use_high_accuracy_detection": "false",
            "rviz": rviz,
        }.items(),
    )

    interface = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(interface_share, "launch", "interface.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    vehicle_interface = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(vehicle_share, "launch", "vehicle_interface.launch.xml")
        )
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                "PATH", ["/usr/bin:/bin:", EnvironmentVariable("PATH")]
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "map_path",
                default_value="/home/klee/autoware.testvehicle/maps/fs_autonomous_sprint",
            ),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("perception", default_value="true"),
            interface,
            vehicle_interface,
            autoware,
        ]
    )
