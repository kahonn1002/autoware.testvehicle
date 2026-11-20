"""Launch Autoware against a CarMaker vehicle/sensor profile.

The profile name is deliberately the only profile selector exposed to the
launch file. Profile files are package data (config/profiles, including the
profiles written by testrun_to_autoware), rather than arbitrary paths accepted
from the command line, so a typo cannot silently select another configuration.
"""

import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _profile_files(launch_share: str) -> dict:
    """Profiles installed as package data, keyed by name."""
    profile_dir = Path(launch_share) / "config" / "profiles"
    return {path.stem: path for path in sorted(profile_dir.glob("*.yaml"))}


def _load_profile(profile_name: str, launch_share: str) -> dict:
    """Load one of the installed profile files."""
    profiles = _profile_files(launch_share)
    try:
        path = profiles[profile_name]
    except KeyError as exc:
        names = ", ".join(profiles) or "none installed"
        raise RuntimeError(
            f"Unknown CarMaker profile '{profile_name}'. Expected one of: {names}."
        ) from exc

    try:
        with path.open(encoding="utf-8") as stream:
            profile = yaml.safe_load(stream)
    except OSError as exc:
        raise RuntimeError(f"Unable to read CarMaker profile '{profile_name}': {path}") from exc

    if not isinstance(profile, dict):
        raise RuntimeError(f"CarMaker profile '{profile_name}' must contain a YAML mapping")
    return profile


def _resolve_map_path(map_path: str, launch_share: str) -> str:
    """Resolve a workspace-relative map while retaining explicit paths."""
    if os.path.isabs(map_path):
        return map_path

    relative = Path(map_path)
    candidates = [Path.cwd() / relative]
    share_path = Path(launch_share)
    candidates.extend(parent / relative for parent in share_path.parents)

    # This is also useful when launching from an installed overlay whose
    # source workspace is not represented by the current working directory.
    workspace_root = os.environ.get("AUTOWARE_TESTVEHICLE_ROOT")
    if workspace_root:
        candidates.insert(0, Path(workspace_root) / relative)

    for candidate in candidates:
        if candidate.exists():
            return str(candidate)

    # Keep a deterministic path for a generated ISO map before it exists.
    return str(candidates[0])


def _profile_actions(context, *_, **__):
    launch_share = get_package_share_directory("autoware_carmaker_launch")
    profile_name = LaunchConfiguration("profile").perform(context)
    profile = _load_profile(profile_name, launch_share)

    requested_map = LaunchConfiguration("map_path").perform(context)
    map_path = requested_map or _resolve_map_path(profile["map_path"], launch_share)

    requested_camera_bridge = LaunchConfiguration("camera_bridge").perform(context)
    camera_bridge = requested_camera_bridge or str(
        bool(profile.get("camera_bridge", False))
    ).lower()

    requested_auto_route = LaunchConfiguration("auto_route").perform(context)
    auto_route = requested_auto_route or str(bool(profile.get("auto_route", False))).lower()
    if auto_route not in ("true", "false"):
        raise RuntimeError(f"auto_route must be true or false, got '{auto_route}'")
    scenario_file = os.path.join(map_path, "scenario.yaml")
    if auto_route == "true" and not os.path.isfile(scenario_file):
        raise RuntimeError(
            f"auto_route needs {scenario_file}; convert the TestRun with testrun_to_autoware"
        )

    autoware_share = get_package_share_directory("autoware_launch")
    interface_share = get_package_share_directory(profile["interface_package"])
    vehicle_share = get_package_share_directory(profile["vehicle_launch_package"])

    diagnostic_graph_path = os.path.join(
        launch_share,
        "config",
        "autoware-carmaker.yaml",
    )
    interface_params_path = os.path.join(
        interface_share,
        "config",
        profile["interface_params"],
    )
    converter_params_path = os.path.join(
        vehicle_share,
        "config",
        profile["converter_params"],
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")
    perception = LaunchConfiguration("perception")

    autoware = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(autoware_share, "launch", "autoware.launch.xml")
        ),
        launch_arguments={
            "map_path": map_path,
            "vehicle_model": profile["vehicle_model"],
            "sensor_model": profile["sensor_model"],
            "use_sim_time": use_sim_time,
            "launch_vehicle": "true",
            # The profile's vehicle interface is included below exactly once.
            "launch_vehicle_interface": "false",
            "launch_system": "true",
            "system_run_mode": "planning_simulation",
            "launch_system_monitor": "false",
            "launch_dummy_diag_publisher": "true",
            "diagnostic_graph_aggregator_graph_path": diagnostic_graph_path,
            # CarMaker's bridge supplies sensing topics and camera topics are
            # published by the selected sensor kit independently.
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
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": interface_params_path,
            "enable_cameras": camera_bridge,
            "publish_vehicle_control": LaunchConfiguration("publish_vehicle_control"),
        }.items(),
    )

    vehicle_interface = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(vehicle_share, "launch", "vehicle_interface.launch.xml")
        ),
        launch_arguments={
            "raw_vehicle_cmd_converter_param_path": converter_params_path,
        }.items(),
    )

    actions = [interface, vehicle_interface, autoware]
    if auto_route == "true":
        actions.append(
            Node(
                package=profile["interface_package"],
                executable="scenario_route_node",
                name="carmaker_scenario_route",
                output="screen",
                parameters=[
                    {
                        "scenario_file": scenario_file,
                        "auto_engage": ParameterValue(
                            LaunchConfiguration("auto_engage"), value_type=bool
                        ),
                        "use_sim_time": use_sim_time,
                    }
                ],
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            SetEnvironmentVariable(
                "PATH", ["/usr/bin:/bin:", EnvironmentVariable("PATH")]
            ),
            DeclareLaunchArgument(
                "profile",
                default_value="fsai_sprint",
                description="CarMaker profile in config/profiles (fsai_sprint, "
                "iso_lanechange_demo or a converted TestRun)",
            ),
            DeclareLaunchArgument(
                "map_path",
                default_value="",
                description="Map directory; empty selects the active profile's map",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("perception", default_value="true"),
            DeclareLaunchArgument(
                "publish_vehicle_control", default_value="true",
                description="Set false for a data-only run; fresh copied TestRun stays in its driver hold",
            ),
            DeclareLaunchArgument(
                "camera_bridge",
                default_value="",
                description="Override profile camera bridge setting (true or false)",
            ),
            DeclareLaunchArgument(
                "auto_route",
                default_value="",
                description="Set the route from <map>/scenario.yaml (default: profile setting)",
            ),
            DeclareLaunchArgument(
                "auto_engage",
                default_value="false",
                description="With auto_route, request autonomous mode once a route is set",
            ),
            OpaqueFunction(function=_profile_actions),
        ]
    )
