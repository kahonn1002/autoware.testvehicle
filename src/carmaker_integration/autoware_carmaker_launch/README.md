# Running Autoware with CarMaker 15

The integration is isolated in `/home/klee/autoware.testvehicle`; the main
Autoware checkout is not used at runtime and is not modified.

## 1. Build

Build the seven integration packages with the system Python. This avoids a
Conda Python version taking precedence over ROS Humble's Python 3.10.

```bash
cd /home/klee/autoware.testvehicle
source /opt/ros/humble/setup.bash
PATH=/usr/bin:/bin:$PATH /usr/bin/colcon build --symlink-install --packages-select \
  vehiclecontrol_msgs \
  autoware_carmaker_interface \
  carmaker_fsai_vehicle_description \
  carmaker_fsai_vehicle_launch \
  carmaker_sensor_kit_description \
  carmaker_sensor_kit_launch \
  autoware_carmaker_launch
```

Build the duplicated CarMaker project:

```bash
cd /home/klee/project/Carmaker_proj/FSAI_LTS_2026_CM15_Autoware
PATH=/usr/bin:/bin:$PATH ./build.sh 2
```

## 2. Start

1. Start the duplicated CarMaker project:

   ```bash
   cd /home/klee/project/Carmaker_proj/FSAI_LTS_2026_CM15_Autoware
   ./CMStart.sh 2
   ```

2. In another terminal, start Autoware before starting the TestRun:

   ```bash
   cd /home/klee/autoware.testvehicle
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ros2 launch autoware_carmaker_launch carmaker.launch.py
   ```

3. In CarMaker, load `FS_autonomous_Sprint_LTS`. Use the CMRosIF launch action
   to start `hellocm`, then start the simulation. If the CMRosIF terminal action
   is unavailable, start it manually from the duplicated CarMaker directory:

   ```bash
   source ros_setup.bash 2
   ros2 launch hellocm hellocm.launch.py use_sim_time:=True rviz2:=False
   ```

CarMaker supplies `/clock`, so the default `use_sim_time:=true` is correct for
the connected run. For an Autoware-only startup check, use
`use_sim_time:=false perception:=false rviz:=false`.
The duplicated CMROSIF keeps `/clock` monotonic across repeated TestRuns. The
interface retimestamps CarMaker sensor data into that clock domain, clears the
old route, and republishes `/initialpose3d` when it detects the raw CarMaker
timestamp returning to zero. Autoware and RViz therefore remain running between
TestRuns.
The Sprint map and sensor kit do not provide an Autoware traffic-light camera,
so the camera6/camera7 traffic-light recognition pipelines are disabled while
LiDAR perception remains enabled.

## 3. Verify and engage

Before engagement, verify that the sensor and localization inputs are live:

```bash
ros2 topic hz /carmaker/odometry
ros2 topic hz /carmaker/lidar/front/points
ros2 topic hz /localization/kinematic_state
ros2 topic hz /perception/occupancy_grid_map/map
ros2 topic echo --once /api/localization/initialization_state
ros2 topic echo --once /initialpose3d
```

Ground-truth localization is aligned to the generated Sprint map, so a manual
initial-pose estimate is unnecessary. The interface publishes the first
CarMaker pose on `/initialpose3d` so Autoware's localization readiness monitor
also becomes healthy. In RViz, wait until the Autoware state is
`WaitingForRoute`, select **2D Goal Pose**, set a forward-facing goal on the
lane centerline, and confirm the route and trajectory:

```bash
ros2 topic echo --once /api/routing/state
ros2 topic echo --once /planning/mission_planning/route
ros2 topic hz /planning/trajectory
ros2 topic echo --once /carmaker/VehicleControl
```

`WaitingForRoute` is not an actuator error: autonomous mode remains unavailable
until a goal has produced a route and trajectory. The launch enables all
planning modules' auto mode, but it intentionally does not choose a destination
or engage driving. Engage Autoware only after the trajectory and command
direction look correct.
Keep CarMaker paused for the first command check. The interface applies 35%
brake if Autoware's actuation command is stale for 0.5 seconds.

A few MRM emergency-stop transitions can occur during the first seconds after
CarMaker data starts, while diagnostic and planning nodes receive their first
samples. Do not request autonomous mode during that startup window; wait for
`WaitingForRoute`, set the goal, and wait for a live trajectory first.
The wrapper uses a CarMaker-specific diagnostic graph. Its three-second
localization and MRM timeouts prevent short perception-load stalls from
repeatedly toggling the emergency stop. A warning-level planner trajectory rate
is allowed because a usable trajectory is still arriving; an `ERROR`, a stale
trajectory, invalid trajectory, missing route, or localization failure still
blocks autonomous mode.

The integration uses Autoware's laser-scan occupancy implementation with the
segmented CarMaker point cloud. Do not change it back to the pointcloud-pair
implementation without also synchronizing the raw and segmented stamps:
CarMaker's ROS job runs every 10 ms while its LiDAR scan updates every 20 ms.
The pair implementation otherwise waits forever for two exactly equal latest
timestamps, leaving behavior planning at `waiting for occupancy_grid`.

## 4. Calibration

The actuation maps are safe initial linear maps. They must be calibrated from
CarMaker step tests before increasing the Sprint speed limit. Confirm
`steering_command_sign`, `steering_feedback_sign`, selector values, and steering
ratio at low speed before a closed-loop lap.

The duplicated FSAI vehicle intentionally has
`PowerTrain.Control.GasInterpret.TrqZero = 0`. The Autoware acceleration map
uses zero pedal as zero acceleration; the original `-55` value made every
normal low-acceleration command below 50% pedal request regenerative torque
instead of forward torque.

## 5. Converting other TestRuns

`testrun_to_autoware` turns a CarMaker TestRun into an Autoware map, a scenario
and a launch profile. Road geometry is evaluated with CarMaker 15's own IPGRoad
Python API (`Python/IPGRoad` in the installation, IPGRoad license required), so
the map lies exactly where CarMaker's ground-truth odometry puts the vehicle.
The newest installed CarMaker is used; pass `--carmaker-home` to override.

```bash
cd /home/klee/autoware.testvehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
P=/home/klee/project/Carmaker_proj/FSAI_LTS_2026_CM15_Autoware
ros2 run autoware_carmaker_launch testrun_to_autoware FS_autonomous_Sprint_LTS --project $P --build
ros2 launch autoware_carmaker_launch carmaker.launch.py profile:=fs_autonomous_sprint_lts
```

Then load the same TestRun in CarMaker and start it. Once CarMaker's pose has
initialized localization, `scenario_route_node` sets the converted goal (and
waypoints); after a TestRun restart it sets the route again. Engage manually,
or add `auto_engage:=true`.

Each conversion writes:

| Output | Content |
|---|---|
| `maps/<name>/lanelet2_map.osm` | Lanelets in CarMaker's global frame (Local projector) |
| `maps/<name>/pointcloud_map.pcd` | Synthetic road surface plus cone/pylon/post columns; not localization-grade |
| `maps/<name>/scenario.yaml` | Source hashes, ego start, goal, waypoints, vehicle check, validation, warnings |
| `config/profiles/<name>.yaml` | Base vehicle/sensor profile with the new map and `auto_route: true` |

The mode is chosen from the ego route (`Vehicle.Routing`):

- `corridor`: a lanelet chain along the route, bounded by `DrvPylon` gates
  (ISO lane change), by the cone rows (Formula Student; blue left, yellow right,
  orange cones ignored), or by the path width. Selected when the route has
  pylons, at least four cones per side, or uses user paths.
- `network`: every drivable lane of every link (shared boundaries, so lane
  changes are possible), junction lanes with `turn_direction`, and IPGRoad's
  path topology. Waypoints on each link of the route make Autoware follow the
  TestRun's route rather than the shortest one.

Before writing, the map is loaded the way Autoware's Local projector loads it
and routed from the ego start through the waypoints to the goal with Lanelet2;
the tool exits non-zero if that fails. Speed limits come from the road
(`--max-speed-kmh` caps them, default 20 km/h, because the actuation maps are
uncalibrated). The CarMaker vehicle must map to an existing profile
(`FS-AI_IMechE_ADS-DV_2026` and `DemoCar` variants); for others pass
`--base-profile`. Differences between the CarMaker vehicle file and that
profile's `vehicle_info.param.yaml` are reported as warnings.

Not converted: traffic lights, stop lines and right-of-way rules (Autoware
does not stop for CarMaker signals), moving traffic (perceived by the LiDAR at
runtime), elevation-dependent bumps, and start positions other than
`Vehicle.StartPos.Type = Route`. A route that drives the same lane twice in the
same direction (SkidPad) produces overlapping lanelets and a warning. TestRuns
saved by older CarMaker versions may need to be opened and saved once in
CarMaker 15 before IPGRoad 15 can read their road.
